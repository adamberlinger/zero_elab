#!/bin/bash

# -r option to remove default rules and significantly speed-up
# compilation in MinGW environment
make="make -r"

folders=`ls target`
if hash nproc 2>/dev/null; then
    num_threads=`nproc`
    num_threads=$((num_threads*2))
else
    num_threads=4;
fi
if [ -n "$1" ]; then
    if [ $1 = "scan" ]; then
        echo "Scanning available targets..."
        targets=()
        dfu_targets=()
        for t in $folders
        do
            if $make TARGET_NAME=$t test_abstract &>/dev/null; then
                targets+=($t);
                echo "$t";
                if $make TARGET_NAME=$t test_dfu &>/dev/null; then
                    dfu_targets+=($t);
                fi
            fi
        done
        rm target_list.txt
        for t in ${targets[@]}
        do
            echo "$t" >> target_list.txt
        done
        rm dfu_list.txt
        for t in ${dfu_targets[@]}
        do
            echo "$t" >> dfu_list.txt
        done
    else
        echo "Using target cache. To refresh this, run './build.sh scan'"
        IFS=$'\r\n' GLOBIGNORE='*' command eval  'targets=($(cat target_list.txt))'
        IFS=$'\r\n' GLOBIGNORE='*' command eval  'dfu_targets=($(cat dfu_list.txt))'
    fi
fi

if [ -z "$1" ]; then
    echo "Usage $0 <command>";
    echo "";
    echo "List of commands:";
    echo "    scan         - scans all available targets creating cached list"
    echo "    list         - lists all available targets for build";
    echo "    configure    - configures Makefile.user (e.g. select target)";
    echo "    clean_all    - runs 'make clean' for all targets";
    echo "    build_all    - runs 'make bin' for all targets";
    echo "    package      - runs 'build_all' and creates zip file containing all binary files";
    echo "    gen_projects - generate projects files for IDEs (e.g. Keil, SCIDE) for all targets"
    echo "";
    echo "All makefile operations are called with '-jN' parameter (number of jobs)
where N/2 is number of cores returned by 'nproc' utility. If nproc is not installed N=4.";
elif [ $1 = "list" ]; then
    echo "Available targets are: "
    i=0
    for t in ${targets[@]}
    do
        echo "($i) $t"
        i=$((i+1))
    done
    echo "Threads: $num_threads"
elif [ $1 = "package" ]; then
    if hash zip 2>/dev/null; then
        files=build/package.zip
        for t in ${targets[@]}
        do
            echo -e "\e[92m\e[1mGenerating BIN file for $t...\e[0m"
            if ! output=$($make "-j$num_threads" TARGET_NAME=$t bin 2>&1); then
                echo -e "\e[31mError occured for '$t'\e[0m"
            fi
            files="$files build/$t/$t.bin"
        done

        for t in ${dfu_targets[@]}
        do
            echo -e "\e[92m\e[1mGenerating DFU file for $t...\e[0m"
            if ! output=$($make "-j$num_threads" TARGET_NAME=$t dfu 2>&1); then
                echo -e "\e[31mError occured for '$t'\e[0m"
            fi
            files="$files build/$t/$t.dfu"
        done
        rm -rf build/package.zip
        zip -j $files
    else
        echo "Error: You need to have 'zip' command Available to make package";
        exit
    fi
elif [ $1 = "build_all" ]; then
    for t in ${targets[@]}
    do
        echo -e "\e[92m\e[1mGenerating BIN file for $t...\e[0m"
        if ! output=$($make "-j$num_threads" TARGET_NAME=$t bin 2>&1); then
            echo -e "\e[31mError occured for '$t'\e[0m"
        fi
        files="$files build/$t/$t.bin"
    done

    for t in ${dfu_targets[@]}
    do
        echo -e "\e[92m\e[1mGenerating DFU file for $t...\e[0m"
        if ! output=$($make "-j$num_threads" TARGET_NAME=$t dfu 2>&1); then
            echo -e "\e[31mError occured for '$t'\e[0m"
        fi
        files="$files build/$t/$t.dfu"
    done
elif [ $1 = "clean_all" ]; then
    for t in ${targets[@]}
    do
        echo "Cleaning files for $t..."
        $make TARGET_NAME=$t clean &>/dev/null
    done
elif [ $1 = "gen_projects" ]; then
    for t in ${targets[@]}
    do
        echo "Generating project files for $t..."
        $make TARGET_NAME=$t gen_project
    done
elif [ $1 = "configure" ]; then
    echo "Available targets are: "
    i=0
    for t in ${targets[@]}
    do
        echo "($i) $t"
        i=$((i+1))
    done
    echo "Please enter target name: "
    read x
    target=${targets[${x}]}
    echo "You selected '$target'"
    echo "TARGET_NAME?=$target" >Makefile.user
fi
