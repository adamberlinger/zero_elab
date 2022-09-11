#!/bin/bash
test_callbacks=()
count=0;
for folder in "$@"
do
    if [ -d "$folder" ]; then
        files=$(ls $folder | grep '\.c$');
        for file in $files
        do
            test_callbacks+=(${file%.*});
            count=$((count+1))
        done
    fi
done

for test_c in ${test_callbacks[@]}
do
    echo "int $test_c(comm_t*);";
done
echo "";
echo "#define TEST_LIST_SIZE ($count)"
echo "test_list_entry_t test_list[TEST_LIST_SIZE] = {";
for test_c in ${test_callbacks[@]}
do
    echo "{$test_c, \"$test_c\"},";
done
echo "};";
