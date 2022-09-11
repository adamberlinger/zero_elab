#!/bin/bash
function join { local IFS="$1"; shift; echo "$*"; }
path_prefix=../../
i=0;
while IFS='' read -r line || [[ -n "$line" ]]; do
    if [ $i = 1 ]; then
        src_folders=$line;
    elif [ $i = 3 ]; then
        inc_paths=$line;
    elif [ $i = 5 ]; then
        defines=$line;
    elif [ $i = 7 ]; then
        module_files=$line;
    elif [ $i = 9 ]; then
        api_files=$line;
    elif [ $i = 13 ]; then
        sct_file=$line;
    elif [ $i = 15 ]; then
        s_file=$line;
    fi
    i=$(($i + 1));
done < "$1"

defines=$(join ',' $defines)

new_inc_paths=()
for i in $inc_paths
do
    new_inc_paths="$new_inc_paths $path_prefix${i}";
done
inc_paths=$(join ';' $new_inc_paths)

cat "tools/mdk_proj/mdk_proj_part1.txt" > "$2";
echo "<TargetName>$3</TargetName>" >> "$2";
echo "<ToolsetNumber>0x4</ToolsetNumber>" >> "$2";
echo "<ToolsetName>ARM-ADS</ToolsetName>" >> "$2";
echo "<TargetOption>" >> "$2";
echo "    <TargetCommonOption>" >> "$2";
echo "      <Device>$6</Device>" >> "$2";
echo "      <Vendor>STMicroelectronics</Vendor>" >> "$2";
echo "      <Cpu>$4</Cpu>" >> "$2";
echo "      <SFDFile>\$\$\$\$$5</SFDFile>" >> "$2";
cat "tools/mdk_proj/mdk_proj_part2.txt" >> "$2";
echo "      <Define>$defines</Define>" | sed -r 's/"/\\"/g' >> "$2";
echo "      <Undefine></Undefine>" >> "$2";
echo "      <IncludePath>$inc_paths</IncludePath>" >> "$2";

cat "tools/mdk_proj/mdk_proj_part3.txt" >> "$2";
echo "<ScatterFile>$path_prefix$sct_file</ScatterFile>" >> "$2"
cat "tools/mdk_proj/mdk_proj_part3b.txt" >> "$2";

for folder in $src_folders
do
    echo "<Group>" >> "$2";
    echo " <GroupName>$folder</GroupName>" >> "$2";
    echo " <Files>" >> "$2";
    files=$(ls $folder | grep '\.c$');
    for file in $files
    do
        echo "<File>" >> "$2";
        echo "<FileName>$file</FileName>" >> "$2";
        echo "<FileType>1</FileType>" >> "$2";
        echo "<FilePath>$path_prefix$folder/$file</FilePath>" >> "$2";
        echo "</File>" >> "$2";
    done
    echo "</Files>" >> "$2";
    echo "</Group>" >> "$2";
done

echo "<Group>" >> "$2";
echo "<GroupName>Keil</GroupName>" >> "$2";
echo "<Files><File>" >> "$2";
echo "<FileName>keil.s</FileName>" >> "$2";
echo "<FileType>2</FileType>" >> "$2";
echo "<FilePath>$path_prefix$s_file</FilePath>" >> "$2";
echo "</File></Files>" >> "$2";
echo "</Group>" >> "$2";

echo "<Group>" >> "$2";
echo " <GroupName>api</GroupName>" >> "$2";
echo " <Files>" >> "$2";
for file in $api_files
do
    echo "<File>" >> "$2";
    echo "<FileName>$file</FileName>" >> "$2";
    echo "<FileType>1</FileType>" >> "$2";
    echo "<FilePath>${path_prefix}api/$file</FilePath>" >> "$2";
    echo "</File>" >> "$2";
done
echo "</Files>" >> "$2";
echo "</Group>" >> "$2";

echo "<Group>" >> "$2";
echo " <GroupName>modules</GroupName>" >> "$2";
echo " <Files>" >> "$2";
for file in $module_files
do
    echo "<File>" >> "$2";
    echo "<FileName>$file</FileName>" >> "$2";
    echo "<FileType>1</FileType>" >> "$2";
    echo "<FilePath>${path_prefix}modules/$file</FilePath>" >> "$2";
    echo "</File>" >> "$2";
done
echo "</Files>" >> "$2";
echo "</Group>" >> "$2";

cat "tools/mdk_proj/mdk_proj_part4.txt" >> "$2";
