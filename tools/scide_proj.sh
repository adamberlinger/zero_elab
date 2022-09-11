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
    elif [ $i = 11 ]; then
        opts=$line;
    fi
    i=$(($i + 1));
done < "$1"

new_inc_paths=()
for i in $inc_paths
do
    new_inc_paths="$new_inc_paths $path_prefix${i}";
done

mkdir -p $2
config_file="$2/configurations.xml"
project_file="$2/project.xml"
makefile="$2/../Makefile"

cat "tools/scide_proj/scide_proj_part1.txt" > "$project_file";
echo "<name>$3</name>" >> "$project_file";
cat "tools/scide_proj/scide_proj_part2.txt" >> "$project_file";

cat "tools/scide_proj/scide_proj_part3.txt" > "$config_file";
for folder in $src_folders
do
    echo "<logicalFolder name=\"$folder\" displayName=\"$folder\" projectFiles=\"true\">" >> "$config_file";
    files=$(ls $folder | grep '\.c$');
    for file in $files
    do
        echo "<itemPath>$path_prefix$folder/$file</itemPath>" >> "$config_file";
    done
    echo "</logicalFolder>" >> "$config_file";
done

echo "<logicalFolder name=\"api\" displayName=\"api\" projectFiles=\"true\">" >> "$config_file";
for file in $api_files
do
    echo "<itemPath>${path_prefix}api/$file</itemPath>" >> "$config_file";
done
echo "</logicalFolder>" >> "$config_file";

echo "<logicalFolder name=\"modules\" displayName=\"modules\" projectFiles=\"true\">" >> "$config_file";
for file in $module_files
do
    echo "<itemPath>${path_prefix}modules/$file</itemPath>" >> "$config_file";
done
echo "</logicalFolder>" >> "$config_file";
echo "</logicalFolder>" >> "$config_file";
echo "</logicalFolder>" >> "$config_file";

echo "<projectmakefile>Makefile</projectmakefile>" >> "$config_file";
echo "<confs>" >> "$config_file";
echo "  <conf name=\"SC-KIT\" type=\"1\">" >> "$config_file";
echo "    <toolsSet>" >> "$config_file";
echo "      <compilerSet>CodeSourceryARM|CodeSourceryARM</compilerSet>" >> "$config_file";
echo "      <dependencyChecking>true</dependencyChecking>" >> "$config_file";
echo "      <rebuildPropChanged>false</rebuildPropChanged>" >> "$config_file";
echo "    </toolsSet>" >> "$config_file";
echo "    <compileType>" >> "$config_file";
echo "      <cTool>" >> "$config_file";
echo "        <developmentMode>0</developmentMode>" >> "$config_file";
echo "        <incDir>" >> "$config_file";
for inc_path in $new_inc_paths
do
    echo "          <pElem>$inc_path</pElem>" >> "$config_file";
done
echo "        </incDir>" >> "$config_file";
echo "        <commandLine>$opts</commandLine>" >> "$config_file";
echo "        <preprocessorList>" >> "$config_file";
for c_def in $defines
do
    echo "          <Elem>$c_def</Elem>" >> "$config_file";
done
echo "        </preprocessorList>" >> "$config_file";
echo "        <warningLevel>2</warningLevel>" >> "$config_file";
echo "      </cTool>" >> "$config_file";
echo "      <asmTool>" >> "$config_file";
echo "        <developmentMode>0</developmentMode>" >> "$config_file";
echo "        <warningLevel>2</warningLevel>" >> "$config_file";
echo "        <commandLine>-c $opts</commandLine>" >> "$config_file";
echo "      </asmTool>" >> "$config_file";
echo "      <linkerTool>" >> "$config_file";
echo '        <output>${CND_DISTDIR}/${CND_CONF}/Application.elf</output>' >> "$config_file";
echo "        <commandLine>$opts -T${path_prefix}target/$3/linkerscript.ld -L../../ -Wl,--gc-sections -Wl,-Map=${CND_DISTDIR}/${CND_CONF}/Application.map</commandLine>" >> "$config_file";
echo "      </linkerTool>" >> "$config_file";
echo "    </compileType>" >> "$config_file";

for folder in $src_folders
do
    files=$(ls $folder | grep '\.c$');
    for file in $files
    do
        echo "<item path=\"$path_prefix$folder/$file\" ex=\"false\" tool=\"0\"></item>" >> "$config_file";
    done
done


echo "  </conf>" >> "$config_file";
echo "</confs>" >> "$config_file";

cat "tools/scide_proj/scide_proj_part4.txt" >> "$config_file";

cat "tools/scide_proj/scide_proj_part5.txt" >> "$makefile";
