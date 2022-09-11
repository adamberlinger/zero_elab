/*
 * BSD 3-Clause License
 * 
 * Copyright (c) 2016-2022, Adam Berlinger
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 #include <iostream>
#include <fstream>
#include <map>
#include <stdint.h>

#define TYPE_SIZE 2

std::string types[TYPE_SIZE] = {
    "FUNC", "OBJECT"
};

void printFile(std::string& fileName,std::map<std::string, int>& sizeMap){
    std::ofstream ofile(fileName.c_str());
    int i = 0;
    for(std::map<std::string,int>::iterator it = sizeMap.begin();it != sizeMap.end();it++){
        std::string f = it->first;
        int x = f.find_last_of('/');
        ofile << i++ << " " << f.substr(x+1) << " " << it->second << std::endl;
    }
    ofile.close();
}

int main(int argc,char* argv[]){
    std::map<std::string,std::string> fileMap;
    std::map<std::string, int> codeSizeMap;
    std::map<std::string, int> ramSizeMap;
    std::map<std::string,std::string> weakFileMap;
    if(argc < 3){
        std::cerr << "Usage: " << argv[0] <<" statistics_files global_statistic_file output_file" << std::endl;
        return -1;
    }
    for(int i = 1;i < (argc - 2);++i){
        int dummy_i;
        char dummy_c;
        std::string symbolName;
        std::ifstream file(argv[i]);
        std::string fileName(argv[i]);
        int run_loop = 1;
        int l = 1;
        while(run_loop){
            std::cout << "Parsing file " << i << " / " << (argc-3) << " line " << l++ <<"\r" << std::flush;
            file >> dummy_i;
            file >> dummy_c;
            run_loop &= (bool)(file >> symbolName);

            std::map<std::string,std::string>& realFileMap = (dummy_c == 'W')?weakFileMap:fileMap;

            realFileMap.insert(std::pair<std::string,std::string>(symbolName,fileName));
        }
        file.close();
    }
    std::cout << std::endl;

    for(std::map<std::string,std::string>::iterator it = weakFileMap.begin(); it != weakFileMap.end();it++){
        if(fileMap.find(it->first) == fileMap.end()){
            fileMap.insert(*it);
        }
    }

    std::ifstream globalFile(argv[argc-2], std::ifstream::in);

    std::string line;
    /*std::getline(globalFile,line);
    std::getline(globalFile,line);
    std::getline(globalFile,line);*/

    int run_loop = 1;
    int l = 0;
    while(run_loop){
        uint32_t size,start;
        char dummy_c;
        run_loop &= (bool)(globalFile >> line);
        if(!run_loop) break;
        globalFile >> dummy_c;
        globalFile >> std::hex >> start;
        globalFile >> std::hex >> size;
        l++;

        std::cout << "Readed " << line << " " << dummy_c << " " << std::hex << start << " " << size << std::endl;

        std::map<std::string,int>& sizeMap = (dummy_c == 'B')?ramSizeMap:codeSizeMap;

        std::map<std::string,std::string>::iterator file_it = fileMap.find(line);
        if(file_it == fileMap.end()){
            std::cerr << "Symbol '" << line << "' not found!" << std::endl;
            continue;
        }

        std::map<std::string,int>::iterator it = sizeMap.find(file_it->second);
        if(it == sizeMap.end()){
            sizeMap.insert(std::pair<std::string,int>(file_it->second,size));
        }
        else {
            it->second += size;
        }
    }

    std::cout << "Maps: " << ramSizeMap.size() << ", " << ramSizeMap.size() << ", " << l << std::endl;

    std::string codeFile(argv[argc-1]);
    codeFile.append(".rom");
    std::string ramFile(argv[argc-1]);
    ramFile.append(".ram");

    printFile(ramFile,ramSizeMap);
    printFile(codeFile,codeSizeMap);


    return 0;
}
