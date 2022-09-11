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
#include "module.h"
#include "meta.h"

Module* Module::root = NULL;

Module::Module(){
    Module** tail = &Module::root;
    while((*tail) != NULL){
        tail = &(*tail)->next;
    }
    *tail = this;
    this->next = NULL;
}

Module::~Module(){

}

void* Module::operator new(size_t size){
    return module_malloc(size);
}

void Module::operator delete(void *ptr){
}

void Module::removeAll(){
    Module* current = Module::root;
    while(current != NULL){
        delete current;
        current = current->next;
    }
    Module::root = NULL;
}

void Module::command(comm_t* comm, const command_t* command){

}

void Module::thread(comm_t* comm){

}

void Module::commandAll(comm_t* comm,const command_t* command){
    Module* current = Module::root;
    if(command->channel == 0){
        Module::mainCommand(comm,command);
        return;
    }
    while(current){
        current->command(comm, command);
        current = current->next;
    }
}

void Module::threadAll(comm_t* comm){
    Module* current = Module::root;
    while(current){
        current->thread(comm);
        current = current->next;
    }
}

void enter_bootloader(void) __attribute__((weak));

void enter_bootloader(){

}

void set_next_device_configuration() __attribute__((weak));

void set_next_device_configuration() {

}

uint32_t get_target_capabilities() __attribute__((weak));

uint32_t get_target_capabilities(){
    return 0xFFFFFFFF;
}

void Module::mainCommand(comm_t* comm,const command_t* command){
    if(command->command_id == 'G'){
        if(command->value == 'N'){
            /* Get target name */
            send_binary_data_header(comm,0,target_name_length);
            comm_write(comm,target_name,target_name_length);
        }
        else if(command->value == 'V'){
            uint32_t value = get_vdda();
            send_binary_data_header(comm,0,4);
            comm_write(comm,(char*)&value,4);
        }
        else if(command->value == 'C'){
            if(target_configuration_name == NULL){
                send_binary_data_header(comm,0,1);
                comm_write(comm,"",1);
            }
            else {
                uint16_t size = strlen(target_configuration_name)+1;
                send_binary_data_header(comm,0,size);
                comm_write(comm,target_configuration_name,size);
            }
        }
        else if(command->value == 'F'){
            uint32_t value = get_target_capabilities();
            send_binary_data_header(comm,0,4);
            comm_write(comm,(char*)&value,4);
        }
    }
    else if(command->command_id == 'C'){
        set_next_device_configuration();
    }
    else if(command->command_id == 'B'){
        enter_bootloader();
    }
}
