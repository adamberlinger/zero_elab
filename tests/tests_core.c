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
#include "tests_core.h"
#include "gpio.h"

#include "tests_list_generated.h"

void test_print_error(comm_t* comm,int value,int expected,
    int line, int user_param){
    print_string(comm,"Expected '");
    print_int_dec(comm,expected);
    print_string(comm,"' but found '");
    print_int_dec(comm,value);
    print_string(comm,"' on line: ");
    print_int_dec(comm,line);
    print_string(comm,"\n\tuser param: ");
    print_int_dec(comm,user_param);
    print_string(comm,"\n");
}

int main(void){
    uint32_t i;
    comm_t *main_comm;

    gpio_init_pins();
    init_cpu();
    target_init();

    main_comm = get_main_comm();

    print_string(main_comm, "Starting tests...\n");
    for(i = 0;i < TEST_LIST_SIZE;++i){
        int status;
        print_string(main_comm,"Running test '");
        print_string(main_comm,test_list[i].test_name);
        print_string(main_comm,"'\n");
        status = test_list[i].test_callback(main_comm);
        if(status == 0){
            print_string(main_comm,"\t\tOK\n");
        }
        else {
            print_string(main_comm,"\t\tFAILED\n");
        }
    }

    while(1){
        /* Nothing to do */
    }
}
