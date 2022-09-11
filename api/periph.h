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
#ifndef _PERIPH_H_
#define _PERIPH_H_

class BasePeriph {
public:
    enum Error {
        /** No error */
        ERROR_NONE =                0x00,
        /** No suitable configuration was found. E.g. no free DMA available */
        ERROR_NO_CONFIGURATION =    0x01,
        /** Peripheral had some wrong arguments during initialization */
        ERROR_BAD_CONFIGURATION =   0x02,
        /** Some process was not done in time and timeout was triggered */
        ERROR_TIMEOUT =             0x04,
        /** Bad ID for peripheral initialization */
        ERROR_INVALID_ID =          0x08,
        /** Program ran out of memory */
        ERROR_OUT_OF_MEMORY =       0x10,

    };
private:
    Error error;
protected:
    void setError(Error error){
        this->error = (Error)((int)this->error | (int)error);
    }
public:
    void* operator new(size_t size){return module_malloc(size);}
    void operator delete(void *ptr){}

    BasePeriph(){
        this->error = ERROR_NONE;
    }
    virtual ~BasePeriph(){}
    Error getErrorFlags()const{return error;}
};

#endif /* _PERIPH_H_ */
