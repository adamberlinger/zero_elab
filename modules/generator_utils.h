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

#ifndef _GENERATOR_UTILS_H_
#define _GENERATOR_UTILS_H_

#ifndef SAMPLE_TABLE_SIZE
    /** \brief Default value for size of table stored in flash */
    #define SAMPLE_TABLE_SIZE   (4096)
#endif

#ifndef SAMPLE_TABLE_RESOLUTION
    /** \brief Default resolution of table stored in flash */
    #define SAMPLE_TABLE_RESOLUTION (16)
#endif

enum GeneratorShape{
    /** \brief Sine signal shape */
    SHAPE_SIN = 0,
    /** \brief Triangle signal shape (ascending and descending) */
    SHAPE_TRIANGLE = 1,
    /** \brief Square signal shape */
    SHAPE_SQUARE = 2,
    /** \brief This settings generates pseudo-random samples */
    SHAPE_NOISE = 3,
    /** \brief Saw signal shape (ascending and jump to bottom) */
    SHAPE_SAW = 4,
};

#endif /* _GENERATOR_UTILS_H_ */