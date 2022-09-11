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
 
/* Intentionally no multiple-include protection */
/* TODO: Should be replaced by C++ template in future */
#ifdef __cplusplus
    extern "C" {
#endif

static void OSC_CALLBACK_NAME(void* _osc_handle,uint8_t* _data,uint32_t _size){
    osc_handle_t* osc_handle = (osc_handle_t*)_osc_handle;
    const osc_data_t* data = (const osc_data_t*)_data;
    uint32_t size = (_size >> OSC_DATA_SHIFT);
    uint32_t i = 0;
    uint32_t callback_offset = (_data - osc_handle->circular_buffer->buffer) >> OSC_DATA_SHIFT;

    if(osc_handle->state == OSC_STATE_RUNNING){
        if((osc_handle->pretrigger - osc_handle->pretrigger_index) < (size / osc_handle->num_channels)){
            osc_handle->state = OSC_STATE_PRETRIGGERED;
            i = (osc_handle->pretrigger - osc_handle->pretrigger_index) * osc_handle->num_channels;
        }
        else {
            osc_handle->pretrigger_index += size / osc_handle->num_channels;
        }
    }

    if(osc_handle->state == OSC_STATE_PRETRIGGERED){
        i+=osc_handle->trigger_channel;
        if(i >= size){
            return;
        }
        int old_comp = data[i] < osc_handle->trigger_level;
        uint32_t i_add = osc_handle->num_channels * osc_handle->trigger_skip;
        /* TODO: move auto-trigger outside ?? */
        for(;i < size;i+=i_add){
            int new_comp = data[i] < osc_handle->trigger_level;
            /* TODO: write filter */
            if(((new_comp != old_comp) && (((osc_handle->trigger_polarity & OSC_TRIGGER_RISING_EDGE) && old_comp && !new_comp) ||
                ((osc_handle->trigger_polarity & OSC_TRIGGER_FALLING_EDGE) && !old_comp && new_comp))) ||
                (osc_handle->osc_flags & OSC_FLAG_AUTO_TRIGGER)){

                /* Search for trigger value */
                if(i_add != osc_handle->num_channels && !(osc_handle->osc_flags & OSC_FLAG_AUTO_TRIGGER)){
                    uint32_t real_i = i - i_add;
                    for(;real_i < i;real_i+=osc_handle->num_channels){
                        old_comp = data[real_i] < osc_handle->trigger_level;
                        if(old_comp == new_comp)
                            break;
                    }
                    i = real_i;
                }

                int32_t sample_index = (int32_t)((i + callback_offset + osc_handle->transfer_size) / osc_handle->num_channels) - osc_handle->pretrigger;
                if(sample_index >= (osc_handle->transfer_size / osc_handle->num_channels)){
                    sample_index -= (osc_handle->transfer_size / osc_handle->num_channels);
                }
                else if (sample_index < 0){
                    sample_index += (osc_handle->transfer_size / osc_handle->num_channels);
                }

                osc_handle->stop_offset = sample_index;
                buffer_stop_precise_rw(osc_handle->circular_buffer, (uint32_t)sample_index);

                osc_handle->state = OSC_STATE_TRIGGERED;
                return;
            }

            old_comp = new_comp;
        }
    }
}
#ifdef __cplusplus
    }
#endif
