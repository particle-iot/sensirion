
/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "SensirionBase.h"

constexpr unsigned int CRC8_POLYNOMIAL = 0x31;
constexpr unsigned int CRC8_INIT = 0xFF;
constexpr unsigned int CRC8_LEN = 1;
constexpr unsigned int SENSIRION_COMMAND_SIZE = 2;
constexpr unsigned int SENSIRION_MAX_BUFFER_WORDS = 32;

Logger SensirionBase::driver_log("sensirion-driver");

bool SensirionBase::init() {
    bool ret = true;
    _i2c.begin();
    _i2c.beginTransmission(_address);
    if (_i2c.endTransmission() != 0) {
        driver_log.error("address 0x%X invalid or device failed", _address);
        ret = false;
    }
    return ret;
}

bool SensirionBase::readCmd(uint16_t command,
                                        uint16_t* data_words,
                                        uint16_t num_words,
                                        uint32_t delay_us) {
    uint8_t buf[SENSIRION_COMMAND_SIZE] {};

    fillCmdBytes(buf, command, NULL, 0);
    size_t ret = writeRegister(buf, SENSIRION_COMMAND_SIZE);

    if (ret != SENSIRION_COMMAND_SIZE) {
        return false;
    }

    if (delay_us) {
        delayMicroseconds(delay_us);
    }

    return readWords(data_words, num_words);
}

bool SensirionBase::writeCmd(uint16_t command) {
    uint8_t buf[SENSIRION_COMMAND_SIZE];
    bool ret = true;

    fillCmdBytes(buf, command, NULL, 0);
    if(writeRegister(buf, SENSIRION_COMMAND_SIZE) !=
                    SENSIRION_COMMAND_SIZE) {
        ret = false;
        driver_log.error("failed write command: 0x%X",command);
    }
    return ret;
}

bool SensirionBase::writeCmdWithArgs(uint16_t command,
                                          const uint16_t* data_words,
                                          uint16_t num_words) {
    uint8_t buf[SENSIRION_MAX_BUFFER_WORDS];
    bool ret = true;

    uint16_t buf_size = fillCmdBytes(buf, command, data_words, num_words);

    if(writeRegister(buf, buf_size) != buf_size) {
        ret = false;
        driver_log.error("failed write command with args");
    }
    return ret;
}

uint16_t SensirionBase::generateCrc(const uint8_t* data, uint8_t len) {
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;

    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < len; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            }
            else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

uint16_t SensirionBase::fillCmdBytes(uint8_t* buf,
                                    uint16_t cmd,
                                    const uint16_t* args,
                                    uint8_t num_args) {
    uint8_t crc;
    uint16_t idx = 0;

    buf[idx++] = (uint8_t)((cmd & 0xFF00) >> 8);
    buf[idx++] = (uint8_t)((cmd & 0x00FF) >> 0);

    for (int i = 0; i < num_args; ++i) {
        buf[idx++] = (uint8_t)((args[i] & 0xFF00) >> 8);
        buf[idx++] = (uint8_t)((args[i] & 0x00FF) >> 0);

        crc = generateCrc((uint8_t*)&buf[idx - 2],
                                            SENSIRION_WORD_SIZE);
        buf[idx++] = crc;
    }
    return idx;
}

bool SensirionBase::readWordsAsBytes(uint8_t* data,
                                                        uint16_t num_words) {
    bool ret = true; //assume success
    int size = num_words * (SENSIRION_WORD_SIZE + CRC8_LEN);
    uint16_t word_buf[SENSIRION_MAX_BUFFER_WORDS] {};
    uint8_t* const buf8 = (uint8_t*)word_buf;

    if (!readRegister(buf8, size)) {
        ret = false;
        driver_log.error("read register fail");
    }
    else {
        /* check the CRC for each word */
        for (int i = 0, j = 0; i < size; i += SENSIRION_WORD_SIZE + CRC8_LEN) {
            if (generateCrc(&buf8[i], SENSIRION_WORD_SIZE) != buf8[i + SENSIRION_WORD_SIZE]) {
                ret = false;
                driver_log.error("checksum match failed");
                break;
            }
            else {
                data[j++] = buf8[i];
                data[j++] = buf8[i + 1];
            }
        }
    }

    return ret;
}

bool SensirionBase::readWords(uint16_t* data_words,
                                                uint16_t num_words) {
    const uint8_t* word_bytes;

    bool ret =
                    readWordsAsBytes((uint8_t*)data_words, num_words);

    if (ret) {
        for (int i = 0; i < num_words; ++i) {
            word_bytes = (uint8_t*)&data_words[i];
            data_words[i] = ((uint16_t)word_bytes[0] << 8) | word_bytes[1];
        }
    }
    else {
        driver_log.error("read words failed");
    }

    return ret;
}

size_t SensirionBase::writeRegister(const uint8_t* buf,
                                size_t length) {
    _i2c.beginTransmission(_address);
    size_t ret = _i2c.write(buf, length);
    _i2c.endTransmission();

    return ret;
}

size_t SensirionBase::readRegister(uint8_t* buf,
                                size_t length) {
    size_t readLength = (int)_i2c.requestFrom(_address, length);
    size_t count = 0;
    if (readLength != length) {
        _i2c.endTransmission();
    }
    else {
        while (_i2c.available() && length--) {
            *buf++ = _i2c.read();
            count++;
        }
    }
    return count;
}
