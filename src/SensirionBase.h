/**
 * Copyright (c) 2022 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include "Particle.h"

#define SENSIRION_WORD_SIZE 2
#define SENSIRION_NUM_WORDS(x) (sizeof(x) / SENSIRION_WORD_SIZE)

class SensirionBase {
protected:
    SensirionBase(TwoWire &i2c, uint8_t address)
      : _i2c(i2c),
        _address(address) {};

    /**
     * @brief Initialize the interface
     *
     * @details Attempts to begin i2c transmission of the sensor to
     * validate the sensor can communicate
     *
     * @return true on success, false on failure
     */
    bool init();

    /**
     * @brief Used to read a sensirion sensor
     *
     * @details Will read multiple words from a read command
     *
     * @param[in] address of sensirion device to read
     * @param[in] command to read
     * @param[out] data_words buffer containing words read
     * @param[in] num_words number of words to read
     * @param[in] delay_us delay seconds between sending the read command,
     * and actually reading the results
     *
     * @return true on success, false on failure
     */
    bool readCmd(
      uint16_t command,
      uint16_t *data_words,
      uint16_t num_words,
      uint32_t delay_us = 0
    );

    /**
     * @brief Used to write a command to a sensirion sensor
     *
     * @details Writes a command to a sensirion sensor
     *
     * @param[in] address of sensirion device to write
     * @param[in] command to write
     *
     * @return true on success, false on failure
     */
    bool writeCmd(uint16_t command);

    /**
     * @brief Used to write a command with multiple arguments to a sensirion
     * sensor
     *
     * @details Writes a a command with multiple arguments to a sensirion
     * sensor
     *
     * @param[in] address of sensirion device to write
     * @param[in] command to write
     * @param[in] data_words arguments to write, 1 argument = 1 word
     * @param[in] num_words number of arguments to write, 1 argument = 1 word
     *
     * @return <what does the function return (optional if void)>
     */
    bool writeCmdWithArgs(
      uint16_t command, const uint16_t *data_words, uint16_t num_words
    );

    /**
     * @brief Read a register from a sensirion device
     *
     * @details Given the address of a sensirion device, read out the register
     * contents of the device. User must first write to the register they want
     * to read, then you can read from that register
     *
     * @param[in] address of sensirion device to read
     * @param[out] buf buffer to store read bytes into
     * @param[in] length number of bytes to read
     *
     * @return number of bytes read
     */
    size_t readRegister(uint8_t *buf, size_t length);

    /**
     * @brief Write a register of a sensirion device
     *
     * @details Given the address of a sensirion device, write to that register
     * of the device.
     *
     * @param[in] address of sensirion device to write
     * @param[in] buf buffer containing payload to write
     * @param[in] length number of bytes to write
     *
     * @return number of bytes written
     */
    size_t writeRegister(const uint8_t *buf, size_t length);

    /**
     * @brief Read words from a sensirion device
     *
     * @details Used to read a single word, or multiple words from a sensirion
     * device. Will read out single bytes (calls readWordsAsBytes() function)
     * from the sensirion device, and then concatenate those bytes to get a
     * full word from a device.
     *
     * @param[in] address of sensirion device to read
     * @param[out] buffer to store read words
     * @param[in] num_words number of words to read
     *
     * @return true on success, false on failure
     */
    bool readWords(uint16_t *data_words, uint16_t num_words);

    TwoWire &_i2c;
    uint8_t _address;
    static Logger driver_log;

private:
    /**
     * @brief Read bytes given a buffer that represents words from a sensirion
     * device
     *
     * @details This is called from the readWords() function, and will take the
     * buffer of uint16_t elements, and read them out byte by byte from the
     * device. This will still check if each word's crc matches the
     * calculated crc for that word.
     *
     * @param[in] address of sensirion device to read
     * @param[in] data buffer repesenting single bytes of a word to store
     * @param[in] num_words number of words to read
     *
     * @return true on success, false on failure
     */
    bool readWordsAsBytes(uint8_t *data, uint16_t num_words);

    /**
     * @brief Fill out command to be sent to a sensirion device
     *
     * @details Used to fill out a command, with multiple arguments in the
     * correct byte order. Calculates the CRC for multiple arguments.
     *
     * @param[out] buf buffer to store the command, args, and CRCs
     * @param[in] cmd command to write
     * @param[in] args buffer containing the arguments to fill, if any
     * @param[in] num_args number of arguments to fill
     *
     * @return number of bytes filled in the buffer
     */
    uint16_t fillCmdBytes(
      uint8_t *buf, uint16_t cmd, const uint16_t *args, uint8_t num_args
    );

    /**
     * @brief Generates a CRC
     *
     * @details Used to generate a CRC from a buffer of a given length
     *
     * @param[in] data buffer of data to calculate CRC
     * @param[in] len length of the buffer
     *
     * @return the generated CRC
     */
    uint16_t generateCrc(const uint8_t *data, uint8_t len);
};
