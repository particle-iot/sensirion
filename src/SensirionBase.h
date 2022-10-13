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

#include <cstdint>

#include "Particle.h"

#define SENSIRION_WORD_SIZE 2
#define SENSIRION_NUM_WORDS(x) (sizeof(x) / SENSIRION_WORD_SIZE)

class SensirionBase {
public:
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
     * @brief Read the status register
     *
     * @details Sends a command to read the SHT status register
     *
     * @param[out] status read from the register
     *
     * @return true on success, false on failure
     */
    bool getStatus(std::uint16_t &status);

    /**
     * @brief CLEAR the status register
     *
     * @details Sends a command to clear the SHT status register
     *
     * @return true on success, false on failure
     */
    bool clearStatus();

    /**
     * @brief Turns the heater on to see plausability of values
     *
     * @details Sends the heater on command
     *
     * @return true on success, false on failure
     */
    bool heaterOn();

    /**
     * @brief Turns the heater off
     *
     * @details Sends the heater off command
     *
     * @return true on success, false on failure
     */
    bool heaterOff();

protected:
    SensirionBase(
      TwoWire &i2c, std::uint8_t address, pin_t alert_pin, RecursiveMutex &mutex
    )
      : _i2c(i2c),
        _address(address),
        _alertPin(alert_pin),
        _mutex(mutex) {};

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
      std::uint16_t command,
      std::uint16_t *data_words,
      std::uint16_t num_words,
      std::uint32_t delay_us = 0
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
    bool writeCmd(std::uint16_t command);

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
      std::uint16_t command,
      const std::uint16_t *data_words,
      std::uint16_t num_words
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
    size_t readRegister(std::uint8_t *buf, size_t length);

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
    size_t writeRegister(const std::uint8_t *buf, size_t length);

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
    bool readWords(std::uint16_t *data_words, std::uint16_t num_words);

    TwoWire &_i2c;
    std::uint8_t _address;
    std::uint16_t _alertPin;
    RecursiveMutex &_mutex;
    static Logger driver_log;

    static constexpr float convert_raw_temp(std::uint16_t temperature_raw)
    {
        return (((RAW_TEMP_ADC_COUNT * temperature_raw) >> DIVIDE_BY_POWER)
                - RAW_TEMP_CONST)
               / SENSIRION_SCALE;
    }

    static constexpr float convert_raw_humidity(std::uint16_t humidity_raw)
    {
        return ((RAW_HUMIDITY_ADC_COUNT * humidity_raw) >> DIVIDE_BY_POWER)
               / SENSIRION_SCALE;
    }

    static constexpr std::uint16_t temperature_to_tick(float temperature)
    {
        return (static_cast<int>(SENSIRION_SCALE * temperature)
                  * TEMP_MULTIPLY_CONSTANT
                + TEMP_ADD_CONSTANT)
               >> DIVIDE_BY_TICK;
    }

    static constexpr std::uint16_t humidity_to_tick(float humidity)
    {
        return (static_cast<int>(SENSIRION_SCALE * humidity)
                * HUMID_MULT_CONSTANT)
               >> DIVIDE_BY_TICK;
    }

private:
    static constexpr int RAW_TEMP_ADC_COUNT = 21875;
    static constexpr int RAW_HUMIDITY_ADC_COUNT = 12500;
    static constexpr int DIVIDE_BY_POWER = 13;
    static constexpr int DIVIDE_BY_TICK = 15;
    static constexpr int TEMP_ADD_CONSTANT = 552195000;
    static constexpr int TEMP_MULTIPLY_CONSTANT = 12271;
    static constexpr int HUMID_MULT_CONSTANT = 21474;
    static constexpr int RAW_TEMP_CONST = 45000;
    static constexpr float SENSIRION_SCALE = 1000.0F;

    /**
     * @brief Read bytes given a buffer that represents words from a sensirion
     * device
     *
     * @details This is called from the readWords() function, and will take the
     * buffer of std::uint16_t elements, and read them out byte by byte from the
     * device. This will still check if each word's crc matches the
     * calculated crc for that word.
     *
     * @param[in] address of sensirion device to read
     * @param[in] data buffer repesenting single bytes of a word to store
     * @param[in] num_words number of words to read
     *
     * @return true on success, false on failure
     */
    bool readWordsAsBytes(std::uint8_t *data, std::uint16_t num_words);

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
    std::uint16_t fillCmdBytes(
      std::uint8_t *buf,
      std::uint16_t cmd,
      const std::uint16_t *args,
      std::uint8_t num_args
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
    std::uint16_t generateCrc(const std::uint8_t *data, std::uint8_t len);
};
