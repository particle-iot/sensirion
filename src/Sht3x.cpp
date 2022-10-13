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
 *
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

#include <cstdint>
#include <mutex>

#include "Sht3x.h"

constexpr std::uint16_t STS3x_PERIODIC_READ_CMD = 0xE000;
constexpr std::uint16_t SHT3X_BREAK_CMD = 0x3093;

constexpr std::uint16_t SHT3X_CMD_DURATION_USEC = 1000;
constexpr std::uint16_t SHT3X_HUMIDITY_LIMIT_MSK = 0xFE00U;
constexpr std::uint16_t SHT3X_TEMPERATURE_LIMIT_MSK = 0x01FFU;

constexpr int ONE_WORD_SIZE = 1;
constexpr int TWO_WORD_SIZE = 2;
constexpr int FOUR_WORD_SIZE = 4;
constexpr int EIGHT_WORD_SIZE = 8;
constexpr int TEN_WORD_SIZE = 10;

RecursiveMutex Sht3x::mutexA;
RecursiveMutex Sht3x::mutexB;

bool Sht3x::singleShotMeasureAndRead(
  float &temperature, float &humidity, SingleMode s_setting
)
{
    bool ret = true;
    const std::lock_guard<RecursiveMutex> lg(_mutex);

    if (measure(Mode::SINGLE_SHOT, s_setting) == true) {
        ret = singleShotRead(temperature, humidity);
    } else {
        driver_log.error("SHT-3x measure failed");
        ret = false;
    }
    return ret;
}

bool Sht3x::measure(Mode mode, SingleMode s_setting, PeriodicMode p_setting)
{
    bool ret = true;
    const std::lock_guard<RecursiveMutex> lg(_mutex);

    _sht3x_cmd_measure = (mode == Mode::SINGLE_SHOT) ? (std::uint16_t)s_setting
                                                     : (std::uint16_t)p_setting;

    // break command to stop a previous periodic mode measure
    ret = writeCmd(SHT3X_BREAK_CMD);
    delay(1); // must delay 1ms to allow SHT3X to stop periodic data

    // do a check here if it happens to fail
    // the break command when in periodic mode
    if (ret) {
        ret = writeCmd(_sht3x_cmd_measure);
    }

    return ret;
}

bool Sht3x::singleShotRead(float &temperature, float &humidity)
{
    std::uint16_t words[2] {};
    const std::lock_guard<RecursiveMutex> lg(_mutex);

    bool ret = readWords(words, 2);

    temperature = convert_raw_temp(words[0]);
    humidity = convert_raw_humidity(words[1]);

    return ret;
}

bool Sht3x::periodicDataRead(Vector<std::pair<float, float>> &data)
{
    const std::lock_guard<RecursiveMutex> lg(_mutex);

    int num_of_words = _get_mps_size_to_words();
    Vector<std::uint16_t> words(num_of_words);
    bool ret = false;

    if (writeCmd(STS3x_PERIODIC_READ_CMD)) {
        ret = readWords(words.data(), num_of_words);
    }

    for (int i = 0; i < num_of_words; i += 2) {
        data.append(std::make_pair(words.at(i), words.at(i + 1)));
    }

    return ret;
}

int Sht3x::_get_mps_size_to_words()
{
    int size {};

    switch (_sht3x_cmd_measure) {
        case HIGH_05_MPS:
        case MEDIUM_05_MPS:
        case LOW_05_MPS:
            size = ONE_WORD_SIZE;
            break;
        case HIGH_1_MPS:
        case MEDIUM_1_MPS:
        case LOW_1_MPS:
            size = TWO_WORD_SIZE;
            break;
        case HIGH_2_MPS:
        case MEDIUM_2_MPS:
        case LOW_2_MPS:
            size = FOUR_WORD_SIZE;
            break;
        case HIGH_4_MPS:
        case MEDIUM_4_MPS:
        case LOW_4_MPS:
            size = EIGHT_WORD_SIZE;
            break;
        case HIGH_10_MPS:
        case MEDIUM_10_MPS:
        case LOW_10_MPS:
            size = TEN_WORD_SIZE;
            break;

        default:
            size = 0;
            break;
    }

    return size;
}

bool Sht3x::setAlertThreshold(
  AlertThreshold limit, float humidity, float temperature
)
{
    std::uint16_t limitVal = 0U;
    std::uint16_t write_cmd {};
    bool ret = true;
    const std::lock_guard<RecursiveMutex> lg(_mutex);

    std::uint16_t rawT = temperature_to_tick(temperature);
    std::uint16_t rawRH = humidity_to_tick(humidity);

    /* convert inputs to alert threshold word */
    limitVal = (rawRH & SHT3X_HUMIDITY_LIMIT_MSK);
    limitVal |= ((rawT >> 7) & SHT3X_TEMPERATURE_LIMIT_MSK);

    switch (limit) {
        case AlertThreshold::SHT3X_HIALRT_SET:
            write_cmd = WRITE_HIALRT_LIM_SET;
            break;
        case AlertThreshold::SHT3X_HIALRT_CLR:
            write_cmd = WRITE_HIALRT_LIM_CLR;
            break;
        case AlertThreshold::SHT3X_LOALRT_CLR:
            write_cmd = WRITE_LOALRT_LIM_CLR;
            break;
        case AlertThreshold::SHT3X_LOALRT_SET:
            write_cmd = WRITE_LOALRT_LIM_SET;
            break;
    }

    if (!writeCmdWithArgs(write_cmd, &limitVal, 1)) {
        ret = false;
        driver_log.info("failed to set alert limit");
    }

    return ret;
}

bool Sht3x::getAlertThreshold(
  AlertThreshold limit, float &humidity, float &temperature
)
{
    std::uint16_t word;
    std::uint16_t read_cmd {};
    bool ret = true;
    const std::lock_guard<RecursiveMutex> lg(_mutex);

    switch (limit) {
        case AlertThreshold::SHT3X_HIALRT_SET:
            read_cmd = READ_HIALRT_LIM_SET;
            break;
        case AlertThreshold::SHT3X_HIALRT_CLR:
            read_cmd = READ_HIALRT_LIM_CLR;
            break;
        case AlertThreshold::SHT3X_LOALRT_CLR:
            read_cmd = READ_LOALRT_LIM_CLR;
            break;
        case AlertThreshold::SHT3X_LOALRT_SET:
            read_cmd = READ_LOALRT_LIM_SET;
            break;
    }

    if (readCmd(read_cmd, &word, 1, SHT3X_CMD_DURATION_USEC)) {
        /* convert threshold word to alert settings in 10*%RH & 10*°C */
        std::uint16_t rawRH = (word & SHT3X_HUMIDITY_LIMIT_MSK);
        std::uint16_t rawT = ((word & SHT3X_TEMPERATURE_LIMIT_MSK) << 7);

        humidity = convert_raw_humidity(rawRH);
        temperature = convert_raw_temp(rawT);
    } else {
        ret = false;
        driver_log.info("failed to get alert limit");
    }

    return ret;
}
