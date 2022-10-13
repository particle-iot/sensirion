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

#include <cstdint>
#include <mutex>

#include "Sts3x.h"

constexpr std::uint16_t STS3x_PERIODIC_READ_CMD = 0xE000;
constexpr std::uint16_t STS3X_BREAK_CMD = 0x3093;
constexpr std::uint16_t STS3X_CMD_READ_STATUS_REG = 0xF32D;
constexpr std::uint16_t STS3X_CMD_CLR_STATUS_REG = 0x3041;
constexpr std::uint16_t STS3X_CMD_HEATER_ON = 0x306D;
constexpr std::uint16_t STS3X_CMD_HEATER_OFF = 0x3066;

constexpr std::uint16_t STS3X_CMD_DURATION_USEC = 1000;
constexpr std::uint16_t STS3X_TEMPERATURE_LIMIT_MSK = 0x01FFU;

constexpr std::uint16_t STS_DELIMITER = 0xFFFF;

constexpr int ONE_WORD_SIZE = 1;
constexpr int TWO_WORD_SIZE = 2;
constexpr int FOUR_WORD_SIZE = 4;
constexpr int EIGHT_WORD_SIZE = 8;
constexpr int TEN_WORD_SIZE = 10;

RecursiveMutex Sts3x::mutexA;
RecursiveMutex Sts3x::mutexB;

bool Sts3x::init()
{
    const std::lock_guard<RecursiveMutex> lg(mutex);
    return SensirionBase::init();
}

bool Sts3x::singleShotMeasureAndRead(float &temperature, SingleMode s_setting)
{
    bool ret = true;
    const std::lock_guard<RecursiveMutex> lg(mutex);

    if (measure(Mode::SINGLE_SHOT, s_setting) == true) {
        ret = singleShotRead(temperature);
    } else {
        driver_log.error("STS-3x measure failed");
        ret = false;
    }
    return ret;
}

bool Sts3x::measure(Mode mode, SingleMode s_setting, PeriodicMode p_setting)
{
    bool ret = true;
    const std::lock_guard<RecursiveMutex> lg(mutex);

    _sts3x_cmd_measure = (mode == Mode::SINGLE_SHOT) ? (std::uint16_t)s_setting
                                                     : (std::uint16_t)p_setting;

    // break command to stop a previous periodic mode measure
    ret = writeCmd(STS3X_BREAK_CMD);
    delay(1); // must delay 1ms to allow STS3X to stop periodic data

    // do a check here if it happens to fail
    // the break command when in periodic mode
    if (ret) {
        ret = writeCmd(_sts3x_cmd_measure);
    }

    return ret;
}

bool Sts3x::singleShotRead(float &temperature)
{
    std::uint16_t raw_temp;
    const std::lock_guard<RecursiveMutex> lg(mutex);

    bool ret = readWords(&raw_temp, 1);

    temperature = convert_raw_temp(raw_temp);

    return ret;
}

bool Sts3x::periodicDataRead(Vector<float> &data)
{
    const std::lock_guard<RecursiveMutex> lg(mutex);
    int num_of_words = _get_mps_size_to_words();
    Vector<std::uint16_t> words(num_of_words);
    bool ret = false;

    if (writeCmd(STS3x_PERIODIC_READ_CMD)) {
        ret = readWords(words.data(), num_of_words);
    }

    for (int i = 0; i < num_of_words; i++) {
        if (words.at(i) != STS_DELIMITER) {
            data.append(convert_raw_temp(words.at(i)));
        }
    }

    return ret;
}

int Sts3x::_get_mps_size_to_words()
{
    int size {};

    switch (_sts3x_cmd_measure) {
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

bool Sts3x::setAlertThreshold(AlertThreshold limit, float temperature)
{
    std::uint16_t limitVal = 0U;
    std::uint16_t write_cmd {};
    bool ret = true;

    std::uint16_t rawT = temperature_to_tick(temperature);

    /* convert inputs to alert threshold word */
    limitVal = ((rawT >> 7) & STS3X_TEMPERATURE_LIMIT_MSK);

    switch (limit) {
        case AlertThreshold::STS3X_HIALRT_SET:
            write_cmd = WRITE_HIALRT_LIM_SET;
            break;
        case AlertThreshold::STS3X_HIALRT_CLR:
            write_cmd = WRITE_HIALRT_LIM_CLR;
            break;
        case AlertThreshold::STS3X_LOALRT_CLR:
            write_cmd = WRITE_LOALRT_LIM_CLR;
            break;
        case AlertThreshold::STS3X_LOALRT_SET:
            write_cmd = WRITE_LOALRT_LIM_SET;
            break;
    }

    if (!writeCmdWithArgs(write_cmd, &limitVal, 1)) {
        ret = false;
        Log.info("failed to set alert limit");
    }

    return ret;
}

bool Sts3x::getAlertThreshold(AlertThreshold limit, float &temperature)
{
    std::uint16_t word;
    std::uint16_t read_cmd {};

    bool ret = true;

    switch (limit) {
        case AlertThreshold::STS3X_HIALRT_SET:
            read_cmd = READ_HIALRT_LIM_SET;
            break;
        case AlertThreshold::STS3X_HIALRT_CLR:
            read_cmd = READ_HIALRT_LIM_CLR;
            break;
        case AlertThreshold::STS3X_LOALRT_CLR:
            read_cmd = READ_LOALRT_LIM_CLR;
            break;
        case AlertThreshold::STS3X_LOALRT_SET:
            read_cmd = READ_LOALRT_LIM_SET;
            break;
    }

    if (writeCmd(read_cmd) && readWords(&word, 1)) {
        /* convert threshold word to alert settings in 10*%RH & 10*°C */
        std::uint16_t rawT = ((word & STS3X_TEMPERATURE_LIMIT_MSK) << 7);
        temperature = convert_raw_temp(rawT);
    } else {
        ret = false;
        Log.info("failed to get alert limit");
    }

    return ret;
}

bool Sts3x::getStatus(std::uint16_t &status)
{
    const std::lock_guard<RecursiveMutex> lg(mutex);
    return readCmd(
      STS3X_CMD_READ_STATUS_REG, &status, 1, STS3X_CMD_DURATION_USEC
    );
}

bool Sts3x::clearStatus()
{
    const std::lock_guard<RecursiveMutex> lg(mutex);
    return writeCmd(STS3X_CMD_CLR_STATUS_REG);
}

bool Sts3x::heaterOn()
{
    const std::lock_guard<RecursiveMutex> lg(mutex);
    return writeCmd(STS3X_CMD_HEATER_ON);
}

bool Sts3x::heaterOff()
{
    const std::lock_guard<RecursiveMutex> lg(mutex);
    return writeCmd(STS3X_CMD_HEATER_OFF);
}
