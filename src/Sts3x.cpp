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

constexpr std::uint16_t STS3X_TEMPERATURE_LIMIT_MSK = 0x01FFU;

RecursiveMutex Sts3x::mutexA;
RecursiveMutex Sts3x::mutexB;

bool Sts3x::init()
{
    bool ret = SensirionBase::init();

    if (ret) {
        pinMode(_alertPin, INPUT);
        ret = writeCmd(STS3X_BREAK_CMD);
    }
    return ret;
}

bool Sts3x::singleMeasurement(float &temperature, SingleMode mode)
{
    constexpr auto delay_high {16u};
    constexpr auto delay_medium {7u};
    constexpr auto delay_low {5u};

    const std::lock_guard<RecursiveMutex> lg(_mutex);
    std::uint16_t data;

    bool ret = writeCmd(static_cast<std::uint16_t>(mode));
    if (!ret) {
        return ret;
    }

    switch (mode) {
        case SingleMode::HIGH_NO_CLOCK_STRETCH:
            delay(delay_high);
            break;
        case SingleMode::MEDIUM_NO_CLOCK_STRETCH:
            delay(delay_medium);
            break;
        case SingleMode::LOW_NO_CLOCK_STRETCH:
            delay(delay_low);
            break;
        default:
            break;
    }

    ret = readWords(&data, 1);
    if (ret) {
        temperature = convert_raw_temp(data);
    }

    return ret;
}

bool Sts3x::startPeriodicMeasurement(PeriodicMode mode)
{
    const std::lock_guard<RecursiveMutex> lg(_mutex);
    return writeCmd(static_cast<std::uint16_t>(mode));
}

bool Sts3x::stopPeriodicMeasurement()
{
    const std::lock_guard<RecursiveMutex> lg(_mutex);
    return writeCmd(STS3X_BREAK_CMD);
}

bool Sts3x::periodicDataRead(float &temperature)
{
    const std::lock_guard<RecursiveMutex> lg(_mutex);

    uint16_t raw_temp;
    bool ret = readCmd(STS3x_PERIODIC_READ_CMD, &raw_temp, 1);

    if (ret) {
        temperature = convert_raw_temp(raw_temp);
    }

    return ret;
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

    if (readCmd(read_cmd, &word, 1)) {
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
    const std::lock_guard<RecursiveMutex> lg(_mutex);
    return readCmd(STS3X_CMD_READ_STATUS_REG, &status, 1);
}

bool Sts3x::clearStatus()
{
    const std::lock_guard<RecursiveMutex> lg(_mutex);
    return writeCmd(STS3X_CMD_CLR_STATUS_REG);
}

bool Sts3x::heaterOn()
{
    const std::lock_guard<RecursiveMutex> lg(_mutex);
    return writeCmd(STS3X_CMD_HEATER_ON);
}

bool Sts3x::heaterOff()
{
    const std::lock_guard<RecursiveMutex> lg(_mutex);
    return writeCmd(STS3X_CMD_HEATER_OFF);
}
