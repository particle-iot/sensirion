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
constexpr std::uint16_t SHT3X_CMD_READ_STATUS_REG = 0xF32D;
constexpr std::uint16_t SHT3X_CMD_CLR_STATUS_REG = 0x3041;
constexpr std::uint16_t SHT3X_CMD_HEATER_ON = 0x306D;
constexpr std::uint16_t SHT3X_CMD_HEATER_OFF = 0x3066;

constexpr std::uint16_t SHT3X_HUMIDITY_LIMIT_MSK = 0xFE00U;
constexpr std::uint16_t SHT3X_TEMPERATURE_LIMIT_MSK = 0x01FFU;

RecursiveMutex Sht3x::mutexA;
RecursiveMutex Sht3x::mutexB;

bool Sht3x::init()
{
    bool ret = SensirionBase::init();

    if (ret) {
        pinMode(_alertPin, INPUT);
        ret = writeCmd(SHT3X_BREAK_CMD);
    }
    return ret;
}

bool Sht3x::singleMeasurement(
  float &temperature, float &humidity, SingleMode mode
)
{
    constexpr auto delay_high {16u};
    constexpr auto delay_medium {7u};
    constexpr auto delay_low {5u};

    const std::lock_guard<RecursiveMutex> lg(_mutex);
    std::uint16_t data[2];

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

    ret = readWords(data, 2);
    if (ret) {
        temperature = convert_raw_temp(data[0]);
        humidity = convert_raw_humidity(data[1]);
    }

    return ret;
}

bool Sht3x::startPeriodicMeasurement(PeriodicMode mode)
{
    const std::lock_guard<RecursiveMutex> lg(_mutex);
    return writeCmd(static_cast<std::uint16_t>(mode));
}

bool Sht3x::stopPeriodicMeasurement()
{
    const std::lock_guard<RecursiveMutex> lg(_mutex);
    return writeCmd(SHT3X_BREAK_CMD);
}

bool Sht3x::periodicDataRead(float &temperature, float &humidity)
{
    const std::lock_guard<RecursiveMutex> lg(_mutex);

    uint16_t data[2];
    bool ret = readCmd(STS3x_PERIODIC_READ_CMD, data, 2);

    if (ret) {
        temperature = convert_raw_temp(data[0]);
        humidity = convert_raw_humidity(data[1]);
    }

    return ret;
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

    if (readCmd(read_cmd, &word, 1)) {
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

bool Sht3x::getStatus(std::uint16_t &status)
{
    const std::lock_guard<RecursiveMutex> lg(_mutex);
    return readCmd(SHT3X_CMD_READ_STATUS_REG, &status, 1);
}

bool Sht3x::clearStatus()
{
    const std::lock_guard<RecursiveMutex> lg(_mutex);
    return writeCmd(SHT3X_CMD_CLR_STATUS_REG);
}

bool Sht3x::heaterOn()
{
    const std::lock_guard<RecursiveMutex> lg(_mutex);
    return writeCmd(SHT3X_CMD_HEATER_ON);
}

bool Sht3x::heaterOff()
{
    const std::lock_guard<RecursiveMutex> lg(_mutex);
    return writeCmd(SHT3X_CMD_HEATER_OFF);
}
