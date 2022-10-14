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
#pragma once

#include <cstdint>

#include "SensirionBase.h"

class Sts3x : public SensirionBase {
public:
    static constexpr std::uint8_t ADDR_A {0x4au};
    static constexpr std::uint8_t ADDR_B {0x4bu};

    enum class Mode {
        SINGLE_SHOT,
        PERIODIC_DATA,
    };

    enum AlertReadCmd : std::uint16_t {
        READ_HIALRT_LIM_SET = 0xE11F,
        READ_HIALRT_LIM_CLR = 0xE114,
        READ_LOALRT_LIM_CLR = 0xE109,
        READ_LOALRT_LIM_SET = 0xE102,
    };

    enum AlertWriteCmd : std::uint16_t {
        WRITE_HIALRT_LIM_SET = 0x611D,
        WRITE_HIALRT_LIM_CLR = 0x6116,
        WRITE_LOALRT_LIM_CLR = 0x610B,
        WRITE_LOALRT_LIM_SET = 0x6100,
    };

    enum SingleMode : std::uint16_t {
        HIGH_CLOCK_STRETCH = 0x2C06,
        MEDIUM_CLOCK_STRETCH = 0x2C0D,
        LOW_CLOCK_STRETCH = 0x2C10,
        HIGH_NO_CLOCK_STRETCH = 0x2400,
        MEDIUM_NO_CLOCK_STRETCH = 0x240B,
        LOW_NO_CLOCK_STRETCH = 0x2416,
        SINGLE_NONE = 0xFFFF,
    };

    enum PeriodicMode : std::uint16_t {
        HIGH_05_MPS = 0x2032,
        MEDIUM_05_MPS = 0x2024,
        LOW_05_MPS = 0x202F,
        HIGH_1_MPS = 0x2130,
        MEDIUM_1_MPS = 0x2126,
        LOW_1_MPS = 0x212D,
        HIGH_2_MPS = 0x2236,
        MEDIUM_2_MPS = 0x2220,
        LOW_2_MPS = 0x222B,
        HIGH_4_MPS = 0x2334,
        MEDIUM_4_MPS = 0x2322,
        LOW_4_MPS = 0x2329,
        HIGH_10_MPS = 0x2737,
        MEDIUM_10_MPS = 0x2721,
        LOW_10_MPS = 0x272A,
        PERIODIC_NONE = 0xFFFF,
    };

    enum class AlertThreshold {
        STS3X_HIALRT_SET,
        STS3X_HIALRT_CLR,
        STS3X_LOALRT_CLR,
        STS3X_LOALRT_SET,
    };

    Sts3x(TwoWire &interface, std::uint8_t address, pin_t alert_pin)
      : SensirionBase(
        interface, address, alert_pin, address == ADDR_A ? mutexA : mutexB
      )
    {}

    /**
     * @brief Measure and read from an STS sensor the temperature and humidity
     *
     * @details Write to an STS sensor the command to start a measurement, and
     * read from the STS sensor the temperature and humidity. Calls the
     * measure() and read() functions internally
     *
     * @param[out] temperature measured and read in Celsius
     *
     * @return true on success, false on failure
     */
    bool singleShotMeasureAndRead(
      float &temperature,
      SingleMode s_setting = SingleMode::HIGH_NO_CLOCK_STRETCH
    );

    /**
     * @brief Start periodic measurement
     *
     * @details Start periodic temperature and humidity measurements at the
     * commanded repeatability and rate
     *
     * @param[in] mode periodic mode to use
     *
     * @return true on success, false on failure
     */
    bool startPeriodicMeasurement(PeriodicMode);

    /**
     * @brief Stop periodic measurement
     *
     * @details Stop any periodic temperature and humidity measurement in
     * progress
     *
     * @return true on success, false on failure
     */
    bool stopPeriodicMeasurement();

    /**
     * @brief Read a started periodic mode measurement from an STS sensor.
     *
     * @details Read from an STS sensor periodic mode measurement(s) that has
     * already started. Each measurement contains a temperature value.
     * The number of measurements read back depends on the MPS for the peridoc
     * mode setting chosen when the measure() function was called
     *
     * @param[out] temperature contains the data read.
     *
     * @return true on success, false on failure
     */
    bool periodicDataRead(float &temperature);

    /**
     * @brief Set thresholds for alert mode
     *
     * @details Set limits for the alert mode. An alert can be disabled
     * by setting the low set point above the high set point.
     *
     * @param[in] limit the limit to set
     * @param[in] temperature temperature threshold value
     *
     * @return true on success, false on failure
     */
    bool setAlertThreshold(AlertThreshold limit, float temperature);

    /**
     * @brief Get tresholds for alert mode
     *
     * @details Read limits for the alert mode
     *
     * @param[in] limit the limit to read
     * @param[out] temperature temperature threshold value
     *
     * @return true on success, false on failure
     */
    bool getAlertThreshold(AlertThreshold limit, float &temperature);

private:
    static RecursiveMutex mutexA;
    static RecursiveMutex mutexB;
};
