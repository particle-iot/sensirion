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
#include <utility>

#include "SensirionBase.h"

class Sht3x : public SensirionBase {
public:
    static constexpr std::uint8_t ADDR_A {0x44u};
    static constexpr std::uint8_t ADDR_B {0x45u};

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
        SHT3X_HIALRT_SET,
        SHT3X_HIALRT_CLR,
        SHT3X_LOALRT_CLR,
        SHT3X_LOALRT_SET,
    };

    Sht3x(TwoWire &interface, std::uint8_t address, pin_t alert_pin)
      : SensirionBase(
        interface, address, alert_pin, address == ADDR_A ? mutexA : mutexB
      ),
        _sht3x_cmd_measure(SingleMode::SINGLE_NONE)
    {}

    /**
     * @brief Measure and read from an SHT sensor the temperature and humidity
     *
     * @details Write to an SHT sensor the command to start a measurement, and
     * read from the SHT sensor the temperature and humidity. Calls the
     * measure() and read() functions internally
     *
     * @param[out] temperature measured and read in Celsius
     * @param[out] humidity measured and read in %
     *
     * @return true on success, false on failure
     */
    bool singleShotMeasureAndRead(
      float &temperature,
      float &humidity,
      SingleMode s_setting = SingleMode::HIGH_NO_CLOCK_STRETCH
    );
    /**
     * @brief Measure from an SHT sensor
     *
     * @details Write to an SHT sensor the command to start a measurement.
     * Either sends the single shot mode, or periodic mode depending on the
     * mode chosen. Default is single shot.
     *
     * @param[in] mode measurement mode to chose from
     * @param[in] s_setting single mode setting
     * @param[in] p_setting periodic mode setting
     *
     * @return true on success, false on failure
     */
    bool measure(
      Mode mode = Mode::SINGLE_SHOT,
      SingleMode s_setting = SingleMode::HIGH_NO_CLOCK_STRETCH,
      PeriodicMode p_setting = PeriodicMode::PERIODIC_NONE
    );

    /**
     * @brief Read a started single shot measurement from an SHT sensor
     *
     * @details Read from an SHT sensor a measurement that has already started
     *
     * @param[out] temperature measured and read in Celsius
     * @param[out] humidity measured and read in %
     *
     * @return true on success, false on failure
     */
    bool singleShotRead(float &temperature, float &humidity);

    /**
     * @brief Read a started periodic mode measurement from an SHT sensor.
     *
     * @details Read from an SHT sensor periodic mode measurement(s) that has
     * already started. Each measurement contains a temperature and humidty.
     * The number of measurements read back depends on the MPS for the peridoc
     * mode setting chosen when the measure() function was called. Must call the
     * measure() function before calling this function.
     *
     * @param[out] data contains the data read.
     *
     * @return true on success, false on failure
     */
    bool periodicDataRead(Vector<std::pair<float, float>> &data);

    /**
     * @brief Set thresholds for alert mode
     *
     * @details <details of the function>
     *
     * @param[in] limit the limit to set
     * @param[in] humidity humidity threshold value
     * @param[in] temperature temperature threshold value
     *
     * @return true on success, false on failure
     */
    bool
    setAlertThreshold(AlertThreshold limit, float humidity, float temperature);

    /**
     * @brief Get tresholds for alert mode
     *
     * @details Read limits for the alert mode
     *
     * @param[in] limit the limit to read
     * @param[out] humidity humidity threshold value
     * @param[out] temperature temperature threshold value
     *
     * @return true on success, false on failure
     */
    bool getAlertThreshold(
      AlertThreshold limit, float &humidity, float &temperature
    );

private:
    /**
     * @brief Returns the MPS word size expected in a single second for
     * reading all of the measuremnts in periodic mode
     *
     * @details Calculates the total number of words needed to read all of
     * the measurements in a single second while in periodic mode. So if you
     * setup the SHT to read 10MPS in periodic mode that is 20 words, 4MPS is
     * 8 words, 2MPS 4 words, etc.
     *
     * @return number of words in a single second read
     */
    int _get_mps_size_to_words();

    static RecursiveMutex mutexA;
    static RecursiveMutex mutexB;
    std::uint16_t _sht3x_cmd_measure;
};
