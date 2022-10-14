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

#include "SensirionBase.h"

class Sht3x : public SensirionBase {
public:
    static constexpr std::uint8_t ADDR_A {0x44u};
    static constexpr std::uint8_t ADDR_B {0x45u};

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
    };

    enum class AlertThreshold {
        SHT3X_HIALRT_SET,
        SHT3X_HIALRT_CLR,
        SHT3X_LOALRT_CLR,
        SHT3X_LOALRT_SET,
    };

    Sht3x(TwoWire &interface, std::uint8_t address, pin_t alert_pin)
      : SensirionBase(interface, address),
        _alertPin(alert_pin),
        _mutex(address == ADDR_A ? mutexA : mutexB)
    {}

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
    bool singleMeasurement(
      float &temperature,
      float &humidity,
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
     * @brief Read a started periodic mode measurement from an SHT sensor.
     *
     * @details Read from an SHT sensor periodic mode measurement(s) that has
     * already started. Must call the startPeriodicMeasurement() function before
     * calling this function.
     *
     * @param[out] temperature measured and read in Celsius
     * @param[out] humidity measured and read in %
     *
     * @return true on success, false on failure
     */
    bool periodicDataRead(float &temperature, float &humidity);

    /**
     * @brief Set thresholds for alert mode
     *
     * @details Set limits for the alert mode. An alert can be disabled
     * by setting the low set point above the high set point.
     *
     * @param[in] limit the limit to set
     * @param[in] temperature temperature threshold value
     * @param[in] humidity humidity threshold value
     *
     * @return true on success, false on failure
     */
    bool
    setAlertThreshold(AlertThreshold limit, float temperature, float humidity);

    /**
     * @brief Get tresholds for alert mode
     *
     * @details Read limits for the alert mode
     *
     * @param[in] limit the limit to read
     * @param[out] temperature temperature threshold value
     * @param[out] humidity humidity threshold value
     *
     * @return true on success, false on failure
     */
    bool getAlertThreshold(
      AlertThreshold limit, float &temperature, float &humidity
    );

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
     * @brief Clear the status register
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

private:
    pin_t _alertPin;
    RecursiveMutex &_mutex;

    // Use separate mutexes per address
    static RecursiveMutex mutexA;
    static RecursiveMutex mutexB;
};
