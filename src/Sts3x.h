
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
#pragma once

#include "SensirionBase.h"

class Sts3x : public SensirionBase {
public:

    enum Addr {
        ADDR_A = 0x4A,
        ADDR_B = 0x4B
    };

    enum class Mode {
        SINGLE_SHOT,
        PERIODIC_DATA,
    };

    enum AlertReadCmd: uint16_t {
        READ_HIALRT_LIM_SET = 0xE11F,
        READ_HIALRT_LIM_CLR = 0xE114,
        READ_LOALRT_LIM_CLR = 0xE109,
        READ_LOALRT_LIM_SET = 0xE102,
    };

    enum AlertWriteCmd: uint16_t {
        WRITE_HIALRT_LIM_SET = 0x611D,
        WRITE_HIALRT_LIM_CLR = 0x6116,
        WRITE_LOALRT_LIM_CLR = 0x610B,
        WRITE_LOALRT_LIM_SET = 0x6100,
    };

    enum SingleMode : uint16_t {
        HIGH_CLOCK_STRETCH = 0x2C06,
        MEDIUM_CLOCK_STRETCH = 0x2C0D,
        LOW_CLOCK_STRETCH = 0x2C10,
        HIGH_NO_CLOCK_STRETCH = 0x2400,
        MEDIUM_NO_CLOCK_STRETCH = 0x240B,
        LOW_NO_CLOCK_STRETCH = 0x2416,
        SINGLE_NONE = 0xFFFF,
    };
    
    enum PeriodicMode : uint16_t {
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

    enum class AlertThd {
        STS3X_HIALRT_SET,
        STS3X_HIALRT_CLR,
        STS3X_LOALRT_CLR,
        STS3X_LOALRT_SET,
    };

    Sts3x(TwoWire& interface, Addr address, pin_t alertPin) : 
            SensirionBase(interface), 
            _address(address),
            _alertPin(alertPin), 
            _sts3x_cmd_measure(SingleMode::SINGLE_NONE) {
        pinMode(_alertPin, INPUT);
    }

    /**
     * @brief Initialize the STS3x interface
     *
     * @details Attempts to begin i2c transmission of the STS3x sensor to 
     * validate the sensor can communicate
     * 
     * @return NO_ERROR on success, ERROR_FAIL on failure
     */
    SensirionBase::ErrorCodes init();

    /**
     * @brief Measure and read from an STS sensor the temperature and humidity
     *
     * @details Write to an STS sensor the command to start a measurement, and
     * read from the STS sensor the temperature and humidity. Calls the 
     * measure() and read() functions internally
     *
     * @param[out] temperature measured and read in Celsius
     *
     * @return NO_ERROR on success, ERROR_FAIL on failure
     */
    SensirionBase::ErrorCodes singleShotMeasureAndRead(float& temperature,
                    SingleMode s_setting = SingleMode::HIGH_NO_CLOCK_STRETCH);
    /**
     * @brief Measure from an STS sensor
     *
     * @details Write to an STS sensor the command to start a measurement.
     * Either sends the single shot mode, or periodic mode depending on the 
     * mode chosen. Default is single shot.
     * 
     * @param[in] mode measurement mode to chose from
     * @param[in] s_setting single mode setting
     * @param[in] p_setting periodic mode setting
     *
     * @return NO_ERROR on success, ERROR_FAIL on failure
     */
    SensirionBase::ErrorCodes measure(Mode mode = Mode::SINGLE_SHOT, 
                    SingleMode s_setting = SingleMode::HIGH_NO_CLOCK_STRETCH,
                    PeriodicMode p_setting = PeriodicMode::PERIODIC_NONE);

    /**
     * @brief Read a started measurement from an STS sensor
     *
     * @details Read from an STS sensor a measurement that has already started
     *
     * @param[out] temperature measured and read in Celsius
     *
     * @return NO_ERROR on success, ERROR_FAIL on failure
     */
    SensirionBase::ErrorCodes singleShotRead(float& temperature);

    /**
     * @brief Read a started periodic mode measurement from an STS sensor. 
     *
     * @details Read from an STS sensor periodic mode measurement(s) that has
     * already started. Each measurement contains a temperature value.
     * The number of measurements read back depends on the MPS for the peridoc 
     * mode setting chosen when the measure() function was called
     *
     * @param[out] data contains the data read.
     *
     * @return NO_ERROR on success, ERROR_FAIL on failure
     */
    SensirionBase::ErrorCodes periodicDataRead(Vector<float>& data);

    // /**
    //  * @brief <enter a brief one sentence description>
    //  *
    //  * @details <details of the function>
    //  *
    //  * @param[in,out] <name of variable> <description of variable>
    //  *
    //  * @return <what does the function return (optional if void)>
    //  */
    // SensirionBase::ErrorCodes setAlertThd(AlertThd thd);

    // /**
    //  * @brief <enter a brief one sentence description>
    //  *
    //  * @details <details of the function>
    //  *
    //  * @param[in,out] <name of variable> <description of variable>
    //  *
    //  * @return <what does the function return (optional if void)>
    //  */
    // SensirionBase::ErrorCodes getAlertThd(AlertThd thd);

    /**
     * @brief Read the STS status register
     *
     * @details Sends a command to read the STS status register
     *
     * @param[out] status read from the register
     *
     * @return NO_ERROR on success, ERROR_FAIL on failure
     */
    SensirionBase::ErrorCodes getStatus(uint16_t& status);

    /**
     * @brief CLEAR the STS status register
     *
     * @details Sends a command to clear the STS status register
     *
     * @return NO_ERROR on success, ERROR_FAIL on failure
     */
    SensirionBase::ErrorCodes clearStatus();

    /**
     * @brief Turns the heater on the STS to see plausability of values
     *
     * @details Sends the heater on command
     *
     * @return NO_ERROR on success, ERROR_FAIL on failure
     */
    SensirionBase::ErrorCodes heaterOn();

    /**
     * @brief Turns the heater on the STS off
     *
     * @details Sends the heater off command
     *
     * @return NO_ERROR on success, ERROR_FAIL on failure
     */
    SensirionBase::ErrorCodes heaterOff();

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

    /**
     * @brief Convert the raw temperature from an STS sensor
     *
     * @details This is explained in the STS data sheet, and is usually
     * optimized for fixed point arithmetic
     *
     * @param[in] temperature_raw raw temp reading from STS sensor
     *
     * @return the read and converted temperature in Celsius
     */
    float _convert_raw_temp(uint16_t temperature_raw);

    /**
     * @brief converts temperature to ADC ticks
     *
     * @param temperature temperature value in T°C*1000
     * @param tick sensor ADC ticks
     */
    uint16_t _temperature_to_tick(int32_t temperature);

    RecursiveMutex mutex;
    Addr _address;
    uint16_t _alertPin;
    uint16_t _sts3x_cmd_measure;
};