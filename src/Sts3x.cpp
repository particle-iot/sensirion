/*
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

#include "Sts3x.h"

constexpr uint16_t STS3x_PERIODIC_READ_CMD = 0xE000;
constexpr uint16_t STS3X_BREAK_CMD = 0x3093;
constexpr uint16_t STS3X_CMD_READ_STATUS_REG = 0xF32D;
constexpr uint16_t STS3X_CMD_CLR_STATUS_REG = 0x3041;
constexpr uint16_t STS3X_CMD_HEATER_ON = 0x306D;
constexpr uint16_t STS3X_CMD_HEATER_OFF = 0x3066;

constexpr uint16_t STS3X_CMD_DURATION_USEC = 1000;
constexpr uint16_t STS3X_HUMIDITY_LIMIT_MSK = 0xFE00U;
constexpr uint16_t STS3X_TEMPERATURE_LIMIT_MSK = 0x01FFU;

constexpr unsigned int RAW_TEMP_ADC_COUNT = 21875;
constexpr unsigned int RAW_HUMIDITY_ADC_COUNT = 12500;
constexpr unsigned int DIVIDE_BY_POWER = 13;
constexpr unsigned int RAW_TEMP_CONST = 45000;
constexpr int DIVIDE_BY_TICK = 15;
constexpr int TEMP_ADD_CONSTANT = 552195000;
constexpr int TEMP_MULTIPLY_CONSTANT = 12271;
constexpr float SENSIRION_SCALE = 1000.0F;
constexpr uint16_t STS_DELIMITER = 0xFFFF;

constexpr int ONE_WORD_SIZE = 1;
constexpr int TWO_WORD_SIZE = 2;
constexpr int FOUR_WORD_SIZE = 4;
constexpr int EIGHT_WORD_SIZE = 8;
constexpr int TEN_WORD_SIZE = 10;

bool Sts3x::singleShotMeasureAndRead(float& temperature,
                                                        SingleMode s_setting) {
    bool ret = true ;
    const std::lock_guard<RecursiveMutex> lg(mutex);

    if (measure(Mode::SINGLE_SHOT, s_setting) ==
                    true) {
        ret = singleShotRead(temperature);
    }
    else {
        driver_log.error("STS-3x measure failed");
        ret = false;
    }
    return ret;
}

bool Sts3x::measure(Mode mode,
                                        SingleMode s_setting,
                                        PeriodicMode p_setting) {
    bool ret = true;
    const std::lock_guard<RecursiveMutex> lg(mutex);

    _sts3x_cmd_measure =
        (mode == Mode::SINGLE_SHOT) ? (uint16_t)s_setting : (uint16_t)p_setting;

    //break command to stop a previous periodic mode measure
    ret = writeCmd(STS3X_BREAK_CMD);
    delay(1);//must delay 1ms to allow STS3X to stop periodic data

    //do a check here if it happens to fail
    //the break command when in periodic mode
    if(ret) {
        ret = writeCmd(_sts3x_cmd_measure);
    }

    return ret;
}

bool Sts3x::singleShotRead(float& temperature) {
    uint16_t words[1] {};
    const std::lock_guard<RecursiveMutex> lg(mutex);

    bool ret =
        readWords(words, SENSIRION_NUM_WORDS(words));

    temperature = _convert_raw_temp(words[0]);

    return ret;
}

bool Sts3x::periodicDataRead(Vector<float>& data) {
    const std::lock_guard<RecursiveMutex> lg(mutex);
    int num_of_words = _get_mps_size_to_words();
    Vector<uint16_t> words(num_of_words);
    bool ret = false;

    if(writeCmd(STS3x_PERIODIC_READ_CMD)) {
        ret = readWords(words.data(), num_of_words);
    }

    for(int i = 0; i < num_of_words; i++) {
        if(words.at(i) != STS_DELIMITER) {
            data.append(_convert_raw_temp(words.at(i)));
        }
    }

    return ret;
}

int Sts3x::_get_mps_size_to_words() {
    int size {};

    switch(_sts3x_cmd_measure) {
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

// bool Sts3x::setAlertThd(AlertThd thd, float temperature) {
//     uint16_t limitVal = 0U;
//     uint16_t write_cmd {};
//     bool ret = true;

//     uint16_t rawT = _temperature_to_tick(temperature * SENSIRION_SCALE);
//     uint16_t rawRH = _humidity_to_tick(humidity * SENSIRION_SCALE);

//     /* convert inputs to alert threshold word */
//     limitVal = (rawRH & SHT3X_HUMIDITY_LIMIT_MSK);
//     limitVal |= ((rawT >> 7) & SHT3X_TEMPERATURE_LIMIT_MSK);

//     switch (thd) {
//         case AlertThd::STS3X_HIALRT_SET:
//             write_cmd = WRITE_HIALRT_LIM_SET;
//         break;

//         case AlertThd::STS3X_HIALRT_CLR:
//             write_cmd = WRITE_HIALRT_LIM_CLR;
//         break;

//         case AlertThd::STS3X_LOALRT_CLR:
//             write_cmd = WRITE_LOALRT_LIM_CLR;
//         break;

//         case AlertThd::STS3X_LOALRT_SET:
//             write_cmd = WRITE_LOALRT_LIM_SET;
//         break;
//     }

//     if(!writeCmdWithArgs(write_cmd, &limitVal, 1)) {
//         ret = false;
//         Log.info("failed to set alert limit");
//     }

//     return ret;
// }

// bool Sts3x::getAlertThd(AlertThd thd,
//                                             float& humidity,
//                                             float& temperature) {
//     uint16_t word;
//     uint16_t read_cmd {};

//     bool ret = true;

//     switch (thd) {
//         case AlertThd::STS3X_HIALRT_SET:
//             read_cmd = READ_HIALRT_LIM_SET;
//         break;

//         case AlertThd::STS3X_HIALRT_CLR:
//             read_cmd = READ_HIALRT_LIM_CLR;
//         break;

//         case AlertThd::STS3X_LOALRT_CLR:
//             read_cmd = READ_LOALRT_LIM_CLR;
//         break;

//         case AlertThd::STS3X_LOALRT_SET:
//             read_cmd = READ_LOALRT_LIM_SET;
//         break;
//     }

//     if(writeCmdWithArgs(read_cmd, &word, 1)) {
//         /* convert threshold word to alert settings in 10*%RH & 10*°C */
//         uint16_t rawRH = (word & STS3X_HUMIDITY_LIMIT_MSK);
//         uint16_t rawT = ((word & STS3X_TEMPERATURE_LIMIT_MSK) << 7);

//         humidity = _convert_raw_humidity(rawRH);
//         temperature = _convert_raw_temp(rawT);
//     }
//     else {
//         ret = false;
//         Log.info("failed to get alert limit");
//     }

//     return ret;
// }

bool Sts3x::getStatus(uint16_t& status) {
    const std::lock_guard<RecursiveMutex> lg(mutex);
    return readCmd(STS3X_CMD_READ_STATUS_REG,
                &status,
                1,
                STS3X_CMD_DURATION_USEC);
}

bool Sts3x::clearStatus() {
    const std::lock_guard<RecursiveMutex> lg(mutex);
    return writeCmd(STS3X_CMD_CLR_STATUS_REG);
}

bool Sts3x::heaterOn() {
    const std::lock_guard<RecursiveMutex> lg(mutex);
    return writeCmd(STS3X_CMD_HEATER_ON);
}

bool Sts3x::heaterOff() {
    const std::lock_guard<RecursiveMutex> lg(mutex);
    return writeCmd(STS3X_CMD_HEATER_OFF);
}

float Sts3x::_convert_raw_temp(uint16_t temperature_raw) {
    return (((RAW_TEMP_ADC_COUNT * (int32_t)temperature_raw) >>
                DIVIDE_BY_POWER) - RAW_TEMP_CONST)/SENSIRION_SCALE;
}

uint16_t Sts3x::_temperature_to_tick(int32_t temperature) {
    return (uint16_t)((temperature * TEMP_MULTIPLY_CONSTANT +
                TEMP_ADD_CONSTANT) >> DIVIDE_BY_TICK);
}
