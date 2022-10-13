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

#define CATCH_CONFIG_MAIN

#include "Particle.h"
#include "catch.h"

#include "Sht3x.h"
#include "Sts3x.h"

constexpr int clear_stat_pass_data[] = {0x83, 0xF0, 0x0D};
constexpr int clear_stat_fail_data[] = {0x83, 0xF0, 0x0E};

Sht3x *sht;
Sts3x *sts;

TEST_CASE("SHT tests")
{
    constexpr int singleshot_pass_data[] = {0x61, 0x21, 0x97, 0x74, 0xF8, 0x02};
    constexpr int singleshot_fail_data[] = {0x61, 0x21, 0x97, 0x74, 0xF8, 0x03};
    constexpr int periodic_pass_data[] = {
      0x62, 0x2E, 0x94, 0x96, 0x83, 0xBF, 0x62, 0x2E, 0x94, 0x96, 0xAA, 0xB1,
      0x62, 0x2E, 0x94, 0x96, 0xF5, 0xE1, 0x62, 0x29, 0x03, 0x96, 0xFB, 0xFE};
    constexpr int periodic_fail_data[] = {
      0x62, 0x2E, 0x94, 0x96, 0x83, 0xBF, 0x62, 0x2E, 0x94, 0x96, 0xAA, 0xB1,
      0x62, 0x2E, 0x94, 0x96, 0xF5, 0xE1, 0x62, 0x29, 0x03, 0x96, 0xFB, 0xFD};

    float temp = 0;
    float humidity = 0;
    Vector<std::pair<float, float>> data;

    sht = new (std::nothrow) Sht3x(Wire, Sht3x::ADDR_A, 0);

    // setup init failure
    Wire.end_transmission_return = endTransmissionReturns::TIMEOUT;
    REQUIRE(sht->init() == false);

    // setup init success
    Wire.end_transmission_return = endTransmissionReturns::SUCCESS;
    REQUIRE(sht->init() == true);

    // setup heaterOn failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(sht->heaterOn() == false);

    // setup heaterOn success
    Wire.num_bytes_to_write = 2;
    REQUIRE(sht->heaterOn() == true);

    // setup heaterOff failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(sht->heaterOff() == false);

    // setup heaterOff success
    Wire.num_bytes_to_write = 2;
    REQUIRE(sht->heaterOff() == true);

    // setup clearStatus failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(sht->clearStatus() == false);

    // setup getStatus success
    Wire.num_bytes_to_write = 2;
    REQUIRE(sht->clearStatus() == true);

    // setup getStatus failure 1
    uint16_t dummy_status {};
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(sht->getStatus(dummy_status) == false);

    // setup getStatus failure 2
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 0;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(sht->getStatus(dummy_status) == false);

    // setup getStatus failure  3
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_fail_data;
    REQUIRE(sht->getStatus(dummy_status) == false);

    // setup getStatus success
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(sht->getStatus(dummy_status) == true);

    // setup singleShotMeasureAndRead failure 1
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 6;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(sht->singleShotMeasureAndRead(temp, humidity) == false);

    // setup singleShotMeasureAndRead failure 2
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 5;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(sht->singleShotMeasureAndRead(temp, humidity) == false);

    // setup singleShotMeasureAndRead failure 3
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 6;
    Wire.data_read = singleshot_fail_data;
    REQUIRE(sht->singleShotMeasureAndRead(temp, humidity) == false);

    // setup singleShotMeasureAndRead success
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 6;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(sht->singleShotMeasureAndRead(temp, humidity) == true);

    // setup the periodicDataRead failure 1
    sht->measure(
      Sht3x::Mode::PERIODIC_DATA,
      Sht3x::SingleMode::SINGLE_NONE,
      Sht3x::PeriodicMode::HIGH_4_MPS
    );
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 24;
    Wire.data_read = periodic_pass_data;
    REQUIRE(sht->periodicDataRead(data) == false);

    // setup the periodicDataRead failure 2
    sht->measure(
      Sht3x::Mode::PERIODIC_DATA,
      Sht3x::SingleMode::SINGLE_NONE,
      Sht3x::PeriodicMode::HIGH_4_MPS
    );
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 23;
    Wire.data_read = periodic_pass_data;
    REQUIRE(sht->periodicDataRead(data) == false);

    // setup the periodicDataRead failure 3
    sht->measure(
      Sht3x::Mode::PERIODIC_DATA,
      Sht3x::SingleMode::SINGLE_NONE,
      Sht3x::PeriodicMode::HIGH_4_MPS
    );
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 24;
    Wire.data_read = periodic_fail_data;
    REQUIRE(sht->periodicDataRead(data) == false);

    // setup the periodicDataRead success
    sht->measure(
      Sht3x::Mode::PERIODIC_DATA,
      Sht3x::SingleMode::SINGLE_NONE,
      Sht3x::PeriodicMode::HIGH_4_MPS
    );
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 24;
    Wire.data_read = periodic_pass_data;
    REQUIRE(sht->periodicDataRead(data) == true);
}

TEST_CASE("STS tests")
{
    constexpr int singleshot_pass_data[] = {0x60, 0xC4, 0x57};
    constexpr int singleshot_fail_data[] = {0x60, 0xC4, 0x56};
    constexpr int periodic_pass_data[] = {
      0x60, 0x55, 0x5F, 0xFF, 0xFF, 0xAC, 0x60, 0x5A, 0x71, 0xFF, 0xFF, 0xAC,
      0x60, 0x4F, 0xC7, 0xFF, 0xFF, 0xAC, 0x60, 0x4F, 0xC7, 0xFF, 0xFF, 0xAC};
    constexpr int periodic_fail_data[] = {
      0x60, 0x55, 0x5F, 0xFF, 0xFF, 0xAC, 0x60, 0x5A, 0x71, 0xFF, 0xFF, 0xAC,
      0x60, 0x4F, 0xC7, 0xFF, 0xFF, 0xAC, 0x60, 0x4F, 0xC7, 0xFF, 0xFF, 0xAB};

    float temp = 0;
    Vector<float> data;

    sts = new (std::nothrow) Sts3x(Wire, Sts3x::ADDR_A, 0);

    // setup init failure
    Wire.end_transmission_return = endTransmissionReturns::TIMEOUT;
    REQUIRE(sts->init() == false);

    // setup init success
    Wire.end_transmission_return = endTransmissionReturns::SUCCESS;
    REQUIRE(sts->init() == true);

    // setup heaterOn failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(sts->heaterOn() == false);

    // setup heaterOn success
    Wire.num_bytes_to_write = 2;
    REQUIRE(sts->heaterOn() == true);

    // setup heaterOff failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(sts->heaterOff() == false);

    // setup heaterOff success
    Wire.num_bytes_to_write = 2;
    REQUIRE(sts->heaterOff() == true);

    // setup clearStatus failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(sts->clearStatus() == false);

    // setup getStatus success
    Wire.num_bytes_to_write = 2;
    REQUIRE(sts->clearStatus() == true);

    // setup getStatus failure 1
    uint16_t dummy_status {};
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(sts->getStatus(dummy_status) == false);

    // setup getStatus failure 2
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 0;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(sts->getStatus(dummy_status) == false);

    // setup getStatus failure  3
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_fail_data;
    REQUIRE(sts->getStatus(dummy_status) == false);

    // setup getStatus success
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(sts->getStatus(dummy_status) == true);

    // setup singleShotMeasureAndRead failure 1
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(sts->singleShotMeasureAndRead(temp) == false);

    // setup singleShotMeasureAndRead failure 2
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 2;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(sts->singleShotMeasureAndRead(temp) == false);

    // setup singleShotMeasureAndRead failure 3
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = singleshot_fail_data;
    REQUIRE(sts->singleShotMeasureAndRead(temp) == false);

    // setup singleShotMeasureAndRead success
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(sts->singleShotMeasureAndRead(temp) == true);

    // setup the periodicDataRead failure 1
    sts->measure(
      Sts3x::Mode::PERIODIC_DATA,
      Sts3x::SingleMode::SINGLE_NONE,
      Sts3x::PeriodicMode::HIGH_4_MPS
    );
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 24;
    Wire.data_read = periodic_pass_data;
    REQUIRE(sts->periodicDataRead(data) == false);

    // setup the periodicDataRead failure 2
    sts->measure(
      Sts3x::Mode::PERIODIC_DATA,
      Sts3x::SingleMode::SINGLE_NONE,
      Sts3x::PeriodicMode::HIGH_4_MPS
    );
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 23;
    Wire.data_read = periodic_pass_data;
    REQUIRE(sts->periodicDataRead(data) == false);

    // setup the periodicDataRead failure 3
    sts->measure(
      Sts3x::Mode::PERIODIC_DATA,
      Sts3x::SingleMode::SINGLE_NONE,
      Sts3x::PeriodicMode::HIGH_4_MPS
    );
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 24;
    Wire.data_read = periodic_fail_data;
    REQUIRE(sts->periodicDataRead(data) == false);

    // setup the periodicDataRead success
    sts->measure(
      Sts3x::Mode::PERIODIC_DATA,
      Sts3x::SingleMode::SINGLE_NONE,
      Sts3x::PeriodicMode::HIGH_4_MPS
    );
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 24;
    Wire.data_read = periodic_pass_data;
    REQUIRE(sts->periodicDataRead(data) == true);
}
