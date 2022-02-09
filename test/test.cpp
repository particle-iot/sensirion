#define CATCH_CONFIG_MAIN

#include "Particle.h"
#include "catch.h"

#include "Sht3x.h"
#include "Sts3x.h"

constexpr int clear_stat_pass_data[] = {0x83, 0xF0, 0x0D};
constexpr int clear_stat_fail_data[] = {0x83, 0xF0, 0x0E};

Sht3x* sht;
Sts3x* sts;

TEST_CASE("SHT tests") {
    constexpr int singleshot_pass_data[] = {0x61, 0x21, 0x97, 0x74, 0xF8, 0x02};
    constexpr int singleshot_fail_data[] = {0x61, 0x21, 0x97, 0x74, 0xF8, 0x03};
    constexpr int periodic_pass_data[] = {0x62, 0x2E, 0x94, 0x96, 0x83, 0xBF, 0x62, 
                0x2E, 0x94, 0x96, 0xAA, 0xB1, 0x62, 0x2E, 0x94, 0x96, 0xF5, 0xE1, 
                0x62, 0x29, 0x03, 0x96, 0xFB, 0xFE};
    constexpr int periodic_fail_data[] = {0x62, 0x2E, 0x94, 0x96, 0x83, 0xBF, 0x62, 
                0x2E, 0x94, 0x96, 0xAA, 0xB1, 0x62, 0x2E, 0x94, 0x96, 0xF5, 0xE1, 
                0x62, 0x29, 0x03, 0x96, 0xFB, 0xFD};

    float temp = 0;
    float humidity = 0;
    Vector<float> data;

    sht  = new (std::nothrow) Sht3x(Wire, Sht3x::Addr::ADDR_A, 0);

    //setup init failure
    Wire.end_transmission_return = endTransmissionReturns::TIMEOUT;
    REQUIRE(sht->init() == SensirionBase::ErrorCodes::ERROR_FAIL);
    
    //setup init success
    Wire.end_transmission_return = endTransmissionReturns::SUCCESS;
    REQUIRE(sht->init() == SensirionBase::ErrorCodes::NO_ERROR);

    //setup heaterOn failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(sht->heaterOn() == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup heaterOn success
    Wire.num_bytes_to_write = 2;
    REQUIRE(sht->heaterOn() == SensirionBase::ErrorCodes::NO_ERROR);

    //setup heaterOff failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(sht->heaterOff() == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup heaterOff success
    Wire.num_bytes_to_write = 2;
    REQUIRE(sht->heaterOff() == SensirionBase::ErrorCodes::NO_ERROR);

    //setup clearStatus failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(sht->clearStatus() == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup getStatus success
    Wire.num_bytes_to_write = 2;
    REQUIRE(sht->clearStatus() == SensirionBase::ErrorCodes::NO_ERROR);

    //setup getStatus failure 1
    uint16_t dummy_status{};
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(sht->getStatus(dummy_status) == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup getStatus failure 2
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 0;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(sht->getStatus(dummy_status) == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup getStatus failure  3
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_fail_data;
    REQUIRE(sht->getStatus(dummy_status) == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup getStatus success
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(sht->getStatus(dummy_status) == SensirionBase::ErrorCodes::NO_ERROR);

    //setup singleShotMeasureAndRead failure 1
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 6;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(sht->singleShotMeasureAndRead(temp, humidity) == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup singleShotMeasureAndRead failure 2
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 5;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(sht->singleShotMeasureAndRead(temp, humidity) == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup singleShotMeasureAndRead failure 3
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 6;
    Wire.data_read = singleshot_fail_data;
    REQUIRE(sht->singleShotMeasureAndRead(temp, humidity) == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup singleShotMeasureAndRead success
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 6;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(sht->singleShotMeasureAndRead(temp, humidity) == SensirionBase::ErrorCodes::NO_ERROR);

    //setup the periodicDataRead failure 1
    sht->measure(Sht3x::Mode::PERIODIC_DATA, 
                Sht3x::SingleMode::SINGLE_NONE, 
                Sht3x::PeriodicMode::HIGH_4_MPS);
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 24;
    Wire.data_read = periodic_pass_data;
    REQUIRE(sht->periodicDataRead(data) == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup the periodicDataRead failure 2
    sht->measure(Sht3x::Mode::PERIODIC_DATA, 
                Sht3x::SingleMode::SINGLE_NONE, 
                Sht3x::PeriodicMode::HIGH_4_MPS);
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 23;
    Wire.data_read = periodic_pass_data;
    REQUIRE(sht->periodicDataRead(data) == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup the periodicDataRead failure 3
    sht->measure(Sht3x::Mode::PERIODIC_DATA, 
                Sht3x::SingleMode::SINGLE_NONE, 
                Sht3x::PeriodicMode::HIGH_4_MPS);
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 24;
    Wire.data_read = periodic_fail_data;
    REQUIRE(sht->periodicDataRead(data) == SensirionBase::ErrorCodes::ERROR_FAIL);


    //setup the periodicDataRead success
    sht->measure(Sht3x::Mode::PERIODIC_DATA, 
                Sht3x::SingleMode::SINGLE_NONE, 
                Sht3x::PeriodicMode::HIGH_4_MPS);
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 24;
    Wire.data_read = periodic_pass_data;
    REQUIRE(sht->periodicDataRead(data) == SensirionBase::ErrorCodes::NO_ERROR);
}

TEST_CASE("STS tests") {
    constexpr int singleshot_pass_data[] = {0x60, 0xC4, 0x57};
    constexpr int singleshot_fail_data[] = {0x60, 0xC4, 0x56};
    constexpr int periodic_pass_data[] = {0x60, 0x55, 0x5F, 0xFF, 0xFF, 0xAC, 
                0x60, 0x5A, 0x71, 0xFF, 0xFF, 0xAC, 0x60, 0x4F, 0xC7, 0xFF, 0xFF,
                0xAC, 0x60, 0x4F, 0xC7, 0xFF, 0xFF, 0xAC};
    constexpr int periodic_fail_data[] = {0x60, 0x55, 0x5F, 0xFF, 0xFF, 0xAC, 
                0x60, 0x5A, 0x71, 0xFF, 0xFF, 0xAC, 0x60, 0x4F, 0xC7, 0xFF, 0xFF,
                0xAC, 0x60, 0x4F, 0xC7, 0xFF, 0xFF, 0xAB};

    float temp = 0;
    Vector<float> data;

    sts  = new (std::nothrow) Sts3x(Wire, Sts3x::Addr::ADDR_A, 0);

    //setup init failure
    Wire.end_transmission_return = endTransmissionReturns::TIMEOUT;
    REQUIRE(sts->init() == SensirionBase::ErrorCodes::ERROR_FAIL);
    
    //setup init success
    Wire.end_transmission_return = endTransmissionReturns::SUCCESS;
    REQUIRE(sts->init() == SensirionBase::ErrorCodes::NO_ERROR);

    //setup heaterOn failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(sts->heaterOn() == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup heaterOn success
    Wire.num_bytes_to_write = 2;
    REQUIRE(sts->heaterOn() == SensirionBase::ErrorCodes::NO_ERROR);

    //setup heaterOff failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(sts->heaterOff() == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup heaterOff success
    Wire.num_bytes_to_write = 2;
    REQUIRE(sts->heaterOff() == SensirionBase::ErrorCodes::NO_ERROR);

    //setup clearStatus failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(sts->clearStatus() == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup getStatus success
    Wire.num_bytes_to_write = 2;
    REQUIRE(sts->clearStatus() == SensirionBase::ErrorCodes::NO_ERROR);

    //setup getStatus failure 1
    uint16_t dummy_status{};
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(sts->getStatus(dummy_status) == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup getStatus failure 2
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 0;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(sts->getStatus(dummy_status) == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup getStatus failure  3
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_fail_data;
    REQUIRE(sts->getStatus(dummy_status) == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup getStatus success
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(sts->getStatus(dummy_status) == SensirionBase::ErrorCodes::NO_ERROR);

    //setup singleShotMeasureAndRead failure 1
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(sts->singleShotMeasureAndRead(temp) == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup singleShotMeasureAndRead failure 2
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 2;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(sts->singleShotMeasureAndRead(temp) == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup singleShotMeasureAndRead failure 3
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = singleshot_fail_data;
    REQUIRE(sts->singleShotMeasureAndRead(temp) == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup singleShotMeasureAndRead success
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(sts->singleShotMeasureAndRead(temp) == SensirionBase::ErrorCodes::NO_ERROR);

    //setup the periodicDataRead failure 1
    sts->measure(Sts3x::Mode::PERIODIC_DATA, 
                Sts3x::SingleMode::SINGLE_NONE, 
                Sts3x::PeriodicMode::HIGH_4_MPS);
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 24;
    Wire.data_read = periodic_pass_data;
    REQUIRE(sts->periodicDataRead(data) == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup the periodicDataRead failure 2
    sts->measure(Sts3x::Mode::PERIODIC_DATA, 
                Sts3x::SingleMode::SINGLE_NONE, 
                Sts3x::PeriodicMode::HIGH_4_MPS);
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 23;
    Wire.data_read = periodic_pass_data;
    REQUIRE(sts->periodicDataRead(data) == SensirionBase::ErrorCodes::ERROR_FAIL);

    //setup the periodicDataRead failure 3
    sts->measure(Sts3x::Mode::PERIODIC_DATA, 
                Sts3x::SingleMode::SINGLE_NONE, 
                Sts3x::PeriodicMode::HIGH_4_MPS);
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 24;
    Wire.data_read = periodic_fail_data;
    REQUIRE(sts->periodicDataRead(data) == SensirionBase::ErrorCodes::ERROR_FAIL);


    //setup the periodicDataRead success
    sts->measure(Sts3x::Mode::PERIODIC_DATA, 
                Sts3x::SingleMode::SINGLE_NONE, 
                Sts3x::PeriodicMode::HIGH_4_MPS);
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 24;
    Wire.data_read = periodic_pass_data;
    REQUIRE(sts->periodicDataRead(data) == SensirionBase::ErrorCodes::NO_ERROR);
}