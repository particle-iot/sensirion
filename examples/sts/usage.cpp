#include "Particle.h"
#include "Sts3x.h"

#define USE_SINGLE_SHOT
//#define USE_PERIODIC_DATA

SerialLogHandler logHandler(115200, LOG_LEVEL_ALL, {
    // { "app", LOG_LEVEL_ALL },
    // { "hal", LOG_LEVEL_NONE},
    // { "lwip", LOG_LEVEL_NONE},
});

STARTUP(
    System.enableFeature(FEATURE_RESET_INFO);
    System.enableFeature(FEATURE_RETAINED_MEMORY);
);

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

Sts3x* sts;

void setup() {
    Wire.setSpeed(CLOCK_SPEED_100KHZ);
    sts  = new (std::nothrow) Sts3x(Wire, Sts3x::Addr::ADDR_A, 0);
    sts->init();

#if defined(USE_PERIODIC_DATA)
    sts->measure(Sts3x::Mode::PERIODIC_DATA, Sts3x::SingleMode::SINGLE_NONE, Sts3x::PeriodicMode::HIGH_10_MPS);
#endif//defined(USE_PERIODIC_DATA)
}

void loop() {
    static uint32_t read_time_ms = millis();
    
    if(millis() - read_time_ms >= 1000) {
#if defined(USE_SINGLE_SHOT)
        float temp = 0;
        if(sts->singleShotMeasureAndRead(temp, Sts3x::SingleMode::LOW_NO_CLOCK_STRETCH) == 
                SensirionBase::ErrorCodes::NO_ERROR) {
            Log.info("temp:%f", temp);
        }
        else {
            Log.info("failed to read temp");
        }
#else
        Vector<float> values {};
        if(sts->periodicDataRead(values) == 
                SensirionBase::ErrorCodes::NO_ERROR) {
            for(auto iter : values) {
                Log.info("values:%f", iter);
            }
        }
        else {
            Log.info("failed to read periodic temp");
        }
#endif //defined(USE_SINGLE_SHOT)
        read_time_ms = millis();
    }
}