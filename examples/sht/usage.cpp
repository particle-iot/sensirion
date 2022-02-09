#include "Particle.h"
#include "Sht3x.h"

//#define USE_SINGLE_SHOT
#define USE_PERIODIC_DATA

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

Sht3x* sht;

void setup() {
    Wire.setSpeed(CLOCK_SPEED_100KHZ);
    sht  = new (std::nothrow) Sht3x(Wire, Sht3x::Addr::ADDR_A, 0);
    sht->init();

#if defined(USE_PERIODIC_DATA)
    sht->measure(Sht3x::Mode::PERIODIC_DATA, Sht3x::SingleMode::SINGLE_NONE, Sht3x::PeriodicMode::HIGH_10_MPS);
#endif//defined(USE_PERIODIC_DATA)
}

void loop() {
    static uint32_t read_time_ms = millis();
    
    if(millis() - read_time_ms >= 1000) {
#if defined(USE_SINGLE_SHOT)
        float temp = 0;
        float humidity = 0;
        if(sht->singleShotMeasureAndRead(temp, humidity) == 
                SensirionBase::ErrorCodes::NO_ERROR) {
            Log.info("temp:%f  humidity:%f", temp, humidity);
        }
        else {
            Log.info("failed to read temp and humidity");
        }
#else
        Vector<float> values {};
        if(sht->periodicDataRead(values) == SensirionBase::ErrorCodes::NO_ERROR) {
            for(auto iter : values) {
                Log.info("values:%f", iter);
            }
        }
        else {
            Log.info("failed to read periodic temp and humidity");
        }
#endif //defined(USE_SINGLE_SHOT)
        read_time_ms = millis();
    }
}