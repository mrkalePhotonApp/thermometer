/*
  NAME:
  Measuring temperature with analog sensor LM35DZ.

  DESCRIPTION:
  Application for observing ambient temperature as well as boot count and RSSI.
  - Physical observation:
    - Ambient temperature by analog sensor LM35DZ in bits within
      range 0 ~ 4095 with 12-bit resolution.
    - The long term minimal and maximal value is calculated.
  - System observation:
    - Number of boots since recent power-up.
    - RSSI wifi signal intensity.
  - All measured values are statistically processed:
    - A batch (burst) of measured values is statistically smoothed by median
      and result is considered as a measured value.
    - Smoothed (measured) values is then processed by exponential filter
      with individual smoothing factor and result is considered as a final
      processed value.
  - Hardware watchdog timers built in microcontroller are utilized for
    reseting the microcontroller when it freezes in order to ensure
    unattended operation.
  - Publishing to cloud services:
    - ThingSpeak for graphing and further analyzing of smoothed
      and filtered values.
    - Blynk for mobile application observation and control.
  - The application utilizes a separate include credentials file
    with credentials to cloud services.
    - The credential file contains just placeholder for credentials.
    - Replace credential placeholders with real ones only in the Particle
      dashboard in order not to share or reveal them.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#define PARTICLE_CLOUD              // Comment to totally ignore Particle Cloud
#define PHOTON_PUBLISH_DEBUG        // Uncomment to publish debug events to Particle
#define PHOTON_PUBLISH_VALUE        // Uncomment to publish regular events to Particle

#define THINGSPEAK_CLOUD            // Comment to totally ignore ThingSpeak Cloud

#define BLYNK_CLOUD                 // Comment to totally ignore Blynk Cloud
#define BLYNK_NOTIFY_TEMP           // Uncomment to send Blynk push notifications
#define BLYNK_SIGNAL_TEMP           // Uncomment to use Blynk LED signalling

// Libraries
#include "watchdogs/watchdogs.h"
#include "smooth-sensor-data/smooth-sensor-data.h"
#include "exponential-filter/exponential-filter.h"
#include "math.h"

#ifdef THINGSPEAK_CLOUD
#include "ThingSpeak/ThingSpeak.h"
#endif

#ifdef BLYNK_CLOUD
#include "blynk/blynk.h"
#endif

// Boot
// STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));
STARTUP(WiFi.selectAntenna(ANT_INTERNAL));
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(AUTOMATIC);
//-------------------------------------------------------------------------
// Temperature sensing and publishing to Particle, ThinkSpeak, and Blynk
//-------------------------------------------------------------------------
#define SKETCH "THERMOMETER 1.0.0"
#include "credentials.h"

const unsigned int TIMEOUT_WATCHDOG = 10000;    // Watchdog timeout in milliseconds

// Hardware configuration
const unsigned char PIN_LM35 = A0;              // Ambient temperature sensor LM35DZ
const unsigned int REF_VOLTAGE = 3300;          // Reference voltage for analog reading in millivolts
const float COEF_LM35 = 0.0805861;              // Centigraged per bit - Resolution 12 bit, reference 3.3V, 10mV/degC

// Measuring, trending, and publishing periods (delays) in milliseconds
const unsigned int PERIOD_MEASURE_RSSI = 2000;
const unsigned int PERIOD_MEASURE_TEMP = 2000;
const unsigned int COUNT_TREND_TEMP = 15;       // Number of measurements to calculate trend

//-------------------------------------------------------------------------
// Particle configuration
//-------------------------------------------------------------------------
#ifdef PARTICLE_CLOUD
const unsigned int PERIOD_PUBLISH_PARTICLE = 15000;
#endif

//-------------------------------------------------------------------------
// ThinkSpeak configuration
//-------------------------------------------------------------------------
#ifdef THINGSPEAK_CLOUD
const unsigned int PERIOD_PUBLISH_THINGSPEAK = 60000;
const char* THINGSPEAK_TOKEN = CREDENTIALS_THINGSPEAK_TOKEN;
const unsigned long CHANNEL_NUMBER = CREDENTIALS_THINGSPEAK_CHANNEL;
#define FIELD_RSSI_VALUE 1
#define FIELD_TEMP_VALUE 2
#define FIELD_TEMP_TREND 3
TCPClient ThingSpeakClient;
#endif

//-------------------------------------------------------------------------
// Blynk configuration
//-------------------------------------------------------------------------
#ifdef BLYNK_CLOUD
const unsigned int PERIOD_PUBLISH_BLYNK = 30000;
char BLYNK_TOKEN[] = CREDENTIALS_BLYNK_TOKEN;
#define VPIN_BOOT_VALUE  V1
#define VPIN_RSSI_VALUE  V2
#define VPIN_LM35_VALUE  V3
#define VPIN_LM35_TREND  V4
#define VPIN_LM35_MIN    V5
#define VPIN_LM35_MAX    V6
#define VPIN_LM35_RESET  V7
#define VPIN_LM35_LED1   V8
#define VPIN_LM35_LED2   V9
#if defined(BLYNK_SIGNAL_TEMP)
WidgetLED ledTempInc(VPIN_LM35_LED1);
WidgetLED ledTempDec(VPIN_LM35_LED2);
#endif
#if defined(BLYNK_NOTIFY_TEMP)
String BLYNK_LABEL_GLUE = String(" -- ");
String BLYNK_LABEL_PREFIX = String("Chalupa");
String BLYNK_LABEL_TEMP = String("temperature");
#endif
#endif

//-------------------------------------------------------------------------
// Measuring configuration
//-------------------------------------------------------------------------
// Measured values
int   rssiValue;
float tempValue, tempValueOld, tempTrend;
unsigned char tempStatus;

// Backup variables (long term statistics)
retained unsigned int bootCount, bootTimeLast, bootRunPeriod;
retained char bootFwVersion[9];
retained float tempValueMin = 100.0, tempValueMax = -25.0;

// Status variables
bool stateAfterBoot = true;

// Statistical smoothing and exponential filtering
const float FACTOR_RSSI = 0.1;    // Filtering factor for RSSI
const float FACTOR_TEMP = 0.2;    // Filtering factor for temperature
ExponentialFilter efRssi = ExponentialFilter(FACTOR_RSSI);
ExponentialFilter efTemp = ExponentialFilter(FACTOR_TEMP);
SmoothSensorData smooth;

// Temperature trigger values
const float TEMP_VALUE_FREEZE  =  5.0;
const float TEMP_VALUE_COLD    = 20.0;
const float TEMP_VALUE_NORMAL  = 24.0;
const float TEMP_VALUE_WARM    = 28.0;
const float TEMP_VALUE_MARGIN  =  0.5;    // Temperature hysteresis in centigrades
const float TEMP_TREND_MARGIN  =  0.01;   // Trend hysteresis in centigrades per minute

// Temperature statuses
const unsigned char TEMP_STATUS_FREEZE = 1;
const unsigned char TEMP_STATUS_COLD   = 2;
const unsigned char TEMP_STATUS_NORMAL = 3;
const unsigned char TEMP_STATUS_WARM   = 4;
const unsigned char TEMP_STATUS_HOT    = 5;

void setup()
{
    // Boot process
    if (bootCount++ > 0) bootRunPeriod = Time.now() - bootTimeLast;
    bootTimeLast = Time.now();
    strcpy(bootFwVersion, System.version());
    // Clouds
#ifdef THINGSPEAK_CLOUD
    ThingSpeak.begin(ThingSpeakClient);
#endif
#ifdef BLYNK_CLOUD
    Blynk.begin(BLYNK_TOKEN);
#endif
    // Start watchdogs
    Watchdogs::begin(TIMEOUT_WATCHDOG);
}

void loop()
{
    Watchdogs::tickle();
#ifdef BLYNK_CLOUD
    Blynk.run();
#endif
    measure();
    publish();
}

void measure()
{
    measureRssi();
    measureTemp();
}

void publish()
{
#ifdef PARTICLE_CLOUD
    publishParticle();
#endif

#ifdef THINGSPEAK_CLOUD
    publishThingspeak();
#endif

#ifdef BLYNK_CLOUD
    publishBlynk();
#endif
}

//-------------------------------------------------------------------------
// Measuring
//-------------------------------------------------------------------------
void measureRssi()
{
    static unsigned long tsMeasure;
    if (millis() - tsMeasure >= PERIOD_MEASURE_RSSI || tsMeasure == 0)
    {
        tsMeasure = millis();
        while(smooth.registerData(-1 * WiFi.RSSI()));
        rssiValue = (int) efRssi.getValue((float) (-1 * smooth.getMidAverage()));
    }
}

void measureTemp()
{
    static unsigned long tsMeasure, tsMeasureOld;
    static unsigned int cntMeasure;
    if (millis() - tsMeasure >= PERIOD_MEASURE_TEMP || tsMeasure == 0)
    {
        tsMeasure = millis();
        while(smooth.registerData(analogRead(PIN_LM35)));
        tempValue = efTemp.getValue(COEF_LM35 * smooth.getMidAverage());
        // Status
        if (tempValue <= TEMP_VALUE_FREEZE)
        {
            tempStatus = TEMP_STATUS_FREEZE;
        }
        else if (tempValue <= TEMP_VALUE_COLD)
        {
            tempStatus = TEMP_STATUS_COLD;
        }
        else if (tempValue <= TEMP_VALUE_NORMAL)
        {
            tempStatus = TEMP_STATUS_NORMAL;
        }
        else if (tempValue <= TEMP_VALUE_WARM)
        {
            tempStatus = TEMP_STATUS_WARM;
        }
        else
        {
            tempStatus = TEMP_STATUS_HOT;
        }
        // Trend and statistics
        if (tsMeasureOld == 0)
        {
            tsMeasureOld = tsMeasure;
            tempValueOld = tempValue;
        }
        else if (tsMeasure > tsMeasureOld)
        {
            if (tempValueMin > tempValue) tempValueMin = tempValue;
            if (tempValueMax < tempValue) tempValueMax = tempValue;
            // Calculate trend
            if (++cntMeasure >= COUNT_TREND_TEMP)
            {
                tempTrend = 6e4 * (tempValue - tempValueOld) / ((float) tsMeasure - (float) tsMeasureOld);
                cntMeasure = 0;
                tsMeasureOld = tsMeasure;
                tempValueOld = tempValue;
            }
        }
    }
}

//-------------------------------------------------------------------------
// Publishing to Particle
//-------------------------------------------------------------------------
#ifdef PARTICLE_CLOUD
void publishParticle()
{
    static unsigned long tsPublish;
    if (millis() - tsPublish >= PERIOD_PUBLISH_PARTICLE)
    {
        tsPublish = millis();
        // Publish boot events
        if (stateAfterBoot)
        {
            bool sent;
            sent = Particle.publish("Boot_Cnt_Run", String::format("%d/%d/%s", bootCount, bootRunPeriod, bootFwVersion));
#ifdef PHOTON_PUBLISH_DEBUG
            if (sent)
            {
                sent = Particle.publish("Sketch", String(SKETCH));
            }
#endif
            if (sent) stateAfterBoot = false;
        }
        // Publish values
        Particle.publish("RSSI", String::format("%3d", rssiValue));
        Particle.publish("Temperature/Trend", String::format("%4.1f/%4.3f", tempValue, tempTrend));
    }
}
#endif


//-------------------------------------------------------------------------
// Publishing to ThingSpeak
//-------------------------------------------------------------------------
#ifdef THINGSPEAK_CLOUD
void publishThingspeak()
{
    static unsigned long tsPublish;
    if (millis() - tsPublish >= PERIOD_PUBLISH_THINGSPEAK)
    {
        tsPublish = millis();
        bool isField = false;

#ifdef FIELD_RSSI_VALUE
        isField = true;
        ThingSpeak.setField(FIELD_RSSI_VALUE, rssiValue);
#endif

#ifdef FIELD_TEMP_VALUE
        isField = true;
        ThingSpeak.setField(FIELD_TEMP_VALUE, tempValue);
#endif

#ifdef FIELD_TEMP_TREND
        isField = true;
        ThingSpeak.setField(FIELD_TEMP_TREND, tempTrend);
#endif

        // Publish if there is something to
        if (isField)
        {
            int result = ThingSpeak.writeFields(CHANNEL_NUMBER, THINGSPEAK_TOKEN);
#if defined(PARTICLE_CLOUD) && defined(PHOTON_PUBLISH_DEBUG)
            Particle.publish("ThingSpeakResult", String(result));
#endif
        }
    }
}
#endif


//-------------------------------------------------------------------------
// Publishing to Blynk
//-------------------------------------------------------------------------
#ifdef BLYNK_CLOUD
void publishBlynk()
{
#if defined(BLYNK_NOTIFY_TEMP) || defined(BLYNK_SIGNAL_TEMP)
#ifdef BLYNK_NOTIFY_TEMP
    static unsigned char tempStatusOld = tempStatus;
#endif
    static unsigned long tsPublish;
    if (millis() - tsPublish >= PERIOD_PUBLISH_BLYNK)
    {
        tsPublish = millis();
        // Temperature status push notification
#ifdef BLYNK_NOTIFY_TEMP
        if (tempStatus != tempStatusOld && fabs(tempValue - tempValueOld) > TEMP_VALUE_MARGIN)
        {
            String txtStatus;
            switch (tempStatus)
            {
                case TEMP_STATUS_FREEZE:
                    txtStatus = String("Freeze");
                    break;
                case TEMP_STATUS_COLD:
                    txtStatus = String("Cold");
                    break;
                case TEMP_STATUS_NORMAL:
                    txtStatus = String("Normal");
                    break;
                case TEMP_STATUS_WARM:
                    txtStatus = String("Warm");
                    break;
                case TEMP_STATUS_HOT:
                    txtStatus = String("Hot");
                    break;
            }
            Blynk.notify(BLYNK_LABEL_PREFIX + BLYNK_LABEL_GLUE + BLYNK_LABEL_TEMP + BLYNK_LABEL_GLUE + txtStatus);
            tempStatusOld = tempStatus;
        }
#endif

        // Temperature trend signalling
#ifdef BLYNK_SIGNAL_TEMP
        if (tempTrend > TEMP_TREND_MARGIN)
        {
            ledTempInc.on();
            ledTempDec.off();
        }
        else if (tempTrend < -1.0 * TEMP_TREND_MARGIN)
        {
            ledTempInc.off();
            ledTempDec.on();
        }
        else
        {
            ledTempInc.off();
            ledTempDec.off();
        }
#endif
    }
#endif
}

#ifdef VPIN_BOOT_VALUE
BLYNK_READ(VPIN_BOOT_VALUE)
{
    Blynk.virtualWrite(VPIN_BOOT_VALUE, bootCount);
}
#endif

#ifdef VPIN_RSSI_VALUE
BLYNK_READ(VPIN_RSSI_VALUE)
{
    Blynk.virtualWrite(VPIN_RSSI_VALUE, rssiValue);
}
#endif

#ifdef VPIN_LM35_VALUE
BLYNK_READ(VPIN_LM35_VALUE)
{
    Blynk.virtualWrite(VPIN_LM35_VALUE, String::format("%4.1f", tempValue));
}
#endif

#ifdef VPIN_LM35_TREND
BLYNK_READ(VPIN_LM35_TREND)
{
    Blynk.virtualWrite(VPIN_LM35_TREND, String::format("%5.3f", tempTrend));
}
#endif

#ifdef VPIN_LM35_MIN
BLYNK_READ(VPIN_LM35_MIN)
{
    Blynk.virtualWrite(VPIN_LM35_MIN, String::format("%4.1f", tempValueMin));
}
#endif

#ifdef VPIN_LM35_MAX
BLYNK_READ(VPIN_LM35_MAX)
{
    Blynk.virtualWrite(VPIN_LM35_MAX, String::format("%4.1f", tempValueMax));
}
#endif

#ifdef VPIN_LM35_RESET
BLYNK_WRITE(VPIN_LM35_RESET)
{
    if (param.asInt() == HIGH)
    {
        tempValueMin = tempValueMax = tempValue;
    }
}
#endif

#endif
