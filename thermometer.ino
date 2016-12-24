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
// SYSTEM_THREAD(ENABLED);
// SYSTEM_THREAD(DISABLED);
// SYSTEM_MODE(AUTOMATIC);

//-------------------------------------------------------------------------
// Temperature sensing and publishing to Particle, ThinkSpeak, and Blynk
//-------------------------------------------------------------------------
#define SKETCH "THERMOMETER 1.1.0"
#include "credentials.h"


//-------------------------------------------------------------------------
// Hardware configuration
//-------------------------------------------------------------------------
const byte PIN_LM35 = A0;                       // Ambient temperature sensor LM35DZ
const unsigned int REF_VOLTAGE = 3300;          // Reference voltage for analog reading in millivolts
const float COEF_LM35 = 0.0805861;              // Centigrades per bit - Resolution 12bit, reference 3.3V, 10mV/degC


//-------------------------------------------------------------------------
// System configuration
//-------------------------------------------------------------------------
const unsigned int TIMEOUT_WATCHDOG = 10000;    // Watchdog timeout in milliseconds
const unsigned int TIMEOUT_RECONNECT = 10000;   // Reconnection timeout in milliseconds
unsigned int reconnects;

//-------------------------------------------------------------------------
// Measuring configuration
//-------------------------------------------------------------------------

// Periods (delays) in milliseconds
const unsigned int PERIOD_MEASURE_RSSI = 2000;
const unsigned int PERIOD_MEASURE_TEMP = 2000;
const unsigned int PERIOD_MEASURE_TEMP_TREND = 60000;

// Temperature processing parameters (in centigrades)
const float TEMP_VALUE_NAN = -999.0;    // Temperature not a number
const float TEMP_VALUE_MARGIN = 0.2;    // Temperature hysteresis
const float TEMP_BUCKET[] = {0.0, 5.0, 16.0, 20.0, 24.0, 28.0};
const char* TEMP_STATUS[] = {"unknown", "Freeze", "Cold", "Lukewarm", "Normal", "Warm", "Hot"};

// Statistical smoothing and exponential filtering
const float FACTOR_RSSI = 0.1;    // Filtering factor for RSSI
const float FACTOR_TEMP = 0.2;    // Filtering factor for temperature
ExponentialFilter efRssi = ExponentialFilter(FACTOR_RSSI);
ExponentialFilter efTemp = ExponentialFilter(FACTOR_TEMP);
SmoothSensorData smooth;

// Measured values
int rssiValue;
float tempValue = TEMP_VALUE_NAN, tempTrend;
byte tempStatus;

// Backup variables (long term statistics)
retained int bootCount, bootTimeLast, bootRunPeriod;
retained float tempValueMin = 100.0, tempValueMax = -25.0;


//-------------------------------------------------------------------------
// Particle configuration
//-------------------------------------------------------------------------
#ifdef PARTICLE_CLOUD
const unsigned int PERIOD_PUBLISH_PARTICLE = 15000;
const byte PARTICLE_BATCH_LIMIT = 4;
bool particlePublishAfterBoot = true;
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
int thingspeakResult;
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
#define VPIN_DISCONNECT  V10
#define VPIN_RECONNECTS  V11
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
// SETUP
//-------------------------------------------------------------------------
void setup()
{
    // Boot process
    if (bootCount++ > 0) bootRunPeriod = max(0, Time.now() - bootTimeLast);
    bootTimeLast = Time.now();
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


//-------------------------------------------------------------------------
// LOOP
//-------------------------------------------------------------------------
void loop()
{
    Watchdogs::tickle();
#ifdef BLYNK_CLOUD
    Blynk.run();
#endif
    watchConnection();
    measure();
    publish();
}

void measure()
{
    measureRssi();
    measureTemp();
    measureTempTrend();
}


//-------------------------------------------------------------------------
// Processing
//-------------------------------------------------------------------------
void watchConnection()
{
    static unsigned long tsMeasure;
    if (millis() - tsMeasure >= TIMEOUT_RECONNECT)
    {
        tsMeasure = millis();
        if (!Particle.connected())
        {
            reconnects++;
            Particle.connect();
        }
    }
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
        int value = WiFi.RSSI();
        if (value < 0)
        {
            rssiValue = efRssi.getValue(WiFi.RSSI());
        }
    }
}

void measureTemp()
{
    static unsigned long tsMeasure;
    if (millis() - tsMeasure >= PERIOD_MEASURE_TEMP || tsMeasure == 0)
    {
        tsMeasure = millis();
        while(smooth.registerData(analogRead(PIN_LM35)));
        tempValue = efTemp.getValue(COEF_LM35 * smooth.getMidAverage());
        tempValueMin = fmin(tempValueMin, tempValue); 
        tempValueMax = fmax(tempValueMax, tempValue);
        // Status
        tempStatus = 0;
        for (byte i = 0; i < sizeof(TEMP_BUCKET)/sizeof(TEMP_BUCKET[0]); i++)
        {
            if (tempValue >= TEMP_BUCKET[i]) tempStatus = i + 1;
        }
    }
}

// Temperature change per minute in centigrades
void measureTempTrend()
{
    static unsigned long tsMeasure, tsMeasureOld;
    static float tempValueOld;
    if (tempValue == TEMP_VALUE_NAN) return;    // No processing before measurement
    if (millis() - tsMeasure >= PERIOD_MEASURE_TEMP_TREND || tsMeasure == 0)
    {
        tsMeasure = millis();
        if (tsMeasureOld > 0)
        {
            tempTrend = 6e4 * (tempValue - tempValueOld) / ((float) tsMeasure - (float) tsMeasureOld);
        }
        tsMeasureOld = tsMeasure;
        tempValueOld = tempValue;
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
        byte batchMsgs = 0;
        batchMsgs = publishParticleInits(batchMsgs);     // Publish boot events
#ifdef PHOTON_PUBLISH_VALUE
        batchMsgs = publishParticleValues(batchMsgs);    // Publish value events
#endif
    }
}

byte publishParticleInits(byte sentBatchMsgs)
{
    static byte sendMsgNo;          // Messages number to send next
    byte batchMsgs = sentBatchMsgs; // Messages sent in the current publish run
    byte startMsgNo = sendMsgNo;    // First message published in this run
    while (particlePublishAfterBoot && batchMsgs < PARTICLE_BATCH_LIMIT)
    {        
        bool publishSuccess;
        switch(sendMsgNo)
        {
            case 0:
                publishSuccess = Particle.publish("Boot", String::format("%d/%d/%s", bootCount, bootRunPeriod, System.version().c_str()));
                break;

#ifdef PHOTON_PUBLISH_DEBUG
            case 1:
                publishSuccess = Particle.publish("Sketch", String(SKETCH));
                break;
                
            case 2:
                publishSuccess = Particle.publish("Library", String(WATCHDOGS_VERSION));
                break;

            case 3:
                publishSuccess = Particle.publish("Library", String(EXPONENTIALFILTER_VERSION));
                break;

            case 4:
                publishSuccess = Particle.publish("Library", String(SMOOTHSENSORDATA_VERSION));
                break; 
#endif                    
            default:
                particlePublishAfterBoot = false;
                sendMsgNo = 0;
                break;
        }
        if (!publishSuccess) break; // Break publishing due to cloud disconnection
        batchMsgs++;
        sendMsgNo++;
    }
    return batchMsgs;
}

#ifdef PHOTON_PUBLISH_VALUE
byte publishParticleValues(byte sentBatchMsgs)
{
    static byte sendMsgNo;          // Messages number to send next
    byte batchMsgs = sentBatchMsgs; // Messages sent in the current publish run
    byte startMsgNo = sendMsgNo;    // First message published in this run
    while (batchMsgs < PARTICLE_BATCH_LIMIT)
    {        
        bool publishSuccess;
        switch(sendMsgNo)
        {
            case 0:
                publishSuccess = Particle.publish("RSSI", String::format("%3d", rssiValue));
                break;

            case 1:
                publishSuccess = Particle.publish("Temperature/Trend", String::format("%4.1f/%4.3f", tempValue, tempTrend));
                break;

            case 2:
                publishSuccess = Particle.publish("Status", String(TEMP_STATUS[tempStatus]));
                break;

#ifdef PHOTON_PUBLISH_DEBUG
            case 3:
                publishSuccess = Particle.publish("Reconnects", String(reconnects));
                break;

#ifdef THINGSPEAK_CLOUD
            case 4:
                publishSuccess = Particle.publish("ThingSpeakResult", String(thingspeakResult));
                break;
#endif

#endif
            default:
                sendMsgNo = 0;
                if (startMsgNo > 0) continue;   // Rotate messages from the first one
                break;
        }
        if (!publishSuccess) break; // Break publishing due to cloud disconnection
        batchMsgs++;
        sendMsgNo++;
    }
    return batchMsgs;
}
#endif

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
            thingspeakResult = ThingSpeak.writeFields(CHANNEL_NUMBER, THINGSPEAK_TOKEN);
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
    static byte tempStatusOld = tempStatus;
#endif
    static float tempValueOld = tempValue;
    static unsigned long tsPublish;
    if (millis() - tsPublish >= PERIOD_PUBLISH_BLYNK)
    {
        tsPublish = millis();
        // Publish only at relevant temperature change
        if (fabs(tempValue - tempValueOld) > TEMP_VALUE_MARGIN)
        {
        
            // Temperature status push notification
#ifdef BLYNK_NOTIFY_TEMP
            if (tempStatus != tempStatusOld)
            {
                Blynk.notify(BLYNK_LABEL_PREFIX + BLYNK_LABEL_GLUE + BLYNK_LABEL_TEMP + BLYNK_LABEL_GLUE + TEMP_STATUS[tempStatus]);
                tempStatusOld = tempStatus;
            }
#endif

            // Temperature change signalling
#ifdef BLYNK_SIGNAL_TEMP
            if (tempValue > tempValueOld)
            {
                ledTempInc.on();
                ledTempDec.off();
            }
            else if (tempValue < tempValueOld)
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
            tempValueOld = tempValue;
        }
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

#ifdef VPIN_RECONNECTS
BLYNK_READ(VPIN_RECONNECTS)
{
    Blynk.virtualWrite(VPIN_RECONNECTS, reconnects);
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

#ifdef VPIN_DISCONNECT
BLYNK_WRITE(VPIN_DISCONNECT)
{
    if (param.asInt() == HIGH)
    {
        Particle.disconnect();
    }
}
#endif

#endif
