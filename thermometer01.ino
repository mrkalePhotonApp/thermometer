/*
  NAME:
  Measuring temperature with analog sensor LM35DZ.

  DESCRIPTION:
  The application utilizes several technologies for publishing measured values. It serves for getting acquainted with those ones and to use all of them concurrently. As a measured physical value the application utilizes ambient temperature along side with some system measures like boot and reconnect count, RSSI, etc. See more about the application in the README.md file.
  - Siplified version.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
// #define PARTICLE_CLOUD              // Comment to totally ignore Particle Cloud
// #define PHOTON_PUBLISH_DEBUG        // Uncomment to publish debug events to Particle Cloud
#define PHOTON_PUBLISH_VALUE        // Uncomment to publish regular events to Particle Cloud

#define THINGSPEAK_CLOUD            // Comment to totally ignore ThingSpeak Cloud

#define BLYNK_CLOUD                 // Comment to totally ignore Blynk Cloud
// #define BLYNK_NOTIFY_TEMP           // Uncomment to send Blynk push notifications
#define BLYNK_SIGNAL_TEMP           // Uncomment to use Blynk LED signalling

#define MQTT_CLOUD                  // Comment to totally ignore MQTT broker
#define MQTT_PUBLISH                // Uncomment to enable publishing to MQTT topics
#define MQTT_SUBSCRIBE              // Uncomment to enable subscribing to MQTT topics


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

#ifdef MQTT_CLOUD
#include "MQTT/MQTT.h"
#endif

//-------------------------------------------------------------------------
// Boot setup
//-------------------------------------------------------------------------
// STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));
STARTUP(WiFi.selectAntenna(ANT_INTERNAL));
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));
SYSTEM_THREAD(ENABLED);
// SYSTEM_THREAD(DISABLED);
SYSTEM_MODE(AUTOMATIC);


//-------------------------------------------------------------------------
// Temperature sensing and publishing to Particle, ThinkSpeak, and Blynk
//-------------------------------------------------------------------------
#define SKETCH "THERMOMETER01 1.5.0"
#include "credentials.h"


//-------------------------------------------------------------------------
// Hardware configuration
//-------------------------------------------------------------------------
const byte PIN_LM35 = A0;                       // Ambient temperature sensor LM35DZ
const unsigned int REF_VOLTAGE = 3300;          // Reference voltage 3.3V
const float COEF_LM35 = 0.0805861;              // Centigrades per bit


//-------------------------------------------------------------------------
// Time periods (milliseconds)
//-------------------------------------------------------------------------
const unsigned int TIMEOUT_WATCHDOG = 10000;
const unsigned int TIMEOUT_RECONNECT = 10000;
const unsigned int PERIOD_MEASURE_RSSI = 1000;
const unsigned int PERIOD_MEASURE_TEMP = 1000;


//-------------------------------------------------------------------------
// Color definitions
//-------------------------------------------------------------------------
#define COLOR_BLYNK_RED       "#D3435C"
#define COLOR_BLYNK_GREEN     "#23C48E"
#define COLOR_BLYNK_BLUE      "#04C0F8"
#define COLOR_BLYNK_YELLOW    "#ED9D00"
#define COLOR_BLYNK_DARK_BLUE "#5F7CD8"


//-------------------------------------------------------------------------
// Measuring configuration
//-------------------------------------------------------------------------
// Temperature processing parameters (in centigrades)
const float TEMP_VALUE_NAN = -999.0;    // Temperature not a number
const float TEMP_VALUE_MARGIN = 0.5;    // Temperature hysteresis
// const float TEMP_BUCKET[] = {0.0, 5.0, 16.0, 20.0, 24.0, 28.0};
// const char* TEMP_STATUS_NAME[] = {"unknown", "Freeze", "Cold", "Lukewarm", "Normal", "Warm", "Hot"};
const float TEMP_BUCKET[] = {0.0, 4.0, 6.0, 8.0}; // Bottom status values
const char* TEMP_STATUS_NAME[]  = {"unknown", "Freeze", "Cold", "Antifrost", "Lukewarm"};
const char* TEMP_STATUS_COLOR[] = {"unknown", COLOR_BLYNK_DARK_BLUE, COLOR_BLYNK_BLUE, COLOR_BLYNK_GREEN, COLOR_BLYNK_YELLOW};

// Statistical smoothing and exponential filtering
const float EXPFILTER_FACTOR_TEMP = 0.2;    // Filtering factor for temperature
ExponentialFilter efTemp = ExponentialFilter(EXPFILTER_FACTOR_TEMP);
SmoothSensorData smooth;

// Measured values
int rssiValue;
float tempValue;
byte tempStatus;

// Backup variables (long term statistics)
retained int boots, bootTimeLast, bootRunPeriod, reconnects;
retained float tempValueMin = 150.0, tempValueMax = -50.0;


#ifdef PARTICLE_CLOUD
//-------------------------------------------------------------------------
// Particle configuration
//-------------------------------------------------------------------------
const unsigned int PERIOD_PUBLISH_PARTICLE = 15000;
const byte PARTICLE_BATCH_LIMIT = 4;    // Don't change it - platform dependent
#endif


#ifdef THINGSPEAK_CLOUD
//-------------------------------------------------------------------------
// ThinkSpeak configuration
//-------------------------------------------------------------------------
const unsigned int PERIOD_PUBLISH_THINGSPEAK = 60000;
const char* THINGSPEAK_TOKEN = CREDENTIALS_THINGSPEAK_TOKEN;
const unsigned long THINGSPEAK_CHANNEL_NUMBER = CREDENTIALS_THINGSPEAK_CHANNEL;
#define THINGSPEAK_FIELD_RSSI_VALUE 1
#define THINGSPEAK_FIELD_TEMP_VALUE 2
#define THINGSPEAK_FIELD_TEMP_DIFF  3
TCPClient ThingSpeakClient;
int thingspeakResult;
#endif


#ifdef BLYNK_CLOUD
//-------------------------------------------------------------------------
// Blynk configuration
//-------------------------------------------------------------------------
const unsigned int PERIOD_PUBLISH_BLYNK = 30000;
char BLYNK_TOKEN[] = CREDENTIALS_BLYNK_TOKEN;
#define BLYNK_VPIN_LCD_STATUS           V0
#define BLYNK_VPIN_BUTTON_STATUS        V1
#define BLYNK_VPIN_BUTTON_RESET_INIT    V2
#define BLYNK_VPIN_VALUE_RSSI           V3
#define BLYNK_VPIN_VALUE_TEMP           V4
#define BLYNK_VPIN_VALUE_TEMP_GAUGE     V4
#define BLYNK_VPIN_BUTTON_RESET_TEMP    V5
// #define BLYNK_VPIN_VALUE_TEMP_DIFF      V5
#define BLYNK_VPIN_VALUE_TEMP_MIN       V6
#define BLYNK_VPIN_VALUE_TEMP_MAX       V7
#define BLYNK_VPIN_LED_TEMP_INC         V8
#define BLYNK_VPIN_LED_TEMP_DEC         V9

#ifdef BLYNK_VPIN_LCD_STATUS
WidgetLCD lcdStatus(BLYNK_VPIN_LCD_STATUS);
#endif

#ifdef BLYNK_SIGNAL_TEMP
    #ifdef BLYNK_VPIN_LED_TEMP_INC
WidgetLED ledTempInc(BLYNK_VPIN_LED_TEMP_INC);
    #endif
    #ifdef BLYNK_VPIN_LED_TEMP_DEC
WidgetLED ledTempDec(BLYNK_VPIN_LED_TEMP_DEC);
    #endif
#endif

#if defined(BLYNK_NOTIFY_TEMP)
String BLYNK_LABEL_GLUE = String(" -- ");
String BLYNK_LABEL_PREFIX = String("Chalupa");
#endif

#endif


#ifdef MQTT_CLOUD
//-------------------------------------------------------------------------
// MQTT configuration
//-------------------------------------------------------------------------
const unsigned int PERIOD_PUBLISH_MQTT = 2000;
byte MQTT_BROKER_IP[] = {192, 168, 0, 9};
const unsigned int MQTT_BROKER_PORT = 1883;

const String MQTT_PREFIX_DEVICE = String("Chalupa/Interior/"); 

const String MQTT_VAR_RESET = String("Reset");
const String MQTT_VAR_GET = String("Get");
const String MQTT_VAR_INITIALS = String("Initials");
const String MQTT_VAR_STATISTICS = String("Statistics");

const String MQTT_VAR_VALUE = String("Value");
const String MQTT_VAR_MINIMUM = String("Minimum");
const String MQTT_VAR_MAXIMUM = String("Maximum");
const String MQTT_VAR_DIFF = String("Diff");
const String MQTT_VAR_STATUS = String("Status");

#ifdef MQTT_PUBLISH
const String MQTT_TOPIC_PUB_STATUS = String(MQTT_PREFIX_DEVICE + MQTT_VAR_STATUS);

const String MQTT_TOPIC_PUB_RSSI = String(MQTT_PREFIX_DEVICE + "RSSI");

const String MQTT_TOPIC_PUB_TEMP = String(MQTT_PREFIX_DEVICE + "Temperature/");
const String MQTT_TOPIC_PUB_TEMP_VALUE   = String(MQTT_TOPIC_PUB_TEMP + MQTT_VAR_VALUE);
const String MQTT_TOPIC_PUB_TEMP_STATUS  = String(MQTT_TOPIC_PUB_TEMP + MQTT_VAR_STATUS);
const String MQTT_TOPIC_PUB_TEMP_DIFF    = String(MQTT_TOPIC_PUB_TEMP + MQTT_VAR_DIFF);
const String MQTT_TOPIC_PUB_TEMP_MINIMUM = String(MQTT_TOPIC_PUB_TEMP + MQTT_VAR_MINIMUM);
const String MQTT_TOPIC_PUB_TEMP_MAXIMUM = String(MQTT_TOPIC_PUB_TEMP + MQTT_VAR_MAXIMUM);
#endif

#ifdef MQTT_SUBSCRIBE
const String MQTT_TOPIC_SUB_RESET = String(MQTT_PREFIX_DEVICE + MQTT_VAR_RESET);
const String MQTT_TOPIC_SUB_GET = String(MQTT_PREFIX_DEVICE + MQTT_VAR_GET);
#endif

MQTT MqttClient(MQTT_BROKER_IP, MQTT_BROKER_PORT, subscriptionMqtt);
#endif


//-------------------------------------------------------------------------
// SETUP
//-------------------------------------------------------------------------
void setup()
{
    Watchdogs::begin(TIMEOUT_WATCHDOG);
    if (boots++ > 0) bootRunPeriod = max(0, Time.now() - bootTimeLast);
    bootTimeLast = Time.now();

    // Clouds
#ifdef THINGSPEAK_CLOUD
    ThingSpeak.begin(ThingSpeakClient);
#endif

#ifdef BLYNK_CLOUD
    Blynk.begin(BLYNK_TOKEN);
#endif

#ifdef MQTT_CLOUD
    MqttClient.connect(System.deviceID());
    #ifdef MQTT_SUBSCRIBE
    if (MqttClient.isConnected())
    {
        MqttClient.subscribe(MQTT_TOPIC_SUB_RESET);
        MqttClient.subscribe(MQTT_TOPIC_SUB_GET);
    }
    #endif
#endif
}


//-------------------------------------------------------------------------
// LOOP
//-------------------------------------------------------------------------
void loop()
{
    Watchdogs::tickle();
    watchConnection();

#ifdef BLYNK_CLOUD
    Blynk.run();
#endif

#ifdef MQTT_CLOUD
    if (MqttClient.isConnected()) MqttClient.loop();
#endif

    measure();
    publish();
}


//-------------------------------------------------------------------------
// Measuring
//-------------------------------------------------------------------------
void measure()
{
    measureRssi();
    measureTemp();
}


//-------------------------------------------------------------------------
// Processing
//-------------------------------------------------------------------------
void watchConnection()
{
    static unsigned long timeStamp;
    if (millis() - timeStamp >= TIMEOUT_RECONNECT)
    {
        timeStamp = millis();
        if (!Particle.connected())
        {
            reconnects++;
            Particle.connect();
        }
    }
}


//-------------------------------------------------------------------------
// Publishing
//-------------------------------------------------------------------------
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

#if defined(MQTT_CLOUD) && defined(MQTT_PUBLISH)
    publishMqtt();
#endif
}

//-------------------------------------------------------------------------
// Measuring
//-------------------------------------------------------------------------
void measureRssi()
{
    static unsigned long timeStamp;
    if (millis() - timeStamp >= PERIOD_MEASURE_RSSI || timeStamp == 0)
    {
        timeStamp = millis();
        rssiValue = WiFi.RSSI();
    }
}

void measureTemp()
{
    static unsigned long timeStamp;
    if (millis() - timeStamp >= PERIOD_MEASURE_TEMP || timeStamp == 0)
    {
        timeStamp = millis();
        while(smooth.registerData(analogRead(PIN_LM35)));
        tempValue = efTemp.getValue(COEF_LM35 * smooth.getMidAverage());
        tempValueMin = fmin(tempValueMin, tempValue);
        tempValueMax = fmax(tempValueMax, tempValue);
        // Temperature status
        tempStatus = 0;
        for (byte i = 0; i < sizeof(TEMP_BUCKET)/sizeof(TEMP_BUCKET[0]); i++)
        {
            if (tempValue >= TEMP_BUCKET[i]) tempStatus = i + 1;
        }
    }
}

#ifdef PARTICLE_CLOUD
//-------------------------------------------------------------------------
// Publishing to Particle
//-------------------------------------------------------------------------
void publishParticle()
{
    static unsigned long timeStamp;
    if (millis() - timeStamp >= PERIOD_PUBLISH_PARTICLE)
    {
        timeStamp = millis();
        byte batchMsgs = 0;
        batchMsgs = publishParticleInits(batchMsgs);     // Publish boot events
#ifdef PHOTON_PUBLISH_VALUE
        batchMsgs = publishParticleValues(batchMsgs);    // Publish value events
#endif
    }
}

byte publishParticleInits(byte sentBatchMsgs)
{
    static bool flagAfterBoot = true;   // Flag about pending activities right after boot
    static byte sendMsgNo;              // Messages number to send next
    byte batchMsgs = sentBatchMsgs;     // Messages sent in the current publish run
    while (flagAfterBoot && batchMsgs < PARTICLE_BATCH_LIMIT)
    {
        bool publishSuccess;
        switch(sendMsgNo)
        {
            case 0:
                publishSuccess = Particle.publish("Boot", String::format("%d/%d/%s", boots, bootRunPeriod, System.version().c_str()));
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
                flagAfterBoot = false;
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
                publishSuccess = Particle.publish("Temp", String::format("%4.1f", tempValue));
                break;

            case 2:
                publishSuccess = Particle.publish("Status", String(TEMP_STATUS_NAME[tempStatus]));
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


#ifdef THINGSPEAK_CLOUD
//-------------------------------------------------------------------------
// Publishing to ThingSpeak
//-------------------------------------------------------------------------
void publishThingspeak()
{
    static float tempValueOld = tempValue;
    static unsigned long timeStamp;
    if (millis() - timeStamp >= PERIOD_PUBLISH_THINGSPEAK)
    {
        timeStamp = millis();
        bool isField = false;

#ifdef THINGSPEAK_FIELD_RSSI_VALUE
        isField = true;
        ThingSpeak.setField(THINGSPEAK_FIELD_RSSI_VALUE, (int)rssiValue);
#endif

#ifdef THINGSPEAK_FIELD_TEMP_VALUE
        isField = true;
        ThingSpeak.setField(THINGSPEAK_FIELD_TEMP_VALUE, (float)tempValue);
#endif

#ifdef THINGSPEAK_FIELD_TEMP_DIFF
        isField = true;
        float tempDiff = tempValue - tempValueOld;
        tempValueOld = tempValue;
        ThingSpeak.setField(THINGSPEAK_FIELD_TEMP_DIFF, (float)tempDiff);
#endif

        // Publish if there is something to
        if (isField)
        {
            thingspeakResult = ThingSpeak.writeFields(THINGSPEAK_CHANNEL_NUMBER, THINGSPEAK_TOKEN);
        }
    }
}
#endif


#ifdef BLYNK_CLOUD
//-------------------------------------------------------------------------
// Publishing to Blynk
//-------------------------------------------------------------------------
void publishBlynk()
{
#if defined(BLYNK_NOTIFY_TEMP) || defined(BLYNK_SIGNAL_TEMP)
    #ifdef BLYNK_NOTIFY_TEMP
    static byte tempStatusOld = tempStatus;
    #endif
    static float tempValueOld = tempValue;
    static unsigned long timeStamp;
    if (millis() - timeStamp >= PERIOD_PUBLISH_BLYNK)
    {
        timeStamp = millis();
        float tempDiff = tempValue - tempValueOld;

    #ifdef BLYNK_VPIN_VALUE_TEMP
        // Pushing temperature value to the cloud for gauge
        Blynk.virtualWrite(BLYNK_VPIN_VALUE_TEMP, tempValue);
        Blynk.setProperty(BLYNK_VPIN_VALUE_TEMP, "label", String(TEMP_STATUS_NAME[tempStatus]));
        Blynk.setProperty(BLYNK_VPIN_VALUE_TEMP, "color", String(TEMP_STATUS_COLOR[tempStatus]));
    #endif

    #ifdef BLYNK_VPIN_VALUE_TEMP_DIFF
        // Pushing temperature change to the cloud
        Blynk.virtualWrite(BLYNK_VPIN_VALUE_TEMP_DIFF, tempDiff);
    #endif

    #ifdef BLYNK_SIGNAL_TEMP
        // Temperature change signalling
        if (tempDiff > 0)
        {
        #ifdef BLYNK_VPIN_LED_TEMP_INC
            ledTempInc.on();
        #endif
        #ifdef BLYNK_VPIN_LED_TEMP_INC
            ledTempDec.off();
        #endif
        }
        else if (tempDiff < 0)
        {
        #ifdef BLYNK_VPIN_LED_TEMP_INC
            ledTempInc.off();
        #endif
        #ifdef BLYNK_VPIN_LED_TEMP_INC
            ledTempDec.on();
        #endif
        }
        else
        {
        #ifdef BLYNK_VPIN_LED_TEMP_INC
            ledTempInc.off();
        #endif
        #ifdef BLYNK_VPIN_LED_TEMP_INC
            ledTempDec.off();
        #endif
        }
    #endif

        // Publishing only at relevant temperature change
        if (fabs(tempDiff) > TEMP_VALUE_MARGIN)
        {
    #ifdef BLYNK_NOTIFY_TEMP
            // Temperature status push notification
            if (tempStatus != tempStatusOld)
            {
                Blynk.notify(BLYNK_LABEL_PREFIX + BLYNK_LABEL_GLUE + String::format("%4.1f �C", tempValue) + BLYNK_LABEL_GLUE + TEMP_STATUS_NAME[tempStatus]);
                tempStatusOld = tempStatus;
            }
    #endif
        }
        tempValueOld = tempValue;
    }
#endif
}

//-------------------------------------------------------------------------
// Blynk reads
//-------------------------------------------------------------------------
#ifdef BLYNK_VPIN_VALUE_RSSI
BLYNK_READ(BLYNK_VPIN_VALUE_RSSI)
{
    Blynk.virtualWrite(BLYNK_VPIN_VALUE_RSSI, rssiValue);
}
#endif

#ifdef BLYNK_VPIN_VALUE_TEMP_MIN
BLYNK_READ(BLYNK_VPIN_VALUE_TEMP_MIN)
{
    Blynk.virtualWrite(BLYNK_VPIN_VALUE_TEMP_MIN, tempValueMin);
}
#endif

#ifdef BLYNK_VPIN_VALUE_TEMP_MAX
BLYNK_READ(BLYNK_VPIN_VALUE_TEMP_MAX)
{
    Blynk.virtualWrite(BLYNK_VPIN_VALUE_TEMP_MAX, tempValueMax);
}
#endif

#ifdef BLYNK_VPIN_VALUE_TEMP_GAUGE
BLYNK_READ(BLYNK_VPIN_VALUE_TEMP_GAUGE)
{
    Blynk.virtualWrite(BLYNK_VPIN_VALUE_TEMP_GAUGE, tempValue);
    Blynk.setProperty(BLYNK_VPIN_VALUE_TEMP_GAUGE, "label", String(TEMP_STATUS_NAME[tempStatus]));
    Blynk.setProperty(BLYNK_VPIN_VALUE_TEMP, "color", String(TEMP_STATUS_COLOR[tempStatus]));
}
#endif


//-------------------------------------------------------------------------
// Blynk actions
//-------------------------------------------------------------------------
#ifdef BLYNK_VPIN_BUTTON_RESET_INIT
BLYNK_WRITE(BLYNK_VPIN_BUTTON_RESET_INIT)
{
    if (param.asInt() == HIGH)
    {
        boots = bootRunPeriod = bootTimeLast = reconnects = 0;
    }
}
#endif

#ifdef BLYNK_VPIN_BUTTON_RESET_TEMP
BLYNK_WRITE(BLYNK_VPIN_BUTTON_RESET_TEMP)
{
    if (param.asInt() == HIGH)
    {
        tempValueMin = tempValueMax = tempValue;
    }
}
#endif


//-------------------------------------------------------------------------
// Blynk system status
//-------------------------------------------------------------------------
#ifdef BLYNK_VPIN_BUTTON_STATUS
BLYNK_WRITE(BLYNK_VPIN_BUTTON_STATUS)
{
    static byte lcdMode;
    if (param.asInt() == HIGH)
    {
    #ifdef BLYNK_VPIN_LCD_STATUS
        lcd:
        lcdMode++;
        lcdStatus.clear();
        switch (lcdMode)
        {
            case 1:
                lcdStatus.print(0, 0, String::format("SSID: %s", WiFi.SSID()));
                lcdStatus.print(0, 1, String::format("Firmware: %s", System.version().c_str()));
                break;

            case 2:
                lcdStatus.print(0, 0, String::format("Boots: %d", boots));
                lcdStatus.print(0, 1, String::format("Reconnects: %d", reconnects));
                break;

            case 3:
                {
                    String name = SKETCH;
                    lcdStatus.print(0, 0, String(name.substring(0, name.indexOf(' '))));
                    lcdStatus.print(0, 1, String(name.substring(name.indexOf(' ') + 1)));
                }
                break;

            default:
                lcdMode = 0;
                goto lcd;
                break;
        }
    #endif
    }
}
#endif

#endif

#ifdef MQTT_CLOUD
//-------------------------------------------------------------------------
// Publishing to MQTT
//-------------------------------------------------------------------------
#ifdef MQTT_PUBLISH
void publishMqtt()
{
    static float tempValueOld = tempValue;
    static unsigned long timeStamp;
    if (millis() - timeStamp >= PERIOD_PUBLISH_MQTT)
    {
        timeStamp = millis();
        float tempDiff = tempValue - tempValueOld;
        if (MqttClient.isConnected())
        {
            MqttClient.publish(MQTT_TOPIC_PUB_RSSI, String(rssiValue));
            MqttClient.publish(MQTT_TOPIC_PUB_TEMP_STATUS, String(TEMP_STATUS_NAME[tempStatus]));
            MqttClient.publish(MQTT_TOPIC_PUB_TEMP_VALUE, String(tempValue));
            MqttClient.publish(MQTT_TOPIC_PUB_TEMP_MINIMUM, String(tempValueMin));
            MqttClient.publish(MQTT_TOPIC_PUB_TEMP_MAXIMUM, String(tempValueMax));
            MqttClient.publish(MQTT_TOPIC_PUB_TEMP_DIFF, String(tempDiff));
        }
        tempValueOld = tempValue;
    }
}
#endif


//-------------------------------------------------------------------------
// Subscriptions to MQTT
//-------------------------------------------------------------------------
#ifdef MQTT_SUBSCRIBE
void subscriptionMqtt(char* topic, byte* payload, unsigned int length)
{
    String msgTopic = String(topic);
    // Message content
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;
    String msgPayload(p);
    
    // Request for action
    if (msgTopic.equals(MQTT_TOPIC_SUB_RESET))
    {
        if (msgPayload.equals(MQTT_VAR_INITIALS))
        {
            boots = bootRunPeriod = bootTimeLast = reconnects = 0;
        }
        if (msgPayload.equals(MQTT_VAR_STATISTICS))
        {
            tempValueMin = tempValueMax = tempValue;
        }
    }
    // Request for value
    if (msgTopic.equals(MQTT_TOPIC_SUB_GET))
    {
        if (msgPayload.equals(MQTT_VAR_INITIALS))
        {
    #ifdef MQTT_PUBLISH
        MqttClient.publish(MQTT_TOPIC_PUB_STATUS, String::format("%s = %d/%d/%s/%d", MQTT_VAR_INITIALS.c_str(), boots, bootRunPeriod, System.version().c_str(), reconnects));
    #endif
        }
    }
}
#endif

#endif
