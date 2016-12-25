# thermometer
Application for observing ambient temperature as well as boot count and RSSI.

- *Physical observation*:
  - The ambient temperature in centigrades with analog temperature sensor LM35DZ.
  - The trend is calculated in centigrades change per minute.
  - The status is determined based on value buckets.
  - The long term minimal and maximal temperature is calculated.


- *System observation*:
  - Number of boots since recent power-up.
  - Number of seconds since recent boot.
  - Firmware version at recent boot.
  - Number of reconnections to the Particle cloud since recent boot.
  - RSSI wifi signal intensity.


- *Hardware watchdog timers* built in microcontroller are utilized for resetting the microcontroller when it freezes in order to ensure unattended operation.


- Publishing to cloud services:
  - **Particle** for debugging purposes.
  - **ThingSpeak** for graphing and further analyzing of smoothed and filtered values.
  - **Blynk** for mobile application control and observation.


- The application utilizes a separate include **credentials** file with credentials to cloud services.
  - The credential file contains just placeholder for credentials.
  - Replace credential placeholders with real ones only in the Particle dashboard in order not to share or reveal them.


## Temperature measurement
The ambient temperature is measured as a non-negative integer by analog sensor **LM35DZ** in bits within range 0 ~ 4095 with 12-bit resolution. The temperature is then calculated from the measured value by multiplying it with sensor coeficient. The application calculates following values.

- **Current temperature**. The value is measured several times in a measurement burst and statistically smoothed with the library *SmoothSensorData* in order to stabilize analog-digital converter. Then the smoothed valued is exponentially filtered by the library *ExponentialFilter* in order to smooth steep changes in measurements.

- **Minute temperature trend**. It is an averaged temperature change speed in centigrades pre minute.

- **Minimal and maximal temperature**. Those are long term statistics stored in backup memory, so that they are retained across booting the microcontroller.


## Particle
For debugging purposes after changing firmware or when something goes wrong. The microcontroller utilizes native publish mechanism of the Particle platform for sending event messaged to the cloud, which are observable in the **Particle Console**.

The application publishes events in batches of maximal 4 of them according to the cloud politics subsequently. Subsequent events batch is published in the next publishing period determined by corresponding configurtion constant.

At first, the application publishes events related to start (boot) of the microcontroller. Those events are published only once and in several batches if needed. After that the regular events (usually with measured values) are published repeatably. They are rotated in several batches if needed.


## ThingSpeak
For graphing and eventually further processing only *current temperature* and *minute temperature trend* is sent to the ThingSpeak cloud.


### Blynk
For Blynk cloud and composing a mobile application following aspects are provided in order to observe and control the temperature measurement.

- For **Blynk push notifications** the expected range of temperature values is divided into several buckets by respective temperature value thresholds. Each bucket is marked by particular temperature status and for each status a corresponding notification featured text is defined. A push notification is generated and sent to a mobile application only when a temperature status is changed at relevant temperature change.

- All measured values a provided to a mobile application only on demand with help related **Blynk methods and virtual pins**. Values are not pushed to the Blynk cloud, just a mobile application reads them at its time intervals.

- The application provides a Blynk method for **resetting statistical extremes**, i.e., minimal and maximal temperature. It is invoked by a mobile application Blynk button press at sending a logical *HIGH* to the microcontroller. Is is useful when we want to start a new statistical observation, because the backup memory is cleared at power down the microcontroller only.

- The application switches on and off **Blynk LEDs** for positive and negative temperature trend, i.e., temperature increasing or decreasing, for better and smarter indication in a mobile application.
