#include <WiFi.h>
#include <PubSubClient.h>

#include "Zanshin_BME680.h"  // Include the BME680 Sensor library
const uint32_t SERIAL_SPEED{115200}; 


// WiFi settings
const char *ssid = "wifi_name";             // Replace with your WiFi name
const char *password = "wifi_password";     // Replace with your WiFi password

// MQTT Broker settings
const char *mqtt_broker = "****";     // EMQX broker endpoint
const char *mqtt_topic = "****";      // MQTT topic
const char *mqtt_username = "****";   // MQTT username for authentication
const char *mqtt_password = "****";   // MQTT password for authentication
const int mqtt_port = ******;         // MQTT port (TCP)


WiFiClient espClient;
PubSubClient mqtt_client(espClient);


BME680_Class BME680;  ///< Create an instance of the BME680 class
///< Forward function declaration with default value for sea level
float altitude(const int32_t press, const float seaLevel = 1013.25);
float altitude(const int32_t press, const float seaLevel) {
  static float Altitude;
  Altitude =
      44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}  // of method altitude()

void setup() {
  // put your setup code here, to run once:
    Serial.begin(SERIAL_SPEED);  // Start serial port at Baud rate
#ifdef __AVR_ATmega32U4__      // If this is a 32U4 processor, then wait 3 seconds to init USB port
  delay(3000);
#endif
  Serial.print(F("- Initializing BME680 sensor\n"));
  while (!BME680.begin(I2C_STANDARD_MODE)) {  // Start BME680 using I2C, use first device found
    Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(5000);
  }  // of loop until device is located
  Serial.print(F("- Setting 16x oversampling for all sensors\n"));
  BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
  BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
  Serial.print(F("- Setting IIR filter to a value of 4 samples\n"));
  BME680.setIIRFilter(IIR4);  // Use enumerated type values
  Serial.print(F("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n"));  // "�C" symbols
  BME680.setGas(320, 150);  // 320�c for 150 milliseconds


   // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  // Setup MQTT broker connection
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setKeepAlive(60);
  mqtt_client.setCallback(mqttCallback);
  while (!mqtt_client.connected()) {
      String client_id = "esp32-client-" + String(WiFi.macAddress());
      Serial.printf("Connecting to MQTT Broker as %s.....\n", client_id.c_str());
      if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
          Serial.println("Connected to MQTT broker");
          mqtt_client.subscribe(mqtt_topic);
          mqtt_client.publish(mqtt_topic, "Your message"); // Publish some message upon successful connection
      } else {
          Serial.print("Failed, rc=");
          Serial.print(mqtt_client.state());
          Serial.println(" try again in 5 seconds");
          delay(5000);
      }
  }
}  // of method setup()

void mqttCallback(char *mqtt_topic, byte *payload, unsigned int length) {
  Serial.print("Message received on mqtt_topic: ");
  Serial.println(mqtt_topic);
  Serial.print("Message: ");
  for (unsigned int i = 0; i < length; i++) {
      Serial.print((char) payload[i]);
  }
  Serial.println("\n-----------------------");
}

void loop() {
  // put your main code here, to run repeatedly:

  static int32_t  temp, humidity, pressure, gas;  // BME readings
  static char     buf[16];                        // sprintf text buffer
  static float    alt;                            // Temporary variable
  static uint16_t loopCounter = 0;                // Display iterations
  if (loopCounter % 25 == 0) {                    // Show header @25 loops
    Serial.print(F("\nLoop Temp\xC2\xB0\x43 Humid% Press hPa   Alt m Air m"));
    Serial.print(F("\xE2\x84\xA6\n==== ====== ====== ========= ======= ======\n"));  // "�C" symbol
  }    
  BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings
  if (loopCounter++ != 0) {                             // Ignore first reading, might be incorrect
    sprintf(buf, "%4d %3d.%02d", (loopCounter - 1) % 9999,  // Clamp to 9999,
            (int8_t)(temp / 100), (uint8_t)(temp % 100));   // Temp in decidegrees
    Serial.print(buf);
    sprintf(buf, "%3d.%03d", (int8_t)(humidity / 1000),
            (uint16_t)(humidity % 1000));  // Humidity milli-pct
    Serial.print(buf);
    sprintf(buf, "%7d.%02d", (int16_t)(pressure / 100),
            (uint8_t)(pressure % 100));  // Pressure Pascals
    Serial.print(buf);
    alt = altitude(pressure);                                                // temp altitude
    sprintf(buf, "%5d.%02d", (int16_t)(alt), ((uint8_t)(alt * 100) % 100));  // Altitude meters
    Serial.print(buf);
    sprintf(buf, "%4d.%02d\n", (int16_t)(gas / 100), (uint8_t)(gas % 100));  // Resistance milliohms
    Serial.print(buf);
    delay(5000);  // Wait 10s
  }                // of ignore first reading

      if (!mqtt_client.connected()) {
      while (!mqtt_client.connected()) {
          String client_id = "esp32-client-" + String(WiFi.macAddress());
          Serial.printf("Connecting to MQTT Broker as %s.....\n", client_id.c_str());
          if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
              Serial.println("Connected to MQTT broker");
              mqtt_client.subscribe(mqtt_topic);
          } else {
              Serial.print("Failed, rc=");
              Serial.print(mqtt_client.state());
              Serial.println(" try again in 5 seconds");
              delay(5000);
          }
      }
    }
    mqtt_client.loop();
}
