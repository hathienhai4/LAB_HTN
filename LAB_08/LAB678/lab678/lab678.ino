#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// WiFi settings
#define WLAN_SSID  "HONOR HOUSE"
#define WLAN_PASS  "bighero6"

// Adafruit IO settings
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "hoangpham2004"
#define AIO_KEY         "aio_xxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Publish LED status
Adafruit_MQTT_Publish light_pub(&mqtt, AIO_USERNAME "/feeds/led");

// Subscribe LED control
Adafruit_MQTT_Subscribe light_sub(&mqtt, AIO_USERNAME "/feeds/led", MQTT_QOS_1);

// Publish temperature
Adafruit_MQTT_Publish temp_pub(&mqtt, AIO_USERNAME "/feeds/temp");

int led_counter = 0;
int led_status = HIGH;

// UART buffer for parsing data from STM32
String uartBuffer = "";

void lightcallback(char *value, uint16_t len) {
    if (value[0] == '0') Serial.print('a');
    if (value[0] == '1') Serial.print('A');
}

void setup() {
    pinMode(2, OUTPUT);
    pinMode(5, OUTPUT);

    // Indicator pin set high during setup
    digitalWrite(5, HIGH);

    Serial.begin(115200);

    // Connect to WiFi
    WiFi.begin(WLAN_SSID, WLAN_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

    // Set callback and subscribe LED feed
    light_sub.setCallback(lightcallback);
    mqtt.subscribe(&light_sub);

    // Connect to Adafruit IO MQTT
    while (mqtt.connect() != 0) {
        mqtt.disconnect();
        delay(500);
    }

    // Setup finished
    digitalWrite(5, LOW);
}

void loop() {
    // Process incoming MQTT packets
    mqtt.processPackets(1);

    // Handle UART data from STM32
    while (Serial.available()) {
        char c = Serial.read();

        // Start of data packet
        if (c == '!') uartBuffer = "";

        uartBuffer += c;

        // End of data packet
        if (c == '#') {
            if (uartBuffer.startsWith("!TEMP:")) {
                String tempStr = uartBuffer.substring(6, uartBuffer.length() - 1);
                float tempValue = tempStr.toFloat();
                temp_pub.publish(tempValue);
            }

            uartBuffer = "";
        }
    }

    // LED control based on UART commands
    if (Serial.available()) {
        int msg = Serial.read();
        if (msg == 'o') Serial.print('O');
        else if (msg == 'a') light_pub.publish(0);
        else if (msg == 'A') light_pub.publish(1);
    }

    // LED blinking logic
    led_counter++;
    if (led_counter == 100) {
        led_counter = 0;
        led_status = !led_status;
        digitalWrite(2, led_status);
    }
}
