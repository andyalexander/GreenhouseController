#include <Arduino.h>
#include "DHT.h"

#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

#include <secret.h>

DHT dht;
const int PIN_DHT = 6;
const int PIN_RELAY1 = 2;
const int PIN_RELAY2 = 3;
const int PIN_RELAY3 = 4;
const int PIN_RELAY4 = 5;

const int PIN_MOTOR_DIRECTION = 7;
const int PIN_MOTOR_SPEED = 8;

// Setup the ethernet / mqtt conection
EthernetClient ethClient;
PubSubClient mqttClient;
uint8_t mac[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x06};
String ip = "";

#define TOPIC_ROOF "garden/greenhouse/roof/"
#define TOPIC_TEMP "garden/greenhouse/temperature"
#define TOPIC_HUMIDITY "garden/greenhouse/humidity"

#define TEMP_MIN 25
#define TEMP_MAX 16
unsigned long UPDATE_DELAY = 5000;      // 5 mins

// void mqttCallback(char* topic, byte* payload, unsigned int length) {
//   char msgBuffer[20];
//   // I am only using one ascii character as command, so do not need to take an entire word as payload
//   // However, if you want to send full word commands, uncomment the next line and use for string comparison
//   // payload[length]='\0';// terminate string with 0
//   // String strPayload = String((char*)payload);  // convert to string
//   // Serial.println(strPayload); 
//   Serial.print("Message arrived [");
//   Serial.print(topic);
//   Serial.print("] ");//MQTT_BROKER
//   for (int i = 0; i < length; i++) {
//     Serial.print((char)payload[i]);
//   }
//   Serial.println();
//   Serial.println(payload[0]);

//   // Examine only the first character of the message
//   if (payload[0] == 49)             // Message "1" in ASCII (turn output ON)
//   {
//     digitalWrite(LED_BUILTIN, HIGH);
//   } else if (payload[0] == 48)      // Message "0" in ASCII (turn output OFF)
//   {
//     digitalWrite(LED_BUILTIN, LOW);
//   } else if (payload[0] == 50)
//   {
//     mqttClient.publish("home/br/nb/ip", ip.c_str());// publish IP nr
//   } else {
//     Serial.println("Unknown value");
//     mqttClient.publish("home/br/nb", "Syntax Error");
//   }
// }

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("arduinoClient1234")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      // mqttClient.publish("outTopic","hello world");
      // ... and resubscribe
      // mqttClient.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(20000);
    }
  }
}

void setup() {
  Serial.begin(9600);

  dht.setup(PIN_DHT); // data pin 2

  // Setup relay pins and initially close
  pinMode(PIN_RELAY1, OUTPUT);
  pinMode(PIN_RELAY2, OUTPUT);
  pinMode(PIN_RELAY3, OUTPUT);
  pinMode(PIN_RELAY4, OUTPUT);
  // digitalWrite(PIN_RELAY1, HIGH);
  // digitalWrite(PIN_RELAY2, LOW);
  // digitalWrite(PIN_RELAY3, LOW);
  // digitalWrite(PIN_RELAY4, LOW);

  pinMode(PIN_MOTOR_DIRECTION, OUTPUT);
  pinMode(PIN_MOTOR_SPEED, OUTPUT);

  // setup ethernet communication using DHCP
  if (Ethernet.begin(mac) == 0) {
    //Serial.println(F("Unable to configure Ethernet using DHCP"));
    for (;;);
  }

  Serial.println(F("Ethernet configured via DHCP"));
  Serial.print("IP address: ");
  Serial.println(Ethernet.localIP());
  Serial.println();
  //convert ip Array into String
  ip = String (Ethernet.localIP()[0]);
  ip = ip + ".";
  ip = ip + String (Ethernet.localIP()[1]);
  ip = ip + ".";
  ip = ip + String (Ethernet.localIP()[2]);
  ip = ip + ".";
  ip = ip + String (Ethernet.localIP()[3]);
  Serial.println(ip);

  // setup mqtt client
  mqttClient.setClient(ethClient);
  mqttClient.setServer(MQTT_SERVER, 1883); 
  mqttClient.connect("greenhouseArduino");
  // mqttClient.setCallback(mqttCallback);
  // mqttClient.publish("home/br/nb/ip", ip.c_str());

    Serial.println("Setup complete");
}

void open_roof(int time){
  mqttClient.publish(TOPIC_ROOF, "OPEN");
  digitalWrite(PIN_MOTOR_DIRECTION, HIGH);
  digitalWrite(PIN_MOTOR_SPEED, HIGH);
  delay(time * 1000);
  digitalWrite(PIN_MOTOR_SPEED, LOW);
}

void close_roof(int time){
  mqttClient.publish(TOPIC_ROOF, "OPEN");
  digitalWrite(PIN_MOTOR_DIRECTION, LOW);
  digitalWrite(PIN_MOTOR_SPEED, HIGH);
  delay(time * 1000);
  digitalWrite(PIN_MOTOR_SPEED, LOW);
}

void adjust_roof(unsigned long temp, unsigned long humidity){
  if (temp > TEMP_MAX) {
    open_roof(1);
  }

  if (temp < TEMP_MIN){
    close_roof(1);
  }
}

unsigned long LAST_UPDATE_TIME = 0;

void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }

  unsigned long current_time = millis();

  if ((current_time - LAST_UPDATE_TIME) > UPDATE_DELAY) {   
    // Publish the humidity and temp
    float humidity = dht.getHumidity();
    float temperature = dht.getTemperature();

    char temp_str[5];
    dtostrf(temperature, 5, 2, temp_str);
    mqttClient.publish(TOPIC_TEMP, temp_str);
        
    temp_str[0] = 0;
    dtostrf(humidity, 5, 2, temp_str);
    mqttClient.publish(TOPIC_HUMIDITY, temp_str);

    // Handle the roof 
    adjust_roof(temperature, humidity);

    // save the current time
    LAST_UPDATE_TIME = current_time;
  }

  mqttClient.loop();
}