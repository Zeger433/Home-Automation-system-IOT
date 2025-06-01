#define BLYNK_TEMPLATE_ID "TMPL6yeDEA3Qm"
#define BLYNK_TEMPLATE_NAME "HOMEAUTOMATION"
#define BLYNK_AUTH_TOKEN "yghuQaeDK58SlOKBVV7L1bLE5tfuF2p_"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>


char auth[] = "yghuQaeDK58SlOKBVV7L1bLE5tfuF2p_";

// WiFi credentials
char ssid[] = "Galaxy Note20 Ultra 5G7321";
char pass[] = "00000513";

// DHT Sensor settings
#define DHTPIN 15       // DHT sensor connected to GPIO15
#define DHTTYPE DHT11   // Use DHT22 if you're using that sensor
DHT dht(DHTPIN, DHTTYPE);

// Virtual pins for humidity and temperature
#define HUMIDITY_VPIN V5
#define TEMPERATURE_VPIN V6

BlynkTimer timer;

// Button and relay pin definitions
#define button1_pin 26
#define button2_pin 25
#define button3_pin 33
#define button4_pin 32

#define relay1_pin 13
#define relay2_pin 12
#define relay3_pin 14
#define relay4_pin 27

int relay1_state = 0;
int relay2_state = 0;
int relay3_state = 0;
int relay4_state = 0;

#define button1_vpin    V1
#define button2_vpin    V2
#define button3_vpin    V3 
#define button4_vpin    V4

// Blynk connect sync
BLYNK_CONNECTED() {
  Blynk.syncVirtual(button1_vpin);
  Blynk.syncVirtual(button2_vpin);
  Blynk.syncVirtual(button3_vpin);
  Blynk.syncVirtual(button4_vpin);
}

// Virtual button handlers
BLYNK_WRITE(button1_vpin) {
  relay1_state = param.asInt();
  digitalWrite(relay1_pin, relay1_state);
}
BLYNK_WRITE(button2_vpin) {
  relay2_state = param.asInt();
  digitalWrite(relay2_pin, relay2_state);
}
BLYNK_WRITE(button3_vpin) {
  relay3_state = param.asInt();
  digitalWrite(relay3_pin, relay3_state);
}
BLYNK_WRITE(button4_vpin) {
  relay4_state = param.asInt();
  digitalWrite(relay4_pin, relay4_state);
}

// Send humidity and temperature data
void sendSensorData() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature(); // Celsius

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" *C");

  Blynk.virtualWrite(HUMIDITY_VPIN, humidity);
  Blynk.virtualWrite(TEMPERATURE_VPIN, temperature);
}

void setup() {
  Serial.begin(115200);

  pinMode(button1_pin, INPUT_PULLUP);
  pinMode(button2_pin, INPUT_PULLUP);
  pinMode(button3_pin, INPUT_PULLUP);
  pinMode(button4_pin, INPUT_PULLUP);

  pinMode(relay1_pin, OUTPUT);
  pinMode(relay2_pin, OUTPUT);
  pinMode(relay3_pin, OUTPUT);
  pinMode(relay4_pin, OUTPUT);

  // Start with all relays OFF
  digitalWrite(relay1_pin, HIGH);
  digitalWrite(relay2_pin, HIGH);
  digitalWrite(relay3_pin, HIGH);
  digitalWrite(relay4_pin, HIGH);

  dht.begin();

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(2000L, sendSensorData); // Read DHT every 2 seconds
}

void loop() {
  Blynk.run();
  timer.run();
  listen_push_buttons();
}

// Button to relay control
void listen_push_buttons() {
  if (digitalRead(button1_pin) == LOW) {
    delay(200);
    control_relay(1);
    Blynk.virtualWrite(button1_vpin, relay1_state);
  } else if (digitalRead(button2_pin) == LOW) {
    delay(200);
    control_relay(2);
    Blynk.virtualWrite(button2_vpin, relay2_state);
  } else if (digitalRead(button3_pin) == LOW) {
    delay(200);
    control_relay(3);
    Blynk.virtualWrite(button3_vpin, relay3_state);
  } else if (digitalRead(button4_pin) == LOW) {
    delay(200);
    control_relay(4);
    Blynk.virtualWrite(button4_vpin, relay4_state);
  }
}

// Relay toggle logic
void control_relay(int relay) {
  if (relay == 1) {
    relay1_state = !relay1_state;
    digitalWrite(relay1_pin, relay1_state);
    Serial.print("Relay1 State = ");
    Serial.println(relay1_state);
  } else if (relay == 2) {
    relay2_state = !relay2_state;
    digitalWrite(relay2_pin, relay2_state);
  } else if (relay == 3) {
    relay3_state = !relay3_state;
    digitalWrite(relay3_pin, relay3_state);
  } else if (relay == 4) {
    relay4_state = !relay4_state;
    digitalWrite(relay4_pin, relay4_state);
  }
}
