#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "Wire.h" 
#define MPU6050_ADDR              0x68 // Alternatively set AD0 to HIGH  --> Address = 0x69
#define MPU6050_ACCEL_CONFIG      0x1C // Accelerometer Configuration Register
#define MPU6050_PWR_MGT_1         0x6B // Power Management 1 Register
#define MPU6050_INT_PIN_CFG       0x37 // Interrupt Pin / Bypass Enable Configuration Register
#define MPU6050_INT_ENABLE        0x38 // Interrupt Enable Register
#define MPU6050_LATCH_INT_EN      0x05 // Latch Enable Bit for Interrupt 
#define MPU6050_ACTL              0x07 // Active-Low Enable Bit
#define MPU6050_WOM_EN            0x06 // Wake on Motion Enable bit
#define MPU6050_WOM_THR           0x1F // Wake on Motion Threshold Register
#define MPU6050_MOT_DUR           0x20 // Motion Detection Duration Register
#define MPU6050_ACCEL_INTEL_CTRL  0x69 // Accelaration Interrupt Control Register
#define MPU6050_SIGNAL_PATH_RESET 0x68 // Signal Path Reset Register
byte interruptPin=13;
byte ledPin=D6; 
volatile bool accEvent = false;

const char* ssid = "FRITZ!Box Fon WLAN 7360"; // Hier  Netzwerknamen einsetzen
const char* password = ""; // Hier Passwort des Netzwerkes einsetzen
const char* mqtt_server = "62.75.187.126";
const char* myTobic = "igelalarm";

long timestamp = 0;

WiFiClient espClient;

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    Serial.println("Ich schalte die LED aus");
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    Serial.println("Ich schalte die LED ein");
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect

    if (client.connect(clientId.c_str())) {
      Serial.println("connected. now subscribing...");
      client.subscribe("LED");
      client.publish(myTobic, "Igel-Alarm-Melder bereit");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Wire.begin();
  writeRegister(MPU6050_PWR_MGT_1, 0);
  setInterrupt(1); // set Wake on Motion Interrupt / Sensitivity; 1(highest sensitivity) - 255
  pinMode(ledPin, OUTPUT);
  digitalWrite(D6, HIGH);
  delay(500);
  digitalWrite(D6, LOW);
  delay(500);  
  digitalWrite(D6, HIGH);
  delay(500);
  digitalWrite(D6, LOW);
  pinMode(interruptPin, INPUT);
  pinMode(A0, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), motion, RISING);
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(9600);
  setup_wifi();
  client.setServer(mqtt_server, 1883); 
  client.setCallback(callback);
}

void loop() {

  if (millis() - timestamp >30000) {
    client.publish(myTobic, "Igel-Alarm-Gerät immer noch betriebsbereit");
    timestamp = millis();
  }

  //Serial.println(analogRead(A0));

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if(accEvent){
    Serial.println("Publishing message: BEWEGUNG REGISTRIERT");
    client.publish(myTobic, "BEWEGUNG REGISTRIERT");
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    accEvent = false;
    attachInterrupt(digitalPinToInterrupt(interruptPin), motion, RISING);   
  } 
  /*
  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    snprintf (msg, MSG_BUFFER_SIZE, "hello world %ld", value);
    Serial.print("Publishing message: ");
    Serial.println(msg);
    client.publish(myTobic, msg);
  } */
}

void setInterrupt(byte threshold){
//writeRegister(MPU6050_SIGNAL_PATH_RESET, 0b00000111);  // not(?) needed
//writeRegister(MPU6050_INT_PIN_CFG, 1<<MPU6050_ACTL); // 1<<MPU6050_LATCH_INT_EN
  writeRegister(MPU6050_ACCEL_CONFIG, 0b00000001);
  writeRegister(MPU6050_WOM_THR, threshold); 
  writeRegister(MPU6050_MOT_DUR, 0b00000001);  // set duration (LSB = 1 ms)
//writeRegister(MPU6050_ACCEL_INTEL_CTRL, 0x15);  // not needed (?)
  writeRegister(MPU6050_INT_ENABLE, 1<<MPU6050_WOM_EN);
}
void writeRegister(uint16_t reg, byte value){
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
void motion(){
  accEvent = true;
  detachInterrupt(interruptPin);
}
