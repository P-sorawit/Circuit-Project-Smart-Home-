#include <WiFi.h>
#include <WiFiClient.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

#define LED_BUILTIN 2

//Wifi
WiFiClient   espClient;
PubSubClient client(espClient);             //สร้างออปเจ็ค สำหรับเชื่อมต่อ mqtt

//MQTT potocol
const char* mqtt_broker = "broker.emqx.io";       //IP mqtt server
const int   mqtt_port = 1883;             //port mqtt server

//LDR-IR remote
#define control_onoff 18
#define control_auto 19
#define Remote 39
int Mode = 0;

//IR car
#define IR_SENSOR_PIN 36   
#define BUZZER_PIN 22      // Connect buzzer to GPIO 22
unsigned long previousMillis = 0;  
const long interval = 200;  // Check sensor every 200ms
bool buzzerState = false;  // Track if the buzzer is ON
bool carDetected = false;  // Track car presence
bool buzzer_stop = false;
unsigned long buzzerStartTime = 0;  
const long buzzerTimeout = 5000;  // Buzzer will turn off after 5 seconds

//THERMISTOR 1,2,3
int THERMISTOR_PIN[]= {34,35,32};  //analog pin 34,35,32 
int NOMINAL_RESISTANCE[] ={11850,11850,11850};  // Resistance at 25°C (10kΩ)
#define SERIES_RESISTOR 10000  // 10kΩ Resistor
#define NOMINAL_TEMPERATURE 25  // 25°C
#define BETA_COEFFICIENT 3950  // Beta coefficient (check datasheet)
#define ADC_MAX 4095  // 12-bit ADC (0-4095)
#define REF_VOLTAGE 3.3  // ESP32 reference voltage (3.3V)
#define temp_turn_on_fan 20

//Touch_sensor

int pressCopper=0;
int pressedCopper =0;
bool isPressedCopper=false;
bool copperLight=false;

//digital pin 
#define LED1 21
#define STAYCAP_STATUS_analog 33

const float VOLTAGE_LOW = 1.5;
const float VOLTAGE_HIGH = 2.5;
const unsigned long DEBOUNCE_DELAY = 200; // 200ms debounce delay
bool lastState = false;
bool outputState = false;
unsigned long lastDebounceTime = 0;

//eieiei
unsigned long timeHigh;
unsigned long timeLow;
unsigned long lastTimePressed=0;
unsigned long timePress=0;
bool delMode=false;



void setup() {
  Serial.begin(115200);

  //wifi
  pinMode(LED_BUILTIN, OUTPUT);
  WiFiManager wm;
  //wm.resetSettings();
  bool res;
  res = wm.autoConnect("อุอุอิอิ");
  //res = wm.autoConnect("อุอุอิอิ","password")
  if(!res) {
    Serial.println("Failed to connect");
    ESP.restart();
  } 
  else {
    Serial.println("connected...yeey :)");
    digitalWrite(LED_BUILTIN, HIGH);
  }
  //LDR-IR remote
  pinMode(control_onoff,OUTPUT);
  pinMode(control_auto,OUTPUT);
	pinMode(Remote,INPUT);

  //IR car
  pinMode(BUZZER_PIN, OUTPUT);
  chk_car_default();

  //Touch sensor
  pinMode(LED1, OUTPUT);

}


void reconnect() {  //ฟังก์ชั่นเชื่อมต่อmqtt
  client.setServer(mqtt_broker, mqtt_port);   //เชื่อมต่อmqtt
  while (!client.connected()) //รอจนกว่าจะเชื่อมต่อmqttสำเร็จ
  {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
    if (client.connect(client_id.c_str()))
      Serial.println("Public emqx mqtt broker connected");
    else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

//sent data
unsigned long previousMillis_sentdata = 0;
const long interval_sentdata = 500; // 500ms interval
String home_led_status = "home_led_status";
String fence_led_status = "fence_led_status";
String temp_status = "temp_status";
String parking_status = "parking_status";

void sent_data(){
  static int step = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis_sentdata >= interval_sentdata) {
    previousMillis_sentdata = currentMillis;

    switch (step) {
      case 0:
        client.publish("KMITL/Circuit/Home/Nut/home_led_status", home_led_status.c_str());
        break;
      case 1:
        client.publish("KMITL/Circuit/Home/Nut/fence_led_status", fence_led_status.c_str());
        break;
      case 2:
        client.publish("KMITL/Circuit/Home/Nut/temp_status", temp_status.c_str());
        break;
      case 3:
        client.publish("KMITL/Circuit/Home/Nut/parking_status", parking_status.c_str());
        break;
    }

    step = (step + 1) % 4; // วนลูป 0-3
  }
}

void IR_Remote() {
	int status_remote = analogRead(Remote);
  // Serial.println(analogRead(Remote));
	if(status_remote > 600){
    timePress = millis();
    
    if(timePress>lastTimePressed&&!delMode){
      Mode++;
      lastTimePressed = millis();
      delMode=true;
    }
	}
  else if(status_remote < 600 && delMode && millis()-timePress>50){
    delMode=false;
  }
  // Serial.println(isChange);
	if(Mode>2){
		Mode = 0;
	}
}

void LDR() {
  if(Mode == 0){
    digitalWrite(control_onoff,LOW);
    digitalWrite(control_auto,LOW);
    fence_led_status = "ปิดไฟอยู่";
  }else if(Mode == 1){
    digitalWrite(control_onoff,HIGH);
    digitalWrite(control_auto,LOW);
    fence_led_status = "เปิดไฟอยู่";
  }else if(Mode == 2){
    digitalWrite(control_onoff,LOW);
    digitalWrite(control_auto,HIGH);
    fence_led_status = "ออโต้";
    
  }
  // Serial.println(Mode);
  // Serial.println(fence_led_status);
}

void IR_car(){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    int sensorValue = analogRead(IR_SENSOR_PIN) > 1000 ? 1 : 0;
    //Serial.println(sensorValue);
    if (sensorValue == HIGH && !buzzerState) {  // Car detected & buzzer OFF
      //Serial.println("Car detected! Buzzer ON");
      tone(BUZZER_PIN, 1000);  // Turn on buzzer
      buzzerState = true;
      buzzerStartTime = currentMillis;  // Start timeout countdown
      carDetected = true;
      parking_status = "มีรถจอดอยู่";
    }else if (sensorValue == LOW && carDetected && buzzer_stop) {  // No car detected
      //Serial.println("No car detected.");
      buzzer_stop = false;
      carDetected = false;
      buzzerState = false;
      parking_status = "ไม่มีรถจอดอยู่";
    }
  }
  // Check if buzzer timeout has elapsed
  if (buzzerState && (currentMillis - buzzerStartTime >= buzzerTimeout)) {
    //Serial.println("Buzzer Timeout! Turning OFF");
    noTone(BUZZER_PIN);  // Turn off buzzer
    buzzer_stop = true;
  }
}

void getTemperature() {
  int adcValue[3];
  float voltage[3];
  float resistance[3];
  float steinhart[3];
  float avg_temp = 0.0;
  for(int i=0;i<3;i++){
    adcValue[i] = analogRead(THERMISTOR_PIN[i]);
    //test 25 Celcius
    //adcValue[i] = 2048;   //2500= 35 celcius, 2048 = 25 celcius

    voltage[i] = (adcValue[i] * REF_VOLTAGE) / ADC_MAX;  // Convert to voltage
    // Calculate resistance of the thermistor
    resistance[i] = SERIES_RESISTOR * ((REF_VOLTAGE / voltage[i]) - 1);
    steinhart[i] = resistance[i] / NOMINAL_RESISTANCE[i];  // (R/Ro)
    steinhart[i] = log(steinhart[i]);  // ln(R/Ro)
    steinhart[i] /= BETA_COEFFICIENT;  // 1/B * ln(R/Ro)
    steinhart[i] += 1.0 / (NOMINAL_TEMPERATURE + 273.15);  // + (1/To)
    steinhart[i] = 1.0 / steinhart[i];  // Invert
    steinhart[i] -= 273.15;  // Convert to Celsius
    avg_temp += steinhart[i];
  }
  avg_temp = avg_temp/3.0;
  temp_status = String(avg_temp); //AVG_temp sent data
  //return avg_temp;
}


void chk_car_default(){
  int sensorValue = !digitalRead(IR_SENSOR_PIN);
  if (sensorValue == LOW) {  // Car detected & buzzer OFF
    parking_status = "มีรถจอดอยู่";
  }else if (sensorValue == HIGH) {  // No car detected
    parking_status = "ไม่มีรถจอดอยู่";
  }
}

bool status_touch_ana = false;

int State_touch_buzzer = HIGH;        // the current state of the output pin
int buttonState_touch_buzzer;            // the current reading from the input pin
int lastButtonState_touch_buzzer = LOW;  // the previous reading from the input pin
unsigned long lastDebounceTime_touch_buzzer = 0;  // the last time the output pin was toggled
unsigned long debounceDelay_touch_buzzer = 50;    // the debounce time; increase if the output flickers

void Touch_sensor(){
  int analogValue = analogRead(STAYCAP_STATUS_analog);
  float voltage = analogValue * (3.3 / 4095.0); // Convert ADC to voltage
  // Serial.print("Voltage: ");
  // Serial.println(voltage);
  // Serial.println(status_touch_ana);
  if(voltage > 2.5 ){
    pressCopper = millis();
    if (pressCopper > pressedCopper && !isPressedCopper){
      copperLight = !copperLight;
      digitalWrite(LED1,copperLight);
      if(copperLight == HIGH){
        home_led_status = "เปิดไฟอยู่";
      }
      else{
        home_led_status = "ปิดไฟอยู่";
      }
      isPressedCopper=true;
      pressedCopper=millis();
    }
  }
  else if (voltage <2.5 && isPressedCopper &&millis()-pressCopper>50){
    isPressedCopper=false;
  }

 
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  if (!client.connected()) {
    reconnect();
  }
  //else{
  //   //tempurature
  //Serial.println(getTemperature());
  getTemperature();
  //   //detect car
  IR_car();
  //   //touch sensor (home led)
  //   Touch_LED();
  //   Touch_buzzer();
  //   //remote (fence led)
  IR_Remote();
  //Serial.println(Mode);
  LDR();
  Touch_sensor();
  sent_data();
  // }
}