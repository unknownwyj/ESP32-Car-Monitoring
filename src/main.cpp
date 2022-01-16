#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>

const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress; 
int numberOfDevices;
float tempC[10];

Adafruit_ADS1115 ads;
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */ 
#define TIME_TO_SLEEP 3 /* Time ESP32 will go to sleep (in seconds) */
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

float supercapmaxvoltage = 36;

float supercap_adc_voltage = 0.0;
float supercap_voltage = 0.0;
float supercap_percentage = 0.0;
float supercap_calibration = 1.003;

float battery_adc_voltage = 0.0;
float battery_voltage = 0.0;
float battery_percentage = 0.0;
float battery_calibration = 1.003;
//SUPERCAP resistance
float R1 = 340900.0;
float R2 = 29770.0; 
//BATTERY resistance
float RR1 = 325400.0;
float RR2 = 99200.0;
float ref_voltage = 3.3;

int adc_value = 0;
void sensorRUN();
void displayOLED(int modes);

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++){
    if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
  }
}

void setup() {
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  setCpuFrequencyMhz(40);
  WiFi.mode(WIFI_OFF);
  btStop();
  Serial.begin(9600);
  ads.setGain(GAIN_ONE);
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");}
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
  }
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");
  for(int i=0;i<numberOfDevices; i++){
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)){
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      printAddress(tempDeviceAddress);
      Serial.println();
    } else {
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorRUN();
  displayOLED(0);
  esp_deep_sleep_start();
}
void sensorRUN(){
  int16_t results[2];
  float multiplier = 0.125F;
  results[0] = ads.readADC_SingleEnded(0);
  results[1] = ads.readADC_SingleEnded(1);
  supercap_adc_voltage = ((results[0] * multiplier)/1000)*supercap_calibration;
  battery_adc_voltage = ((results[1] * multiplier)/1000)*battery_calibration;
  Serial.println(results[0]);
  Serial.println(supercap_adc_voltage,4);
  supercap_voltage = supercap_adc_voltage / (R2/(R1+R2));
  battery_voltage = battery_adc_voltage / (RR2/(RR1+RR2));
  supercap_percentage = supercap_voltage / supercapmaxvoltage;
  //temperature
  sensors.requestTemperatures(); 
  for(int i=0;i<numberOfDevices; i++){
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)){
      // Output the device ID
      Serial.print("Temperature for device: ");
      Serial.println(i,DEC);
      // Print the data
      tempC[i] = sensors.getTempC(tempDeviceAddress);
      Serial.print("Temp C: ");
      Serial.print(tempC[i]);
      Serial.print(" Temp F: ");
      Serial.println(DallasTemperature::toFahrenheit(tempC[i])); // Converts tempC to Fahrenheit
    }
  }
}
void displayOLED(int modes){
  switch(modes){
    case 0:
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 10);
      // Display static text
      display.println("SuperCap Voltage:");
      display.setTextSize(3);
      display.setCursor (20, 30);
      display.print(supercap_voltage, 2);
      display.println("V");
      display.setTextSize(1);
      display.print("SoC:");
      display.print(supercap_percentage*100, 2);
      display.print("%");
      display.display(); 
      break;
    case 1:
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 10);
      // Display static text
      display.println("Battery Voltage:");
      display.setTextSize(3);
      display.setCursor (20, 30);
      display.print(battery_voltage, 2);
      display.println("V");
      display.display(); 
      break;
    case 2:
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 10);
      // Display static text
      display.println("Temperature:");
      display.println("SuperCap:");
      display.setTextSize(2);
      display.print(tempC[0], 2);
      display.println("°C");
      display.setTextSize(1);
      display.print("UnderSeat:");
      display.print(tempC[1], 2);
      display.println("°C");
      display.display(); 
      break;
  }
}