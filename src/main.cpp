#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define LOG_to_SD
#ifdef LOG_to_SD
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#endif
const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress; 
int numberOfDevices;
float tempC[10];

Adafruit_ADS1115 ads;
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */ 
#define TIME_TO_SLEEP 3 /* Time ESP32 will go to sleep (in seconds) */
int modes=0;
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
void wakeupreason();
#ifdef LOG_to_SD
void loggingtoSD();
#endif
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++){
    if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
  }
}

void setup() {
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1);
  setCpuFrequencyMhz(240);
  WiFi.mode(WIFI_OFF);
  btStop();
  Serial.begin(9600);
  ads.setGain(GAIN_ONE);
  if (!ads.begin(0x48)) {
    Serial.println("Failed to initialize ADS.");}
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }
  #ifdef LOG_to_SD
  if(!SD.begin()){
  Serial.println("Card Mount Failed");
  return;
  }
  else{
  Serial.println("Card Mounted");
  if (!SD.exists("/SensorData.txt")){
    Serial.println("File doesn't exist creating new file");
    File dataFile = SD.open("/SensorData.txt", FILE_WRITE);
    dataFile.close();
  }
  }
  #endif
  display.clearDisplay();
  display.dim(true);
  /*
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
  } */
}

void loop() {
  // put your main code here, to run repeatedly:
  wakeupreason();
  sensorRUN();
  displayOLED(0);
  esp_light_sleep_start();
}
void wakeupreason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by external signal using RTC_IO");
      if (modes <= 1) modes++;
      else modes = 0;
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by external signal using RTC_CNTL");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      Serial.println("Wakeup caused by touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP:
      Serial.println("Wakeup caused by ULP program");
      break;
    default:
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
  }
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
  /*
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
  }*/
}
void displayOLED(int modes){
  switch(modes){
    case 0:
      display.clearDisplay();
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
      display.clearDisplay();
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
      display.clearDisplay();
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
      display.print("Lithium Battery:");
      display.setTextSize(2);
      display.print(tempC[1], 2);
      display.println("°C");
      display.setTextSize(1);
      display.display(); 
      break;
  }
}
#ifdef LOG_to_SD
void loggingtoSD(){
  char message[100] = "";
  dtostrf (supercap_voltage, 4, 2, message);
  strcat(message, ",");
  dtostrf (battery_voltage, 4, 2, message);
  strcat(message, ",");
  dtostrf (tempC[0], 4, 2, message);
  strcat(message, ",");
  dtostrf (tempC[1], 4, 2, message);
  strcat(message, ",");
  char currenttime[100];
  sprintf(currenttime, "%d", esp_timer_get_time()/1000000);
  strcat(message, currenttime);
  File file = SD.open("/SensorData.txt", FILE_APPEND);
  if(file.println(message)){
    Serial.println("Logging to SD");
  }
  else{
    Serial.println("Failed to log to SD");
  }
  file.close();
}

#endif