#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1015 ads;
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */ 
#define TIME_TO_SLEEP 3 /* Time ESP32 will go to sleep (in seconds) */
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define ANALOG_IN_PIN 34
float supercapmaxvoltage = 36;

float supercap_adc_voltage = 0.0;
float supercap_voltage = 0.0;
float supercap_percentage = 0.0;

float R1 = 340900.0;
float R2 = 29770.0; 

float ref_voltage = 3.3;

int adc_value = 0;
void sensorRUN();
void displayOLED(int modes);
void setup() {
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  setCpuFrequencyMhz(40);
  WiFi.mode(WIFI_OFF);
  btStop();
  Serial.begin(9600);
  ads.setGain(GAIN_TWOTHIRDS);
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");}
  pinMode(ANALOG_IN_PIN, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorRUN();
  displayOLED(1);
  esp_deep_sleep_start();
}
void sensorRUN(){/*
  for(int i= 1;i < 201;i++){
    supercap_adc_voltage = supercap_adc_voltage + analogReadMilliVolts(ANALOG_IN_PIN);
  }
  supercap_adc_voltage = supercap_adc_voltage/200;
  supercap_adc_voltage = supercap_adc_voltage/1000;
  supercap_voltage = supercap_adc_voltage / (R2/(R1+R2));
  supercap_percentage = supercap_voltage / supercapmaxvoltage; */
  int16_t results;
  float multiplier = 0.1875F;
  results = ads.readADC_SingleEnded(0);
  supercap_adc_voltage = ads.computeVolts(results);
  supercap_voltage = supercap_adc_voltage / (R2/(R1+R2));
  supercap_voltage = supercap_voltage / supercapmaxvoltage;
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
      display.println(supercap_voltage, 3);
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
      display.println(supercap_voltage, 3);
      display.display(); 
      break;
  }
}