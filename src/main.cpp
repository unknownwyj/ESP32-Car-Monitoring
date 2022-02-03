#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>
#include <ESP32Time.h>
#define time_offset 28800 // UTC+8
#define forcecarstarted
#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCK 18
#define SD_CS 15 // SD Adapter
ESP32Time rtc;
//#define LOG_to_SD
#ifdef LOG_to_SD
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#endif
#define DELAYTIMER 1 // 1 second delay
bool ChangedModecleardisplaycheck = true;
struct Button {
  const uint8_t PIN;
  bool pressed;
};
Button button1 = {33,false};
//gps stuff
struct tm timeinfo;
static const uint32_t GPSBaud = 9600;
#define RXD2 16
#define TXD2 17
TinyGPSPlus gps;
TaskHandle_t gpstask;
bool gpsupdated = false;
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
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET,800000U,100000U);

float supercapmaxvoltage = 36;

float supercap_adc_voltage = 0.0;
float supercap_voltage = 0.0;
float supercap_percentage = 0.0;
float supercap_calibration = 1.003;

float battery_adc_voltage = 0.0;
float battery_voltage = 0.0;
float battery_percentage = 0.0;
float battery_calibration = 1.003;
float accdect_adc_voltage = 0.0;
float accdect_voltage = 0.0;
float accdect_calibration = 1.003;
//SUPERCAP resistance
float R1 = 340900.0;
float R2 = 29770.0; 
//BATTERY resistance
float RR1 = 325400.0;
float RR2 = 99200.0;

float RRR1 = 325400.0;
float RRR2 = 99200.0;
float carstartedref_voltage = 12.0;
float ref_voltage = 3.3;
bool Car_started = true;
int64_t last_time = 0;
int64_t current_time = 0;
int64_t Timerforbutton = 0;
int adc_value = 0;
void sensorRUN();
void displayOLED(int modes);
void wakeupreason();
int GPSTime();
void gpsRUN(void *pvParameters);
void drawCentreString(const String &buf, int x, int y);
#ifdef LOG_to_SD
void loggingtoSD();
#endif
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++){
    if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
  }
}
void IRAM_ATTR isr() {
  uint64_t now = esp_timer_get_time();
  if(now - Timerforbutton > 1000000) {
    button1.pressed = true;
    Timerforbutton = esp_timer_get_time();
  }
}
void setup() {
  pinMode(button1.PIN, INPUT);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1);
  attachInterrupt(button1.PIN, isr, FALLING);
  setCpuFrequencyMhz(240);
  WiFi.mode(WIFI_OFF);
  pinMode(SD_CS, OUTPUT);
  btStop();
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  ads.setGain(GAIN_ONE);
  if (!ads.begin(0x48)) {
    Serial.println("Failed to initialize ADS.");}
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }
  #ifdef LOG_to_SD
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  delay(1000);
  if(!SD.begin(SD_CS, SPI, 4000000)){
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
  xTaskCreatePinnedToCore(gpsRUN,"trackgps",10000,NULL,5,&gpstask,0);
  /*
  display.clearDisplay();
  display.setTextSize(1);
  display.println("Starting...");
  display.display();
  display.print("Waiting for GPS");
  for(int i = 0; i < 10; i++){
    display.print(".");
    display.display();
    if(gps.time.isUpdated()){
      display.clearDisplay();
      display.setTextSize(1);
      display.println("GPS Found");
      display.display();
      int time = GPSTime();
      if (time == 0){
        display.clearDisplay();
        display.setTextSize(1);
        display.println("GPS Time 0 Error");
        display.display();
      }
      else{
        rtc.setTime(time+time_offset);
        i = 20;
      }
      break;
    }
    vTaskDelay(1000);
  }*/
}
void gpsRUN(void *pvParameters) {
  while(1){
    if(Serial2.available() > 0){
      while(Serial2.available() > 0){
        gps.encode(Serial2.read());
        gpsupdated = true;
      }
    }
    vTaskDelay(1);
    getLocalTime(&timeinfo);
  }
}
void loop() {
  // put your main code here, to run repeatedly:
   if (button1.pressed) {
    Serial.println("Button Pressed");
    button1.pressed = false;
    ChangedModecleardisplaycheck = true;
    modes++;
    if(modes>=4){
      modes=0;
    }
    displayOLED(modes);
    last_time = esp_timer_get_time();
  }
  if(!Car_started){
    wakeupreason();
    sensorRUN();
    displayOLED(modes);
    int i = 0;
    while(!gpsupdated){
      Serial.println("waiting for gps");
      i++;
      if(i > 50){
        gpsupdated = true; 
      }
      vTaskDelay(100);
    }
    gpsupdated = false;
    #ifdef LOG_to_SD
    loggingtoSD();
    #endif
    esp_light_sleep_start();
  }
  else{
    if((esp_timer_get_time() - last_time)/500000 > DELAYTIMER){
      Serial.print("Time Used:");
      last_time = esp_timer_get_time();
      sensorRUN();
      displayOLED(modes);
      #ifdef LOG_to_SD
      Serial.print("|loggingtoSD = ");
      loggingtoSD();
      #endif
      Serial.println(esp_timer_get_time() - last_time);
    }
  }
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
  int16_t results[3];
  float multiplier = 0.125F;
  results[0] = ads.readADC_SingleEnded(0);
  results[1] = ads.readADC_SingleEnded(1);
  results[2] = ads.readADC_SingleEnded(2);
  supercap_adc_voltage = ((results[0] * multiplier)/1000)*supercap_calibration;
  battery_adc_voltage = ((results[1] * multiplier)/1000)*battery_calibration;
  accdect_adc_voltage = ((results[2] * multiplier)/1000)*accdect_calibration;
  //Serial.println(results[0]);
  //Serial.println(supercap_adc_voltage,4);
  supercap_voltage = supercap_adc_voltage / (R2/(R1+R2));
  battery_voltage = battery_adc_voltage / (RR2/(RR1+RR2));
  supercap_percentage = supercap_voltage / supercapmaxvoltage;
  accdect_voltage = accdect_adc_voltage / (RRR2/(RRR1+RRR2));
  if (accdect_voltage > carstartedref_voltage){
    Car_started = true;
  }
  else{
    Car_started = false;
    #ifdef forcecarstarted
    Car_started = true;
    #endif
  }
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
  if(ChangedModecleardisplaycheck){
      ChangedModecleardisplaycheck = false;
      display.clearDisplay();
    }
  switch(modes){
    case 0:
      display.setTextSize(1);
      display.setTextColor(WHITE,BLACK);
      display.setCursor(40,0);
      display.print(&timeinfo,"%H:%M:%S");
      display.setCursor(0, 15);
      // Display static text
      display.println("SuperCap Voltage:");
      display.setTextSize(3);
      display.setCursor (12, 30);
      if(supercap_voltage < 10){
        display.print("0");
      }
      display.print(supercap_voltage, 2);
      display.println("V");
      display.setTextSize(1);
      display.print("SoC:");
      if(supercap_percentage*100 < 10){
        display.print("0");
      }
      display.print(supercap_percentage*100, 2);
      display.print("%");
      display.display(); 
      break;
    case 1:
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE,BLACK);
      display.setCursor(40,0);
      display.print(&timeinfo,"%H:%M:%S");
      display.setCursor(0, 15);
      // Display static text
      display.println("Battery Voltage:");
      display.setTextSize(3);
      display.setCursor (12, 30);
      if(battery_voltage < 10){
        display.print("0");
      }
      display.print(battery_voltage, 2);
      display.println("V");
      display.display(); 
      break; 
    case 2:
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE,BLACK);
      display.setCursor(0, 0);
      // Display static text
      display.println("Temperature:");
      display.println("SuperCap:");
      display.setTextSize(2);
      display.print(tempC[0], 2);
      display.print((char)247);display.println("C");
      display.setTextSize(1);
      display.println("Lithium Battery:");
      display.setTextSize(2);
      display.print(tempC[1], 2);
      display.print((char)247);display.println("C");
      display.display(); 
      break;
    case 3:
      display.setCursor(0,0);
      display.setTextSize(1);
      display.setTextColor(WHITE,BLACK);
      display.print("Time:");
      display.setCursor(40,0);
      display.println(&timeinfo,"%H:%M:%S");
      display.setTextSize(4);
      display.fillRect(3,11,99,33,BLACK);
      char s[10];
      sprintf(s,"%.0f", gps.speed.kmph());
      drawCentreString(s,64,12);
      display.setCursor(100,30);
      display.setTextSize(1);
      display.println("km/h");
      display.setCursor(0,45);
      if(gps.satellites.value() > 9){
        display.print(gps.satellites.value());
      }
      else{
        display.print(gps.satellites.value());
        display.print(" ");
      }
      display.println("-Sats connected");
      if(gps.hdop.value() > 9){
        display.print(gps.hdop.value());
      }
      else{
        display.print(gps.hdop.value());
        display.print(" ");
      }
      display.println("-HDOP");
      display.display();
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
  strcat(message, ",");
  strcat(message, rtc.getTime().c_str());
  strcat(message, 0); //terminate correctly 
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

int GPSTime() {
    time_t t_of_day; 
    struct tm t;
    if (gps.date.isUpdated())
    {
    t.tm_year = gps.date.year()-1900;
    t.tm_mon = gps.date.month()-1;           // Month, 0 - jan
    t.tm_mday = gps.date.day();          // Day of the month
    t.tm_hour = gps.time.hour();
    t.tm_min =  gps.time.minute();
    t.tm_sec = gps.time.second();
    t_of_day = mktime(&t);
    return t_of_day;
    }
    else{
      return 0;
    }
}
void drawCentreString(const String &buf, int x, int y)
{
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(buf, x, y, &x1, &y1, &w, &h); //calc width of new string
    Serial.printf("x:%d, y:%d, w:%d, h:%d", x1, y1, w, h);
    Serial.print("\n");
    display.setCursor(x - w / 2, y);
    display.print(buf);
}