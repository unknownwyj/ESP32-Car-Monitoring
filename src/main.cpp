//#define useSSD1327
#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>
#include <ESP32Time.h>
#include <Adafruit_SSD1327.h>

#define time_offset 28800 // UTC+8
#define forcecarstarted
#define longpress_time 20000000 // 2s
#define shortpressdelay 200000 // 0.2s
time_t t_of_day; // time of day
//#define LOG_to_SD
#ifdef LOG_to_SD
#include "FS.h"
#include "SD_MMC.h"
File Sensorsfile;
#endif
#define DELAYTIMER 1 // 1 second delay
bool ChangedModecleardisplaycheck = true;
struct Button {
  const uint8_t PIN;
  bool pressed;
  bool longpressed;
};
Button button1 = {33,false,false};
//gps stuff
struct tm timeinfo;
static const uint32_t GPSBaud = 9600;
#define RXD2 25
#define TXD2 26
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
#define OLED_MOSI   23
#define OLED_CLK    18
#define OLED_DC     16
#define OLED_CS     5
#define OLED_RESET  17
Adafruit_SSD1327 display(128, 128, &SPI, OLED_DC, OLED_RESET, OLED_CS, 10000000UL);

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
bool SDcard_mounted = false;

int64_t last_time = 0;
int64_t current_time = 0;
int64_t Timerforbutton = 0;
int adc_value = 0;
void sensorRUN();
void displayOLED(int modes);
void wakeupreason();
void updateGPSTime();
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
  if(now - Timerforbutton > 500000) {
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
  btStop();
  
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  if (!display.begin()) {
     Serial.println("Unable to initialize OLED");
  }
  display.clearDisplay();
  display.display();
  ads.setGain(GAIN_ONE);
  if (!ads.begin(0x48)) {
    Serial.println("Failed to initialize ADS.");}
  //if failed will cause esp32 to stuck 
  #ifdef LOG_to_SD
  if (!SD_MMC.begin("/sd", true)) {
    Serial.println("Card Mount Failed");
  }
  else{
  Serial.println("Card Mounted");
  SDcard_mounted = true;
  if (!SD_MMC.exists("/SensorData.txt")){
    Serial.println("File doesn't exist creating new file");
    File dataFile = SD_MMC.open("/SensorData.txt", FILE_WRITE);
    dataFile.close();
  }
  }
  #endif
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
    updateGPSTime();
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
    if(button1.longpressed){
      Serial.println("Button Long Pressed");
      button1.longpressed = false;
      ChangedModecleardisplaycheck = true;
      //sdcard check 
      #ifdef LOG_to_SD
      if (!SDcard_mounted){
        if (!SD_MMC.begin("/sd", true)) {
        Serial.println("Card Mount Failed");
       return;
       } 
      else{
      Serial.println("Card Mounted");
      SDcard_mounted = true;
      }
      }
      else if (SDcard_mounted){
      SD_MMC.end();
      SDcard_mounted = false;
    }
    #endif
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
    if(SDcard_mounted){
      loggingtoSD();
      //maybe add a sdcard mount check ?
    }
    #endif
    esp_light_sleep_start();
  }
  else{
    if((esp_timer_get_time() - last_time)/500000 > DELAYTIMER){
      last_time = esp_timer_get_time();
      sensorRUN();
      displayOLED(modes);
      #ifdef LOG_to_SD
      if(SDcard_mounted){
        Serial.print("loggingtoSD");
        loggingtoSD();
        //maybe add a sdcard mount check ?
      }
      else{
        Serial.println("SDcard not mounted");
      }
      #endif
      Serial.print("Time Used:");
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
      display.setTextColor(SSD1327_WHITE, SSD1327_BLACK);
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
      display.println("%");
      display.setTextSize(1);
      display.setTextColor(SSD1327_WHITE, SSD1327_BLACK);
      display.setCursor(0, display.getCursorY()+9);
      display.println("Battery Voltage:");
      display.setTextSize(3);
      display.setCursor (12, display.getCursorY()+5);
      if(battery_voltage < 10){
        display.print("0");
      }
      display.print(battery_voltage, 2);
      display.println("V");
      display.setTextSize(1);
      display.setCursor(0, display.getCursorY()+5);
      display.print("SD Status:");
      if(!SDcard_mounted){
        display.println("error");
      }
      else{
        display.println("mounted");
      }
      display.display(); 
      break;
    case 1:
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1327_WHITE, SSD1327_BLACK);
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
      display.setTextSize(1);
      if(!SDcard_mounted){
        display.print("error");
      }
      else{
        display.print("mounted");
      }
      display.display(); 
      break; 
    case 2:
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1327_WHITE, SSD1327_BLACK);
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
      display.setTextColor(SSD1327_WHITE, SSD1327_BLACK);
      display.print("Time:");
      display.setCursor(40,0);
      display.println(&timeinfo,"%H:%M:%S");
      display.setTextSize(5);
      display.fillRect(3,11,99,33,SSD1327_BLACK);
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
      break;
    case 4:
      display.setCursor(0,0);
      display.setTextSize(1);
      display.setTextColor(SSD1327_WHITE, SSD1327_BLACK);
      display.println("Settings:");
      display.print("SDcard Status:");
      if(!SDcard_mounted){
        display.println("Not Mounted");
      }
      else{
        display.println("Mounted");
      }
  }
}
#ifdef LOG_to_SD
void loggingtoSD(){
  char message[400]="";
  char buffer[5][40] = {"","","","",""};
  dtostrf (supercap_voltage, 4, 2, buffer[0]);
  dtostrf (battery_voltage, 4, 2, buffer[1]);
  dtostrf (tempC[0], 4, 2, buffer[2]);
  dtostrf (tempC[1], 4, 2, buffer[3]);
  //char currenttime[100];
  //sprintf(currenttime, "%d", esp_timer_get_time()/1000000);
  //strcat(message, currenttime);
  //strcat(message, rtc.getTime().c_str());
  strcat(message, buffer[0]);
  strcat(message, ",");
  strcat(message, buffer[1]);
  strcat(message, ",");
  strcat(message, buffer[2]);
  strcat(message, ",");
  strcat(message, buffer[3]);
  strcat(message, ",");
  strcat(message, rtc.getTime().c_str());
  if(Sensorsfile.available()){
    Sensorsfile.println(message);
  }
  else{
    Sensorsfile = SD_MMC.open("/SensorData.txt", FILE_APPEND);
    Sensorsfile.println(message);
    Sensorsfile.flush();
  }
  /*
  File file = SD_MMC.open("/SensorData.txt", FILE_APPEND);
  if(Sensorsfile.println(message)){
    Serial.println("Logged to SD");
  }
  else{
    Serial.println("Failed to log to SD");
  }
  file.close(); */
}
#endif

void updateGPSTime() {
    if (gps.date.isUpdated())
    {
    timeinfo.tm_year = gps.date.year()-1900;
    timeinfo.tm_mon = gps.date.month()-1;           // Month, 0 - jan
    timeinfo.tm_mday = gps.date.day();          // Day of the month
    timeinfo.tm_hour = gps.time.hour();
    timeinfo.tm_min =  gps.time.minute();
    timeinfo.tm_sec = gps.time.second();
    t_of_day = mktime(&timeinfo);
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