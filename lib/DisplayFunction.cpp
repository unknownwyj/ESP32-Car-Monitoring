#include <Arduino.h>
#include <Adafruit_SSD1327.h>
#include <Math.h>

namespace {
  const int OLED_RESET = 17;
  const int TEXT_SIZE_SMALL = 1;
  const int TEXT_SIZE_LARGE = 2;
  const int ONE_K = 1000;
  
  const int OLED_HEIGHT = 64;
  const int OLED_WIDTH = 128;
  const int YELLOW_SEGMENT_HEIGHT = 16;
  const int DISPLAY_FULL_BRIGHTNESS = 255;
  const int DISPLAY_DIM_BRIGHTNESS = 0;
  
  const int IR_LED_PIN_3 = 3;
  const int PHOTODIODE_PIN_2 = 2;
  const int INTERRUPT_ZERO_ON_PIN_2 = 0;
  
  const uint16_t DIAL_CENTER_X = OLED_WIDTH / 2;
  const uint16_t DIAL_RADIUS = (OLED_HEIGHT - YELLOW_SEGMENT_HEIGHT) - 1;
  const uint16_t DIAL_CENTER_Y = OLED_HEIGHT - 1;
  const uint16_t INDICATOR_LENGTH = DIAL_RADIUS - 5;
  const uint16_t INDICATOR_WIDTH = 5;
  const uint16_t LABEL_RADIUS = DIAL_RADIUS - 18;
  const int DIAL_LABEL_Y_OFFSET = 6;
  const int DIAL_LABEL_X_OFFSET = 4;
  
  const long MAJOR_TICKS[] = { 0, 10000, 20000, 30000 };
  const int MAJOR_TICK_COUNT = sizeof(MAJOR_TICKS) / sizeof(MAJOR_TICKS[0]);
  const int  MAJOR_TICK_LENGTH = 7;
  const long MINOR_TICKS[] = {5000, 15000, 25000};
  const int MINOR_TICK_COUNT = sizeof(MINOR_TICKS) / sizeof(MINOR_TICKS[0]);
  const int MINOR_TICK_LENGTH = 3;
  
  const uint16_t DIAL_MAX_RPM = MAJOR_TICKS[MAJOR_TICK_COUNT-1];
  
  const int HALF_CIRCLE_DEGREES = 180;
  const float PI_RADIANS = PI/HALF_CIRCLE_DEGREES;
  
  const double MILLIS_PER_SECOND = 1000.0;
  const double SECONDS_PER_MINUTE = 60.0;
  const long DISPLAY_TIMEOUT_INTERVAL = 120 * MILLIS_PER_SECOND;
  const long DISPLAY_DIM_INTERVAL = DISPLAY_TIMEOUT_INTERVAL/2;
  const long DISPLAY_UPDATE_INTERVAL = 250;
  const int  DISPLAY_AVERAGE_INTERVALS = 4;
  
  volatile unsigned long revolutions;
  
  unsigned long previous_revolutions = 0;
  unsigned long revolution_count[DISPLAY_AVERAGE_INTERVALS]; 
  unsigned long interval_millis[DISPLAY_AVERAGE_INTERVALS]; 
  unsigned int interval_index = 0;
  unsigned long previous_millis = 0;
  unsigned long last_sensor_time = 0;
  bool is_oled_display_on = false;
  bool is_oled_display_dim = false;
}
void draw_dial(Adafruit_SSD1327& display, uint16_t rpm, bool is_dim){
    display.fillScreen(SSD1327_BLACK);
    display.drawCircle(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS, SSD1327_WHITE);
    display.drawCircle(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS - 1, SSD1327_WHITE);
    display.drawCircle(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS - 2, SSD1327_WHITE);
    display.drawCircle(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS - 3, SSD1327_WHITE);
    display.drawCircle(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS - 4, SSD1327_WHITE);
    display.drawCircle(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS - 5, SSD1327_WHITE);
    display.drawCircle(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS - 6, SSD1327_WHITE);
    display.drawCircle(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS - 7, SSD1327_WHITE);
    display.drawCircle(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS - 8, SSD1327_WHITE);
    display.drawCircle(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS - 9, SSD1327_WHITE);
    display.drawCircle(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS - 10, SSD1327_WHITE);
    display.drawCircle(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS - 11, SSD1327_WHITE);
    display.drawCircle(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS - 12, SSD1327_WHITE);
}
#include "Arduino.h"
#include <Adafruit_SSD1327.h>
#include <Math.h>
#include <Adafruit_GFX.h>

#define OLED_MOSI   23
#define OLED_CLK    18
#define OLED_DC     16
#define OLED_CS     5
#define OLED_RESET  17
//Adafruit_SSD1327 display(128, 128, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// software SPI
//Adafruit_SSD1327 display(128, 128, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
// hardware SPI
Adafruit_SSD1327 display(128, 128, &SPI, OLED_DC, OLED_RESET, OLED_CS);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
int rpm;
int speedkmh;
#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };
// 'x1000tacho', 39x6px
const unsigned char epd_bitmap_x1000tacho [] PROGMEM = {
	0x02, 0x63, 0x18, 0x67, 0x22, 0x8a, 0x94, 0xa4, 0x94, 0xb6, 0x52, 0x94, 0xa4, 0x94, 0xaa, 0x22, 
	0x94, 0xa4, 0xe7, 0x22, 0x52, 0x94, 0xa4, 0xa4, 0x22, 0x8a, 0x63, 0x18, 0x94, 0x22
};
// 'engineoil', 41x17px
const unsigned char epd_bitmap_engineoil [] PROGMEM = {
	0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x7e, 0x00, 0x00, 0x10, 0x00, 0x18, 0x18, 0x00, 0x00, 
	0xf8, 0x00, 0x30, 0x18, 0x00, 0x03, 0xfe, 0x00, 0x60, 0x1c, 0x00, 0x0f, 0xff, 0x00, 0xc7, 0xff, 
	0xf8, 0x3f, 0xc7, 0x80, 0x87, 0xff, 0xfc, 0xff, 0x83, 0x00, 0x07, 0x00, 0x1f, 0xff, 0x00, 0x00, 
	0x06, 0x00, 0x0f, 0xcf, 0x00, 0x00, 0x06, 0x00, 0x07, 0x0e, 0x00, 0x00, 0x06, 0x00, 0x00, 0x1c, 
	0x03, 0x00, 0x06, 0x00, 0x00, 0x1c, 0x03, 0x80, 0x06, 0x00, 0x00, 0x38, 0x03, 0x80, 0x06, 0x00, 
	0x00, 0x78, 0x03, 0x80, 0x07, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x07, 0xff, 0xff, 0xe0, 0x00, 0x00, 
	0x07, 0xff, 0xff, 0xe0, 0x00, 0x00
};
// 'engine-oil-temperature-warning-light', 27x17px
const unsigned char epd_bitmap_engine_oil_temperature_warning_light [] PROGMEM = {
	0x00, 0x70, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 
	0x00, 0x7c, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x60, 0x7e, 0x00, 0x00, 
	0x78, 0x7f, 0x01, 0xc0, 0xfe, 0x7e, 0x0f, 0xe0, 0xff, 0x7e, 0x7f, 0xe0, 0xff, 0xff, 0xfe, 0x20, 
	0x3e, 0x7b, 0xfc, 0x60, 0x0e, 0x70, 0x38, 0x60, 0x06, 0x00, 0x70, 0x60, 0x07, 0xff, 0xe0, 0x00, 
	0x07, 0xff, 0xc0, 0x00
};
// 'watertempbmp', 21x20px
const unsigned char watertempbmp [] PROGMEM = {
	0x00, 0x70, 0x00, 0x00, 0x70, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x70, 0x00, 0x00, 
	0x70, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x70, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x7f, 
	0x00, 0x00, 0x70, 0x00, 0x70, 0x70, 0x70, 0xfe, 0xfb, 0xf8, 0x8e, 0xfb, 0xc8, 0x00, 0x70, 0x00, 
	0x21, 0x8c, 0x20, 0x7f, 0xff, 0xf0, 0x1f, 0xfb, 0xc0, 0xff, 0xff, 0xf8
};
namespace {
  const int TEXT_SIZE_SMALL = 1;
  const int TEXT_SIZE_LARGE = 2;
  const int ONE_K = 1000;
  
  const int OLED_HEIGHT = 128;
  const int OLED_WIDTH = 128;
  const int YELLOW_SEGMENT_HEIGHT = 16;
  const int DISPLAY_FULL_BRIGHTNESS = 255;
  const int DISPLAY_DIM_BRIGHTNESS = 0;

  
  const uint16_t DIAL_CENTER_X = OLED_WIDTH / 2;
  const uint16_t DIAL_RADIUS = OLED_WIDTH / 2 - 2;
  const uint16_t DIAL_CENTER_Y = OLED_HEIGHT - 20;
  const uint16_t INDICATOR_LENGTH = DIAL_RADIUS - 5;
  const uint16_t INDICATOR_WIDTH = 5;
  const uint16_t LABEL_RADIUS = DIAL_RADIUS - 18;
  const int DIAL_LABEL_Y_OFFSET = 6;
  const int DIAL_LABEL_X_OFFSET = 4;
  
  const long MAJOR_TICKS[] = { 0, 10000, 20000, 30000 };
  const int MAJOR_TICK_COUNT = sizeof(MAJOR_TICKS) / sizeof(MAJOR_TICKS[0]);
  const int  MAJOR_TICK_LENGTH = 7;
  const long MINOR_TICKS[] = {5000, 15000, 25000};
  const int MINOR_TICK_COUNT = sizeof(MINOR_TICKS) / sizeof(MINOR_TICKS[0]);
  const int MINOR_TICK_LENGTH = 3;
  
  const uint16_t DIAL_MAX_RPM = 8000;
  
  const int HALF_CIRCLE_DEGREES = 180;
  const float PI_RADIANS = PI/HALF_CIRCLE_DEGREES;
  
  const double MILLIS_PER_SECOND = 1000.0;
  const double SECONDS_PER_MINUTE = 60.0;
  const long DISPLAY_TIMEOUT_INTERVAL = 120 * MILLIS_PER_SECOND;
  const long DISPLAY_DIM_INTERVAL = DISPLAY_TIMEOUT_INTERVAL/2;
  const long DISPLAY_UPDATE_INTERVAL = 250;
  const int  DISPLAY_AVERAGE_INTERVALS = 4;
  
  volatile unsigned long revolutions;
  
  unsigned long previous_revolutions = 0;
  unsigned long revolution_count[DISPLAY_AVERAGE_INTERVALS]; 
  unsigned long interval_millis[DISPLAY_AVERAGE_INTERVALS]; 
  unsigned int interval_index = 0;
  unsigned long previous_millis = 0;
  unsigned long last_sensor_time = 0;
  bool is_oled_display_on = false;
  bool is_oled_display_dim = false;
}
void drawHalfCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color,Adafruit_SSD1327& display) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  //90 degrees
  display.drawPixel(x0  , y0-r, color);
  //180 degrees
  display.drawPixel(x0+r, y0  , color);
  //0 degrees
  display.drawPixel(x0-r, y0  , color);


  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

	//90-135 degrees
    display.drawPixel(x0 + x, y0 - y, color);
	//45-90 degrees
    display.drawPixel(x0 - x, y0 - y, color);
	//135-180 degrees
    display.drawPixel(x0 + y, y0 - x, color);
	//0-45 degrees
    display.drawPixel(x0 - y, y0 - x, color);
  }
}
void drawNeedle(int16_t cx, int16_t cy, int16_t r,int16_t w,uint16_t color,Adafruit_SSD1327& display,int16_t rpm,int16_t maxrpm) {
  double anglerpm = abs(((double)rpm / maxrpm)-1);
  double angle = (double)anglerpm * (double)PI;
  double angle2 = (double)angle + (PI/2);
  double angle2n = (double)angle - (PI/2);
  int16_t ww = w*2;
  int16_t xx1 = cx + (ww * cos(angle2));
  int16_t yy1 = cy - (ww * sin(angle2));
  int16_t xx2 = cx + (ww * cos(angle2n));
  int16_t yy2 = cy - (ww * sin(angle2n));
  int16_t x = cx + (r * cos(angle));
  int16_t y = cy - (r * sin(angle));
  display.fillTriangle(xx1,yy1,x,y,xx2,yy2,color);
  display.drawLine(xx1,yy1,xx2,yy2,SSD1327_BLACK);
  display.fillCircle(cx,cy,ww,color);
}
void drawGauge(int16_t x0, int16_t y0, int16_t r, int16_t w, uint16_t color,Adafruit_SSD1327& display) {

	//Outside Border
	drawHalfCircle(x0, y0, r, color,display);
	//Inside Border
  drawHalfCircle(x0, y0, r-w, color,display);
	//Close off bottom
	display.drawFastHLine(x0 - r, y0, w, color);
	display.drawFastHLine(x0 + r - w, y0, w, color);
}
void fillgauge(int16_t cx, int16_t cy, int16_t r, int16_t w,int16_t lw,int16_t maxrpm, uint16_t color,Adafruit_SSD1327& display) {
  int maxrpmx1000 =maxrpm/1000;
  double special_angle = 180/(double)maxrpmx1000;
  double halfspecial_angle = special_angle/2;
  int rpmx1000 = 0;
  display.drawBitmap(cx-(39/2),cy-20,epd_bitmap_x1000tacho,39,6,SSD1327_WHITE);
  for(double i = 0; i <= 181; i+=special_angle) {
    double anglerpm = abs(((double)i / 180)-1);
    double angle = anglerpm * PI;
    int16_t x = cx + (r * cos(angle));
    int16_t y = cy - (r * sin(angle));
    int16_t x1 = cx + ((r-lw) * cos(angle));
    int16_t y1 = cy - ((r-lw) * sin(angle));
    int16_t xx,yy;
    uint16_t w,h;
    display.setTextSize(1);
    display.getTextBounds(String(rpmx1000),0,0,&xx,&yy,&w,&h);
    int16_t x2 = cx + ((r-lw-w) * cos(angle));
    int16_t y2 = cy - ((r-lw-h) * sin(angle));
    display.drawLine(x,y,x1,y1,color);
    display.setCursor(x2-(w/2),y2-(h/2));
    display.print((int)rpmx1000);
    rpmx1000++;
  }
  for(double i = 0; i <= 180; i+=halfspecial_angle) {
    double anglerpm = abs(((double)i / 180)-1);
    double angle = anglerpm * PI;
    int16_t x = cx + (r * cos(angle));
    int16_t y = cy - (r * sin(angle));
    int16_t x1 = cx + ((r-w) * cos(angle));
    int16_t y1 = cy - ((r-w) * sin(angle));
    display.drawLine(x,y,x1,y1,color);
  }
}
void setup()   {                
  Serial.begin(115200);
  //while (! Serial) delay(100);
  Serial.println("SSD1327 OLED test");
  
  if ( ! display.begin() ) {
     Serial.println("Unable to initialize OLED");
     while (1) yield();
  }
  display.clearDisplay();
  display.display();
}


void loop() {
  if(rpm <6800){
    rpm = rpm + 20;
  }
  else{
    rpm = random(2000,5000);
  }
  display.clearDisplay();
  display.drawBitmap(0,0,epd_bitmap_engine_oil_temperature_warning_light ,27,17,SSD1327_WHITE);
  display.setCursor(27,0);
  display.setTextSize(2);
  display.print(": ");
  display.print("85.3C");
  display.drawBitmap(0,18,watertempbmp ,21,20,SSD1327_WHITE);
  display.setCursor(27,22);
  display.setTextSize(2);
  display.print(": ");
  display.print("85.3C");
  drawGauge(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS, 5, SSD1327_WHITE,display);
  fillgauge(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS, 7,14, DIAL_MAX_RPM, SSD1327_WHITE,display);
  drawNeedle(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS, 2, SSD1327_WHITE,display,rpm,DIAL_MAX_RPM);
  display.setTextSize(2);
  speedkmh = (rpm/60)*1.609;
  int16_t xx,yy;
  uint16_t w,h;
  display.getTextBounds((String)speedkmh,0,0,&xx,&yy,&w,&h);
  display.setCursor(DIAL_CENTER_X-(w/2), DIAL_CENTER_Y+5);
  display.print(speedkmh);
  display.setCursor(90, DIAL_CENTER_Y+13);
  display.setTextSize(1);
  display.print("km/h");
  display.display();
}