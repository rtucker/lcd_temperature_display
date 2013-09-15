#ifndef __LCD_TEMPERATURE_DISPLAY_H
#define __LCD_TEMPERATURE_DISPLAY_H

#define TRUE 1
#define FALSE 0

//#define DEBUG

// Define constants
#define AVERAGE_BINS_BITS 8   // 10 = 2^10 = 1024
#define AVERAGE_BINS (1 << AVERAGE_BINS_BITS)
#define SLEEP_TIME 30
#define DEBOUNCE_DELAY 50
#define NUMBER_COLORS 8
#define STEP_SHIFT 3
#define TEMPERATURE_MULTIPLIER float(5)/float(1024)/float(0.01)
#define TEMPERATURE_ADDEND 0

//                         0         1
//                         0123456789012345
#define STARTUP_MESSAGE_0 " FC Thermometer "
#define STARTUP_MESSAGE_1 "R Tucker Sep2013"
// "FC" stands for "Feature Creep"

// Define types
struct ColorValue
{
  int red_channel;
  int green_channel;
  int blue_channel;
};

struct TemperatureRecord
{
  float temperature;
  unsigned long time;
};

enum ColorName
{
    RED_COLOR,
    ORANGE_COLOR,
    YELLOW_COLOR,
    GREEN_COLOR,
    BLUE_COLOR,
    INDIGO_COLOR,
    VIOLET_COLOR,
    NO_COLOR
};



// Define LED pins
#define RED_PIN 11
#define GREEN_PIN 9
#define BLUE_PIN 10
#define LED_PIN 13

#define BUTTON_PIN 8

#define TEMPERATURE_PIN A0

#define LCD_RS_PIN 12
#define LCD_EN_PIN 7
#define LCD_D4_PIN 2
#define LCD_D5_PIN 3
#define LCD_D6_PIN 5
#define LCD_D7_PIN 4

#endif

