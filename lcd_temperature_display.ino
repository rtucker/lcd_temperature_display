// vim: ts=2 et
/*
 *
 *  LCD Temperature Display
 *
 *  Assumes:
 *    LM35 temperature device (analog pin 0)
 *    16x2 display connected to WickedDevices LCD shield
 *    Pushbutton on digital pin 7 held high by pin 8
 *    Red/Green/Blue backlights on digital pins 6, 9, and 10 respectively
 *
 *  Ryan Tucker <rtucker@gmail.com>
 */

// include the libraries
#include <stdarg.h>
#include <LiquidCrystal.h>
#include "lcd_temperature_display.h"

// Define colors corresponding to the ColorNames
const byte BacklightColors[][3] = {
  {255, 0, 0}, {255, 127, 0}, {255, 255, 0}, {0, 255, 0},
  {0, 0, 255}, {111, 0, 255}, {191, 0, 255}, {0, 0, 0}
};

// Declare Global variables
byte buttonIsPressed = FALSE;
byte buttonState;
byte lastButtonState = LOW;
unsigned long lastDebounceTime = 0;

ColorValue current_color;
ColorValue new_color;
ColorName next_color;
int voltageBins[AVERAGE_BINS] = {0};
int currentVoltageBin = -1;
TemperatureRecord high_temp = {0.0, 0};
TemperatureRecord low_temp = {999.9, 0};

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

void processButton()
{
  // Debounce and read the state of the input pin, setting the
  // buttonIsPressed once when the button is pressed.
  byte reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) lastDebounceTime = millis();
  lastButtonState = reading;
  digitalWrite(LED_PIN, !reading);

  if ((reading != buttonState) && ((millis() - lastDebounceTime) > DEBOUNCE_DELAY))
  {
    buttonState = reading;
    if (buttonState == LOW) buttonIsPressed = TRUE;
  }
}

void getTemperature(float *averageval)
{
  // Reads the temperature from the LM35 device, and computes
  // a rolling average.
  int readval = analogRead(TEMPERATURE_PIN);
  byte readval_shifted;
  long bigsum = 0;

  if (currentVoltageBin < 0)
  {
    // special case: first time through.
    currentVoltageBin = 0;
    for (int i = 0; i < AVERAGE_BINS; i++)
    {
      voltageBins[i] = readval;
    }
    #ifdef DEBUG
      Serial.print("getTemperature: voltageBins initialized to ");
      Serial.println(readval);
    #endif
  } else {
    voltageBins[currentVoltageBin] = readval;
  }

  // increment the current bin, but wrap if we're at the end
  currentVoltageBin = (currentVoltageBin + 1) % AVERAGE_BINS;

  // sum all of the bins
  for (int i = 0; i < AVERAGE_BINS; i++)
  {
    bigsum += voltageBins[i];
  }

  // Compute the average, scaled from ADC value to temperature.
  *averageval  = float(bigsum) / float(AVERAGE_BINS);
  *averageval *= (float)TEMPERATURE_MULTIPLIER;
  *averageval += (float)TEMPERATURE_ADDEND;

  #ifdef DEBUG
    Serial.print("getTemperature: readval=");
    Serial.print(readval);
    Serial.print(" currentVoltageBin=");
    Serial.print(currentVoltageBin);
    Serial.print(" bigsum=");
    Serial.print(bigsum);
    Serial.print(" *averageval=");
    Serial.print(*averageval);
    Serial.println("");
  #endif
}

void getColorByName(ColorValue* result, ColorName color)
{
  // Converts a ColorName to a ColorValue
  result->red_channel   = BacklightColors[(int)color][0];
  result->green_channel = BacklightColors[(int)color][1];
  result->blue_channel  = BacklightColors[(int)color][2];
}

void setColor(ColorValue* color)
{
  // Sets a color on the pins by varying the PWM brightness
  analogWrite(RED_PIN,   255-(color->red_channel));
  analogWrite(GREEN_PIN, 255-(color->green_channel));
  analogWrite(BLUE_PIN,  255-(color->blue_channel));
}

byte determineStep(byte old_val, byte target)
{
  // Compute the next value so that we transition from
  // old_val to target in a smooth timeframe.
  byte new_val = old_val;
  byte delta   = abs(old_val - target);

  // just return the new value if it's less than 2
  // otherwise, bump it up/down by the step shift value
  if      (delta < 2)        new_val = target;
  else if (old_val < target) new_val += (delta >> STEP_SHIFT);
  else                       new_val -= (delta >> STEP_SHIFT);

  #ifdef DEBUG
    if (old_val != new_val)
    {
      Serial.print("determineStep: old_val=");
      Serial.print(old_val);
      Serial.print(" target=");
      Serial.print(target);
      Serial.print(" delta=");
      Serial.print(delta);
      Serial.print(" new_val=");
      Serial.print(new_val);
      Serial.println("");
    }
  #endif

  return new_val;
}

void fadeTowardsColor(ColorValue* active_color, ColorValue* target_color)
{
  // steps towards a color in small, even increments
  active_color->red_channel   = determineStep(active_color->red_channel,   target_color->red_channel);
  active_color->green_channel = determineStep(active_color->green_channel, target_color->green_channel);
  active_color->blue_channel  = determineStep(active_color->blue_channel,  target_color->blue_channel);
}

int minutesAgo(unsigned long msec)
{
  // Returns integer number of minutes since the given millis count passed
  return max(0, ((millis() - msec) / 1000 / 60));
}

void incrementColor(ColorName* the_color)
{
  // Increments the color by one step.
  *the_color = (ColorName) (((int)*the_color + 1) % NUMBER_COLORS);
}

void setup()
{
  // init LED pins
  pinMode(RED_PIN,         OUTPUT);
  pinMode(GREEN_PIN,       OUTPUT);
  pinMode(BLUE_PIN,        OUTPUT);
  pinMode(LED_PIN,         OUTPUT);
  pinMode(BUTTON_PIN,      INPUT_PULLUP);
  pinMode(TEMPERATURE_PIN, INPUT);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // serial for troubleshooting
  #ifdef DEBUG
    Serial.begin(115200);
  #endif
}

void normalizeTime(int* timevalue, char* timeunit)
{
  // Convert a count of minutes into a count of minutes or hours or days
  // It is worth noting that neither minutes nor hours will exceed 99,
  // and days is unlikely to surpass 40 days or so due to millis()
  // overflowing its unsigned long.
  if (*timevalue > 1439)
  {
    *timevalue /= 1400;
    *timeunit = 'd';
  }
  else if (*timevalue > 59)
  {
    *timevalue /= 60;
    *timeunit = 'h';
  }
  else
  {
    *timeunit = 'm';
  }
}

void loop()
{
  float current_temp;
  int timediff;
  char timespec;
  char outstr[17];

  // Get the current temperature.
  getTemperature(&current_temp);

  if (millis() > (3*1000))
  {
    // We are past the startup screen.

    // Check for button presses.
    processButton();

    // Increment color if the button was pressed
    if (buttonIsPressed)
    {
      incrementColor(&next_color);
      buttonIsPressed = FALSE;
    }

    // Check for records
    if (current_temp > high_temp.temperature)
    {
      high_temp.temperature = current_temp;
      high_temp.time = millis();
    }

    if (current_temp < low_temp.temperature)
    {
      low_temp.temperature = current_temp;
      low_temp.time = millis();
    }

    // Update the display once in awhile
    if (currentVoltageBin == 0)
    {
      // Print the records.
      // Start with the high
      timediff = minutesAgo(high_temp.time);
      normalizeTime(&timediff, &timespec);

      lcd.setCursor(0, 0);
      snprintf(outstr, 17, "H%3d.%1d@%2d%c %3d%1c%1d", (int)high_temp.temperature,
               (int)(high_temp.temperature * 10) % 10, timediff, timespec,
               (int)current_temp, '.',
               (int)(current_temp * 10) % 10
              );
      lcd.print(outstr);

      // Then do the low
      timediff = minutesAgo(low_temp.time);
      normalizeTime(&timediff, &timespec);
      timespec = 'm';

      lcd.setCursor(0, 1);
      snprintf(outstr, 17, "L%3d.%1d@%2d%c  deg%1c", (int)low_temp.temperature,
               (int)(low_temp.temperature * 10) % 10, timediff, timespec, 'C'
              );
      lcd.print(outstr);
    }
  } else {
    // Startup message
    lcd.setCursor(0, 0);
    lcd.print(STARTUP_MESSAGE_0);
    lcd.setCursor(0, 1);
    lcd.print(STARTUP_MESSAGE_1);

    // read, but ignore, button presses
    processButton();
    buttonIsPressed = FALSE;

    next_color = VIOLET_COLOR;

  }

  // Manage colors
  getColorByName(&new_color, next_color);
  fadeTowardsColor(&current_color, &new_color);
  setColor(&current_color);

  // Sleep for a bit
  delay(SLEEP_TIME);
}
