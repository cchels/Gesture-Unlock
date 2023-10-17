#include <Arduino.h>
#include <acc.h>
#define percentage_decrease_threshold 0.2
AccelerometerBuffer saved_accelorometer_data_array;
AccelerometerBuffer current_accelorometer_data_array;
bool left_button_pressed;

void setup()
{
  // put your setup code here, to run once:
  CircuitPlayground.begin();
  Serial.begin(9600);

  // Initialize saved and current accelerometer data array
  saved_accelorometer_data_array = AccelerometerBuffer();
  current_accelorometer_data_array = AccelerometerBuffer();

  // Initialize left button pressed to false
  left_button_pressed = false;
}

void left_button_pressed_callback()
{
  // Reset data arrays
  Serial.println("BUTTON PRESSED");
  saved_accelorometer_data_array.reset();
  current_accelorometer_data_array.reset();
  left_button_pressed = true;
  CircuitPlayground.redLED(true);
}

void left_button_released_callback()
{
  Serial.println("BUTTON RELEASED");
  current_accelorometer_data_array.set_max_size(saved_accelorometer_data_array.get_size());
  left_button_pressed = false;
  CircuitPlayground.redLED(false);
}

void loop()
{
  // Check if button pressed for FIRST TIME only then reset saved data array
  if (CircuitPlayground.leftButton() && !left_button_pressed)
  {
    left_button_pressed_callback();
  }

  if (left_button_pressed)
  {
    // Call sample
    saved_accelorometer_data_array.sample_data();
  }
  else
  {
    if (saved_accelorometer_data_array.get_size() == 0)
    {
      Serial.println("NO SAVED DATA");
    }
    else if (current_accelorometer_data_array.get_size() < saved_accelorometer_data_array.get_size())
    {
      Serial.println("Not enough data to compare");
      // Call sample
      current_accelorometer_data_array.sample_data();
    }
    else
    {
      // Call sample
      current_accelorometer_data_array.sample_data(saved_accelorometer_data_array);
      Serial.print("DTW: ");
      Serial.println(current_accelorometer_data_array.dtw_value);
      Serial.print("DTW Percentage Decrease: ");
      Serial.println(current_accelorometer_data_array.dtw_percentage_decrease);

      // Check if DTW value is -1
      if (current_accelorometer_data_array.dtw_value != -1 && current_accelorometer_data_array.dtw_percentage_decrease > percentage_decrease_threshold)
      {

        // karim NEOPIXEL LIGHT UP AND PLAYTONE
        for (unsigned short i = 0; i <= 9; i++)
        {
          CircuitPlayground.setPixelColor(i, 0, 255, 0);
        }
        CircuitPlayground.clearPixels();
        // Play tone
        // CircuitPlayground.playTone(440, 1200);
        Serial.println("MATCH");
        delay(1000);
      }
      else
      {
        // turn neopixels red
        for (unsigned short i = 0; i <= 9; i++)
        {
          CircuitPlayground.setPixelColor(i, 255, 0, 0);
        }
        CircuitPlayground.clearPixels();
      }
    }
  }

  // Check if button released to reset left_button_pressed
  if (!CircuitPlayground.leftButton() && left_button_pressed)
  {
    left_button_released_callback();
  }

  delay(LOOP_DELAY);
}
