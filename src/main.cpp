#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_LcdDisplay.h"
#include "Adafruit_VEML7700.h"

#define SERIAL_DEBUG_BAUD 115200
#define LCD_ADDRESS 0x2c

DFRobot_Lcd_IIC lcd(&Wire, LCD_ADDRESS);

Adafruit_VEML7700 lightSensor = Adafruit_VEML7700();

uint8_t gaugeID;

float lux = 0.0f;

void lightSensorSetup()
{
  if (!lightSensor.begin())
  {
      Serial.println("Sensor not found");
      while(1);
  }
  Serial.println("Sensor found");

  Serial.print(F("Gain: "));
  switch (lightSensor.getGain())
  {
      case VEML7700_GAIN_1: Serial.println("1"); break;
      case VEML7700_GAIN_2: Serial.println("2"); break;
      case VEML7700_GAIN_1_4: Serial.println("1/4"); break;
      case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
  }

  Serial.print(F("Integration Time (ms): "));
  switch (lightSensor.getIntegrationTime())
  {
      case VEML7700_IT_25MS: Serial.println("25"); break;
      case VEML7700_IT_50MS: Serial.println("50"); break;
      case VEML7700_IT_100MS: Serial.println("100"); break;
      case VEML7700_IT_200MS: Serial.println("200"); break;
      case VEML7700_IT_400MS: Serial.println("400"); break;
      case VEML7700_IT_800MS: Serial.println("800"); break;
  }

  lightSensor.setLowThreshold(10000);
  lightSensor.setHighThreshold(20000);
  lightSensor.interruptEnable(true);

}

void UpdateGauge()
{
  lcd.setGaugeValue(gaugeID, lux);
}

void setup()
{
  Serial.begin(SERIAL_DEBUG_BAUD);

  lcd.begin();
  lcd.cleanScreen();
  delay(500);
  lcd.setBackgroundColor(WHITE);

  lightSensorSetup();

  gaugeID = lcd.creatGauge(30, 0, 230, 0, 200, BLACK, GREEN);
  delay(2000);
}

void loop()
{
  lux = lightSensor.readLux();
  UpdateGauge();
  delay(1000);
}