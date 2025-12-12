#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <GyverButton.h>

const byte OC1A_PIN = 9;
const byte SENSOR_PIN = 11;
const byte BTN_PIN = 2;
const word INTERVAL_UPDATES = 1000;
const byte MIN_TEMP_START = 30;
const byte MAX_TEMP_START = 50;
const byte DUTY_HYST = 3;
const word PWM_FREQ_HZ = 25000;
const word TCNT1_TOP = 16000000 / (2 * PWM_FREQ_HZ);
unsigned long prevMillis = 0;
byte lastDuty = 0;
OneWire oneWire(SENSOR_PIN);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 20, 4);
GButton butt1(BTN_PIN);

bool isMenuShowing = false;
byte menuSelected = 0;
bool backlightOn = true;

byte arrowRight[8] = {
    B00000,
    B00100,
    B00010,
    B11111,
    B00010,
    B00100,
    B00000,
    B00000};

void setPwmDuty(byte duty);
byte mapTemperatureToDuty(float tempC);
byte applyHysteresis(byte newDuty);
bool isValidTemp(float t);
void setDisplay(float temp, byte speed);
void initOrErrorMsgDisplay(bool init);
void buttonClickHandler();

void setup()
{
  pinMode(OC1A_PIN, OUTPUT);
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;
  sensors.begin();
  lcd.init();
  if (backlightOn)
    lcd.backlight();
  lcd.createChar(0, arrowRight);
}

void loop()
{
  buttonClickHandler();
  unsigned long currentMillis = millis();
  if (currentMillis - prevMillis >= INTERVAL_UPDATES)
  {
    prevMillis = currentMillis;
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
    bool initMode = (currentMillis < 7000);
    if (!isValidTemp(tempC) || initMode)
    {
      initOrErrorMsgDisplay(initMode);
      setPwmDuty(100);
      return;
    }
    byte duty = mapTemperatureToDuty(tempC);
    byte adjustedDuty = applyHysteresis(duty);
    setPwmDuty(adjustedDuty);
    setDisplay(tempC, adjustedDuty);
  }
}

void setPwmDuty(byte duty)
{
  OCR1A = (word)(duty * TCNT1_TOP) / 100;
}

byte mapTemperatureToDuty(float temp)
{
  if (temp <= MIN_TEMP_START)
    return 0;
  if (temp >= MAX_TEMP_START)
    return 100;
  return map((int)(temp * 10), (MIN_TEMP_START * 10), (MAX_TEMP_START * 10), 0, 100);
}

byte applyHysteresis(byte duty)
{
  if (abs((int)duty - (int)lastDuty) < DUTY_HYST)
    return lastDuty;

  lastDuty = duty;
  return duty;
}

bool isValidTemp(float t)
{
  if (t == DEVICE_DISCONNECTED_C)
    return false;
  if (isnan(t))
    return false;
  return true;
}

void setDisplay(float temp, byte speed = 0)
{
  lcd.clear();
  if (isMenuShowing)
  {
    if (menuSelected == 0)
    {
      lcd.setCursor(0, 0);
      lcd.write(0);
    }

    lcd.setCursor(1, 0);
    lcd.print("Start Temp: ");
    lcd.print(MIN_TEMP_START);
    lcd.write((uint8_t)223);
    lcd.print("C");

    if (menuSelected == 1)
    {
      lcd.setCursor(0, 1);
      lcd.write(0);
    }
    lcd.setCursor(1, 1);
    lcd.print("End Temp: ");
    lcd.print(MAX_TEMP_START);
    lcd.write((uint8_t)223);
    lcd.print("C");

    if (menuSelected == 2)
    {
      lcd.setCursor(0, 2);
      lcd.write(0);
    }
    lcd.setCursor(1, 2);
    lcd.print("Backlight on: ");
    lcd.print(backlightOn ? "Yes" : "No");
  }
  else
  {

    lcd.setCursor(0, 0);
    lcd.print("Temperature: ");
    lcd.print(round(temp * 10.0) / 10.0, 1);
    lcd.write((uint8_t)223);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Fans speed: ");
    lcd.setCursor(13, 1);
    lcd.print(speed);
    lcd.print("%");
  }
}

void initOrErrorMsgDisplay(bool init)
{
  lcd.clear();
  if (init)
  {
    lcd.setCursor(3, 1);
    lcd.print("Initializing...");
  }
  else
  {
    lcd.setCursor(4, 1);
    lcd.print("Sensor Error!");
  }
}

void buttonClickHandler()
{
  butt1.tick();
  if (butt1.isSingle()) {
    isMenuShowing = !isMenuShowing;
  }
}