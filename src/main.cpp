#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <GyverButton.h>
#include <EEPROM.h>

struct Settings
{
  byte startTemp;
  byte endTemp;
  byte dutyHyst;
  byte lightOn;
};

const byte OC1A_PIN = 9;
const byte SENSOR_PIN = 11;
const byte BTN_PIN = 2;
const word INTERVAL_UPDATES = 1000;
byte MIN_TEMP_START = 30; // default
byte MAX_TEMP_START = 50; // default
const byte MIN_CTR_TEMP = 20;
const byte MAX_CTR_TEMP = 80;
bool BACK_LIGHT_ON = true; // default
byte DUTY_HYST = 3;        // default
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
float tempC = 0.0;
byte adjustedDuty = 0;
Settings cfg;

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
byte mapTemperatureToDuty();
byte applyHysteresis(byte newDuty);
bool isValidTemp();
void updateDisplay();
void initOrErrorMsgDisplay(bool init);
void buttonClickHandler();
bool checkUnsavedDate();

void setup()
{
  EEPROM.get(0, cfg);
  MIN_TEMP_START = cfg.startTemp != 255 ? cfg.startTemp : MIN_TEMP_START; // 255 default unprogrammed EEPROM value
  MAX_TEMP_START = cfg.endTemp != 255 ? cfg.endTemp : MAX_TEMP_START;
  DUTY_HYST = cfg.dutyHyst != 255 ? cfg.dutyHyst : DUTY_HYST;
  // BACK_LIGHT_ON = cfg.lightOn;
  pinMode(OC1A_PIN, OUTPUT);
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;
  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.setResolution(10);
  lcd.init();
  if (BACK_LIGHT_ON)
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
    tempC = sensors.getTempCByIndex(0);
    bool initMode = (currentMillis < 7000);
    if (!isValidTemp() || initMode)
    {
      initOrErrorMsgDisplay(initMode);
      setPwmDuty(100);
      return;
    }
    byte duty = mapTemperatureToDuty();
    adjustedDuty = applyHysteresis(duty);
    setPwmDuty(adjustedDuty);
    updateDisplay();
  }
}

void setPwmDuty(byte duty)
{
  OCR1A = (word)(duty * TCNT1_TOP) / 100;
}

byte mapTemperatureToDuty()
{
  if (tempC <= MIN_TEMP_START)
    return 0;
  if (tempC >= MAX_TEMP_START)
    return 100;
  return map((int)(tempC * 10), (MIN_TEMP_START * 10), (MAX_TEMP_START * 10), 0, 100);
}

byte applyHysteresis(byte duty)
{
  if (abs((int)duty - (int)lastDuty) < DUTY_HYST)
    return lastDuty;

  lastDuty = duty;
  return duty;
}

bool isValidTemp()
{
  if (tempC == DEVICE_DISCONNECTED_C)
    return false;
  if (isnan(tempC))
    return false;
  return true;
}

void updateDisplay()
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
    // lcd.print("Backlight on: ");
    // lcd.print(BACK_LIGHT_ON ? "Yes" : "No");
    lcd.print("Hysteresis: ");
    lcd.print(DUTY_HYST);
    lcd.write((uint8_t)223);
    lcd.print("C");
    lcd.setCursor(0, 3);
    lcd.print("Double click to exit");
  }
  else
  {
    lcd.setCursor(0, 0);
    lcd.print("Temperature: ");
    lcd.print(round(tempC * 10.0) / 10.0, 1);
    lcd.write((uint8_t)223);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Fans speed: ");
    lcd.setCursor(13, 1);
    lcd.print(adjustedDuty);
    lcd.print("%");
    lcd.setCursor(0, 2);
    for (int i = 0; i < adjustedDuty / 5; i++)
    {
      lcd.print("*");
    }
    lcd.setCursor(0, 3);
    lcd.print("Double click to menu");
  }
}

void initOrErrorMsgDisplay(bool init)
{
  lcd.clear();
  byte idx = init ? 3 : 4;
  lcd.setCursor(idx, 1);
  lcd.print(init ? "Initializing..." : "Sensor Error!");
}

void buttonClickHandler()
{
  butt1.tick();
  if (butt1.isDouble())
  {
    if (isMenuShowing)
    {
      cfg.startTemp = MIN_TEMP_START;
      cfg.endTemp = MAX_TEMP_START;
      cfg.dutyHyst = DUTY_HYST;
      EEPROM.put(0, cfg);
    }
    isMenuShowing = !isMenuShowing;
    updateDisplay();
  }
  if (butt1.isSingle() && isMenuShowing)
  {
    menuSelected = (menuSelected + 1) % 3;
    updateDisplay();
  }
  if (butt1.isStep() && isMenuShowing)
  {
    switch (menuSelected)
    {
    case 0:
      MIN_TEMP_START = (MIN_TEMP_START + 1) < MAX_TEMP_START ? MIN_TEMP_START + 1 : MIN_CTR_TEMP;
      break;
    case 1:
      MAX_TEMP_START = (MAX_TEMP_START + 1 > MAX_CTR_TEMP) ? MIN_TEMP_START + 1 : MAX_TEMP_START + 1;
      break;
    case 2:
      DUTY_HYST = (DUTY_HYST % 10) + 1;
      break;
    }
    updateDisplay();
  }
}