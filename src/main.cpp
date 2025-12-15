#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <GyverButton.h>
#include <EEPROM.h>
#include <PID_v1.h>
#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))

struct Settings
{
  bool isPIDmode;
  byte ctrTemp;
  byte startTemp;
  byte endTemp;
  byte dutyHyst;
  byte lightOn;
};

enum ValueType
{
  TYPE_BOOL,
  TYPE_BYTE,
  TYPE_DOUBLE
};

struct MenuItem
{
  const char *label;
  void *value;
  ValueType type;
};

enum ScreenMode
{
  SCREEN_MAIN,
  SCREEN_MENU,
  SCREEN_INIT
};

const byte OC1A_PIN = 9;
const byte SENSOR_PIN = 11;
const byte BTN_PIN = 2;
const int INTERVAL_UPDATES = 1000;
const int INIT_START_TIME = 7000;
bool IS_PID_MODE = false;
double CTR_PID_TEMP = 50;
byte MIN_TEMP_START = 30;
byte MAX_TEMP_START = 50;
const byte MIN_CTR_TEMP = 20;
const byte MAX_CTR_TEMP = 80;
bool BACK_LIGHT_ON = true;
byte DUTY_HYST = 3;
const word PWM_FREQ_HZ = 25000;
const word TCNT1_TOP = 16000000 / (2 * PWM_FREQ_HZ);
const byte LCD_ROWS = 4;
const byte LCD_COLS = 20;
unsigned long prevMillis = 0;
OneWire oneWire(SENSOR_PIN);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS);
GButton btn(BTN_PIN);

ScreenMode lastScreen = SCREEN_INIT;
bool isMenuShowing = false;
byte menuSelected = 0;
float tempC = 0.0;
byte adjustedDuty = 0;
Settings cfg;

const MenuItem menuPIDon[] = {
    {"PID enabled: ", &IS_PID_MODE, TYPE_BOOL},
    {"PID Temp: ", &CTR_PID_TEMP, TYPE_DOUBLE}};

const MenuItem menuPIDoff[] = {
    {"PID enabled: ", &IS_PID_MODE, TYPE_BOOL},
    {"Start Temp: ", &MIN_TEMP_START, TYPE_BYTE},
    {"End Temp: ", &MAX_TEMP_START, TYPE_BYTE},
    {"Hysteresis: ", &DUTY_HYST, TYPE_BYTE}};

const MenuItem mainView[] = {
    {"Temperature: ", &tempC, TYPE_DOUBLE},
    {"Fans speed: ", &adjustedDuty, TYPE_BYTE}};

double inputPID, outputPID;
double Kp = 2, Ki = 5, Kd = 1;
PID fanPID(&inputPID, &outputPID, &CTR_PID_TEMP, Kp, Ki, Kd, DIRECT);

byte arrowRight[8] = {
    B00000,
    B00100,
    B00010,
    B11111,
    B00010,
    B00100,
    B00000,
    B00000};

byte arrowUp[8] = {
    B00100,
    B01110,
    B10101,
    B00100,
    B00100,
    B00100,
    B00100,
    B00000};

byte arrowDown[8] = {
    B00100,
    B00100,
    B00100,
    B00100,
    B10101,
    B01110,
    B00100,
    B00000};

void setPwmDuty(byte duty);
byte mapTemperatureToDuty();
byte applyHysteresis(byte newDuty);
bool isValidTemp();
void updateDisplay(bool forceUpdate = false);
void initOrErrorMsgDisplay(bool init);
void buttonClickHandler();
byte digitAmount(float data);
bool hasTempChanges(float data);
bool hasDutyChanges(float data);
bool hasProgBarChanges(int data);
void clearRow(byte row);
void printDegreeC();
void printMenuValue(MenuItem menu);

void setup()
{
  EEPROM.get(0, cfg);
  IS_PID_MODE = cfg.isPIDmode != 255 ? (bool)cfg.isPIDmode : IS_PID_MODE;
  CTR_PID_TEMP = cfg.ctrTemp != 255 ? cfg.ctrTemp : CTR_PID_TEMP;
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
  fanPID.SetOutputLimits(0, 100);
  fanPID.SetSampleTime(INTERVAL_UPDATES);
  fanPID.SetMode(AUTOMATIC); /// check
  lcd.init();
  if (BACK_LIGHT_ON)
    lcd.backlight();
  const byte *const symbols[] = {arrowRight, arrowUp, arrowDown};
  for (byte i = 0; i < 3; i++)
  {
    lcd.createChar(i, (byte *)symbols[i]);
  }
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
    inputPID = tempC;
    bool initMode = (currentMillis < INIT_START_TIME);
    if (!isValidTemp() || initMode)
    {
      initOrErrorMsgDisplay(initMode);
      setPwmDuty(100);
      return;
    }
    if (IS_PID_MODE)
    {
      fanPID.Compute();
      adjustedDuty = (byte)outputPID;
    }
    else
    {
      byte duty = mapTemperatureToDuty();
      adjustedDuty = applyHysteresis(duty);
    }

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
  static byte lastDuty = 0;
  if (duty == 100 || duty == 0)
    return duty;
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

void updateDisplay(bool forceUpdate = false)
{
  ScreenMode currentScreen = isMenuShowing ? SCREEN_MENU : SCREEN_MAIN;
  static float previousTemp = 0;
  if (forceUpdate || lastScreen != currentScreen)
  {
    lcd.clear();
    lastScreen = currentScreen;
  }

  if (isMenuShowing)
  {
    const MenuItem *menu;
    byte menuSize;

    if (IS_PID_MODE)
    {
      menu = menuPIDon;
      menuSize = ARRAY_LEN(menuPIDon);
    }
    else
    {
      menu = menuPIDoff;
      menuSize = ARRAY_LEN(menuPIDoff);
    }
    for (byte i = 0; i < menuSize; i++)
    {
      lcd.setCursor(1, i);
      lcd.print(menu[i].label);
      printMenuValue(menu[i]);
      if (i > 0)
        i == 3 ? (void)lcd.print("%") : printDegreeC();
      if (menuSelected == i)
      {
        lcd.setCursor(0, i);
        lcd.write(0); // arrow
      }
    }
    // lcd.setCursor(0, 3);
    // lcd.print("Double click to exit");
  }
  else
  {
    for (byte i = 0; i < 2; i++)
    {

      if (i == 0)
      {
        double val = *(double *)mainView[i].value;
        if (hasTempChanges(val))
          clearRow(i);
        lcd.setCursor(0, i);
        lcd.print(mainView[i].label);
        lcd.print(round(val * 10.0) / 10.0, 1);
        printDegreeC();
        if (previousTemp != tempC)
          previousTemp < tempC ? lcd.write(1) : lcd.write(2);
        else
          lcd.print(" ");
        previousTemp = tempC;
      }
      else
      {
        byte val = *(byte *)mainView[i].value;
        if (hasDutyChanges(val))
          clearRow(i);
        lcd.setCursor(0, i);
        lcd.print(mainView[i].label);
        lcd.print(val);
        lcd.print("%");
        lcd.setCursor(16, i);
        lcd.print(IS_PID_MODE ? "-PID" : "-LIN");
      }
    }

    ///////////////////////////////////////////
    if (hasProgBarChanges(adjustedDuty))
      clearRow(2);
    lcd.setCursor(0, 2);
    for (int i = 0; i < adjustedDuty / 5; i++)
      lcd.print("*");
    ///////////////////////////////////////////
    lcd.setCursor(0, 3);
    lcd.print("Double click to menu");
  }
}

void initOrErrorMsgDisplay(bool init)
{
  lastScreen = SCREEN_INIT;
  lcd.clear();
  byte idx = init ? 3 : 4;
  lcd.setCursor(idx, 1);
  lcd.print(init ? "Initializing..." : "Sensor Error!");
}

void buttonClickHandler()
{
  btn.tick();
  if (btn.isDouble())
  {
    if (isMenuShowing)
    {
      cfg.isPIDmode = IS_PID_MODE;
      cfg.ctrTemp = CTR_PID_TEMP;
      cfg.startTemp = MIN_TEMP_START;
      cfg.endTemp = MAX_TEMP_START;
      cfg.dutyHyst = DUTY_HYST;
      EEPROM.put(0, cfg);
    }
    isMenuShowing = !isMenuShowing;
    updateDisplay(true);
  }
  if (btn.isSingle() && isMenuShowing)
  {
    const byte menuSize = IS_PID_MODE ? ARRAY_LEN(menuPIDon) : ARRAY_LEN(menuPIDoff);
    menuSelected = (menuSelected + 1) % menuSize;
    updateDisplay(true);
  }
  if (btn.isStep() && isMenuShowing)
  {
    switch (menuSelected)
    {
    case 0:
      IS_PID_MODE = !IS_PID_MODE;
      break;
    case 1:
      if (IS_PID_MODE)
      {
        CTR_PID_TEMP = (CTR_PID_TEMP + 1) > MAX_CTR_TEMP ? MIN_CTR_TEMP : CTR_PID_TEMP + 1;
      }
      else
        MIN_TEMP_START = (MIN_TEMP_START + 1) < MAX_TEMP_START ? MIN_TEMP_START + 1 : MIN_CTR_TEMP;
      break;
    case 2:
      if (IS_PID_MODE)
      {
      }
      else
        MAX_TEMP_START = (MAX_TEMP_START + 1 > MAX_CTR_TEMP) ? MIN_TEMP_START + 1 : MAX_TEMP_START + 1;
      break;
    case 3:
      if (IS_PID_MODE)
      {
      }
      else
        DUTY_HYST = (DUTY_HYST % 10) + 1;
      break;
    }
    updateDisplay(true);
  }
}

byte digitAmount(float data)
{
  int n = abs(data);
  if (n < 10)
    return 1;
  if (n < 100)
    return 2;
  if (n < 1000)
    return 3;
  return 4;
}

bool hasTempChanges(float data)
{
  static int lastDigits = 1;
  byte digits = digitAmount(data);
  bool changed = (digits != lastDigits);
  lastDigits = digits;
  return changed;
}

bool hasDutyChanges(float data)
{
  static int lastDigits = 1;
  byte digits = digitAmount(data);
  bool changed = (digits != lastDigits);
  lastDigits = digits;
  return changed;
}

bool hasProgBarChanges(int data)
{
  static int lastData = 0;
  byte progBarData = data % 10;
  bool changed = (lastData != progBarData);
  lastData = progBarData;
  return changed;
}

void clearRow(byte row)
{
  lcd.setCursor(0, row);
  for (byte i = 0; i < LCD_COLS; i++)
    lcd.print(" ");
}

void printDegreeC()
{
  lcd.write((uint8_t)223);
  lcd.print("C");
}

void printMenuValue(MenuItem menu)
{
  switch (menu.type)
  {
  case TYPE_BOOL:
    lcd.print(*(bool *)menu.value ? "Yes" : "No");
    break;

  case TYPE_BYTE:
    lcd.print(*(byte *)menu.value);
    break;

  case TYPE_DOUBLE:
    lcd.print((int)(*(double *)menu.value));
    break;
  }
}