#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <GyverButton.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <math.h>
#define MAGIC_SUM 0xA56A
#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))

struct Settings
{
  uint16_t magic;
  bool isPIDmode;
  double ctrTemp;
  byte startTemp;
  byte endTemp;
  byte dutyHyst;
  double Kp;
  double Ki;
  double Kd;
};

enum ValueType
{
  TYPE_BOOL,
  TYPE_BYTE,
  TYPE_FLOAT
};

enum DisplaySignType
{
  TYPE_NONE,
  TYPE_DEGREE,
  TYPE_PERCENT,
};

enum DisplayValueType
{
  VAL_INT,
  VAL_FLOAT
};

struct MenuItem
{
  const char *label;
  void *value;
  ValueType type;
  DisplaySignType signType;
  DisplayValueType valueType;
};

enum ScreenMode
{
  SCREEN_MAIN,
  SCREEN_MENU,
  SCREEN_INIT
};

const byte OC1A_PIN = 9; //pin for fan control
const byte SENSOR_PIN = 11; // pin for temp sensor
const byte BTN_PIN = 2; // pin for button
const int INTERVAL_UPDATES = 1000;
const int INIT_START_TIME = 7000;
bool IS_PID_MODE = false;
double CTR_PID_TEMP = 50;
double K_P = 2;
double K_I = 5;
double K_D = 1;
byte MIN_TEMP_START = 30;
byte MAX_TEMP_START = 50;
const byte MIN_CTR_TEMP = 20;
const byte MAX_CTR_TEMP = 80;
byte DUTY_HYST = 3;
const word PWM_FREQ_HZ = 25000;                      // 25 kHz of fan PWM frequency
const word TCNT1_TOP = 16000000 / (2 * PWM_FREQ_HZ); // CPU clock
const byte LCD_ROWS = 4;
const byte LCD_COLS = 20;
unsigned long prevMillis = 0;
OneWire oneWire(SENSOR_PIN);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS);
GButton btn(BTN_PIN);

ScreenMode lastScreen = SCREEN_INIT;
bool isMenuShowing = false;
byte rowSelected = 0;
byte idxFirstRowMenuItem = 0;
float tempC = 0.0;
byte adjustedDuty = 0;
Settings cfg;

const MenuItem menuPIDon[] = {
    {"PID enabled: ", &IS_PID_MODE, TYPE_BOOL, TYPE_NONE},
    {"PID Temp: ", &CTR_PID_TEMP, TYPE_FLOAT, TYPE_DEGREE, VAL_INT},
    {"Kp: ", &K_P, TYPE_FLOAT, TYPE_NONE, VAL_FLOAT},
    {"Ki: ", &K_I, TYPE_FLOAT, TYPE_NONE, VAL_FLOAT},
    {"Kd: ", &K_D, TYPE_FLOAT, TYPE_NONE, VAL_FLOAT}};

const MenuItem menuPIDoff[] = {
    {"PID enabled: ", &IS_PID_MODE, TYPE_BOOL, TYPE_NONE},
    {"Start Temp: ", &MIN_TEMP_START, TYPE_BYTE, TYPE_DEGREE},
    {"End Temp: ", &MAX_TEMP_START, TYPE_BYTE, TYPE_DEGREE},
    {"Hysteresis: ", &DUTY_HYST, TYPE_BYTE, TYPE_PERCENT}};

const MenuItem mainView[] = {
    {"Temperature: ", &tempC, TYPE_FLOAT, TYPE_DEGREE, VAL_FLOAT},
    {"Fans speed: ", &adjustedDuty, TYPE_BYTE, TYPE_PERCENT}};

double inputPID, outputPID;
PID fanPID(&inputPID, &outputPID, &CTR_PID_TEMP, K_P, K_I, K_D, REVERSE);

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

byte vertLine[8] = {
    B00100,
    B00100,
    B00100,
    B00100,
    B00100,
    B00100,
    B00100,
    B00100};

byte vertBar[8] = {
    B00100,
    B00100,
    B01110,
    B01110,
    B01110,
    B01110,
    B00100,
    B00100};

void setPwmDuty(byte duty);
byte mapTemperatureToDuty();
byte applyHysteresis(byte newDuty);
bool isValidTemp();
void updateDisplay(bool forceUpdate = false);
void initOrErrorMsgDisplay(bool init);
void buttonClickHandler();
byte digitsAmount(float data);
bool hasTempChanges(float data);
bool hasDutyChanges(float data);
bool hasProgBarChanges(int data);
void clearRow(byte row);
void printValue(MenuItem menu);
void stepMenuValue(const MenuItem &item);
void getSettingsFromEEPROM();
void saveSettingsToEEPROM();

void setup()
{
  getSettingsFromEEPROM();
  pinMode(OC1A_PIN, OUTPUT);
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;
  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.setResolution(11);
  fanPID.SetOutputLimits(0, 100);
  fanPID.SetSampleTime(INTERVAL_UPDATES);
  fanPID.SetTunings(K_P, K_I, K_D);
  fanPID.SetMode(AUTOMATIC); /// check
  lcd.init();
  lcd.backlight();
  const byte *const symbols[] = {arrowRight, arrowUp, arrowDown, vertLine, vertBar};
  for (byte i = 0; i < ARRAY_LEN(symbols); i++)
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

void updateDisplay(bool forceUpdate)
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
    const byte allowShowRows = LCD_ROWS - 1;
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
      if (i > 2)
        break; // Temporary crutch to show only 3 menu items
      const byte idx = idxFirstRowMenuItem + i;
      lcd.setCursor(1, i);
      printValue(menu[idx]);
      if (rowSelected == i)
      {
        lcd.setCursor(0, i);
        lcd.write(0); // arrow
      }
      if (menuSize >= allowShowRows && i < allowShowRows)
      {
        lcd.setCursor(19, i);
        if (i == 0)
          idxFirstRowMenuItem == 0 ? lcd.write(4) : lcd.write(3); // vertLine = 3, vertBar = 4
        else if (i == 1)
          idxFirstRowMenuItem > 0 && (idxFirstRowMenuItem + allowShowRows) < menuSize ? lcd.write(4) : lcd.write(3);
        else if (i == 2)
          (idxFirstRowMenuItem + allowShowRows) < menuSize ? lcd.write(3) : lcd.write(4);
      }
    }
    lcd.setCursor(0, 3);
    lcd.print("Double click to exit");
  }
  else
  {
    const byte mainShowRowsSize = ARRAY_LEN(mainView);
    for (byte i = 0; i < mainShowRowsSize; i++)
    {
      if (i == 0)
      {
        float v = *(float *)mainView[i].value;
        if (hasTempChanges(v))
          clearRow(i);
        lcd.setCursor(0, i);
        printValue(mainView[i]);
        if (previousTemp != tempC)
          previousTemp < tempC ? lcd.write(1) : lcd.write(2);
        else
          lcd.print(" ");
        previousTemp = tempC;
      }
      else if (i == 1)
      {
        byte v = *(byte *)mainView[i].value;
        if (hasDutyChanges(v))
          clearRow(i);
        lcd.setCursor(0, i);
        printValue(mainView[i]);
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
      saveSettingsToEEPROM();
      if (IS_PID_MODE)
        fanPID.SetTunings(K_P, K_I, K_D);
    }
    isMenuShowing = !isMenuShowing;
    updateDisplay(true);
  }
  if (btn.isSingle() && isMenuShowing)
  {
    const byte menuItems = IS_PID_MODE ? ARRAY_LEN(menuPIDon) : ARRAY_LEN(menuPIDoff);
    const byte allowToShowRows = LCD_ROWS - 1;
    const byte divider = menuItems <= allowToShowRows ? menuItems : allowToShowRows;

    if (rowSelected == (allowToShowRows - 1) && menuItems > allowToShowRows)
    {
      if ((idxFirstRowMenuItem + allowToShowRows) < menuItems)
        idxFirstRowMenuItem = idxFirstRowMenuItem + 1;
      else
      {
        idxFirstRowMenuItem = 0;
        rowSelected = 0;
      }
    }
    else
      rowSelected = (rowSelected + 1) % divider;

    updateDisplay(true);
  }
  if (btn.isStep() && isMenuShowing)
  {
    byte activeIndex = idxFirstRowMenuItem + rowSelected;

    if (IS_PID_MODE)
      stepMenuValue(menuPIDon[activeIndex]);
    else
      stepMenuValue(menuPIDoff[activeIndex]);

    updateDisplay(true);
  }
}

byte digitsAmount(float data)
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
  byte digits = digitsAmount(data);
  bool changed = (digits != lastDigits);
  lastDigits = digits;
  return changed;
}

bool hasDutyChanges(float data)
{
  static int lastDigits = 1;
  byte digits = digitsAmount(data);
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

void printValue(MenuItem menu)
{
  lcd.print(menu.label);
  switch (menu.type)
  {
  case TYPE_BOOL:
    lcd.print(*(bool *)menu.value ? "Yes" : "No");
    break;

  case TYPE_BYTE:
    lcd.print(*(byte *)menu.value);
    break;

  case TYPE_FLOAT:
    bool isFloat = menu.valueType == VAL_FLOAT;
    lcd.print(*(float *)menu.value, isFloat ? 1 : 0);
    break;
  }

  if (menu.signType == TYPE_DEGREE)
  {
    lcd.write((uint8_t)223);
    lcd.print("C");
  }
  else if (menu.signType == TYPE_PERCENT)
    lcd.print("%");
}

void stepMenuValue(const MenuItem &item)
{
  switch (item.type)
  {
  case TYPE_BOOL:
    *(bool *)item.value = !*(bool *)item.value;

    break;
  case TYPE_BYTE:
  {
    byte *v = (byte *)item.value;
    if (item.value == &MIN_TEMP_START)
      *v = (*v + 1 < MAX_TEMP_START) ? *v + 1 : MIN_CTR_TEMP;
    else if (item.value == &MAX_TEMP_START)
      *v = (*v + 1 > MAX_CTR_TEMP) ? MIN_TEMP_START + 1 : *v + 1;
    else if (item.value == &DUTY_HYST)
      *v = (*v % 10) + 1;
    break;
  }
  case TYPE_FLOAT:
  {
    float *v = (float *)item.value;

    if (item.value == &CTR_PID_TEMP)
      *v = (*v + 1 > MAX_CTR_TEMP) ? MIN_CTR_TEMP : *v + 1;
    if (item.value == &K_P || item.value == &K_I || item.value == &K_D)
    {
      *v += 0.1;
      if (*v > 10.0)
        *v = 0.1;
    }
    break;
  }
  }
}

void getSettingsFromEEPROM()
{
  EEPROM.get(0, cfg);
  if (cfg.magic == MAGIC_SUM)
  {
    IS_PID_MODE = (bool)cfg.isPIDmode;
    CTR_PID_TEMP = cfg.ctrTemp;
    MIN_TEMP_START = cfg.startTemp;
    MAX_TEMP_START = cfg.endTemp;
    DUTY_HYST = cfg.dutyHyst;
    K_P = cfg.Kp;
    K_I = cfg.Ki;
    K_D = cfg.Kd;
  }
}

void saveSettingsToEEPROM()
{
  cfg.magic = MAGIC_SUM;
  cfg.isPIDmode = IS_PID_MODE;
  cfg.ctrTemp = CTR_PID_TEMP;
  cfg.startTemp = MIN_TEMP_START;
  cfg.endTemp = MAX_TEMP_START;
  cfg.dutyHyst = DUTY_HYST;
  cfg.Kp = K_P;
  cfg.Ki = K_I;
  cfg.Kd = K_D;
  EEPROM.put(0, cfg);
}