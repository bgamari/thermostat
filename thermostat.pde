#include <LiquidCrystal.h>

enum temp_unit_t {
  KELVIN, CELCIUS, FAHRENHEIT
};

// Configuration:
unsigned int feedbackPeriod = 1000; // Milliseconds

double beta = 3470;
double T0 = 25 + 273; // Kelvin
double R0 = 10e3; // Ohms
double Vcc = 5; // Volts
double Rdiv = 10e3; // Divider resistor, Ohms

double tempUnit = CELCIUS;
double maxTemp = 273 + 40; // Kelvin
double minTemp = 273 + 10; // Kelvin

boolean override = false;
double maxOverrideTime = 5; // Minutes

boolean echo = false;

int thermistorPin = A0;
int heaterPin = 7;
int redPin = 11;
int greenPin = 10;
int encoderAPin = A2;
int encoderBPin = A1;
int backlightPin = 6;

LiquidCrystal lcd(12, 13, 5, 4, 3, 2);
// End of configuration

#include <EEPROM.h>

// EEPROMM address of setpoint
int configAddr = 0;

struct config_t {
  double hysteresis;
  double setpoint;
};
config_t config;
double temperature=0;
boolean heaterOn;

long overrideTime;
long lastFeedback; // Milliseconds

char cmd[256], *cmd_tail=cmd;


void saveConfig() {
  unsigned char *p = (unsigned char *)&config;
  for (int i=0; i<sizeof(config_t); i++)
    EEPROM.write(configAddr+i, p[i]);
}

void restoreConfig() {
  unsigned char *p = (unsigned char *)&config;
  for (int i=0; i<sizeof(config_t); i++)
    p[i] = EEPROM.read(configAddr+i);

  if (config.setpoint < minTemp
   || config.setpoint > maxTemp)
    config.setpoint = 293;
    
  if (config.hysteresis > 100 || config.hysteresis < 0)
    config.hysteresis = 5;
}

class Encoder {
  int pinA, pinB;
  int lastA;
public:
  int pos;
  
 public:
  Encoder(int pinA, int pinB) : pinA(pinA), pinB(pinB), lastA(0), pos(0) {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
  }
  
  virtual void stepped(int dir) {}
  
  void update() {
    int curA = digitalRead(pinA);
    if (!lastA && curA) {
      int curB = digitalRead(pinB);
      int step = curB ? -1 : +1;
      pos += step;
      stepped(step);
    }
    lastA = curA;
  }
};

class EncoderTest : public Encoder {
public:
  EncoderTest(int pinA, int pinB) : Encoder(pinA, pinB) {}
  void stepped(int dir) {
    Serial.println(pos);
  }
};

EncoderTest enc(encoderAPin, encoderBPin);

double tempFromCode(int code) {
  double Vth = Vcc / 1023 * code;
  double Rth = Vth / (Vcc-Vth) * Rdiv;
  double T = 1. / (1./T0 + 1./beta * log(Rth/R0));
  return T;
}

double tempToDisplay(double temp) {
  if (tempUnit == KELVIN)
    return temp;
  else if (tempUnit == CELCIUS)
    return temp - 273;
  else if (tempUnit == FAHRENHEIT)
    return 9/5*(temp - 273) + 32;
}

double tempDeltaToDisplay(double tempDelta) {
  if (tempUnit == KELVIN)
    return tempDelta;
  else if (tempUnit == CELCIUS)
    return tempDelta;
  else if (tempUnit == FAHRENHEIT)
    return 9/5*tempDelta;
}

void setColor(int red, int green) {
  analogWrite(greenPin, green);
  analogWrite(redPin, red);
}

void setBacklight(int brightness) {
  analogWrite(backlightPin, 255-brightness);
}

void setHeater(boolean on) {
  digitalWrite(heaterPin, on);
  heaterOn = on;
  if (on) setColor(255, 0);
  else setColor(0, 255);
}

void setup() {
  Serial.begin(9600);
  pinMode(thermistorPin, INPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  lastFeedback = millis();
  restoreConfig();
  setColor(128, 128);
  
  pinMode(backlightPin, OUTPUT);
  setBacklight(255);
  lcd.begin(16,2);
  lcd.print("Hello World!");
}

void handleInput() {
  int a = Serial.read();
  if (a != -1) {
    if (echo) Serial.write(a);
    *cmd_tail = a;
    cmd_tail++;
  }
  
  if (cmd_tail == &cmd[255]) {
    // Ignore commands over 255 characters long
    cmd_tail = cmd;
    return;
  }
    
  if (*(cmd_tail-1) != '\n')
    return;
   
  if (strncmp("set tar", cmd, 7) == 0) {
    char *c = cmd, *tmp;
    while (isalpha(*c) || isblank(*c)) c++;
    double newT = strtod(c, &tmp);
    if (tmp == c)
      Serial.println("!ERR: Invalid number");      
    else if (tmp != c && newT > minTemp && newT < maxTemp) {
      config.setpoint = newT;
      Serial.print("!OK ");
      Serial.print("@");
      Serial.println(config.setpoint);
    } else
      Serial.println("!ERR Out of range");
  }
  else if (strncmp("set hyst", cmd, 8) == 0) {
    char *c = cmd, *tmp;
    while (isalpha(*c) || isblank(*c)) c++;
    double hyst = strtod(c, &tmp);
    
    if (tmp == c)
      Serial.println("!ERR Invalid number");
    else {
      config.hysteresis = hyst;
      Serial.print("!OK ");
      Serial.println(config.hysteresis);
    }
  }
  else if (strncmp("set heater", cmd, 10) == 0) {
    char *c = cmd, *tmp = cmd_tail;
    while (isalpha(*c) || isblank(*c)) c++;
    long on = strtol(c, &tmp, 10);
    if (tmp == c || !(on == 0 || on == 1))
      Serial.println("!ERR Invalid boolean");
    else if (!override)
      Serial.println("!ERR Override not active");
    else {
      setHeater(on);
      Serial.println("!OK");
    }
  }
  else if (strncmp("hyst", cmd, 4) == 0) {
    Serial.print("!OK ");
    Serial.println(config.hysteresis); 
  }
  else if (strncmp("tar", cmd, 3) == 0) {
    Serial.print("!OK ");
    Serial.println(config.setpoint);
  }
  else if (strncmp("temp", cmd, 4) == 0) {
    Serial.print("!OK ");
    Serial.println(temperature);
  }
  else if (strncmp("heater", cmd, 6) == 0) {
    Serial.print("!OK ");
    Serial.println(heaterOn ? "on" : "off");
  }
  else if (strncmp("echo", cmd, 4) == 0) {
    Serial.print("!OK ");
    echo = 1;
  }
  else if (strncmp("override", cmd, 8) == 0) {
    Serial.println("!OK");
    override = 1;
    overrideTime = millis();
  }
  else if (strncmp("save", cmd, 4) == 0) {
    saveConfig();
    Serial.println("!OK");
  }
  else if (strncmp("no echo", cmd, 7) == 0) {
    Serial.print("!OK ");
    echo = 0;
  } 
  else Serial.println("!ERR Unknown command");
    
  cmd_tail = cmd;
}

void updateTemperature() {
  int code = analogRead(A0);
  temperature = tempFromCode(code);
}

void loop() {
  if (override && millis() - overrideTime > maxOverrideTime*60*1000)
    override = 0;

  updateTemperature();
  handleInput();
  
  if (!override && millis() - lastFeedback > feedbackPeriod)
  {
    doFeedback();
    lastFeedback = millis();
  }

  enc.update();

  updateLcd();
}

void updateLcd() {
  static int state = 0;
  static long lastUpdate = 0;
  static long nUpdates = 0;
  
  if (millis() - lastUpdate < 1000) return;
  lastUpdate = millis();
  nUpdates++;
  if (nUpdates % 20 == 0)
    state = (state+1) % 3;
  
  lcd.clear();
  lcd.print("Temp = ");
  lcd.print(tempToDisplay(temperature));
  
  lcd.setCursor(0,1);
  switch (state) {
  case 0:
    lcd.print("Target = ");
    lcd.print(tempToDisplay(config.setpoint));
    break;
  case 1:
    lcd.print("Hyst = ");
    lcd.print(tempDeltaToDisplay(config.hysteresis));
    break;
  case 2:
    lcd.print("Heater ");
    lcd.print(heaterOn ? "on" : "off");
    break;
  }
}

void doFeedback() {
  if (temperature > config.setpoint + config.hysteresis)
    setHeater(LOW);
  else if (temperature < config.setpoint - config.hysteresis)
    setHeater(HIGH);
}

