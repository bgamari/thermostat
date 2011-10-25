// Configuration:
unsigned int feedbackPeriod = 1000; // Milliseconds

double beta = 3470;
double T0 = 25 + 273; // Kelvin
double R0 = 10e3; // Ohms
double Vcc = 5; // Volts
double Rdiv = 10e3; // Divider resistor, Ohms

double maxTemp = 273 + 40; // Kelvin
double minTemp = 273 + 10; // Kelvin

boolean echo = true;

int thermistorPin = A0;
int heaterPin = PD2;
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

unsigned long lastFeedback; // Milliseconds

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

void setup() {
  Serial.begin(9600);
  pinMode(thermistorPin, INPUT);
  pinMode(heaterPin, OUTPUT);
  lastFeedback = millis();
  restoreConfig();
}

double tempFromCode(int code) {
  double Vth = Vcc / 1023 * code;
  double Rth = Vth / (Vcc-Vth) * Rdiv;
  double T = 1. / (1./T0 + 1./beta * log(Rth/R0));
  return T;
}

void reportHeater() {
  Serial.print("OK ");
  Serial.println(heaterOn ? "on" : "off");
}

void handleInput() {
  int a = Serial.read();
  if (a != -1) {
    if (echo) Serial.write(a);
    *cmd_tail = a;
    cmd_tail++;
  }
  
  if (cmd_tail == &cmd[255]) {
    cmd_tail = cmd;
    return;
  }
    
  if (*(cmd_tail-1) != '\n')
    return;
   
  if (strncmp("set tar", cmd, 5) == 0) {
    char *c = cmd, *tmp;
    while (isalpha(*c) || isblank(*c)) c++;
    double newT = strtod(c, &tmp);
    if (tmp == c)
      Serial.println("ERR: Invalid number");      
    else if (tmp != c && newT > minTemp && newT < maxTemp) {
      config.setpoint = newT;
      Serial.print("OK ");
      Serial.println(config.setpoint);
    } else
      Serial.println("ERR: Out of range");
  }
  else if (strncmp("set hyst", cmd, 5) == 0) {
    char *c = cmd, *tmp;
    while (isalpha(*c) || isblank(*c)) c++;
    double hyst = strtod(c, &tmp);
    if (tmp == c)
      Serial.println("ERR: Invalid number");
    else {
      config.hysteresis = hyst;
      Serial.print("OK ");
      Serial.println(config.hysteresis);
    }
  }
  else if (strncmp("hyst", cmd, 4) == 0) {
    Serial.print("OK ");
    Serial.println(config.hysteresis); 
  }
  else if (strncmp("tar", cmd, 3) == 0) {
    Serial.print("OK ");
    Serial.println(config.setpoint);
  }
  else if (strncmp("temp", cmd, 4) == 0) {
    Serial.print("OK ");
    Serial.println(temperature);
  }
  else if (strncmp("heater", cmd, 6) == 0)
    reportHeater();
  else if (strncmp("echo", cmd, 4) == 0) {
    Serial.print("OK ");
    echo = 1;
  }
  else if (strncmp("save", cmd, 4) == 0) {
    saveConfig();
    Serial.println("OK");
  }
  else if (strncmp("no echo", cmd, 7) == 0) {
    Serial.print("OK ");
    echo = 0;
  } 
  else Serial.println("ERR Unknown command");
    
  cmd_tail = cmd;
}

void setHeater(boolean on) {
  digitalWrite(heaterPin, on);
  heaterOn = on;
}

void updateTemperature() {
  int code = analogRead(A0);
  temperature = tempFromCode(code);
}

void loop() {
  updateTemperature();
  handleInput();
  
  if (millis() > lastFeedback + feedbackPeriod
   || millis() < lastFeedback - 10000) // Handle wrap-around
  {
    doFeedback();
    lastFeedback = millis();
  }
}

void doFeedback() {
  if (temperature > config.setpoint + config.hysteresis)
    setHeater(LOW);
  else if (temperature < config.setpoint - config.hysteresis)
    setHeater(HIGH);
}

