#include <Adafruit_ADS1X15.h>
#include <SSD1306Wire.h>
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <SimpleTimer.h>
#include <wifi_provisioning/manager.h>

#define SDA 1
#define SCL 2
#define ADS_POS_ACS_CURRENT 1   // Posição fisicamente conectado ao ADS1115
#define ADS_POS_VOLTAGE     2   // Posição fisicamente conectado ao ADS1115
#define RELAY_PIN 11

#define DELAY 10000
#define DELAY_SHOW 2000
#define DELAY_BETWEEN_SAMPLES 1163  //8333.33 // Atraso entre as amostras (em us)

#define ZERO 0
#define MIN_DIV 1
#define SAMPLES 3

#define MAX_ZERO_SAMPLES 150

#define SAMPLES_PER_CYCLE     200     // Número de amostras por ciclo
#define HOUR_MILLIS           3600000 // 60 Min * 60 Sec * 1000 millis 

/*Medido*/
#define V_R1 1008000
#define V_R2 9960
#define V_REQ 1017960
#define VOLTAGE_MULTIPLICATOR 102.2 //V_REQ / V_R2 = 1017960 / 9960 = 111.1

#define MINIMAL_VOLTAGE 0.035
#define MINIMAL_CURRENT 0.007

/*Calculado*/
/*
#define ACS_MULTIPLICATOR 10
#define ACS_LIMITS 0.0575
#define ACS_REQ 7100
#define ACS_R3  2000
#define ACS_R4  5100*/

/*Medido*/
#define ACS_MULTIPLICATOR 10   //(100mV/A = 0,1V/A)
#define ACS_LIMITS 0.0575
#define ACS_REQ 7043
#define ACS_R3  1998
#define ACS_R4  5045

#define DEFAULT_RELAY_MODE false
#define DEFAULT_POWER_MODE true

SSD1306Wire display(0x3c, SDA, SCL);
Adafruit_ADS1115 ads;

extern float checkSensors();
extern void show_display();

unsigned long lastMillis, lastMillisRead, lastMillisShow;
extern float cVoltage, cCurrent, cEnergy, cPower;
float ZERO_CURRENT_OUTPUT = 1.7958, LIMIT_INF_ZERO_CURRENT = 1.7383, LIMIT_SUP_ZERO_CURRENT = 1.8533;

/*----------------------------------------Read Sensor----------------------------------------*/
/*-------------------------------------Calculated values-------------------------------------*/
extern float checkSensors() {
  int16_t adcCurrent = ZERO, adcVoltage = ZERO;
  float vCurrent = ZERO, finalCurrent = ZERO, sumCurrent = ZERO, finalVoltage = ZERO, volts = ZERO, sumVoltage = ZERO;
  uint8_t currentCounter = ZERO;
  for(int i=ZERO; i < SAMPLES; i++) {
    for (int j=ZERO; j < SAMPLES_PER_CYCLE; j++) {
      adcCurrent = ads.readADC_SingleEnded(ADS_POS_ACS_CURRENT);
      adcVoltage = ads.readADC_SingleEnded(ADS_POS_VOLTAGE);

      vCurrent   = ads.computeVolts(adcCurrent);
      volts      = ads.computeVolts(adcVoltage);
      sumVoltage += (volts * volts);

      if (vCurrent < LIMIT_SUP_ZERO_CURRENT && vCurrent > LIMIT_INF_ZERO_CURRENT) {
        currentCounter++;
      } else {
        vCurrent = (vCurrent - ZERO_CURRENT_OUTPUT) * ACS_REQ / ACS_R4;
        sumCurrent += (vCurrent * vCurrent);
      }
      delayMicroseconds(DELAY_BETWEEN_SAMPLES);
    }
    if (currentCounter >= MAX_ZERO_SAMPLES) {
      finalCurrent += ZERO;
    } else {
      finalCurrent += (sumCurrent/(SAMPLES_PER_CYCLE-currentCounter));
    }
    finalVoltage += (sumVoltage/SAMPLES_PER_CYCLE);
    sumVoltage = ZERO;
    currentCounter = ZERO;
    sumCurrent = ZERO;
  }
  finalCurrent /= SAMPLES;
  finalCurrent = sqrt(finalCurrent);
  finalCurrent > MINIMAL_CURRENT ? cCurrent = finalCurrent * ACS_MULTIPLICATOR : cCurrent = ZERO;
  
  finalVoltage /= SAMPLES;
  finalVoltage = sqrt(finalVoltage);
  finalVoltage > MINIMAL_VOLTAGE ? cVoltage = ((finalVoltage * V_REQ) / V_R2) : cVoltage = ZERO;
  
  if(cCurrent == ZERO) {
    cEnergy = ZERO;
    cPower = ZERO;
  } else {
    cPower = cVoltage * cCurrent;
    cEnergy += cPower * (millis() - lastMillisRead) / HOUR_MILLIS;
  } 
  lastMillisRead = millis();
  return cEnergy, cPower, cVoltage, cCurrent;
}

/*--------------------------------------------Display------------------------------------------*/
extern void show_display() {                                              //Função de exibição no display
  display.clear();                                                        //Limpa os dados no display
  display.drawString(0,  0, "Energia: "  + String(cEnergy,  3) + " W/h"); //Começa a escrever nos espaços x e y, coluna  0 e linha  0
  display.drawString(0, 15, "Potência: " + String(cPower,   3) + " W");   //Começa a escrever nos espaços x e y, coluna  0 e linha 15
  display.drawString(0, 30, "Tensão: "   + String(cVoltage, 3) + " V");   //Começa a escrever nos espaços x e y, coluna  0 e linha 30
  display.drawString(0, 45, "Corrente: " + String(cCurrent, 3) + " A");   //Começa a escrever nos espaços x e y, coluna  0 e linha 45 
  display.display();                                                      //Exibe no display o que foi escrito com o comando display.drawString
}

void acs_calibrated() {
  digitalWrite(RELAY_PIN, HIGH);
  int16_t adcCurrent = ZERO;
  float vCurrent = ZERO, finalCurrent = ZERO, sumCurrent = ZERO;

  display.clear();
  display.drawString(0,  0, "Calibrando sensor de corrente");
  display.display();

  for(int i=ZERO; i < SAMPLES; i++) {
    for (int j=ZERO; j < SAMPLES_PER_CYCLE; j++) {
      adcCurrent = ads.readADC_SingleEnded(ADS_POS_ACS_CURRENT);
      vCurrent   = ads.computeVolts(adcCurrent);
      sumCurrent += vCurrent;
      delayMicroseconds(DELAY_BETWEEN_SAMPLES);
    }
    finalCurrent += (sumCurrent/SAMPLES_PER_CYCLE);
    sumCurrent = ZERO;
  }
  ZERO_CURRENT_OUTPUT    = finalCurrent / SAMPLES;
  LIMIT_INF_ZERO_CURRENT = ZERO_CURRENT_OUTPUT - ACS_LIMITS;
  LIMIT_SUP_ZERO_CURRENT = ZERO_CURRENT_OUTPUT + ACS_LIMITS;
  digitalWrite(RELAY_PIN, LOW);

  display.clear();
  display.drawString(0,  0, "Zero Current: "     + String(ZERO_CURRENT_OUTPUT, 4)    + " V");
  display.drawString(0, 15, "Zero Current Inf: " + String(LIMIT_INF_ZERO_CURRENT, 4) + " V"); 
  display.drawString(0, 30, "Zero Current Sup: " + String(LIMIT_SUP_ZERO_CURRENT, 4) + " V");
  display.display();
  delay(2000);
  display.clear();
}
