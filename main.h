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

#define DELAY 30000
#define DELAY_SHOW 2500
#define DELAY_BETWEEN_SAMPLES 1163  //8333.33 // Atraso entre as amostras (em us)
#define DELAY_PUBLISH 15000

#define ZERO    0
#define MIN_DIV 1
#define SQUARE  2
#define SAMPLES 3

#define SAMPLES_PER_CYCLE     200     // Número de amostras por ciclo
#define HOUR_MILLIS           3600000 // 60 Min * 60 Sec * 1000 millis 

#define VOLTAGE_MULTIPLICATOR 101 // (R1+R2) / R2 => 1010000 / 10000 => 101 
#define ACS_MULTIPLICATOR 15.14
#define MINIMAL_VOLTAGE 0.005
#define MINIMAL_CURRENT 0.004

#define ZERO_CURRENT_OUTPUT    1.650
#define LIMIT_INF_ZERO_CURRENT 1.642
#define LIMIT_SUP_ZERO_CURRENT 1.658

#define DEFAULT_RELAY_MODE false
#define DEFAULT_POWER_MODE true

SSD1306Wire display(0x3c, SDA, SCL);
Adafruit_ADS1115 ads;

extern float checkSensors();
extern void show_display();

unsigned long lastMillis, lastMillisRead, lastMillisShow;
extern float cVoltage, cCurrent, cEnergy, cPower;

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
      sumVoltage += pow(volts, SQUARE);

      if (vCurrent < LIMIT_SUP_ZERO_CURRENT && vCurrent > LIMIT_INF_ZERO_CURRENT) {
        currentCounter++;
      } else {
        sumCurrent += pow((vCurrent - ZERO_CURRENT_OUTPUT), SQUARE);
      }
      delayMicroseconds(DELAY_BETWEEN_SAMPLES);
    }

    finalCurrent += (sumCurrent/((SAMPLES_PER_CYCLE-currentCounter) == ZERO ? MIN_DIV : SAMPLES_PER_CYCLE-currentCounter));
    finalVoltage += (sumVoltage/SAMPLES_PER_CYCLE);
    sumVoltage = ZERO;
    currentCounter = ZERO;
    sumCurrent = ZERO;
  }
  
  finalCurrent /= SAMPLES;
  finalCurrent = sqrt(finalCurrent);
  finalCurrent > MINIMAL_CURRENT ? cCurrent = finalCurrent * ACS_MULTIPLICATOR : cCurrent = ZERO;
  if(cCurrent == ZERO) {
    cVoltage = ZERO;
    cEnergy = ZERO;
    cPower = ZERO;
  } else {
    finalVoltage /= SAMPLES;
    finalVoltage = sqrt(finalVoltage);
    finalVoltage > MINIMAL_VOLTAGE ? cVoltage = finalVoltage * VOLTAGE_MULTIPLICATOR : cVoltage = ZERO;
    cPower = cVoltage * cCurrent;
    cEnergy += cPower * (millis() - lastMillisRead) / HOUR_MILLIS;
    lastMillisRead = millis();
  }
  return cEnergy, cPower, cVoltage, cCurrent;
}

/*--------------------------------------------Display------------------------------------------*/
extern void show_display() {                                                  //Função de exibição no display
  display.clear();                                                     //Limpa os dados no display
  display.drawString(0,  0, "Energia: "  + String(cEnergy) + " W/h");  //Começa a escrever nos espaços x e y, coluna  0 e linha  0
  display.drawString(0, 15, "Potência: " + String(cPower) + " W");     //Começa a escrever nos espaços x e y, coluna  0 e linha 15
  display.drawString(0, 30, "Tensão: "   + String(cVoltage) + " V");   //Começa a escrever nos espaços x e y, coluna  0 e linha 30
  display.drawString(0, 45, "Corrente: " + String(cCurrent) + " A");   //Começa a escrever nos espaços x e y, coluna  0 e linha 45 
  display.display();                                                   //Exibe no display o que foi escrito com o comando display.drawString
}
