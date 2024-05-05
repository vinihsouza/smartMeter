#include "main.h"

// BLE Credentils
const char *service_name = "PROV_12345";
const char *pop = "1234567";

float cCurrent, cVoltage, cEnergy, cPower;
bool relay = true, wifi_connected = ZERO;
static uint8_t gpio_reset = ZERO, RELAY = RELAY_PIN;

//---------------------------------------------- Declaring Devices --------------------------------------------------//

//The framework provides some standard device types like switch, lightbulb, fan, temperature sensor.
static TemperatureSensor energy("Energy");
static TemperatureSensor power("Power");
static TemperatureSensor voltage("Voltage");
static TemperatureSensor current("Current");
static Switch my_switch("Relay", &relay);

void sysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
      Serial.printf("\nName \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
      printQR(service_name, pop, "ble");
#else
      Serial.printf("\nName \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
      printQR(service_name, pop, "softap");
#endif
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.printf("\nConnected to Wi-Fi!\n");
      wifi_connected = 1;
      delay(500);
      break;
    case ARDUINO_EVENT_PROV_CRED_RECV: {
        Serial.println("\nReceived Wi-Fi credentials");
        Serial.print("\tSSID : ");
        Serial.println((const char *) sys_event->event_info.prov_cred_recv.ssid);
        Serial.print("\tPassword : ");
        Serial.println((char const *) sys_event->event_info.prov_cred_recv.password);
        break;
      }
    case ARDUINO_EVENT_PROV_INIT:
      wifi_prov_mgr_disable_auto_stop(DELAY);
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      Serial.println("Stopping Provisioning!!!");
      wifi_prov_mgr_stop_provisioning();
      break;
  }
}

void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx) {
  const char *device_name = device->getDeviceName();
  Serial.println(device_name);
  const char *param_name = param->getParamName();

  if (strcmp(device_name, "Relay") == ZERO) {
    if (strcmp(param_name, "Power") == ZERO) {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      relay = val.val.b;
      (relay == false) ? digitalWrite(RELAY, LOW) : digitalWrite(RELAY, HIGH);
      param->updateAndReport(val);
    }
  }
}

void setup() {
  pinMode(gpio_reset, INPUT);
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);

  Serial.begin(115200);
  Wire.setPins(SDA, SCL);

  display.init();
  display.flipScreenVertically();
  display.clear();
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS."); while (true);
  }
  ads.setDataRate(RATE_ADS1115_860SPS);
  acs_calibrated();

  //------------------------------------------- Declaring Node -----------------------------------------------------//
  Node my_node;
  my_node = RMaker.initNode("SmartMeter");

  my_switch.addCb(write_callback);

  my_node.addDevice(energy);
  my_node.addDevice(power);
  my_node.addDevice(voltage);
  my_node.addDevice(current);
  my_node.addDevice(my_switch);

  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.setTimeZone("America/Sao_Paulo");
  RMaker.enableTZService();
  RMaker.enableSchedule();

  Serial.printf("\nStarting ESP-RainMaker\n");
  RMaker.start();

  WiFi.onEvent(sysProvEvent);

#if CONFIG_IDF_TARGET_ESP32
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#else
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
#endif
cEnergy, cPower, cVoltage, cCurrent = checkSensors();
lastMillis = millis();
}

void loop() {
  if (wifi_connected && millis() - lastMillis > DELAY) {
    Serial.println("Sending Sensor's Data");
    Send_Sensor();
    lastMillis = millis();
  }

  if (millis() - lastMillisShow > DELAY_SHOW) {
    cEnergy, cPower, cVoltage, cCurrent = checkSensors();
    show_display();
    lastMillisShow = millis();
  }

  //-----------------------------------------------------------  Logic to Reset RainMaker

  // Read GPIO0 (external button to reset device
  if (digitalRead(gpio_reset) == LOW) { //Push button pressed
    Serial.printf("Reset Button Pressed!\n");
    // Key debounce handling
    delay(100);
    int startTime = millis();
    while (digitalRead(gpio_reset) == LOW) delay(50);
    int endTime = millis();

    if ((endTime - startTime) > DELAY) {
      // If key pressed for more than 10secs, reset all
      Serial.printf("Reset to factory.\n");
      wifi_connected = ZERO;
      RMakerFactoryReset(2);
    } else if ((endTime - startTime) > 3000) {
      Serial.printf("Reset Wi-Fi.\n");
      wifi_connected = ZERO;
      // If key pressed for more than 3secs, but less than 10, reset Wi-Fi
      RMakerWiFiReset(2);
    }
  }
  delay(100);
}

void Send_Sensor() {
  energy.updateAndReportParam("Temperature", cEnergy);
  power.updateAndReportParam("Temperature", cPower);
  voltage.updateAndReportParam("Temperature", cVoltage);
  current.updateAndReportParam("Temperature", cCurrent);
}
