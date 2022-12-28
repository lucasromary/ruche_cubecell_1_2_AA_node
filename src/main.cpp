#include <Arduino.h>
#include <HX711_ADC.h>
#include <EEPROM.h>
#include "LoRaWan_APP.h"
#include "LoRa_APP.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "string.h"
#include <CayenneLPP.h>

/*  Capteur de température  */
#define ONE_WIRE_PIN GPIO11
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature temp_sensor(&oneWire);
#define SENSOR_RESOLUTION 9
#define SENSOR_INDEX 0
DeviceAddress sensorDeviceAddress;
float temp = 0;
float last_temp = 0;

/*  Capteur de poids  */
const int HX711_dout_1 = GPIO7; // mcu > HX711 no 1 dout pin
const int HX711_sck_1 = GPIO8;  // mcu > HX711 no 1 sck pin
const int HX711_dout_2 = GPIO5; // mcu > HX711 no 2 dout pin
const int HX711_sck_2 = GPIO6;  // mcu > HX711 no 2 sck pin

HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); // HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); // HX711 2
long tare_1 = 0;
long tare_2 = 0;
float weight_1 = 0;
float weight_2 = 0;
float last_weight_1 = 0;
float last_weight_2 = 0;
float calibrationValue_1 = 43967.0;   // calibration value (see example file "Calibration.ino")
float calibrationValue_2 = 43967.0;   // calibration value (see example file "Calibration.ino")
unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time

/*  Address EEPROM  */
const int tare_adress_1 = 10;
const int tare_adress_2 = 20;
const int temp_adress = 30;
const int weight_adress_1 = 40;
const int weight_adress_2 = 50;
const int voltage_adress = 60;

/* Bouton */
int pin_button = GPIO9;
int pin_LED_button = GPIO10;
bool val_button = 1;
bool last_val_button = 1;
int number_button_press = 0;
long last_trigger = 0;
bool trigger_button = 0;

/* Batterie */
float voltage = 0;
float last_voltage = 0;

/* Timers */
long timer = 0;
long start_timer_button = 0;

/* Paramètres LORAWAN */
uint8_t devEui[] = {0x56, 0xE1, 0x95, 0xE0, 0x2F, 0xA7, 0x9F, 0x42}; // 56E195E02FA79F42
uint8_t appEui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appKey[] = {0x21, 0xFF, 0x38, 0xC9, 0x10, 0x5D, 0x1F, 0x4C, 0x86, 0xF6, 0x33, 0x4A, 0xF9, 0xCA, 0xF1, 0x13}; // 21FF38C9105D1F4C86F6334AF9CAF113
// 70B3D57ED004E7CD
// 19C94F11AA26C1D8D13135E9AF4B56E6
uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
/* ABP para*/
uint8_t nwkSKey[] = {};
uint8_t appSKey[] = {};
uint32_t devAddr = (uint32_t)0x000000000;

LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_EU868;
DeviceClass_t loraWanClass = CLASS_A;

uint32_t appTxDutyCycle = 0.5 * 60 * 1000; // 1 minute

bool overTheAirActivation = true;
bool loraWanAdr = true;
/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = true;
/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = false;
/* Application port */
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

CayenneLPP lpp(160);

void tareLoadCell()
{
  Serial.println("Tare BOTH load cells");
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;

  while ((loadcell_1_rdy + loadcell_2_rdy) < 2)
  { // run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_1_rdy)
      loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, true);
    if (!loadcell_2_rdy)
      loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, true);
  }
  if (LoadCell_1.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }
  else
  {
    Serial.println("Putting Tare no.1 in EEPROM");
    EEPROM.put(tare_adress_1, LoadCell_1.getTareOffset());
    EEPROM.commit();
  }
  if (LoadCell_2.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }
  else
  {
    Serial.println("Putting Tare no.2 in EEPROM");
    EEPROM.put(tare_adress_2, LoadCell_2.getTareOffset());
    EEPROM.commit();
  }
  LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value (float)
  LoadCell_2.setCalFactor(calibrationValue_2); // user set calibration value (float)
  Serial.println("Tare is complete");

  while (!LoadCell_1.update())
    ;
  while (!LoadCell_2.update())
    ;
  /*
    Serial.print("Calibration value: ");
    Serial.print(LoadCell_1.getCalFactor());
    Serial.print(" / ");
    Serial.println(LoadCell_2.getCalFactor());

    Serial.print("HX711 measured conversion time ms: ");
    Serial.print(LoadCell_1.getConversionTime());
    Serial.print(" / ");
    Serial.println(LoadCell_2.getConversionTime());

    Serial.print("HX711 measured sampling rate HZ: ");
    Serial.print(LoadCell_1.getSPS());
    Serial.print(" / ");
    Serial.println(LoadCell_2.getSPS());

    Serial.print("HX711 measured settlingtime ms: ");
    Serial.print(LoadCell_1.getSettlingTime());
    Serial.print(" / ");
    Serial.println(LoadCell_2.getSettlingTime());
  */
}

void tareLoadCell_1()
{
  Serial.println("Tare 1st load cell");
  LoadCell_1.start(stabilizingtime, true);
  if (LoadCell_1.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1)
      ;
  }
  else
  {
    LoadCell_1.setCalFactor(calibrationValue_1); // set calibration value (float)
    Serial.println("Startup is complete");
    Serial.println("Putting Tare no.1 in EEPROM");
    EEPROM.put(tare_adress_1, LoadCell_1.getTareOffset());
    EEPROM.commit();
  }
}

void tareLoadCell_2()
{
  Serial.println("Tare 2nd load cell");
  LoadCell_2.start(stabilizingtime, true);
  if (LoadCell_2.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1)
      ;
  }
  else
  {
    LoadCell_2.setCalFactor(calibrationValue_2); // set calibration value (float)
    Serial.println("Startup is complete");
    Serial.println("Putting Tare no.1 in EEPROM");
    EEPROM.put(tare_adress_2, LoadCell_2.getTareOffset());
    EEPROM.commit();
  }
}

void refreshDataLoadCell()
{
  LoadCell_1.refreshDataSet();
  LoadCell_2.refreshDataSet();
  weight_1 = LoadCell_1.getData();
  weight_2 = LoadCell_2.getData();
}

void wake_up_tare()
{
  Serial.println("Wake up ! ");
  trigger_button = 1;
}

void getSensorsData()
{

  timer = millis();
  // Serial.print("get sensors data ... ");

  bool load_cell_ok = 0;
  temp_sensor.requestTemperatures();
  // Measurement may take 500ms
  while (millis() - timer < 500)
  {
    if (load_cell_ok == 0)
    {
      refreshDataLoadCell();
    }
    load_cell_ok = 1;
    delay(10);
  }
  temp = temp_sensor.getTempCByIndex(SENSOR_INDEX);

  // Serial.print(millis() - timer); // 446 que les cellules de poids
  // Serial.println(" done ");

  /* Print valeurs capteur */
  // Serial.print(weight_1);
  // Serial.print(" / ");
  // Serial.print(weight_2);
  // Serial.print(" / ");
  // Serial.println(temp, 4);
}

void buttonPressed()
{
  if (trigger_button == 1)
  {
    digitalWrite(Vext, LOW);
    digitalWrite(pin_LED_button, HIGH);
    delay(1000);
    detachInterrupt(pin_button);
    start_timer_button = millis();
    number_button_press = 0;

    while (millis() - start_timer_button < 9000)
    {
      val_button = digitalRead(pin_button);
      if (val_button == 0 && last_val_button == 1 && millis() - last_trigger > 1000)
      {
        number_button_press++;
        last_trigger = millis();
        Serial.println("Pressed !");
      }
      last_val_button = val_button;
    }
    Serial.println(number_button_press);
    digitalWrite(pin_LED_button, LOW);
    delay(1000);
    if (number_button_press == 1)
    {
      tareLoadCell_1();
      digitalWrite(pin_LED_button, HIGH);
      delay(500);
      digitalWrite(pin_LED_button, LOW);
    }
    else if (number_button_press == 2)
    {
      tareLoadCell_2();
      for (int i = 0; i < 2; i++)
      {
        digitalWrite(pin_LED_button, HIGH);
        delay(500);
        digitalWrite(pin_LED_button, LOW);
        delay(500);
      }
    }
    else if (number_button_press == 3)
    {
      tareLoadCell();
      for (int i = 0; i < 3; i++)
      {
        digitalWrite(pin_LED_button, HIGH);
        delay(500);
        digitalWrite(pin_LED_button, LOW);
        delay(500);
      }
    }

    digitalWrite(pin_LED_button, LOW);
    attachInterrupt(pin_button, wake_up_tare, FALLING);
    trigger_button = 0;
  }
}

static void getBatteryVoltageFloat(void)
{
  pinMode(VBAT_ADC_CTL, OUTPUT);
  digitalWrite(VBAT_ADC_CTL, LOW);

  uint16_t u16BatteryVoltage = analogRead(ADC) * 2;
  pinMode(VBAT_ADC_CTL, INPUT);
  // fBatteryVoltage = (float)u16BatteryVoltage / 1000;
  // Serial.print("Battery Voltage: ");
  // Serial.print(u16BatteryVoltage);
  // Serial.println("mV");

  voltage = getBatteryVoltage();
  // Serial.print("Battery Voltage Cubecell: ");
  // Serial.print(voltage);
  // Serial.println("mV");
}

static void prepareTxFrame(uint8_t port)
{
  EEPROM.get(weight_adress_1, last_weight_1);
  EEPROM.get(weight_adress_2, last_weight_2);
  EEPROM.get(temp_adress, last_temp);
  EEPROM.get(voltage_adress, last_voltage);

  digitalWrite(Vext, LOW);

  getSensorsData();
  getBatteryVoltageFloat();

  // voltage = 3850;
  // weight_1 = 47.75;
  // weight_2 = 56.75;
  // temp = 18.56;

  if (last_temp == temp && weight_1 == last_weight_1 && weight_2 == last_weight_2 && voltage == last_voltage)
  {
    Serial.println("même valeur que l'eeprom, bizarre, Reset ! ");
    last_temp = 0;
    last_voltage = 0;
    last_weight_1 = 0;
    last_weight_2 = 0;

    EEPROM.put(temp_adress, last_temp);
    EEPROM.put(voltage_adress, voltage);
    EEPROM.put(weight_adress_1, last_weight_1);
    EEPROM.put(weight_adress_2, last_weight_2);
    EEPROM.commit();
    delay(500);
    HW_Reset(0);
  }
  else
  {
    EEPROM.put(temp_adress, last_temp);
    EEPROM.put(voltage_adress, voltage);
    EEPROM.put(weight_adress_1, last_weight_1);
    EEPROM.put(weight_adress_2, last_weight_2);
    EEPROM.commit();
  }

  lpp.reset();
  lpp.addAnalogInput(1, voltage / 1000);
  lpp.addAnalogInput(2, weight_1);
  lpp.addAnalogInput(3, weight_2);
  lpp.addTemperature(4, temp);
  appDataSize = lpp.getSize();
  memcpy(appData, lpp.getBuffer(), lpp.getSize());

  // Vext OFF
  // pinMode(GPIO1, INPUT_PULLDOWN);
  digitalWrite(Vext, HIGH);
}

void setup()
{
  boardInitMcu();
  EEPROM.begin(512);
  Serial.begin(115200);
  pinMode(Vext, OUTPUT);
  pinMode(pin_button, INPUT_PULLUP);
  pinMode(pin_LED_button, OUTPUT);
  pinMode(ONE_WIRE_PIN, INPUT);

  digitalWrite(Vext, LOW);

  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();

  // delay(20);

  temp_sensor.begin();
  temp_sensor.setWaitForConversion(false);
  temp_sensor.getAddress(sensorDeviceAddress, 0);
  temp_sensor.setResolution(sensorDeviceAddress, 10);

  LoadCell_1.begin();
  LoadCell_2.begin();

  EEPROM.get(tare_adress_1, tare_1);
  Serial.println(tare_1);
  LoadCell_1.setTareOffset(tare_1);
  LoadCell_1.setCalFactor(calibrationValue_1);

  EEPROM.get(tare_adress_2, tare_2);
  Serial.println(tare_2);
  LoadCell_2.setTareOffset(tare_2);
  LoadCell_2.setCalFactor(calibrationValue_2);

  attachInterrupt(pin_button, wake_up_tare, FALLING);
  // put your setup code here, to run once:
}

void loop()
{
  buttonPressed();

  switch (deviceState)
  {
  case DEVICE_STATE_INIT:
  {
    printDevParam();
    LoRaWAN.init(loraWanClass, loraWanRegion);
    deviceState = DEVICE_STATE_JOIN;
    break;
  }
  case DEVICE_STATE_JOIN:
  {
    // LoRaWAN.displayJoining();
    LoRaWAN.join();
    break;
  }
  case DEVICE_STATE_SEND:
  {
    // LoRaWAN.displaySending();
    prepareTxFrame(appPort);
    LoRaWAN.send();
    deviceState = DEVICE_STATE_CYCLE;
    // delay(100);
    break;
  }
  case DEVICE_STATE_CYCLE:
  {
    //  Schedule next packet transmission
    txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
    LoRaWAN.cycle(txDutyCycleTime);
    deviceState = DEVICE_STATE_SLEEP;
    break;
  }
  case DEVICE_STATE_SLEEP:
  {
    // LoRaWAN.displayAck();
    LoRaWAN.sleep();
    break;
  }
  default:
  {
    deviceState = DEVICE_STATE_INIT;
    break;
  }
  }
}