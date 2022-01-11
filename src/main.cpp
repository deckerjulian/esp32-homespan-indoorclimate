
#include <Arduino.h>
#include "HomeSpan.h"
#include "bsec.h"
#include <EEPROM.h>

// BME680 BSEC Definitions
// =====================================================================================
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};

#define STATE_SAVE_PERIOD UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day

// Helper functions declarations
void checkIaqSensorStatus(void);
void loadState(void);
void updateState(void);

Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

String output;

// DEV_TempSensor
// =====================================================================================
struct DEV_TempSensor : Service::TemperatureSensor
{

  SpanCharacteristic *temp;

  DEV_TempSensor() : Service::TemperatureSensor()
  {

    temp = new Characteristic::CurrentTemperature(0.0);
    temp->setRange(-50, 100);
    Serial.print("Configuring Temperature Sensor");
    Serial.print("\n");
  }

  void loop()
  {

    if (temp->timeVal() > 5000)
    { // check time elapsed since last update and proceed only if greater than 5 seconds
      unsigned long time_trigger = millis();
      if (iaqSensor.run())
      { // If new data is available
        output = String(time_trigger);
        output += ", " + String(iaqSensor.rawTemperature);
        output += ", " + String(iaqSensor.pressure);
        output += ", " + String(iaqSensor.rawHumidity);
        output += ", " + String(iaqSensor.gasResistance);
        output += ", " + String(iaqSensor.iaq);
        output += ", " + String(iaqSensor.iaqAccuracy);
        output += ", " + String(iaqSensor.temperature);
        output += ", " + String(iaqSensor.humidity);
        Serial.println(output);
        updateState();
        temp->setVal(iaqSensor.temperature);
      }
      else
      {
        checkIaqSensorStatus();
      }
      LOG1("Temperature Update");
      LOG1("\n");
    }

  } // loop
};

// DEV_TempSensor
// =====================================================================================
struct DEV_AirQualitySensor : Service::AirQualitySensor
{

  SpanCharacteristic *airQuality;
  SpanCharacteristic *voc;

  DEV_AirQualitySensor() : Service::AirQualitySensor()
  {

    voc = new Characteristic::VOCDensity(0);
    airQuality = new Characteristic::AirQuality(1);

    Serial.print("Configuring Air Quality Sensor");
    Serial.print("\n");

  } // end constructor

  void loop()
  {

    if (airQuality->timeVal() > 7500)
    {
      unsigned long time_trigger = millis();
      if (iaqSensor.run())
      { // If new data is available
        output = String(time_trigger);
        output += ", " + String(iaqSensor.rawTemperature);
        output += ", " + String(iaqSensor.pressure);
        output += ", " + String(iaqSensor.rawHumidity);
        output += ", " + String(iaqSensor.gasResistance);
        output += ", " + String(iaqSensor.iaq);
        output += ", " + String(iaqSensor.iaqAccuracy);
        output += ", " + String(iaqSensor.temperature);
        output += ", " + String(iaqSensor.humidity);
        Serial.println(output);

        voc->setVal(iaqSensor.gasResistance);

        float air_quality;
        air_quality = 1 + (5 - 1) * ((iaqSensor.iaq - 0) / (500 - 0));

        airQuality->setVal(air_quality);
      }

      LOG1("AirQuality Update");
      LOG1("\n");
    } // loop
  }
};

// SETUP
// =====================================================================================
void setup()
{

  // Init Serial and Wire
  Serial.begin(115200);
  Wire.begin();

  EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1);

  // BME680 Setup
  // =====================================================================================
  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire); // adapt to I2C - BME680_I2C_ADDR_PRIMARY e.x.
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  iaqSensor.setConfig(bsec_config_iaq);
  checkIaqSensorStatus();

  loadState();

  bsec_virtual_sensor_t sensorList[7] = {
      BSEC_OUTPUT_RAW_TEMPERATURE,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_HUMIDITY,
      BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 7, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  // Print the header
  output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%]";
  Serial.println(output);

  // Homespan Setup
  // =====================================================================================

  homeSpan.begin(Category::Sensors, "Raumklima");
  // homeSpan.setPairingCode("46637726");
  // homeSpan.setLogLevel(2);
  // homeSpan.setWifiCredentials("SSID","PASSWORD");

  new SpanAccessory();

  new Service::AccessoryInformation();
  new Characteristic::Name("Temperatur");
  new Characteristic::Manufacturer("atlane");
  new Characteristic::SerialNumber("321-ATL");
  new Characteristic::Model("BME680 Sensor");
  new Characteristic::FirmwareRevision("1.0");
  new Characteristic::Identify();
  new Service::HAPProtocolInformation();
  new Characteristic::Version("1.1.0");

  new DEV_TempSensor();

  new SpanAccessory();

  new Service::AccessoryInformation();
  new Characteristic::Name("Raumklima");
  new Characteristic::Manufacturer("atlane");
  new Characteristic::SerialNumber("546-ATL");
  new Characteristic::Model("BME680 Sensor");
  new Characteristic::FirmwareRevision("1.0");
  new Characteristic::Identify();
  new Service::HAPProtocolInformation();
  new Characteristic::Version("1.1.0");

  new DEV_AirQualitySensor();
}

void loop()
{
  homeSpan.poll();
}

// BME680 Helper Functions
// =====================================================================================
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK)
  {
    if (iaqSensor.status < BSEC_OK)
    {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
    }
    else
    {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK)
  {
    if (iaqSensor.bme680Status < BME680_OK)
    {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
    else
    {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}

void loadState(void)
{
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE)
  {
    // Existing state in EEPROM
    Serial.println("Reading state from EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
      bsecState[i] = EEPROM.read(i + 1);
      Serial.println(bsecState[i], HEX);
    }

    iaqSensor.setState(bsecState);
    checkIaqSensorStatus();
  }
  else
  {
    // Erase the EEPROM with zeroes
    Serial.println("Erasing EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
      EEPROM.write(i, 0);

    EEPROM.commit();
  }
}

void updateState(void)
{
  bool update = false;
  if (stateUpdateCounter == 0)
  {
    /* First state update when IAQ accuracy is >= 3 */
    if (iaqSensor.iaqAccuracy >= 3)
    {
      update = true;
      stateUpdateCounter++;
    }
  }
  else
  {
    /* Update every STATE_SAVE_PERIOD minutes */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis())
    {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update)
  {
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();

    Serial.println("Writing state to EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
      EEPROM.write(i + 1, bsecState[i]);
      Serial.println(bsecState[i], HEX);
    }

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }
}