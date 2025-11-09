/*
    Copyright (C) <2020>  <Mike Roberts>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <ESPTelnet.h>
#include <ArduinoHA.h>

#include "Ecodan.h"
#include "config.h"
#include "MQTTConfig.h"
#include "TimerCallBack.h"
#include "Debug.h"
#include "OTA.h"

#define HEATPUMP_STREAM Serial2
#define SERIAL_BAUD 2400
#define SERIAL_CONFIG SERIAL_8E1

ECODAN HeatPump;

WiFiClient NetworkClient;
// PubSubClient MQTTClient(NetworkClient);
ESPTelnet TelnetServer;
String HostName;
HADevice device;
HAMqtt mqtt(NetworkClient, device, 32);

void HeatPumpQueryStateEngine(void);
void setupTelnet(void);
void HeatPumpKeepAlive(void);
void Zone1Report(void);
void Zone1Report(void);
void HotWaterReport(void);
void SystemReport(void);
void TestReport(void);

TimerCallBack HeatPumpQuery1(500, HeatPumpQueryStateEngine);
TimerCallBack HeatPumpQuery2(60 * 1000, HeatPumpKeepAlive);

HASensorNumber outsideTemp("outsideTemp", HASensorNumber::PrecisionP1);
HASensorNumber legionellaSetpoint("legionellaSetpoint", HASensorNumber::PrecisionP1);
HASensorNumber hotWaterMaximumTempDrop("hotWaterMaximumTempDrop", HASensorNumber::PrecisionP1);
HASensorNumber heaterOutputFlowTemperature("heaterOutputFlowTemperature", HASensorNumber::PrecisionP1);
HASensorNumber heaterReturnFlowTemperature("heaterReturnFlowTemperature", HASensorNumber::PrecisionP1);
HASensorNumber heaterFlowSetpoint("heaterFlowSetpoint", HASensorNumber::PrecisionP1);
HASensorNumber outputPower("outputPower", HASensorNumber::PrecisionP1);
HASensorNumber primaryFlowRate("primaryFlowRate", HASensorNumber::PrecisionP0);
HASensorNumber zone1FlowTemperatureSetpoint("zone1FlowTemperatureSetpoint", HASensorNumber::PrecisionP1);
HASensorNumber zone2FlowTemperatureSetpoint("zone2FlowTemperatureSetpoint", HASensorNumber::PrecisionP1);
HASensorNumber consumedHeatingEnergy("consumedHeatingEnergy", HASensorNumber::PrecisionP1);
HASensorNumber consumedHotWaterEnergy("consumedHotWaterEnergy", HASensorNumber::PrecisionP1);
HASensorNumber deliveredHeatingEnergy("deliveredHeatingEnergy", HASensorNumber::PrecisionP1);
HASensorNumber deliveredHotWaterEnergy("deliveredHotWaterEnergy", HASensorNumber::PrecisionP1);
HASensorNumber compressorFrequency("compressorFrequency", HASensorNumber::PrecisionP0);
HASensorNumber runHours("runHours", HASensorNumber::PrecisionP0);
HASensorNumber flowTempMax("flowTempMax", HASensorNumber::PrecisionP1);
HASensorNumber flowTempMin("flowTempMin", HASensorNumber::PrecisionP1);
HASensorNumber unknownMSG5("unknownMSG5", HASensorNumber::PrecisionP0);

HABinarySensor hotWaterTimerActive("hotWaterTimerActive");
HABinarySensor defrost("defrost");

HASensor hotWaterControlMode("hotWaterControlMode");
HASensor systemOperationMode("systemOperationMode");
HASelect heatingControlMode("heatingControlMode");
HASwitch DHWBoost("DHWBoost");
HASelect systemPowerMode("systemPowerMode");

HAHVAC zone1(
    "zone1",
    HAHVAC::TargetTemperatureFeature | HAHVAC::ModesFeature);

HAHVAC zone2(
    "zone2",
    HAHVAC::TargetTemperatureFeature| HAHVAC::ModesFeature);

HAHVAC hotWater(
    "hotWater",
    HAHVAC::TargetTemperatureFeature| HAHVAC::ModesFeature);

void loop()
{
  mqtt.loop();
  TelnetServer.loop();

  HeatPumpQuery1.Process();
  HeatPumpQuery2.Process();

  HeatPump.Process();
  ArduinoOTA.handle();
}

void onPowerCommand(bool state, HAHVAC *sender)
{
  String ON = "On";
  String Standby = "Standby";
  if (state)
  {

    HeatPump.SetSystemPowerMode(ECODAN::PowerState::On);
  }
  else
  {
    HeatPump.SetSystemPowerMode(ECODAN::PowerState::StandBy);
  }
}
void onTargetTemperatureCommand(HANumeric temperature, HAHVAC *sender)
{
  float temperatureFloat = temperature.toFloat();

  HeatPump.SetHotWaterSetpoint(temperatureFloat);

  sender->setTargetTemperature(temperature); // report target temperature back to the HA panel
}

void HeatPumpKeepAlive(void)
{
  HeatPump.KeepAlive();
  HeatPump.TriggerStatusStateMachine();
}

void Zone1Report(void)
{
  zone1.setCurrentTemperature(HeatPump.Status.Zone1Temperature);
  zone1.setTargetTemperature(HeatPump.Status.Zone1TemperatureSetpoint);
}

void Zone2Report(void)
{
  zone2.setCurrentTemperature(HeatPump.Status.Zone2Temperature);
  zone2.setTargetTemperature(HeatPump.Status.Zone2TemperatureSetpoint);
}

void HotWaterReport(void)
{
  hotWater.setCurrentTemperature(HeatPump.Status.HotWaterTemperature);
  hotWater.setTargetTemperature(HeatPump.Status.HotWaterSetpoint);

  DHWBoost.setState(HeatPump.Status.HotWaterBoostActive);
  hotWaterTimerActive.setState(HeatPump.Status.HotWaterTimerActive);
  hotWaterControlMode.setValue(HowWaterControlModeString[HeatPump.Status.HotWaterControlMode]);
  legionellaSetpoint.setValue(HeatPump.Status.LegionellaSetpoint);
  hotWaterMaximumTempDrop.setValue(HeatPump.Status.HotWaterMaximumTempDrop);
}

void SystemReport(void)
{
  heaterOutputFlowTemperature.setValue(HeatPump.Status.HeaterOutputFlowTemperature);
  heaterReturnFlowTemperature.setValue(HeatPump.Status.HeaterReturnFlowTemperature);
  heaterFlowSetpoint.setValue(HeatPump.Status.HeaterFlowSetpoint);
  outputPower.setValue(HeatPump.Status.OutputPower);
  systemPowerMode.setState(HeatPump.Status.SystemPowerMode);
  systemOperationMode.setValue(SystemOperationModeString[HeatPump.Status.SystemOperationMode]);
  heatingControlMode.setState(HeatPump.Status.HeatingControlMode);
  primaryFlowRate.setValue(HeatPump.Status.PrimaryFlowRate);
  outsideTemp.setValue(HeatPump.Status.OutsideTemperature);
}

void TestReport(void)
{
  zone1FlowTemperatureSetpoint.setValue(HeatPump.Status.Zone1FlowTemperatureSetpoint);
  zone2FlowTemperatureSetpoint.setValue(HeatPump.Status.Zone2FlowTemperatureSetpoint);
  consumedHeatingEnergy.setValue(HeatPump.Status.ConsumedHeatingEnergy);
  consumedHotWaterEnergy.setValue(HeatPump.Status.ConsumedHotWaterEnergy);
  deliveredHeatingEnergy.setValue(HeatPump.Status.DeliveredHeatingEnergy);
  deliveredHeatingEnergy.setValue(HeatPump.Status.DeliveredHeatingEnergy);
  compressorFrequency.setValue(HeatPump.Status.CompressorFrequency);
  runHours.setValue(HeatPump.Status.RunHours);
  flowTempMax.setValue(HeatPump.Status.FlowTempMax);
  flowTempMin.setValue(HeatPump.Status.FlowTempMin);
  unknownMSG5.setValue(HeatPump.Status.UnknownMSG5);
  defrost.setState(HeatPump.Status.Defrost);
}

void onTelnetConnect(String ip)
{
  DEBUG_PRINT("Telnet: ");
  DEBUG_PRINT(ip);
  DEBUG_PRINTLN(" connected");
  TelnetServer.println("\nWelcome " + TelnetServer.getIP());
  TelnetServer.println("(Use ^] + q  to disconnect.)");
}

void onTelnetDisconnect(String ip)
{
  DEBUG_PRINT("Telnet: ");
  DEBUG_PRINT(ip);
  DEBUG_PRINTLN(" disconnected");
}

void onTelnetReconnect(String ip)
{
  DEBUG_PRINT("Telnet: ");
  DEBUG_PRINT(ip);
  DEBUG_PRINTLN(" reconnected");
}

void onTelnetConnectionAttempt(String ip)
{
  DEBUG_PRINT("Telnet: ");
  DEBUG_PRINT(ip);
  DEBUG_PRINTLN(" tried to connected");
}
void HeatPumpQueryStateEngine(void)
{
  HeatPump.StatusStateMachine();
  if (HeatPump.UpdateComplete())
  {
    DEBUG_PRINTLN("Update Complete");
    Zone1Report();
    Zone2Report();
    HotWaterReport();
    SystemReport();
    TestReport();
  }
}
void setupTelnet()
{
  TelnetServer.onConnect(onTelnetConnect);
  TelnetServer.onConnectionAttempt(onTelnetConnectionAttempt);
  TelnetServer.onReconnect(onTelnetReconnect);
  TelnetServer.onDisconnect(onTelnetDisconnect);

  DEBUG_PRINT("Telnet: ");
  if (TelnetServer.begin())
  {
    DEBUG_PRINTLN("running");
  }
  else
  {
    DEBUG_PRINTLN("error.");
    // errorMsg("Will reboot...");
  }
}

void onZone1TargetTemperatureCommand(HANumeric temperature, HAHVAC *sender)
{
  float temperatureFloat = temperature.toFloat();

  HeatPump.SetZoneTempSetpoint(temperatureFloat, ZONE1);

  sender->setTargetTemperature(temperature); // report target temperature back to the HA panel
}
void onZone2TargetTemperatureCommand(HANumeric temperature, HAHVAC *sender)
{
  float temperatureFloat = temperature.toFloat();

  HeatPump.SetZoneTempSetpoint(temperatureFloat, ZONE2);

  sender->setTargetTemperature(temperature); // report target temperature back to the HA panel
}
void onhotWaterSetpointCommand(HANumeric number, HAHVAC *sender)
{
  float temperatureFloat = number.toFloat();
  HeatPump.SetHotWaterSetpoint(temperatureFloat);
  sender->setTargetTemperature(number); // report the selected option back to the HA panel
}
void onSelectPowerCommand(int8_t index, HASelect *sender)
{
  HeatPump.SetSystemPowerMode(static_cast<ECODAN::PowerState>(index));
  sender->setState(index); // report the selected option back to the HA panel
}
void onSelectHeatingCommand(int8_t index, HASelect *sender)
{
  String command(HeatingControlModeString[index]);
  HeatPump.SetHeatingControlMode(&command, BOTH);
  sender->setState(index); // report the selected option back to the HA panel
}
void onDHWBoostCommand(bool state, HASwitch* sender){
  HeatPump.ForceDHW(state);
  sender->setState(state);
}
void setupSensors()
{
  outsideTemp.setIcon("mdi:thermometer");
  outsideTemp.setName("Temperature outside");
  outsideTemp.setUnitOfMeasurement("°C");

  hotWaterTimerActive.setIcon("mdi:timer");
  hotWaterTimerActive.setName("Hot water timer active");

  hotWaterControlMode.setIcon("mdi:home");
  hotWaterControlMode.setName("Hot water control mode");

  legionellaSetpoint.setIcon("mdi:thermometer");
  legionellaSetpoint.setName("Legionella setpoint");
  legionellaSetpoint.setUnitOfMeasurement("°C");

  hotWaterMaximumTempDrop.setIcon("mdi:thermometer");
  hotWaterMaximumTempDrop.setName("Hot water maximum temperature drop");
  hotWaterMaximumTempDrop.setUnitOfMeasurement("°C");

  heaterOutputFlowTemperature.setIcon("mdi:thermometer");
  heaterOutputFlowTemperature.setName("Heater output flow temperature");
  heaterOutputFlowTemperature.setUnitOfMeasurement("°C");

  heaterReturnFlowTemperature.setIcon("mdi:thermometer");
  heaterReturnFlowTemperature.setName("Heater return flow temperature");
  heaterReturnFlowTemperature.setUnitOfMeasurement("°C");

  heaterFlowSetpoint.setIcon("mdi:thermometer");
  heaterFlowSetpoint.setName("Heater flow setpoint");
  heaterFlowSetpoint.setUnitOfMeasurement("°C");

  outputPower.setIcon("mdi:power-plug");
  outputPower.setName("Output power");
  outputPower.setUnitOfMeasurement("W");

  primaryFlowRate.setIcon("mdi:water-pump");
  primaryFlowRate.setName("Primary flow rate");
  primaryFlowRate.setUnitOfMeasurement("L/min");

  systemOperationMode.setIcon("mdi:home");
  systemOperationMode.setName("System operation mode");

  systemPowerMode.setOptions("Standby;On");
  systemPowerMode.onCommand(onSelectPowerCommand);
  systemPowerMode.setIcon("mdi:power");
  systemPowerMode.setName("System power mode");

  heatingControlMode.setOptions("Temperature Control;Fixed Flow;Compensation Flow");
  heatingControlMode.onCommand(onSelectHeatingCommand);
  heatingControlMode.setIcon("mdi:fire");
  heatingControlMode.setName("Heating control mode");

  zone1.onTargetTemperatureCommand(onZone1TargetTemperatureCommand);
  zone1.setName("Zone 1");
  zone1.setMinTemp(5);
  zone1.setMaxTemp(60);
  zone1.setTempStep(1);
  zone1.setModes(HAHVAC::UnknownMode);

  zone2.onTargetTemperatureCommand(onZone2TargetTemperatureCommand);
  zone2.setName("Zone 2");
  zone2.setMinTemp(5);
  zone2.setMaxTemp(60);
  zone2.setTempStep(1);
  zone2.setModes(HAHVAC::UnknownMode);

  hotWater.onTargetTemperatureCommand(onhotWaterSetpointCommand);
  hotWater.setName("Hotwater");
  hotWater.setMinTemp(5);
  hotWater.setMaxTemp(60);
  hotWater.setTempStep(1);
  hotWater.setModes(HAHVAC::UnknownMode);

  zone1FlowTemperatureSetpoint.setIcon("mdi:thermometer");
  zone1FlowTemperatureSetpoint.setName("Zone 1 flow temperature setpoint");
  zone1FlowTemperatureSetpoint.setUnitOfMeasurement("°C");

  zone2FlowTemperatureSetpoint.setIcon("mdi:thermometer");
  zone2FlowTemperatureSetpoint.setName("Zone 2 flow temperature setpoint");
  zone2FlowTemperatureSetpoint.setUnitOfMeasurement("°C");

  consumedHeatingEnergy.setIcon("mdi:power-plug");
  consumedHeatingEnergy.setName("Consumed heating energy");
  consumedHeatingEnergy.setUnitOfMeasurement("kWh");

  consumedHotWaterEnergy.setIcon("mdi:power-plug");
  consumedHotWaterEnergy.setName("Consumed hot water energy");
  consumedHotWaterEnergy.setUnitOfMeasurement("kWh");

  deliveredHeatingEnergy.setIcon("mdi:power-plug");
  deliveredHeatingEnergy.setName("Delivered heating energy");
  deliveredHeatingEnergy.setUnitOfMeasurement("kWh");

  deliveredHotWaterEnergy.setIcon("mdi:power-plug");
  deliveredHotWaterEnergy.setName("Delivered hot water energy");
  deliveredHotWaterEnergy.setUnitOfMeasurement("kWh");

  compressorFrequency.setIcon("mdi:flash");
  compressorFrequency.setName("Compressor frequency");
  compressorFrequency.setUnitOfMeasurement("Hz");

  runHours.setIcon("mdi:timer");
  runHours.setName("Run hours");
  runHours.setUnitOfMeasurement("h");

  flowTempMax.setIcon("mdi:thermometer");
  flowTempMax.setName("Flow temperature max");
  flowTempMax.setUnitOfMeasurement("°C");

  flowTempMin.setIcon("mdi:thermometer");
  flowTempMin.setName("Flow temperature min");
  flowTempMin.setUnitOfMeasurement("°C");

  unknownMSG5.setIcon("mdi:alert");
  unknownMSG5.setName("Unknown MSG5");

  defrost.setIcon("mdi:snowflake-melt");
  defrost.setName("Defrost");

  DHWBoost.setIcon("mdi:fire");
  DHWBoost.setName("Hot Water Boost");
  DHWBoost.onCommand(onDHWBoostCommand);
}

void setup()
{
  byte mac[6];
  WiFi.macAddress(mac);
  device.setUniqueId(mac, sizeof(mac));
  HEATPUMP_STREAM.begin(SERIAL_BAUD, SERIAL_CONFIG); // Rx, Tx

  HeatPump.SetStream(&HEATPUMP_STREAM);

  WiFiManager MyWifiManager;
  Serial.begin(115200);
  // wifiManager.resetSettings(); //reset settings - for testing

  MyWifiManager.setTimeout(180);
  Serial.println("Starting Wifi Manager");
  if (!MyWifiManager.autoConnect("Ecodan Bridge AP"))
  {
    Serial.println("failed to connect and hit timeout");
    ESP.restart();
    delay(5000);
  }
  HostName = "EcodanBridge-";
  uint32_t chipID = ESP.getEfuseMac();
  HostName += String(chipID, HEX);
  WiFi.hostname(HostName);

  setupTelnet();

  OTASetup(HostName.c_str());

  device.setName("ESP32-EcodanBridge");
  device.setSoftwareVersion("1.0.0");

  setupSensors();

  mqtt.begin(BROKER_ADDR);
}