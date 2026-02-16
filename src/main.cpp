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
#include <math.h>
#include <string.h>

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
HASensorNumber inputPowerBand("inputPowerBand", HASensorNumber::PrecisionP0);
HASensorNumber heaterPower("heaterPower", HASensorNumber::PrecisionP0);
HASensorNumber totalInputEnergy("totalInputEnergy", HASensorNumber::PrecisionP1);
HASensorNumber dhwTempDropMode("dhwTempDropMode", HASensorNumber::PrecisionP0);
HASensorNumber primaryFlowRate("primaryFlowRate", HASensorNumber::PrecisionP0);
HASensorNumber externalBoilerFlowTemperature("externalBoilerFlowTemperature", HASensorNumber::PrecisionP1);
HASensorNumber externalBoilerReturnTemperature("externalBoilerReturnTemperature", HASensorNumber::PrecisionP1);
HASensorNumber zone1FlowTemperature("zone1FlowTemperature", HASensorNumber::PrecisionP1);
HASensorNumber zone1ReturnTemperature("zone1ReturnTemperature", HASensorNumber::PrecisionP1);
HASensorNumber zone2FlowTemperature("zone2FlowTemperature", HASensorNumber::PrecisionP1);
HASensorNumber zone2ReturnTemperature("zone2ReturnTemperature", HASensorNumber::PrecisionP1);
HASensorNumber mixingTankTemperature("mixingTankTemperature", HASensorNumber::PrecisionP1);
HASensorNumber condensingTemperature("condensingTemperature", HASensorNumber::PrecisionP1);
HASensorNumber thermistorUnknownTemperature("thermistorUnknownTemperature", HASensorNumber::PrecisionP1);
HASensorNumber outdoorDischargeTemperature("outdoorDischargeTemperature", HASensorNumber::PrecisionP0);
HASensorNumber outdoorLiquidPipeTemperature("outdoorLiquidPipeTemperature", HASensorNumber::PrecisionP1);
HASensorNumber outdoorTwoPhaseTemperature("outdoorTwoPhaseTemperature", HASensorNumber::PrecisionP1);
HASensorNumber outdoorSuctionTemperature("outdoorSuctionTemperature", HASensorNumber::PrecisionP1);
HASensorNumber outdoorHeatSinkTemperature("outdoorHeatSinkTemperature", HASensorNumber::PrecisionP0);
HASensorNumber outdoorCompressorSurfaceTemperature("outdoorCompressorSurfaceTemperature", HASensorNumber::PrecisionP0);
HASensorNumber superheat("superheat", HASensorNumber::PrecisionP0);
HASensorNumber subcooling("subcooling", HASensorNumber::PrecisionP1);
HASensorNumber refrigerantTemperature("refrigerantTemperature", HASensorNumber::PrecisionP1);
HASensorNumber unknownTemperature("unknownTemperature", HASensorNumber::PrecisionP1);
HASensorNumber protocolVersion("protocolVersion", HASensorNumber::PrecisionP0);
HASensorNumber modelVersion("modelVersion", HASensorNumber::PrecisionP0);
HASensorNumber supplyCapacity("supplyCapacity", HASensorNumber::PrecisionP0);
HASensorNumber ftcVersion("ftcVersion", HASensorNumber::PrecisionP0);
HASensorNumber zone1FlowTemperatureSetpoint("zone1FlowTemperatureSetpoint", HASensorNumber::PrecisionP1);
HASensorNumber zone2FlowTemperatureSetpoint("zone2FlowTemperatureSetpoint", HASensorNumber::PrecisionP1);
HASensorNumber consumedHeatingEnergy("consumedHeatingEnergy", HASensorNumber::PrecisionP1);
HASensorNumber consumedCoolingEnergy("consumedCoolingEnergy", HASensorNumber::PrecisionP1);
HASensorNumber consumedHotWaterEnergy("consumedHotWaterEnergy", HASensorNumber::PrecisionP1);
HASensorNumber deliveredHeatingEnergy("deliveredHeatingEnergy", HASensorNumber::PrecisionP1);
HASensorNumber deliveredCoolingEnergy("deliveredCoolingEnergy", HASensorNumber::PrecisionP1);
HASensorNumber deliveredHotWaterEnergy("deliveredHotWaterEnergy", HASensorNumber::PrecisionP1);
HASensorNumber hotWaterTemperature2("hotWaterTemperature2", HASensorNumber::PrecisionP1);
HASensorNumber faultCodeNumber("faultCodeNumber", HASensorNumber::PrecisionP0);
HASensorNumber refrigerantFaultCode("refrigerantFaultCode", HASensorNumber::PrecisionP0);
HASensorNumber multiZoneRunning("multiZoneRunning", HASensorNumber::PrecisionP0);
HASensorNumber faultStatus("faultStatus", HASensorNumber::PrecisionP0);
HASensorNumber compressorFrequency("compressorFrequency", HASensorNumber::PrecisionP0);
HASensorNumber runHours("runHours", HASensorNumber::PrecisionP0);
HASensorNumber flowTempMax("flowTempMax", HASensorNumber::PrecisionP1);
HASensorNumber flowTempMin("flowTempMin", HASensorNumber::PrecisionP1);
HASensorNumber mixingValveStatus("mixingValveStatus", HASensorNumber::PrecisionP0);

HABinarySensor hotWaterTimerActive("hotWaterTimerActive");
HABinarySensor defrost("defrost");
HABinarySensor holidayModeActive("holidayModeActive");
HABinarySensor forcedDHWActive("forcedDHWActive");
HABinarySensor serverControlModeActive("serverControlModeActive");
HABinarySensor thermostat1Active("thermostat1Active");
HABinarySensor thermostat2Active("thermostat2Active");
HABinarySensor outdoorThermostatActive("outdoorThermostatActive");
HABinarySensor boosterHeater1Active("boosterHeater1Active");
HABinarySensor boosterHeater2Active("boosterHeater2Active");
HABinarySensor immersionHeaterActive("immersionHeaterActive");
HABinarySensor pump1Active("pump1Active");
HABinarySensor pump2Active("pump2Active");
HABinarySensor pump3Active("pump3Active");
HABinarySensor valve1Active("valve1Active");
HABinarySensor valve2Active("valve2Active");

HASensor hotWaterControlMode("hotWaterControlMode");
HASensor systemOperationMode("systemOperationMode");
HASensor systemDateTime("systemDateTime");
HASensor consumedEnergyDateTime("consumedEnergyDateTime");
HASensor deliveredEnergyDateTime("deliveredEnergyDateTime");
HASensor heatSource("heatSource");
HASensor heatSourcePhase("heatSourcePhase");
HASensor errorCode("errorCode");
HASelect heatingControlMode("heatingControlMode");
HASwitch DHWBoost("DHWBoost");
HASelect systemPowerMode("systemPowerMode");

HAHVAC zone1(
    "zone1",
    HAHVAC::TargetTemperatureFeature | HAHVAC::ModesFeature);

HAHVAC zone2(
    "zone2",
    HAHVAC::TargetTemperatureFeature | HAHVAC::ModesFeature);

HAHVAC hotWater(
    "hotWater",
    HAHVAC::TargetTemperatureFeature | HAHVAC::ModesFeature);

namespace
{
bool updateNumberIfChanged(HASensorNumber &sensor, float value, float &last)
{
  if (isnan(last) || fabsf(value - last) > 0.01f)
  {
    sensor.setValue(value);
    last = value;
    return true;
  }
  return false;
}

bool updateUInt8IfChanged(HASensorNumber &sensor, uint8_t value, uint8_t &last)
{
  if (value != last)
  {
    sensor.setValue(value);
    last = value;
    return true;
  }
  return false;
}

bool updateBinaryIfChanged(HABinarySensor &sensor, bool value, bool &last)
{
  if (value != last)
  {
    sensor.setState(value);
    last = value;
    return true;
  }
  return false;
}

bool updateSelectIfChanged(HASelect &sensor, uint8_t value, uint8_t &last)
{
  if (value != last)
  {
    sensor.setState(value);
    last = value;
    return true;
  }
  return false;
}

bool updateTextIfChanged(HASensor &sensor, const char *value, char *last, size_t lastSize)
{
  if (strncmp(value, last, lastSize) != 0)
  {
    sensor.setValue(value);
    strncpy(last, value, lastSize - 1);
    last[lastSize - 1] = '\0';
    return true;
  }
  return false;
}

bool formatTimestamp(const tm &timestamp, char *buffer, size_t bufferSize)
{
  if (timestamp.tm_year == 0 && timestamp.tm_mon == 0 && timestamp.tm_mday == 0 &&
      timestamp.tm_hour == 0 && timestamp.tm_min == 0 && timestamp.tm_sec == 0)
  {
    return false;
  }

  int year = 2000 + timestamp.tm_year;
  snprintf(buffer, bufferSize, "%04d-%02d-%02d %02d:%02d:%02d",
           year,
           timestamp.tm_mon,
           timestamp.tm_mday,
           timestamp.tm_hour,
           timestamp.tm_min,
           timestamp.tm_sec);
  return true;
}

const char *heatSourceLabel(uint8_t value)
{
  switch (value)
  {
    case 0:
      return "Heatpump";
    case 1:
      return "Immersion";
    case 2:
      return "Booster";
    case 3:
      return "Booster+Immersion";
    case 4:
      return "Boiler";
    default:
      return "Unknown";
  }
}

const char *heatSourcePhaseLabel(uint8_t value)
{
  switch (value)
  {
    case 0:
      return "Normal";
    case 1:
      return "Heatpump";
    case 2:
      return "Heater";
    default:
      return "Unknown";
  }
}

char safeLetter(char value)
{
  if (value >= 32 && value <= 126)
  {
    return value;
  }
  return '-';
}
}

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
  static float lastCurrent = NAN;
  static float lastTarget = NAN;

  if (isnan(lastCurrent) || fabsf(HeatPump.Status.Zone1Temperature - lastCurrent) > 0.01f)
  {
    zone1.setCurrentTemperature(HeatPump.Status.Zone1Temperature);
    lastCurrent = HeatPump.Status.Zone1Temperature;
  }
  if (isnan(lastTarget) || fabsf(HeatPump.Status.Zone1TemperatureSetpoint - lastTarget) > 0.01f)
  {
    zone1.setTargetTemperature(HeatPump.Status.Zone1TemperatureSetpoint);
    lastTarget = HeatPump.Status.Zone1TemperatureSetpoint;
  }
}

void Zone2Report(void)
{
  static float lastCurrent = NAN;
  static float lastTarget = NAN;

  if (isnan(lastCurrent) || fabsf(HeatPump.Status.Zone2Temperature - lastCurrent) > 0.01f)
  {
    zone2.setCurrentTemperature(HeatPump.Status.Zone2Temperature);
    lastCurrent = HeatPump.Status.Zone2Temperature;
  }
  if (isnan(lastTarget) || fabsf(HeatPump.Status.Zone2TemperatureSetpoint - lastTarget) > 0.01f)
  {
    zone2.setTargetTemperature(HeatPump.Status.Zone2TemperatureSetpoint);
    lastTarget = HeatPump.Status.Zone2TemperatureSetpoint;
  }
}

void HotWaterReport(void)
{
  static float lastCurrent = NAN;
  static float lastTarget = NAN;
  static bool lastBoost = false;
  static bool lastTimerActive = false;
  static uint8_t lastControlMode = 0xFF;
  static float lastLegionella = NAN;
  static float lastDrop = NAN;

  if (isnan(lastCurrent) || fabsf(HeatPump.Status.HotWaterTemperature - lastCurrent) > 0.01f)
  {
    hotWater.setCurrentTemperature(HeatPump.Status.HotWaterTemperature);
    lastCurrent = HeatPump.Status.HotWaterTemperature;
  }
  if (isnan(lastTarget) || fabsf(HeatPump.Status.HotWaterSetpoint - lastTarget) > 0.01f)
  {
    hotWater.setTargetTemperature(HeatPump.Status.HotWaterSetpoint);
    lastTarget = HeatPump.Status.HotWaterSetpoint;
  }

  if (HeatPump.Status.HotWaterBoostActive != lastBoost)
  {
    DHWBoost.setState(HeatPump.Status.HotWaterBoostActive);
    lastBoost = HeatPump.Status.HotWaterBoostActive;
  }
  updateBinaryIfChanged(hotWaterTimerActive, HeatPump.Status.HotWaterTimerActive, lastTimerActive);
  if (HeatPump.Status.HotWaterControlMode != lastControlMode)
  {
    hotWaterControlMode.setValue(HowWaterControlModeString[HeatPump.Status.HotWaterControlMode]);
    lastControlMode = HeatPump.Status.HotWaterControlMode;
  }
  updateNumberIfChanged(legionellaSetpoint, HeatPump.Status.LegionellaSetpoint, lastLegionella);
  updateNumberIfChanged(hotWaterMaximumTempDrop, HeatPump.Status.HotWaterMaximumTempDrop, lastDrop);
}

void SystemReport(void)
{
  static float lastOutputFlow = NAN;
  static float lastReturnFlow = NAN;
  static float lastFlowSetpoint = NAN;
  static uint8_t lastOutputPower = 0xFF;
  static uint8_t lastSystemPowerMode = 0xFF;
  static uint8_t lastSystemOperationMode = 0xFF;
  static uint8_t lastHeatingControlMode = 0xFF;
  static uint8_t lastPrimaryFlow = 0xFF;
  static float lastOutsideTemp = NAN;
  static float lastExternalBoilerFlow = NAN;
  static float lastExternalBoilerReturn = NAN;
  static float lastZone1FlowTemp = NAN;
  static float lastZone1ReturnTemp = NAN;
  static float lastZone2FlowTemp = NAN;
  static float lastZone2ReturnTemp = NAN;
  static float lastMixingTankTemp = NAN;
  static float lastCondensingTemp = NAN;
  static float lastThermistorUnknownTemp = NAN;
  static float lastOutdoorDischargeTemp = NAN;
  static float lastOutdoorLiquidPipeTemp = NAN;
  static float lastOutdoorTwoPhaseTemp = NAN;
  static float lastOutdoorSuctionTemp = NAN;
  static float lastOutdoorHeatSinkTemp = NAN;
  static float lastOutdoorCompressorSurfaceTemp = NAN;
  static float lastSuperheat = NAN;
  static float lastSubcooling = NAN;
  static float lastRefrigerantTemp = NAN;
  static uint8_t lastProtocolVersion = 0xFF;
  static uint8_t lastModelVersion = 0xFF;
  static uint8_t lastSupplyCapacity = 0xFF;
  static uint8_t lastFtcVersion = 0xFF;
  static float lastUnknownTemp = NAN;
  static float lastHotWaterTemp2 = NAN;
  static uint8_t lastInputPowerBand = 0xFF;
  static uint8_t lastHeaterPower = 0xFF;
  static float lastTotalInputEnergy = NAN;
  static uint8_t lastDhwTempDropMode = 0xFF;
  static char lastHeatSource[24] = "";
  static char lastHeatSourcePhase[16] = "";
  static char lastSystemTimestamp[20] = "";
  char timestamp[20];

  updateNumberIfChanged(heaterOutputFlowTemperature, HeatPump.Status.HeaterOutputFlowTemperature, lastOutputFlow);
  updateNumberIfChanged(heaterReturnFlowTemperature, HeatPump.Status.HeaterReturnFlowTemperature, lastReturnFlow);
  updateNumberIfChanged(heaterFlowSetpoint, HeatPump.Status.HeaterFlowSetpoint, lastFlowSetpoint);
  updateNumberIfChanged(externalBoilerFlowTemperature, HeatPump.Status.ExternalBoilerFlowTemperature, lastExternalBoilerFlow);
  updateNumberIfChanged(externalBoilerReturnTemperature, HeatPump.Status.ExternalBoilerReturnTemperature, lastExternalBoilerReturn);
  updateNumberIfChanged(zone1FlowTemperature, HeatPump.Status.Zone1FlowTemperature, lastZone1FlowTemp);
  updateNumberIfChanged(zone1ReturnTemperature, HeatPump.Status.Zone1ReturnTemperature, lastZone1ReturnTemp);
  updateNumberIfChanged(zone2FlowTemperature, HeatPump.Status.Zone2FlowTemperature, lastZone2FlowTemp);
  updateNumberIfChanged(zone2ReturnTemperature, HeatPump.Status.Zone2ReturnTemperature, lastZone2ReturnTemp);
  updateNumberIfChanged(mixingTankTemperature, HeatPump.Status.MixingTankTemperature, lastMixingTankTemp);
  updateNumberIfChanged(condensingTemperature, HeatPump.Status.CondensingTemperature, lastCondensingTemp);
  updateNumberIfChanged(thermistorUnknownTemperature, HeatPump.Status.ThermistorUnknownTemperature, lastThermistorUnknownTemp);
  updateNumberIfChanged(outdoorDischargeTemperature, HeatPump.Status.OutdoorDischargeTemperature, lastOutdoorDischargeTemp);
  updateNumberIfChanged(outdoorLiquidPipeTemperature, HeatPump.Status.OutdoorLiquidPipeTemperature, lastOutdoorLiquidPipeTemp);
  updateNumberIfChanged(outdoorTwoPhaseTemperature, HeatPump.Status.OutdoorTwoPhaseTemperature, lastOutdoorTwoPhaseTemp);
  updateNumberIfChanged(outdoorSuctionTemperature, HeatPump.Status.OutdoorSuctionTemperature, lastOutdoorSuctionTemp);
  updateNumberIfChanged(outdoorHeatSinkTemperature, HeatPump.Status.OutdoorHeatSinkTemperature, lastOutdoorHeatSinkTemp);
  updateNumberIfChanged(outdoorCompressorSurfaceTemperature, HeatPump.Status.OutdoorCompressorSurfaceTemperature, lastOutdoorCompressorSurfaceTemp);
  updateNumberIfChanged(superheat, HeatPump.Status.Superheat, lastSuperheat);
  updateNumberIfChanged(subcooling, HeatPump.Status.Subcooling, lastSubcooling);
  updateNumberIfChanged(refrigerantTemperature, HeatPump.Status.RefrigerantTemperature, lastRefrigerantTemp);
  updateUInt8IfChanged(protocolVersion, HeatPump.Status.ProtocolVersion, lastProtocolVersion);
  updateUInt8IfChanged(modelVersion, HeatPump.Status.ModelVersion, lastModelVersion);
  updateUInt8IfChanged(supplyCapacity, HeatPump.Status.SupplyCapacity, lastSupplyCapacity);
  updateUInt8IfChanged(ftcVersion, HeatPump.Status.FtcVersion, lastFtcVersion);
  updateNumberIfChanged(unknownTemperature, HeatPump.Status.UnknownTemperature, lastUnknownTemp);
  updateNumberIfChanged(hotWaterTemperature2, HeatPump.Status.HotWaterTemperature2, lastHotWaterTemp2);
  updateUInt8IfChanged(outputPower, HeatPump.Status.OutputPower, lastOutputPower);
  updateUInt8IfChanged(inputPowerBand, HeatPump.Status.InputPowerBand, lastInputPowerBand);
  updateUInt8IfChanged(heaterPower, HeatPump.Status.HeaterPower, lastHeaterPower);
  updateNumberIfChanged(totalInputEnergy, HeatPump.Status.TotalInputEnergy, lastTotalInputEnergy);
  updateUInt8IfChanged(dhwTempDropMode, HeatPump.Status.DhwTempDropMode, lastDhwTempDropMode);
  updateTextIfChanged(heatSource, heatSourceLabel(HeatPump.Status.HeatSource), lastHeatSource, sizeof(lastHeatSource));
  updateTextIfChanged(heatSourcePhase, heatSourcePhaseLabel(HeatPump.Status.HeatSourcePhase), lastHeatSourcePhase, sizeof(lastHeatSourcePhase));
  updateSelectIfChanged(systemPowerMode, HeatPump.Status.SystemPowerMode, lastSystemPowerMode);
  if (HeatPump.Status.SystemOperationMode != lastSystemOperationMode)
  {
    systemOperationMode.setValue(SystemOperationModeString[HeatPump.Status.SystemOperationMode]);
    lastSystemOperationMode = HeatPump.Status.SystemOperationMode;
  }
  updateSelectIfChanged(heatingControlMode, HeatPump.Status.HeatingControlMode, lastHeatingControlMode);
  updateUInt8IfChanged(primaryFlowRate, HeatPump.Status.PrimaryFlowRate, lastPrimaryFlow);
  updateNumberIfChanged(outsideTemp, HeatPump.Status.OutsideTemperature, lastOutsideTemp);
  if (formatTimestamp(HeatPump.Status.DateTimeStamp, timestamp, sizeof(timestamp)))
  {
    updateTextIfChanged(systemDateTime, timestamp, lastSystemTimestamp, sizeof(lastSystemTimestamp));
  }
}

void TestReport(void)
{
  static float lastZone1Flow = NAN;
  static float lastZone2Flow = NAN;
  static float lastConsumedHeating = NAN;
  static float lastConsumedCooling = NAN;
  static float lastConsumedHotWater = NAN;
  static float lastDeliveredHeating = NAN;
  static float lastDeliveredCooling = NAN;
  static float lastDeliveredHotWater = NAN;
  static uint8_t lastCompressorFrequency = 0xFF;
  static uint32_t lastRunHours = 0xFFFFFFFF;
  static float lastFlowTempMax = NAN;
  static float lastFlowTempMin = NAN;
  static bool lastDefrost = false;
  static bool lastHolidayMode = false;
  static bool lastForcedDHW = false;
  static bool lastServerControlMode = false;
  static bool lastThermostat1 = false;
  static bool lastThermostat2 = false;
  static bool lastOutdoorThermostat = false;
  static bool lastBoosterHeater1 = false;
  static bool lastBoosterHeater2 = false;
  static bool lastImmersionHeater = false;
  static bool lastPump1 = false;
  static bool lastPump2 = false;
  static bool lastPump3 = false;
  static bool lastValve1 = false;
  static bool lastValve2 = false;
  static uint8_t lastMixingValveStatus = 0xFF;
  static uint16_t lastFaultCodeNumber = 0xFFFF;
  static uint8_t lastRefrigerantFaultCode = 0xFF;
  static uint8_t lastMultiZoneRunning = 0xFF;
  static uint8_t lastFaultStatus = 0xFF;
  static char lastErrorCode[24] = "";
  static char lastConsumedTimestamp[20] = "";
  static char lastDeliveredTimestamp[20] = "";
  char timestamp[20];
  char errorText[24];

  updateNumberIfChanged(zone1FlowTemperatureSetpoint, HeatPump.Status.Zone1FlowTemperatureSetpoint, lastZone1Flow);
  updateNumberIfChanged(zone2FlowTemperatureSetpoint, HeatPump.Status.Zone2FlowTemperatureSetpoint, lastZone2Flow);
  updateNumberIfChanged(consumedHeatingEnergy, HeatPump.Status.ConsumedHeatingEnergy, lastConsumedHeating);
  updateNumberIfChanged(consumedCoolingEnergy, HeatPump.Status.ConsumedCoolingEnergy, lastConsumedCooling);
  updateNumberIfChanged(consumedHotWaterEnergy, HeatPump.Status.ConsumedHotWaterEnergy, lastConsumedHotWater);
  updateNumberIfChanged(deliveredHeatingEnergy, HeatPump.Status.DeliveredHeatingEnergy, lastDeliveredHeating);
  updateNumberIfChanged(deliveredCoolingEnergy, HeatPump.Status.DeliveredCoolingEnergy, lastDeliveredCooling);
  updateNumberIfChanged(deliveredHotWaterEnergy, HeatPump.Status.DeliveredHotWaterEnergy, lastDeliveredHotWater);
  updateUInt8IfChanged(compressorFrequency, HeatPump.Status.CompressorFrequency, lastCompressorFrequency);
  if (HeatPump.Status.RunHours != lastRunHours)
  {
    runHours.setValue(HeatPump.Status.RunHours);
    lastRunHours = HeatPump.Status.RunHours;
  }
  updateNumberIfChanged(flowTempMax, HeatPump.Status.FlowTempMax, lastFlowTempMax);
  updateNumberIfChanged(flowTempMin, HeatPump.Status.FlowTempMin, lastFlowTempMin);
  updateBinaryIfChanged(defrost, HeatPump.Status.Defrost, lastDefrost);
  updateBinaryIfChanged(holidayModeActive, HeatPump.Status.HolidayModeActive, lastHolidayMode);
  updateBinaryIfChanged(forcedDHWActive, HeatPump.Status.ForcedDHWActive, lastForcedDHW);
  updateBinaryIfChanged(serverControlModeActive, HeatPump.Status.ServerControlModeActive, lastServerControlMode);
  updateBinaryIfChanged(thermostat1Active, HeatPump.Status.Thermostat1Active, lastThermostat1);
  updateBinaryIfChanged(thermostat2Active, HeatPump.Status.Thermostat2Active, lastThermostat2);
  updateBinaryIfChanged(outdoorThermostatActive, HeatPump.Status.OutdoorThermostatActive, lastOutdoorThermostat);
  updateBinaryIfChanged(boosterHeater1Active, HeatPump.Status.BoosterHeater1Active, lastBoosterHeater1);
  updateBinaryIfChanged(boosterHeater2Active, HeatPump.Status.BoosterHeater2Active, lastBoosterHeater2);
  updateBinaryIfChanged(immersionHeaterActive, HeatPump.Status.ImmersionHeaterActive, lastImmersionHeater);
  updateBinaryIfChanged(pump1Active, HeatPump.Status.Pump1Active, lastPump1);
  updateBinaryIfChanged(pump2Active, HeatPump.Status.Pump2Active, lastPump2);
  updateBinaryIfChanged(pump3Active, HeatPump.Status.Pump3Active, lastPump3);
  updateBinaryIfChanged(valve1Active, HeatPump.Status.Valve1Active, lastValve1);
  updateBinaryIfChanged(valve2Active, HeatPump.Status.Valve2Active, lastValve2);
  if (HeatPump.Status.MixingValveStatus != lastMixingValveStatus)
  {
    mixingValveStatus.setValue(HeatPump.Status.MixingValveStatus);
    lastMixingValveStatus = HeatPump.Status.MixingValveStatus;
  }
  if (HeatPump.Status.FaultCodeNumber != lastFaultCodeNumber)
  {
    faultCodeNumber.setValue(HeatPump.Status.FaultCodeNumber);
    lastFaultCodeNumber = HeatPump.Status.FaultCodeNumber;
  }
  updateUInt8IfChanged(refrigerantFaultCode, HeatPump.Status.RefrigerantFaultCode, lastRefrigerantFaultCode);
  updateUInt8IfChanged(multiZoneRunning, HeatPump.Status.MultiZoneRunning, lastMultiZoneRunning);
  updateUInt8IfChanged(faultStatus, HeatPump.Status.FaultStatus, lastFaultStatus);
  if (HeatPump.Status.FaultCodeNumber == 0 &&
      safeLetter(HeatPump.Status.FaultCodeLetter1) == '-' &&
      safeLetter(HeatPump.Status.FaultCodeLetter2) == '-')
  {
    strncpy(errorText, "OK", sizeof(errorText));
    errorText[sizeof(errorText) - 1] = '\0';
  }
  else
  {
    snprintf(errorText,
             sizeof(errorText),
             "%u%c%c",
             HeatPump.Status.FaultCodeNumber,
             safeLetter(HeatPump.Status.FaultCodeLetter1),
             safeLetter(HeatPump.Status.FaultCodeLetter2));
  }
  updateTextIfChanged(errorCode, errorText, lastErrorCode, sizeof(lastErrorCode));
  if (formatTimestamp(HeatPump.Status.ConsumedDateTimeStamp, timestamp, sizeof(timestamp)))
  {
    updateTextIfChanged(consumedEnergyDateTime, timestamp, lastConsumedTimestamp, sizeof(lastConsumedTimestamp));
  }
  if (formatTimestamp(HeatPump.Status.DeliveredDateTimeStamp, timestamp, sizeof(timestamp)))
  {
    updateTextIfChanged(deliveredEnergyDateTime, timestamp, lastDeliveredTimestamp, sizeof(lastDeliveredTimestamp));
  }
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
  if (index < 0 || index >= 6)
  {
    return;
  }
  String command(HeatingControlModeString[index]);
  HeatPump.SetHeatingControlMode(&command, BOTH);
  sender->setState(index); // report the selected option back to the HA panel
}

void onDHWBoostCommand(bool state, HASwitch *sender)
{
  HeatPump.ForceDHW(state);
  sender->setState(state);
}

void setupSensors()
{
  outsideTemp.setIcon("mdi:thermometer");
  outsideTemp.setName("Temperature outside");
  outsideTemp.setUnitOfMeasurement("°C");
  outsideTemp.setDeviceClass("temperature");
  outsideTemp.setStateClass("measurement");

  hotWaterTimerActive.setIcon("mdi:timer");
  hotWaterTimerActive.setName("Hot water timer active");

  hotWaterControlMode.setIcon("mdi:home");
  hotWaterControlMode.setName("Hot water control mode");

  legionellaSetpoint.setIcon("mdi:thermometer");
  legionellaSetpoint.setName("Legionella setpoint");
  legionellaSetpoint.setUnitOfMeasurement("°C");
  legionellaSetpoint.setDeviceClass("temperature");
  legionellaSetpoint.setStateClass("measurement");

  hotWaterMaximumTempDrop.setIcon("mdi:thermometer");
  hotWaterMaximumTempDrop.setName("Hot water maximum temperature drop");
  hotWaterMaximumTempDrop.setUnitOfMeasurement("°C");
  hotWaterMaximumTempDrop.setDeviceClass("temperature");
  hotWaterMaximumTempDrop.setStateClass("measurement");

  heaterOutputFlowTemperature.setIcon("mdi:thermometer");
  heaterOutputFlowTemperature.setName("Heater output flow temperature");
  heaterOutputFlowTemperature.setUnitOfMeasurement("°C");
  heaterOutputFlowTemperature.setDeviceClass("temperature");
  heaterOutputFlowTemperature.setStateClass("measurement");

  heaterReturnFlowTemperature.setIcon("mdi:thermometer");
  heaterReturnFlowTemperature.setName("Heater return flow temperature");
  heaterReturnFlowTemperature.setUnitOfMeasurement("°C");
  heaterReturnFlowTemperature.setDeviceClass("temperature");
  heaterReturnFlowTemperature.setStateClass("measurement");

  heaterFlowSetpoint.setIcon("mdi:thermometer");
  heaterFlowSetpoint.setName("Heater flow setpoint");
  heaterFlowSetpoint.setUnitOfMeasurement("°C");
  heaterFlowSetpoint.setDeviceClass("temperature");
  heaterFlowSetpoint.setStateClass("measurement");

  inputPowerBand.setIcon("mdi:flash");
  inputPowerBand.setName("Input power band");
  inputPowerBand.setUnitOfMeasurement("kW");
  inputPowerBand.setDeviceClass("power");
  inputPowerBand.setStateClass("measurement");

  heaterPower.setIcon("mdi:flash");
  heaterPower.setName("Heater power");
  heaterPower.setUnitOfMeasurement("kW");
  heaterPower.setDeviceClass("power");
  heaterPower.setStateClass("measurement");

  totalInputEnergy.setIcon("mdi:counter");
  totalInputEnergy.setName("Total input energy");
  totalInputEnergy.setUnitOfMeasurement("kWh");
  totalInputEnergy.setDeviceClass("energy");
  totalInputEnergy.setStateClass("total_increasing");

  dhwTempDropMode.setIcon("mdi:water");
  dhwTempDropMode.setName("DHW temp drop mode");

  externalBoilerFlowTemperature.setIcon("mdi:thermometer");
  externalBoilerFlowTemperature.setName("External boiler flow temperature");
  externalBoilerFlowTemperature.setUnitOfMeasurement("°C");
  externalBoilerFlowTemperature.setDeviceClass("temperature");
  externalBoilerFlowTemperature.setStateClass("measurement");

  externalBoilerReturnTemperature.setIcon("mdi:thermometer");
  externalBoilerReturnTemperature.setName("External boiler return temperature");
  externalBoilerReturnTemperature.setUnitOfMeasurement("°C");
  externalBoilerReturnTemperature.setDeviceClass("temperature");
  externalBoilerReturnTemperature.setStateClass("measurement");

  zone1FlowTemperature.setIcon("mdi:thermometer");
  zone1FlowTemperature.setName("Zone 1 flow temperature");
  zone1FlowTemperature.setUnitOfMeasurement("°C");
  zone1FlowTemperature.setDeviceClass("temperature");
  zone1FlowTemperature.setStateClass("measurement");

  zone1ReturnTemperature.setIcon("mdi:thermometer");
  zone1ReturnTemperature.setName("Zone 1 return temperature");
  zone1ReturnTemperature.setUnitOfMeasurement("°C");
  zone1ReturnTemperature.setDeviceClass("temperature");
  zone1ReturnTemperature.setStateClass("measurement");

  zone2FlowTemperature.setIcon("mdi:thermometer");
  zone2FlowTemperature.setName("Zone 2 flow temperature");
  zone2FlowTemperature.setUnitOfMeasurement("°C");
  zone2FlowTemperature.setDeviceClass("temperature");
  zone2FlowTemperature.setStateClass("measurement");

  zone2ReturnTemperature.setIcon("mdi:thermometer");
  zone2ReturnTemperature.setName("Zone 2 return temperature");
  zone2ReturnTemperature.setUnitOfMeasurement("°C");
  zone2ReturnTemperature.setDeviceClass("temperature");
  zone2ReturnTemperature.setStateClass("measurement");

  mixingTankTemperature.setIcon("mdi:thermometer");
  mixingTankTemperature.setName("Mixing tank temperature");
  mixingTankTemperature.setUnitOfMeasurement("°C");
  mixingTankTemperature.setDeviceClass("temperature");
  mixingTankTemperature.setStateClass("measurement");

  condensingTemperature.setIcon("mdi:thermometer");
  condensingTemperature.setName("Condensing temperature");
  condensingTemperature.setUnitOfMeasurement("°C");
  condensingTemperature.setDeviceClass("temperature");
  condensingTemperature.setStateClass("measurement");

  thermistorUnknownTemperature.setIcon("mdi:thermometer");
  thermistorUnknownTemperature.setName("Thermistor unknown temperature");
  thermistorUnknownTemperature.setUnitOfMeasurement("°C");
  thermistorUnknownTemperature.setDeviceClass("temperature");
  thermistorUnknownTemperature.setStateClass("measurement");

  outdoorDischargeTemperature.setIcon("mdi:thermometer");
  outdoorDischargeTemperature.setName("Outdoor discharge temperature");
  outdoorDischargeTemperature.setUnitOfMeasurement("°C");
  outdoorDischargeTemperature.setDeviceClass("temperature");
  outdoorDischargeTemperature.setStateClass("measurement");

  outdoorLiquidPipeTemperature.setIcon("mdi:thermometer");
  outdoorLiquidPipeTemperature.setName("Outdoor liquid pipe temperature");
  outdoorLiquidPipeTemperature.setUnitOfMeasurement("°C");
  outdoorLiquidPipeTemperature.setDeviceClass("temperature");
  outdoorLiquidPipeTemperature.setStateClass("measurement");

  outdoorTwoPhaseTemperature.setIcon("mdi:thermometer");
  outdoorTwoPhaseTemperature.setName("Outdoor two-phase temperature");
  outdoorTwoPhaseTemperature.setUnitOfMeasurement("°C");
  outdoorTwoPhaseTemperature.setDeviceClass("temperature");
  outdoorTwoPhaseTemperature.setStateClass("measurement");

  outdoorSuctionTemperature.setIcon("mdi:thermometer");
  outdoorSuctionTemperature.setName("Outdoor suction temperature");
  outdoorSuctionTemperature.setUnitOfMeasurement("°C");
  outdoorSuctionTemperature.setDeviceClass("temperature");
  outdoorSuctionTemperature.setStateClass("measurement");

  outdoorHeatSinkTemperature.setIcon("mdi:thermometer");
  outdoorHeatSinkTemperature.setName("Outdoor heat sink temperature");
  outdoorHeatSinkTemperature.setUnitOfMeasurement("°C");
  outdoorHeatSinkTemperature.setDeviceClass("temperature");
  outdoorHeatSinkTemperature.setStateClass("measurement");

  outdoorCompressorSurfaceTemperature.setIcon("mdi:thermometer");
  outdoorCompressorSurfaceTemperature.setName("Outdoor compressor surface temperature");
  outdoorCompressorSurfaceTemperature.setUnitOfMeasurement("°C");
  outdoorCompressorSurfaceTemperature.setDeviceClass("temperature");
  outdoorCompressorSurfaceTemperature.setStateClass("measurement");

  superheat.setIcon("mdi:thermometer");
  superheat.setName("Superheat");
  superheat.setUnitOfMeasurement("°C");
  superheat.setDeviceClass("temperature");
  superheat.setStateClass("measurement");

  subcooling.setIcon("mdi:thermometer");
  subcooling.setName("Subcooling");
  subcooling.setUnitOfMeasurement("°C");
  subcooling.setDeviceClass("temperature");
  subcooling.setStateClass("measurement");

  protocolVersion.setIcon("mdi:chip");
  protocolVersion.setName("Protocol version");

  modelVersion.setIcon("mdi:chip");
  modelVersion.setName("Model version");

  supplyCapacity.setIcon("mdi:gauge");
  supplyCapacity.setName("Supply capacity");

  ftcVersion.setIcon("mdi:chip");
  ftcVersion.setName("FTC version");

  refrigerantTemperature.setIcon("mdi:thermometer");
  refrigerantTemperature.setName("Refrigerant temperature");
  refrigerantTemperature.setUnitOfMeasurement("°C");
  refrigerantTemperature.setDeviceClass("temperature");
  refrigerantTemperature.setStateClass("measurement");

  unknownTemperature.setIcon("mdi:thermometer");
  unknownTemperature.setName("Unknown temperature");
  unknownTemperature.setUnitOfMeasurement("°C");
  unknownTemperature.setDeviceClass("temperature");
  unknownTemperature.setStateClass("measurement");

  outputPower.setIcon("mdi:power-plug");
  outputPower.setName("Output power");
  outputPower.setUnitOfMeasurement("W");
  outputPower.setDeviceClass("power");
  outputPower.setStateClass("measurement");

  primaryFlowRate.setIcon("mdi:water-pump");
  primaryFlowRate.setName("Primary flow rate");
  primaryFlowRate.setUnitOfMeasurement("L/min");
  primaryFlowRate.setStateClass("measurement");

  systemOperationMode.setIcon("mdi:home");
  systemOperationMode.setName("System operation mode");

  systemDateTime.setIcon("mdi:clock-outline");
  systemDateTime.setName("System time");

  heatSource.setIcon("mdi:fire");
  heatSource.setName("Heat source");

  heatSourcePhase.setIcon("mdi:thermometer");
  heatSourcePhase.setName("Heat source phase");

  errorCode.setIcon("mdi:alert");
  errorCode.setName("Fault code");

  faultCodeNumber.setIcon("mdi:alert");
  faultCodeNumber.setName("Fault code number");

  refrigerantFaultCode.setIcon("mdi:alert");
  refrigerantFaultCode.setName("Refrigerant fault code");

  multiZoneRunning.setIcon("mdi:home-group");
  multiZoneRunning.setName("Multi-zone running");

  faultStatus.setIcon("mdi:alert");
  faultStatus.setName("Fault status");

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
  zone1FlowTemperatureSetpoint.setDeviceClass("temperature");
  zone1FlowTemperatureSetpoint.setStateClass("measurement");

  zone2FlowTemperatureSetpoint.setIcon("mdi:thermometer");
  zone2FlowTemperatureSetpoint.setName("Zone 2 flow temperature setpoint");
  zone2FlowTemperatureSetpoint.setUnitOfMeasurement("°C");
  zone2FlowTemperatureSetpoint.setDeviceClass("temperature");
  zone2FlowTemperatureSetpoint.setStateClass("measurement");

  consumedHeatingEnergy.setIcon("mdi:power-plug");
  consumedHeatingEnergy.setName("Consumed heating energy");
  consumedHeatingEnergy.setUnitOfMeasurement("kWh");
  consumedHeatingEnergy.setDeviceClass("energy");
  consumedHeatingEnergy.setStateClass("total_increasing");

  consumedCoolingEnergy.setIcon("mdi:power-plug");
  consumedCoolingEnergy.setName("Consumed cooling energy");
  consumedCoolingEnergy.setUnitOfMeasurement("kWh");
  consumedCoolingEnergy.setDeviceClass("energy");
  consumedCoolingEnergy.setStateClass("total_increasing");

  consumedHotWaterEnergy.setIcon("mdi:power-plug");
  consumedHotWaterEnergy.setName("Consumed hot water energy");
  consumedHotWaterEnergy.setUnitOfMeasurement("kWh");
  consumedHotWaterEnergy.setDeviceClass("energy");
  consumedHotWaterEnergy.setStateClass("total_increasing");

  deliveredHeatingEnergy.setIcon("mdi:power-plug");
  deliveredHeatingEnergy.setName("Delivered heating energy");
  deliveredHeatingEnergy.setUnitOfMeasurement("kWh");
  deliveredHeatingEnergy.setDeviceClass("energy");
  deliveredHeatingEnergy.setStateClass("total_increasing");

  deliveredCoolingEnergy.setIcon("mdi:power-plug");
  deliveredCoolingEnergy.setName("Delivered cooling energy");
  deliveredCoolingEnergy.setUnitOfMeasurement("kWh");
  deliveredCoolingEnergy.setDeviceClass("energy");
  deliveredCoolingEnergy.setStateClass("total_increasing");

  deliveredHotWaterEnergy.setIcon("mdi:power-plug");
  deliveredHotWaterEnergy.setName("Delivered hot water energy");
  deliveredHotWaterEnergy.setUnitOfMeasurement("kWh");
  deliveredHotWaterEnergy.setDeviceClass("energy");
  deliveredHotWaterEnergy.setStateClass("total_increasing");

  hotWaterTemperature2.setIcon("mdi:thermometer");
  hotWaterTemperature2.setName("Hot water temperature 2");
  hotWaterTemperature2.setUnitOfMeasurement("°C");
  hotWaterTemperature2.setDeviceClass("temperature");
  hotWaterTemperature2.setStateClass("measurement");

  consumedEnergyDateTime.setIcon("mdi:clock-outline");
  consumedEnergyDateTime.setName("Consumed energy timestamp");

  deliveredEnergyDateTime.setIcon("mdi:clock-outline");
  deliveredEnergyDateTime.setName("Delivered energy timestamp");

  compressorFrequency.setIcon("mdi:flash");
  compressorFrequency.setName("Compressor frequency");
  compressorFrequency.setUnitOfMeasurement("Hz");
  compressorFrequency.setStateClass("measurement");

  runHours.setIcon("mdi:timer");
  runHours.setName("Run hours");
  runHours.setUnitOfMeasurement("h");

  flowTempMax.setIcon("mdi:thermometer");
  flowTempMax.setName("Flow temperature max");
  flowTempMax.setUnitOfMeasurement("°C");
  flowTempMax.setDeviceClass("temperature");
  flowTempMax.setStateClass("measurement");

  flowTempMin.setIcon("mdi:thermometer");
  flowTempMin.setName("Flow temperature min");
  flowTempMin.setUnitOfMeasurement("°C");
  flowTempMin.setDeviceClass("temperature");
  flowTempMin.setStateClass("measurement");

  mixingValveStatus.setIcon("mdi:valve");
  mixingValveStatus.setName("Mixing valve status");

  defrost.setIcon("mdi:snowflake-melt");
  defrost.setName("Defrost");

  holidayModeActive.setIcon("mdi:beach");
  holidayModeActive.setName("Holiday mode active");

  forcedDHWActive.setIcon("mdi:water-boiler");
  forcedDHWActive.setName("Forced DHW active");

  serverControlModeActive.setIcon("mdi:server");
  serverControlModeActive.setName("Server control mode active");

  thermostat1Active.setIcon("mdi:thermostat");
  thermostat1Active.setName("Thermostat 1 active");

  thermostat2Active.setIcon("mdi:thermostat");
  thermostat2Active.setName("Thermostat 2 active");

  outdoorThermostatActive.setIcon("mdi:thermostat");
  outdoorThermostatActive.setName("Outdoor thermostat active");

  boosterHeater1Active.setIcon("mdi:fire");
  boosterHeater1Active.setName("Booster heater 1 active");

  boosterHeater2Active.setIcon("mdi:fire");
  boosterHeater2Active.setName("Booster heater 2 active");

  immersionHeaterActive.setIcon("mdi:water-boiler");
  immersionHeaterActive.setName("Immersion heater active");

  pump1Active.setIcon("mdi:water-pump");
  pump1Active.setName("Pump 1 active");

  pump2Active.setIcon("mdi:water-pump");
  pump2Active.setName("Pump 2 active");

  pump3Active.setIcon("mdi:water-pump");
  pump3Active.setName("Pump 3 active");

  valve1Active.setIcon("mdi:valve");
  valve1Active.setName("Valve 1 active");

  valve2Active.setIcon("mdi:valve");
  valve2Active.setName("Valve 2 active");

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

  HostName = "EcodanBridge-";
  uint32_t chipID = ESP.getEfuseMac();
  HostName += String(chipID, HEX);
  WiFi.hostname(HostName);

  MyWifiManager.setTimeout(180);
  Serial.println("Starting Wifi Manager");
  if (!MyWifiManager.autoConnect("Ecodan Bridge AP"))
  {
    Serial.println("failed to connect and hit timeout");
    ESP.restart();
    delay(5000);
  }

  setupTelnet();

  OTASetup(HostName.c_str());

  device.setName("ESP32-EcodanBridge");
  device.setSoftwareVersion("1.0.0");

  setupSensors();

  mqtt.begin(BROKER_ADDR);
}