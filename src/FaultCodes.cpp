#include <string.h>
#include "FaultCodes.h"

struct FaultCodeEntry
{
  const char *code;
  const char *description;
};

// Ecodan FTC / outdoor unit codes as shown on the main remote controller
const FaultCodeEntry faultCodeTable[] = {
    {"8000", "No fault"},
    {"L3", "Circulation water temperature overheat protection"},
    {"L4", "DHW tank water temperature overheat protection"},
    {"L5", "Indoor unit temperature thermistor failure"},
    {"L6", "Circulation water freeze protection"},
    {"L8", "Heating operation error"},
    {"L9", "Low primary circuit flow rate"},
    {"LA", "Pressure sensor failure"},
    {"LB", "High pressure protection"},
    {"LC", "Boiler circulation water temperature overheat protection"},
    {"LD", "Boiler temperature thermistor failure"},
    {"LE", "Boiler operation error"},
    {"LF", "Flow sensor failure"},
    {"LH", "Boiler circulation water freeze protection"},
    {"LJ", "DHW operation error (external plate heat exchanger)"},
    {"LL", "FTC DIP switch setting error"},
    {"P1", "Room temperature thermistor (TH1) failure"},
    {"P2", "Refrigerant liquid temperature thermistor (TH2) failure"},
    {"P6", "Plate heat exchanger anti-freeze protection"},
    {"J0", "Communication failure between FTC and wireless receiver"},
    {"J1", "Wireless receiver / remote controller communication failure"},
    {"J2", "Wireless receiver / remote controller communication failure"},
    {"J3", "Wireless receiver / remote controller communication failure"},
    {"J4", "Wireless receiver / remote controller communication failure"},
    {"J5", "Wireless receiver / remote controller communication failure"},
    {"J6", "Wireless receiver / remote controller communication failure"},
    {"J7", "Wireless receiver / remote controller communication failure"},
    {"J8", "Wireless receiver / remote controller communication failure"},
    {"E0", "Main remote controller / FTC communication failure"},
    {"E1", "Main remote controller / FTC communication failure"},
    {"E2", "Main remote controller / FTC communication failure"},
    {"E3", "Main remote controller / FTC communication failure"},
    {"E4", "Main remote controller / FTC communication failure"},
    {"E5", "Main remote controller / FTC communication failure"},
    {"E6", "FTC / outdoor unit communication failure"},
    {"E7", "FTC / outdoor unit communication failure"},
    {"E8", "FTC / outdoor unit communication failure"},
    {"E9", "FTC / outdoor unit communication failure"},
    {"EA", "FTC / outdoor unit communication failure"},
    {"EB", "FTC / outdoor unit communication failure"},
    {"EC", "FTC / outdoor unit communication failure"},
    {"ED", "FTC / outdoor unit communication failure"},
    {"EE", "FTC / outdoor unit communication failure"},
    {"EF", "FTC / outdoor unit communication failure"},
    {"U1", "Outdoor: abnormal high pressure (63H operated)"},
    {"U2", "Outdoor: high discharge temperature / insufficient refrigerant"},
    {"U3", "Outdoor: discharge thermistor (TH4) open/short"},
    {"U4", "Outdoor: thermistor (TH3/TH6/TH7/TH8) open/short"},
    {"U5", "Outdoor: abnormal heat sink temperature"},
    {"U6", "Outdoor: power module abnormality"},
    {"U7", "Outdoor: abnormal superheat due to low discharge temperature"},
    {"U8", "Outdoor: fan motor abnormality"},
    {"U9", "Outdoor: abnormal supply voltage"},
    {"UF", "Outdoor: compressor overcurrent (compressor locked)"},
    {"UH", "Outdoor: current sensor error"},
    {"UL", "Outdoor: abnormal low pressure"},
    {"UP", "Outdoor: compressor overcurrent interruption"},
    {"F3", "Outdoor: 63L connector open"},
    {"F5", "Outdoor: 63H connector open"},
    {"F9", "Outdoor: 63H and 63L connectors open"},
};

const char *faultCodeDescription(const char *code)
{
  for (const FaultCodeEntry &entry : faultCodeTable)
  {
    if (strcmp(entry.code, code) == 0)
    {
      return entry.description;
    }
  }
  return "Unknown fault";
}

bool isFaultLetter(char value)
{
  return (value >= '0' && value <= '9') || (value >= 'A' && value <= 'Z');
}
