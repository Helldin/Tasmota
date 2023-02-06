/*
  xnrg_10_sdm630.ino - Eastron SDM630-Modbus energy meter support for Tasmota

  Copyright (C) 2019  Gennaro Tortone, Theo Arends and Pablo Zerón

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

#ifdef USE_ENERGY_SENSOR
#ifdef USE_SDM630
/*********************************************************************************************\
 * Eastron SDM630-Modbus energy meter
 *
 * Based on: https://github.com/reaper7/SDM_Energy_Meter
\*********************************************************************************************/

#define XNRG_10             10

// can be user defined in my_user_config.h
#ifndef SDM630_SPEED
  #define SDM630_SPEED      9600    // default SDM630 Modbus address
#endif
// can be user defined in my_user_config.h
#ifndef SDM630_ADDR
  #define SDM630_ADDR       1       // default SDM630 Modbus address
#endif

#include <TasmotaModbus.h>
TasmotaModbus *Sdm630Modbus;

const uint16_t sdm630_start_addresses[] {
  0x0000,  // L1 - SDM630_VOLTAGE [V]
  0x0002,  // L2 - SDM630_VOLTAGE [V]
  0x0004,  // L3 - SDM630_VOLTAGE [V]
  0x0006,  // L1 - SDM630_CURRENT [A]
  0x0008,  // L2 - SDM630_CURRENT [A]
  0x000A,  // L3 - SDM630_CURRENT [A]
  0x000C,  // L1 - SDM630_POWER [W]
  0x000E,  // L2 - SDM630_POWER [W]
  0x0010,  // L3 - SDM630_POWER [W]
  0x0018,  // L1 - SDM630_REACTIVE_POWER [VAR]
  0x001A,  // L2 - SDM630_REACTIVE_POWER [VAR]
  0x001C,  // L3 - SDM630_REACTIVE_POWER [VAR]
  0x001E,  // L1 - SDM630_POWER_FACTOR
  0x0020,  // L2 - SDM630_POWER_FACTOR
  0x0022,  // L3 - SDM630_POWER_FACTOR
  0x0024,  // L1 - SD630_FASE_ANGLE [DEGREES]
  0x0026,  // L2 - SD630_FASE_ANGLE [DEGREES]
  0x0028,  // L3 - SD630_FASE_ANGLE [DEGREES]
  0x0042,  // TOTAL FASE ANGLE [DEGREES]
  0x0046,  // FREQUENCY [HZ]
  0x0048,  // IMPORT ACTIVE [W]
  0x004A,  // EXPORT ACTIVE [W]
  0x004C,  // IMPORT REACTIVE [VAR]
  0x004E,  // EXPORT REACTIVE [VAR]
  0x0156  // Total - SDM630_TOTAL_ACTIVE_ENERGY [Wh]
};

struct SDM630 {
  float import_active = NAN;
  float import_reactive = 0;
  float export_reactive = 0;
  float phase_angle[4] = {0};
  uint8_t read_state = 0;
  uint8_t send_retry = 0;
} Sdm630;

/*********************************************************************************************/

void SDM630Every250ms(void)
{
  bool data_ready = Sdm630Modbus->ReceiveReady();

  if (data_ready) {
    uint8_t buffer[14];  // At least 5 + (2 * 2) = 9

    uint32_t error = Sdm630Modbus->ReceiveBuffer(buffer, 2);
    AddLogBuffer(LOG_LEVEL_DEBUG_MORE, buffer, Sdm630Modbus->ReceiveCount());

    if (error) {
      AddLog_P2(LOG_LEVEL_DEBUG, PSTR("SDM: SDM630 error %d"), error);
    } else {
      Energy.data_valid[0] = 0;
      Energy.data_valid[1] = 0;
      Energy.data_valid[2] = 0;

      //  0  1  2  3  4  5  6  7  8
      // SA FC BC Fh Fl Sh Sl Cl Ch
      // 01 04 04 43 66 33 34 1B 38 = 230.2 Volt
      float value;
      ((uint8_t*)&value)[3] = buffer[3];   // Get float values
      ((uint8_t*)&value)[2] = buffer[4];
      ((uint8_t*)&value)[1] = buffer[5];
      ((uint8_t*)&value)[0] = buffer[6];

      switch(Sdm630.read_state) {
        case 0:
          Energy.voltage[0] = value;
          break;

        case 1:
          Energy.voltage[1] = value;
          break;

        case 2:
          Energy.voltage[2] = value;
          break;

        case 3:
          Energy.current[0] = value;
          break;

        case 4:
          Energy.current[1] = value;
          break;

        case 5:
          Energy.current[2] = value;
          break;

        case 6:
          Energy.active_power[0] = value;
          break;

        case 7:
          Energy.active_power[1] = value;
          break;

        case 8:
          Energy.active_power[2] = value;
          break;

        case 9:
          Energy.reactive_power[0] = value;
          break;

        case 10:
          Energy.reactive_power[1] = value;
          break;

        case 11:
          Energy.reactive_power[2] = value;
          break;

        case 12:
          Energy.power_factor[0] = value;
          break;

        case 13:
          Energy.power_factor[1] = value;
          break;

        case 14:
          Energy.power_factor[2] = value;
          break;
        
        case 15:
          Sdm630.phase_angle[0] = value;
          break;
        
        case 16:
          Sdm630.phase_angle[1] = value;
          break;
        
        case 17:
          Sdm630.phase_angle[2] = value;
          break;
        
        case 18:
          Sdm630.phase_angle[3] = value;
          break;
        
        case 19:
          Energy.frequency[0] = value;        // 50.0 Hz
          break;
        
        case 20:
          Sdm630.import_active = value;    // 478.492 kWh
          break;

        case 21:
          Energy.export_active = value;    // 6.216 kWh
          break;

        case 22:
          Sdm630.import_reactive = value;  // 172.750 kVArh
          break;

        case 23:
          Sdm630.export_reactive = value;  // 2.844 kVArh
          break;

        case 24:
          EnergyUpdateTotal(value, true);
          break;
      }

      Sdm630.read_state++;
      if (25 == Sdm630.read_state) {
        Sdm630.read_state = 0;
      }
    }
  } // end data ready

  if (0 == Sdm630.send_retry || data_ready) {
    Sdm630.send_retry = 5;
    Sdm630Modbus->Send(SDM630_ADDR, 0x04, sdm630_start_addresses[Sdm630.read_state], 2);
  } else {
    Sdm630.send_retry--;
  }
}

void Sdm630SnsInit(void)
{
  Sdm630Modbus = new TasmotaModbus(pin[GPIO_SDM630_RX], pin[GPIO_SDM630_TX]);
  uint8_t result = Sdm630Modbus->Begin(SDM630_SPEED);
  if (result) {
    if (2 == result) { ClaimSerial(); }
    Energy.phase_count = 3;
  } else {
    energy_flg = ENERGY_NONE;
  }
}

void Sdm630DrvInit(void)
{
  if ((pin[GPIO_SDM630_RX] < 99) && (pin[GPIO_SDM630_TX] < 99)) {
    energy_flg = XNRG_10;
  }
}

void Sdm630Reset(void)
{
  if (isnan(Sdm630.import_active)) { return; }

  Sdm630.import_active = 0;
  Sdm630.import_reactive = 0;
  Sdm630.export_reactive = 0;
  Sdm630.phase_angle[0] = 0;
  Sdm630.phase_angle[1] = 0;
  Sdm630.phase_angle[2] = 0;
  Sdm630.phase_angle[3] = 0;
}

#ifdef USE_WEBSERVER
const char HTTP_ENERGY_SDM630[] PROGMEM =
  "{s}" D_IMPORT_REACTIVE "{m}%s " D_UNIT_KWARH "{e}"
  "{s}" D_EXPORT_REACTIVE "{m}%s " D_UNIT_KWARH "{e}"
  "{s}" D_PHASE_ANGLE " L1{m}%s " D_UNIT_ANGLE "{e}"
  "{s}" D_PHASE_ANGLE " L2{m}%s " D_UNIT_ANGLE "{e}"
  "{s}" D_PHASE_ANGLE " L3{m}%s " D_UNIT_ANGLE "{e}"
  "{s}" D_PHASE_ANGLE " TOTAL{m}%s " D_UNIT_ANGLE "{e}";
#endif  // USE_WEBSERVER

void Sdm630Show(bool json)
{
  if (isnan(Sdm630.import_active)) { return; }

  char import_active_chr[FLOATSZ];
  dtostrfd(Sdm630.import_active, Settings.flag2.energy_resolution, import_active_chr);
  char import_reactive_chr[FLOATSZ];
  dtostrfd(Sdm630.import_reactive, Settings.flag2.energy_resolution, import_reactive_chr);
  char export_reactive_chr[FLOATSZ];
  dtostrfd(Sdm630.export_reactive, Settings.flag2.energy_resolution, export_reactive_chr);
  char phase_angle_l1_chr[FLOATSZ];
  dtostrfd(Sdm630.phase_angle[0], 2, phase_angle_l1_chr);
  char phase_angle_l2_chr[FLOATSZ];
  dtostrfd(Sdm630.phase_angle[1], 2, phase_angle_l2_chr);
  char phase_angle_l3_chr[FLOATSZ];
  dtostrfd(Sdm630.phase_angle[2], 2, phase_angle_l3_chr);
  char phase_angle_total_chr[FLOATSZ];
  dtostrfd(Sdm630.phase_angle[3], 2, phase_angle_total_chr);

  if (json) {
    ResponseAppend_P(PSTR(",\"" D_JSON_IMPORT_ACTIVE "\":%s,\"" D_JSON_IMPORT_REACTIVE "\":%s,\"" D_JSON_EXPORT_REACTIVE "\":%s,\"" D_JSON_PHASE_ANGLE "\":%s"),
      import_active_chr, import_reactive_chr, export_reactive_chr, phase_angle_total_chr);
#ifdef USE_WEBSERVER
  } else {
    WSContentSend_PD(HTTP_ENERGY_SDM630, import_reactive_chr, export_reactive_chr,  phase_angle_l1_chr,  phase_angle_l2_chr,  phase_angle_l3_chr, phase_angle_total_chr);
#endif  // USE_WEBSERVER
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xnrg10(uint8_t function)
{
  bool result = false;

  switch (function) {
    case FUNC_EVERY_250_MSECOND:
      if (uptime > 4) { SDM630Every250ms(); }
      break;
    case FUNC_JSON_APPEND:
      Sdm630Show(1);
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      Sdm630Show(0);
      break;
#endif  // USE_WEBSERVER
    case FUNC_ENERGY_RESET:
      Sdm630Reset();
      break;
    case FUNC_INIT:
      Sdm630SnsInit();
      break;
    case FUNC_PRE_INIT:
      Sdm630DrvInit();
      break;
  }
  return result;
}

#endif  // USE_SDM630
#endif  // USE_ENERGY_SENSOR
