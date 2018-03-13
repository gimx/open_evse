/*
   This file is part of Open EVSE.

   Open EVSE is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3, or (at your option)
   any later version.

   Open EVSE is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Open EVSE; see the file COPYING.  If not, write to the
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
*/
#include "open_evse.h"

#ifdef FT_ENDURANCE
int g_CycleCnt = -1;
long g_CycleHalfStart;
uint8_t g_CycleState;
#endif

THRESH_DATA g_DefaultThreshData = {875, 780, 690, 0, 260};

J1772EVSEController g_EvseController;


J1772EVSEController::J1772EVSEController() 

#ifdef CURRENT_PIN
  : adcCurrent(CURRENT_PIN)
#endif
#ifdef VOLTMETER_PIN
  , adcVoltMeter(VOLTMETER_PIN)
#endif
{
}

void J1772EVSEController::SaveSettings()
{
  uint8_t *dest;
  // n.b. should we add dirty bits so we only write the changed values? or should we just write them on the fly when necessary?
  if (GetCurSvcLevel() == 1) {
    dest = (uint8_t *)EOFS_CURRENT_CAPACITY_L1;
  }
  else {
    dest = (uint8_t *)EOFS_CURRENT_CAPACITY_L2;
  }
  eeprom_write_byte(dest, GetCurrentCapacity());
  SaveEvseFlags();
}



// use watchdog to perform a reset
void J1772EVSEController::Reboot()
{
  m_Pilot.SetState(PILOT_STATE_P12);

#ifdef LCD16X2
  g_OBD.LcdPrint_P(1, PSTR("Resetting..."));
#endif

  if (chargingIsOn()) {
    // give the EV some time to open its contactor in response to P12
    delay(3000);
  }

  // hardware reset by forcing watchdog to timeout
  wdt_enable(WDTO_1S);   // enable watchdog timer
  delay(1500);
}



#ifdef SHOW_DISABLED_TESTS
void J1772EVSEController::DisabledTest_P(PGM_P message)
{
  g_OBD.LcdMsg_P(g_psDisabledTests, message);
  delay(SHOW_DISABLED_DELAY);
}

void J1772EVSEController::ShowDisabledTests()
{
  if (m_wFlags & (ECF_DIODE_CHK_DISABLED |
                  ECF_VENT_REQ_DISABLED |
                  ECF_GND_CHK_DISABLED |
                  ECF_STUCK_RELAY_CHK_DISABLED |
                  ECF_GFI_TEST_DISABLED |
                  ECF_TEMP_CHK_DISABLED)) {
    g_OBD.LcdSetBacklightColor(YELLOW);

    if (!DiodeCheckEnabled()) {
      DisabledTest_P(g_psDiodeCheck);
    }
    if (!VentReqEnabled()) {
      DisabledTest_P(g_psVentReqChk);
    }
#ifdef ADVPWR
    if (!GndChkEnabled()) {
      DisabledTest_P(g_psGndChk);
    }
    if (!StuckRelayChkEnabled()) {
      DisabledTest_P(g_psRlyChk);
    }
#endif // ADVPWR
#ifdef GFI_SELFTEST
    if (!GfiSelfTestEnabled()) {
      DisabledTest_P(g_psGfiTest);
    }
#endif // GFI_SELFTEST
#ifdef TEMPERATURE_MONITORING
    if (!TempChkEnabled()) {
      DisabledTest_P(g_psTempChk);
    }
#endif // TEMPERATURE_MONITORING

    g_OBD.LcdSetBacklightColor(WHITE);
  }
}
#endif //SHOW_DISABLED_TESTS

void J1772EVSEController::chargingOn()
{ // turn on charging current
  pinCharging.write(1);
#ifdef CHARGING2_REG
  pinCharging2.write(1);
#endif
#ifdef CHARGINGAC_REG
  pinChargingAC.write(1);
#endif
  m_bVFlags |= ECVF_CHARGING_ON;

  m_ChargeOnTime = now();
  m_ChargeOnTimeMS = millis();
}

void J1772EVSEController::chargingOff()
{ // turn off charging current
  pinCharging.write(0);
#ifdef CHARGING2_REG
  pinCharging2.write(0);
#endif
#ifdef CHARGINGAC_REG
  pinChargingAC.write(0);
#endif
  m_bVFlags &= ~ECVF_CHARGING_ON;

  m_ChargeOffTime = now();
  m_ChargeOffTimeMS = millis();

#ifdef AMMETER
  m_ChargingCurrent = 0;
#endif
}


#ifdef GFI
void J1772EVSEController::SetGfiTripped()
{
#ifdef GFI_SELFTEST
  if (m_Gfi.SelfTestInProgress()) {
    m_Gfi.SetTestSuccess();
    return;
  }
#endif
  m_bVFlags |= ECVF_GFI_TRIPPED;

  // this is repeated in Update(), but we want to keep latency as low as possible
  // for safety so we do it here first anyway
  chargingOff(); // turn off charging current
  // turn off the PWM
  m_Pilot.SetState(PILOT_STATE_P12);

  m_Gfi.SetFault();
  // the rest of the logic will be handled in Update()
}
#endif // GFI

void J1772EVSEController::EnableDiodeCheck(uint8_t tf)
{
  if (tf) {
    m_wFlags &= ~ECF_DIODE_CHK_DISABLED;
  }
  else {
    m_wFlags |= ECF_DIODE_CHK_DISABLED;
  }
  SaveEvseFlags();
}

#ifdef GFI_SELFTEST
void J1772EVSEController::EnableGfiSelfTest(uint8_t tf)
{
  if (tf) {
    m_wFlags &= ~ECF_GFI_TEST_DISABLED;
  }
  else {
    m_wFlags |= ECF_GFI_TEST_DISABLED;
  }
  SaveEvseFlags();
}
#endif

#ifdef TEMPERATURE_MONITORING
void J1772EVSEController::EnableTempChk(uint8_t tf)
{
  if (tf) {
    m_wFlags &= ~ECF_TEMP_CHK_DISABLED;
  }
  else {
    m_wFlags |= ECF_TEMP_CHK_DISABLED;
  }
  SaveEvseFlags();
}
#endif // TEMPERATURE_MONITORING

void J1772EVSEController::EnableVentReq(uint8_t tf)
{
  if (tf) {
    m_wFlags &= ~ECF_VENT_REQ_DISABLED;
  }
  else {
    m_wFlags |= ECF_VENT_REQ_DISABLED;
  }
  SaveEvseFlags();
}

#ifdef ADVPWR
void J1772EVSEController::EnableGndChk(uint8_t tf)
{
  if (tf) {
    m_wFlags &= ~ECF_GND_CHK_DISABLED;
  }
  else {
    m_NoGndRetryCnt = 0;
    m_NoGndStart = 0;
    m_wFlags |= ECF_GND_CHK_DISABLED;
  }
  SaveEvseFlags();
}

void J1772EVSEController::EnableStuckRelayChk(uint8_t tf)
{
  if (tf) {
    m_wFlags &= ~ECF_STUCK_RELAY_CHK_DISABLED;
  }
  else {
    m_wFlags |= ECF_STUCK_RELAY_CHK_DISABLED;
  }
  SaveEvseFlags();
}

void J1772EVSEController::EnableAutoSvcLevel(uint8_t tf)
{
  if (tf) {
    m_wFlags &= ~ECF_AUTO_SVC_LEVEL_DISABLED;
  }
  else {
    m_wFlags |= ECF_AUTO_SVC_LEVEL_DISABLED;
  }
  SaveEvseFlags();
}


#endif // ADVPWR

void J1772EVSEController::EnableSerDbg(uint8_t tf)
{
  if (tf) {
    m_wFlags |= ECF_SERIAL_DBG;
  }
  else {
    m_wFlags &= ~ECF_SERIAL_DBG;
  }
  SaveEvseFlags();
}

#ifdef RGBLCD
int J1772EVSEController::SetBacklightType(uint8_t t, uint8_t update)
{
#ifdef RGBLCD
  g_OBD.LcdSetBacklightType(t, update);
  if (t == BKL_TYPE_MONO) m_wFlags |= ECF_MONO_LCD;
  else m_wFlags &= ~ECF_MONO_LCD;
  SaveEvseFlags();
#endif // RGBLCD
  return 0;
}
#endif // RGBLCD
void J1772EVSEController::Enable()
{
#ifdef SLEEP_STATUS_REG
  pinSleepStatus.write(1);
#endif // SLEEP_STATUS_REG

  if ((m_EvseState == EVSE_STATE_DISABLED) ||
      (m_EvseState == EVSE_STATE_SLEEPING)) {

#if defined(TIME_LIMIT) || defined(CHARGE_LIMIT)
    SetLimitSleep(0);
#endif //defined(TIME_LIMIT) || defined(CHARGE_LIMIT)

    m_PrevEvseState = EVSE_STATE_DISABLED;
    m_EvseState = EVSE_STATE_UNKNOWN;
    m_Pilot.SetState(PILOT_STATE_P12);
  }
}

void J1772EVSEController::Disable()
{
  if (m_EvseState != EVSE_STATE_DISABLED) {
#ifdef SLEEP_STATUS_REG

    pinSleepStatus.write(0);

#endif // SLEEP_STATUS_REG 
    m_Pilot.SetState(PILOT_STATE_N12);
    m_EvseState = EVSE_STATE_DISABLED;
    // panic stop so we won't wait for EV to open its contacts first
    chargingOff();
    g_OBD.Update(OBD_UPD_FORCE);
#ifdef RAPI
    g_ERP.sendEvseState();
#endif // RAPI
  }
}


void J1772EVSEController::Sleep()
{
#ifdef KWH_RECORDING   // Reset the Wh when exiting State A for any reason
  if (m_EvseState == EVSE_STATE_A) {
    g_WattSeconds = 0;
  }
#endif

  if (m_EvseState != EVSE_STATE_SLEEPING) {
    m_Pilot.SetState(PILOT_STATE_P12);
    m_EvseState = EVSE_STATE_SLEEPING;
#ifdef SLEEP_STATUS_REG
    pinSleepStatus.write(0);
#endif // SLEEP_STATUS_REG

    g_OBD.Update(OBD_UPD_FORCE);
#ifdef RAPI
    g_ERP.sendEvseState();
#endif // RAPI
    // try to prevent arcing of our relay by waiting for EV to open its contacts first
    // use the charge end time variable temporarily to count down
    // when to open the contacts in Update()
    m_ChargeOffTimeMS = millis();
  }
}

void J1772EVSEController::LoadThresholds()
{
  memcpy(&m_ThreshData, &g_DefaultThreshData, sizeof(m_ThreshData));
}

void J1772EVSEController::SetSvcLevel(uint8_t svclvl, uint8_t updatelcd)
{
#ifdef SERDBG
  if (SerDbgEnabled()) {
    Serial.print("SetSvcLevel: "); Serial.println((int)svclvl);
  }
#endif //#ifdef SERDBG
  if (svclvl == 2) {
    m_wFlags |= ECF_L2; // set to Level 2
  }
  else {
    svclvl = 1;
    m_wFlags &= ~ECF_L2; // set to Level 1
  }

  SaveEvseFlags();

  uint8_t ampacity =  eeprom_read_byte((uint8_t*)((svclvl == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2));

  if ((ampacity == 0xff) || (ampacity == 0)) {
    ampacity = (svclvl == 1) ? DEFAULT_CURRENT_CAPACITY_L1 : DEFAULT_CURRENT_CAPACITY_L2;
  }

  if (ampacity < MIN_CURRENT_CAPACITY_L1) {
    ampacity = MIN_CURRENT_CAPACITY_L1;
  }
  else {
    if (svclvl == 1) { // L1
      if (ampacity > MAX_CURRENT_CAPACITY_L1) {
        ampacity = MAX_CURRENT_CAPACITY_L1;
      }
    }
    else {
      if (ampacity > MAX_CURRENT_CAPACITY_L2) {
        ampacity = MAX_CURRENT_CAPACITY_L2;
      }
    }
  }


  LoadThresholds();

  SetCurrentCapacity(ampacity);

  if (updatelcd) {
    g_OBD.Update(OBD_UPD_FORCE);
  }
}


void J1772EVSEController::Init()
{
  // read settings from EEPROM
  uint16_t rflgs = eeprom_read_word((uint16_t*)EOFS_FLAGS);

#ifdef RGBLCD
  if ((rflgs != 0xffff) && (rflgs & ECF_MONO_LCD)) {
    g_OBD.LcdSetBacklightType(BKL_TYPE_MONO);
  }
#endif // RGBLCD

  pinCharging.init(CHARGING_REG, CHARGING_IDX, DigitalPin::OUT);
#ifdef CHARGING2_REG
  pinCharging2.init(CHARGING2_REG, CHARGING2_IDX, DigitalPin::OUT);
#endif
#ifdef CHARGINGAC_REG
  pinChargingAC.init(CHARGINGAC_REG, CHARGINGAC_IDX, DigitalPin::OUT);
#endif
#ifdef ACLINE1_REG
  pinAC1.init(ACLINE1_REG, ACLINE1_IDX, DigitalPin::INP_PU);
#endif
#ifdef ACLINE2_REG
  pinAC2.init(ACLINE2_REG, ACLINE2_IDX, DigitalPin::INP_PU);
#endif
#ifdef SLEEP_STATUS_REG
  pinSleepStatus.init(SLEEP_STATUS_REG, SLEEP_STATUS_IDX, DigitalPin::OUT);
  pinSleepStatus.write(1);
#endif

#ifdef GFI
  m_Gfi.Init();
#endif // GFI

  chargingOff();

  m_Pilot.Init(); // init the pilot

  uint8_t svclvl = (uint8_t)DEFAULT_SERVICE_LEVEL;

  if (rflgs == 0xffff) { // uninitialized EEPROM
    m_wFlags = ECF_DEFAULT;
#ifdef RGBLCD
    if (DEFAULT_LCD_BKL_TYPE == BKL_TYPE_MONO) {
      m_wFlags |= ECF_MONO_LCD;
    }
#endif // RGBLCD
  }
  else {
    m_wFlags = rflgs;
    svclvl = GetCurSvcLevel();

  }

#ifdef NOCHECKS
  m_wFlags |= ECF_DIODE_CHK_DISABLED | ECF_VENT_REQ_DISABLED | ECF_GND_CHK_DISABLED | ECF_STUCK_RELAY_CHK_DISABLED | ECF_GFI_TEST_DISABLED | ECF_TEMP_CHK_DISABLED;
#endif

#ifdef SERDBG
  EnableSerDbg(1);
#endif

#ifdef AMMETER
  m_AmmeterCurrentOffset = eeprom_read_word((uint16_t*)EOFS_AMMETER_CURR_OFFSET);
  m_CurrentScaleFactor = eeprom_read_word((uint16_t*)EOFS_CURRENT_SCALE_FACTOR);

  if (m_AmmeterCurrentOffset == (int16_t)0xffff) {
    m_AmmeterCurrentOffset = DEFAULT_AMMETER_CURRENT_OFFSET;
  }
  if (m_CurrentScaleFactor == (int16_t)0xffff) {
    m_CurrentScaleFactor = DEFAULT_CURRENT_SCALE_FACTOR;
  }

  m_AmmeterReading = 0;
  m_ChargingCurrent = 0;
  //  m_LastAmmeterReadMs = 0;
#endif // AMMETER

#ifdef VOLTMETER
  m_VoltOffset = eeprom_read_dword((uint32_t*)EOFS_VOLT_OFFSET);
  m_VoltScaleFactor = eeprom_read_word((uint16_t*)EOFS_VOLT_SCALE_FACTOR);

  if (m_VoltOffset == 0xffffffff) {
    m_VoltOffset = DEFAULT_VOLT_OFFSET;
  }
  if (m_VoltScaleFactor == 0xffff) {
    m_VoltScaleFactor = DEFAULT_VOLT_SCALE_FACTOR;
  }
#endif // VOLTMETER

#ifndef RGBLCD
  m_wFlags |= ECF_MONO_LCD;
#endif

  m_bVFlags = ECVF_DEFAULT;
#ifdef GFI
  m_GfiRetryCnt = 0;
  m_GfiTripCnt = eeprom_read_byte((uint8_t*)EOFS_GFI_TRIP_CNT);
#endif // GFI
#ifdef ADVPWR
  m_NoGndRetryCnt = 0;
  m_NoGndTripCnt = eeprom_read_byte((uint8_t*)EOFS_NOGND_TRIP_CNT);

  m_StuckRelayStartTimeMS = 0;
  m_StuckRelayTripCnt = eeprom_read_byte((uint8_t*)EOFS_STUCK_RELAY_TRIP_CNT);

  m_NoGndRetryCnt = 0;
  m_NoGndStart = 0;
#endif // ADVPWR

  m_EvseState = EVSE_STATE_A;
  m_PrevEvseState = EVSE_STATE_UNKNOWN;


  SetSvcLevel(svclvl);

#ifdef DELAYTIMER
  if (g_DelayTimer.IsTimerEnabled()) {
    Sleep();
  }
#endif

  g_OBD.SetGreenLed(0);
}


//TABLE A1 - PILOT LINE VOLTAGE RANGES (recommended.. adjust as necessary
//                           Minimum Nominal Maximum
//Positive Voltage, State A  11.40 12.00 12.60
//Positive Voltage, State B  8.36 9.00 9.56
//Positive Voltage, State C  5.48 6.00 6.49
//Positive Voltage, State D  2.62 3.00 3.25
//Negative Voltage - States B, C, D, and F -11.40 -12.00 -12.60
//Thresholds {875,780,690,0,260};
//            AB   BC  CD D  DS
void J1772EVSEController::Update()
{
  uint16_t plow;
  uint16_t phigh = 0xffff;

  unsigned long curms = millis();

  uint8_t prevevsestate = m_EvseState;
  uint8_t tmpevsestate = EVSE_STATE_UNKNOWN;
  int16_t deltap = 0;

  m_Pilot.ReadPilot(&plow, &phigh);

  deltap = phigh - plow;

  if (deltap < 10) { //master pilot steady
    if (phigh >= m_ThreshData.m_ThreshAB) {
      tmpevsestate = EVSE_STATE_A;
    }
    if (plow < 500) {
      tmpevsestate = EVSE_STATE_GFCI_FAULT;
    }
  }
  else if (DiodeCheckEnabled() && (plow >= m_ThreshData.m_ThreshDS)) {
    // diode check failed
    tmpevsestate = EVSE_STATE_DIODE_CHK_FAILED;
  }
  else if (phigh >= m_ThreshData.m_ThreshAB) {
    // 12V EV not connected
    tmpevsestate = EVSE_STATE_A;
  }
  else if (phigh >= m_ThreshData.m_ThreshBC) {
    // 9V EV connected, waiting for ready to charge
    tmpevsestate = EVSE_STATE_B;
  }
  else if (phigh  >= m_ThreshData.m_ThreshCD) {
    // 6V ready to charge
    tmpevsestate = EVSE_STATE_C;
  }
  else if (phigh > m_ThreshData.m_ThreshD) {
    // 3V ready to charge vent required
    if (VentReqEnabled()) {
      tmpevsestate = EVSE_STATE_D;
    }
    else {
      //	      tmpevsestate = EVSE_STATE_C;
    }
  }
  else {
    tmpevsestate = EVSE_STATE_UNKNOWN;
  }

  // debounce state transitions
  if (tmpevsestate != prevevsestate) {
    if (tmpevsestate != m_TmpEvseState) {
      m_TmpEvseStateStart = curms;
    }
    else if ((curms - m_TmpEvseStateStart) >= ((tmpevsestate == EVSE_STATE_A) ? DELAY_STATE_TRANSITION_A : DELAY_STATE_TRANSITION)) {
      m_EvseState = tmpevsestate;
    }
  }


  m_TmpEvseState = tmpevsestate;

  //  Serial.println(m_EvseState);
  // state transition
  if (m_EvseState != prevevsestate) {
    if (m_EvseState == EVSE_STATE_A) { // EV not connected
      chargingOff(); // turn off charging current
      ClrHardFault();
      m_Pilot.SetState(PILOT_STATE_P12);
#ifdef KWH_RECORDING
      g_WattHours_accumulated = g_WattHours_accumulated + (g_WattSeconds / 3600);
      eeprom_write_dword((uint32_t*)EOFS_KWH_ACCUMULATED, g_WattHours_accumulated);
#endif // KWH_RECORDING
#ifdef CHARGE_LIMIT
      SetChargeLimit(0);
#endif // CHARGE_LIMIT
#ifdef TIME_LIMIT
      SetTimeLimit(0);
#endif // TIME_LIMIT
    }
    else if (m_EvseState == EVSE_STATE_B) { // connected
      chargingOff(); // turn off charging current
      ClrHardFault();
      m_Pilot.SetPWM(m_CurrentCapacity);
    }
    else if (m_EvseState == EVSE_STATE_C) {
      m_Pilot.SetPWM(m_CurrentCapacity);
      ClrHardFault();
      chargingOn(); // turn on charging current
    }
    else if (m_EvseState == EVSE_STATE_D) {
      // vent required not supported
      chargingOff(); // turn off charging current
      m_Pilot.SetState(PILOT_STATE_P12);
      SetHardFault();
    }
    else {
      // vehicle state F
      chargingOff(); // turn off charging current
      SetHardFault();
      m_Pilot.SetState(PILOT_STATE_N12);
    }


#ifdef RAPI
    g_ERP.sendEvseState();
#endif // RAPI
#ifdef SERDBG
    if (SerDbgEnabled()) {
      Serial.print("state: ");
      switch (m_Pilot.GetState()) {
        case PILOT_STATE_P12: Serial.print("P12"); break;
        case PILOT_STATE_PWM: Serial.print("PWM"); break;
        case PILOT_STATE_N12: Serial.print("N12"); break;
      }
      Serial.print(" ");
      Serial.print((int)prevevsestate);
      Serial.print("->");
      Serial.print((int)m_EvseState);
      Serial.print(" p ");
      Serial.print(plow);
      Serial.print(" ");
      Serial.println(phigh);
    }
#endif //#ifdef SERDBG
#ifdef KWH_RECORDING          // Reset the Wh when exiting State A for any reason
    if (prevevsestate == EVSE_STATE_A) {
      g_WattSeconds = 0;
    }
#endif
  } // state transition

  m_PrevEvseState = prevevsestate;

#ifdef VOLTMETER
  ReadVoltmeter();
#endif // VOLTMETER
#ifdef AMMETER
  if (((m_EvseState == EVSE_STATE_C) && (m_CurrentScaleFactor > 0)) || AmmeterCalEnabled()) {

    if (m_ChargingCurrent < 0) {
      m_ChargingCurrent = 0;
    }
    g_OBD.SetAmmeterDirty(1);
  
}
#endif // AMMETER

if (m_EvseState == EVSE_STATE_C) {
  m_ElapsedChargeTimePrev = m_ElapsedChargeTime;
  m_ElapsedChargeTime = now() - m_ChargeOnTime;

#ifdef TEMPERATURE_MONITORING
  if (TempChkEnabled()) {
    if (m_ElapsedChargeTime != m_ElapsedChargeTimePrev) {
      uint8_t currcap = eeprom_read_byte((uint8_t*) ((GetCurSvcLevel() == 2) ? EOFS_CURRENT_CAPACITY_L2 : EOFS_CURRENT_CAPACITY_L1));
      uint8_t setit = 0;
      //   g_TempMonitor.Read();  // moved this to main update loop so it reads temperatures in all EVSE states
      if (!g_TempMonitor.OverTemperature() && ((g_TempMonitor.m_TMP007_temperature   >= TEMPERATURE_INFRARED_THROTTLE_DOWN ) ||  // any sensor reaching threshold trips action
          (g_TempMonitor.m_MCP9808_temperature  >= TEMPERATURE_AMBIENT_THROTTLE_DOWN ) ||
          (g_TempMonitor.m_DS3231_temperature  >= TEMPERATURE_AMBIENT_THROTTLE_DOWN ))) {   // Throttle back the L2 current advice to the EV
        currcap /= 2;   // set to the throttled back level
        setit = 2;
      }

      else if (g_TempMonitor.OverTemperature() && ((g_TempMonitor.m_TMP007_temperature   <= TEMPERATURE_INFRARED_RESTORE_AMPERAGE ) &&  // all sensors need to show return to lower levels
               (g_TempMonitor.m_MCP9808_temperature  <= TEMPERATURE_AMBIENT_RESTORE_AMPERAGE  ) &&
               (g_TempMonitor.m_DS3231_temperature  <= TEMPERATURE_AMBIENT_RESTORE_AMPERAGE  ))) {  // restore the original L2 current advice to the EV
        setit = 1;    // set to the user's original setting for current
      }


      else if (!g_TempMonitor.OverTemperatureShutdown() && ((g_TempMonitor.m_TMP007_temperature   >= TEMPERATURE_INFRARED_SHUTDOWN ) ||  // any sensor reaching threshold trips action
               (g_TempMonitor.m_MCP9808_temperature  >= TEMPERATURE_AMBIENT_SHUTDOWN  )  ||
               (g_TempMonitor.m_DS3231_temperature  >= TEMPERATURE_AMBIENT_SHUTDOWN  ))) {   // Throttle back the L2 current advice to the EV
        currcap /= 4;
        setit = 4;
      }

      else if (g_TempMonitor.OverTemperatureShutdown() && ((g_TempMonitor.m_TMP007_temperature   <= TEMPERATURE_INFRARED_THROTTLE_DOWN ) &&  // all sensors need to show return to lower levels
               (g_TempMonitor.m_MCP9808_temperature  <= TEMPERATURE_AMBIENT_THROTTLE_DOWN )  &&
               (g_TempMonitor.m_DS3231_temperature  <= TEMPERATURE_AMBIENT_THROTTLE_DOWN ))) {   //  restore the throttled down current advice to the EV since things have cooled down again
        currcap /= 2;    // set to the throttled back level
        setit = 3;
      }
      if (setit) {
        if (setit <= 2)
          g_TempMonitor.SetOverTemperature(setit - 1);
        else
          g_TempMonitor.SetOverTemperatureShutdown(setit - 3);
        SetCurrentCapacity(currcap, 0, 1);
        if (m_Pilot.GetState() != PILOT_STATE_PWM) {
          m_Pilot.SetPWM(m_CurrentCapacity);
        }
      }
    }
  }
#endif // TEMPERATURE_MONITORING
#ifdef CHARGE_LIMIT
  if (m_chargeLimit && (g_WattSeconds >= 3600000 * (uint32_t)m_chargeLimit)) {
    SetChargeLimit(0); // reset charge limit
    SetLimitSleep(1);
    Sleep();
  }
#endif
#ifdef TIME_LIMIT
  if (m_timeLimit) {
    // must call millis() below because curms is sampled before transition to
    // to State C, so m_ChargeOnTimeMS will be > curms from the start
    if ((millis() - m_ChargeOnTimeMS) >= (15lu * 60000lu * (unsigned long)m_timeLimit)) {
      SetTimeLimit(0); // reset time limit
      SetLimitSleep(1);
      Sleep();
    }
  }
#endif
}
}


int J1772EVSEController::SetCurrentCapacity(uint8_t amps, uint8_t updatelcd, uint8_t nosave)
{
  int rc = 0;
  uint8_t maxcurrentcap = (GetCurSvcLevel() == 1) ? MAX_CURRENT_CAPACITY_L1 : MAX_CURRENT_CAPACITY_L2;

  if ((amps >= MIN_CURRENT_CAPACITY_L1) && (amps <= maxcurrentcap)) {
    m_CurrentCapacity = amps;
  }
  else if (amps < MIN_CURRENT_CAPACITY_L1) {
    m_CurrentCapacity = MIN_CURRENT_CAPACITY_L1;
    rc = 1;
  }
  else {
    m_CurrentCapacity = maxcurrentcap;
    rc = 2;
  }

  if (!nosave) {
    eeprom_write_byte((uint8_t*)((GetCurSvcLevel() == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2), (byte)m_CurrentCapacity);
  }

  if (m_Pilot.GetState() == PILOT_STATE_PWM) {
    m_Pilot.SetPWM(m_CurrentCapacity);
  }

  if (updatelcd) {
    g_OBD.Update(OBD_UPD_FORCE);
  }
  return rc;
}

unsigned long J1772EVSEController::GetResetMs()
{
  return 0; //GFI_TIMEOUT - (millis() - ((m_EvseState == EVSE_STATE_GFCI_FAULT) ? m_GfiFaultStartMs : m_NoGndStart));
}


#ifdef VOLTMETER
void J1772EVSEController::SetVoltmeter(uint16_t scale, uint32_t offset)
{
  m_VoltScaleFactor = scale;
  eeprom_write_word((uint16_t*)EOFS_VOLT_SCALE_FACTOR, scale);
  m_VoltOffset = offset;
  eeprom_write_dword((uint32_t*)EOFS_VOLT_OFFSET, offset);
}

uint32_t J1772EVSEController::ReadVoltmeter()
{
  unsigned int peak = 0;
  for (uint32_t start_time = millis(); (millis() - start_time) < VOLTMETER_POLL_INTERVAL; ) {
    unsigned int val = adcVoltMeter.read();
    if (val > peak) peak = val;
  }
  m_Voltage = ((uint32_t)peak) * ((uint32_t)m_VoltScaleFactor) + m_VoltOffset;
  return m_Voltage;
}
#endif // VOLTMETER


//-- end J1772EVSEController
