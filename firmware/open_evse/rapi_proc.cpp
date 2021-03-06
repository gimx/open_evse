// -*- C++ -*-
/*
 * Open EVSE Firmware
 *
<<<<<<< HEAD
 * Copyright (c) 2013-2014 Sam C. Lin <lincomatic@gmail.com>
=======
 * Copyright (c) 2013-2016 Sam C. Lin <lincomatic@gmail.com>
>>>>>>> upstream/stable
 *
 * This file is part of Open EVSE.

 * Open EVSE is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.

 * Open EVSE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with Open EVSE; see the file COPYING.  If not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#if defined(ARDUINO) && (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h" // shouldn't need this but arduino sometimes messes up and puts inside an #ifdef
#endif // ARDUINO
#include "open_evse.h"

#ifdef RAPI
const char RAPI_VER[] PROGMEM = RAPIVER;


// convert 2-digit hex string to uint8_t
uint8_t htou8(const char *s)
{
  uint8_t u = 0;
  for (int i=0;i < 2;i++) {
<<<<<<< HEAD
    if (i == 1) u <<= 4;
    char c = s[i];
=======
    char c = s[i];
    if (c != '\0') {
      if (i == 1) u <<= 4;
>>>>>>> upstream/stable
    if ((c >= '0') && (c <= '9')) {
      u += c - '0';
    }
    else if ((c >= 'A') && (c <= 'F')) {
      u += c - 'A' + 10;
    }
<<<<<<< HEAD
=======
      else if ((c >= 'a') && (c <= 'f')) {
	u += c - 'a' + 10;
      }
      else {
	// invalid character received
	return 0;
      }
    }
>>>>>>> upstream/stable
  }
  return u;
}

// convert decimal string to uint32_t
uint32_t dtou32(const char *s)
{
  uint32_t u = 0;
  while (*s) {
    u *= 10;
    u += *(s++) - '0';
  }
  return u;
}

<<<<<<< HEAD
EvseRapiProcessor::EvseRapiProcessor()
{
=======
#ifdef RAPI_I2C
//get data from master - HINT: this is a ISR call!
//HINT2: do not handle stuff here!! this will NOT work
//collect only data here and process it in the main loop!
void receiveEvent(int numBytes)
{
  //do nothing here
}
#endif // RAPI_I2C

EvseRapiProcessor::EvseRapiProcessor()
{
#ifdef RAPI_SEQUENCE_ID
  curReceivedSeqId = INVALID_SEQUENCE_ID;
#ifdef RAPI_SENDER
  curSentSeqId = INVALID_SEQUENCE_ID;
#endif
#endif
}

void EvseRapiProcessor::init()
{
>>>>>>> upstream/stable
  echo = 0;
  reset();
}

<<<<<<< HEAD
//extern HardwareSerial Serial;
=======
>>>>>>> upstream/stable
int EvseRapiProcessor::doCmd()
{
  int rc = 1;

  int bcnt = available();
  if (bcnt) {
    for (int i=0;i < bcnt;i++) {
      char c = read();
      if (echo) write(c);

      if (c == ESRAPI_SOC) {
	buffer[0] = ESRAPI_SOC;
	bufCnt = 1;
      }
      else if (buffer[0] == ESRAPI_SOC) {
	if (bufCnt < ESRAPI_BUFLEN) {
	  if (c == ESRAPI_EOC) {
	    buffer[bufCnt++] = 0;
<<<<<<< HEAD
	    if (!tokenize()) {
=======
	    if (!tokenize(buffer)) {
>>>>>>> upstream/stable
	      rc = processCmd();
	    }
	    else {
	      reset();
<<<<<<< HEAD
=======
#ifdef RAPI_SEQUENCE_ID
	      curReceivedSeqId = INVALID_SEQUENCE_ID;
#endif // RAPI_SEQUENCE_ID
>>>>>>> upstream/stable
	      response(0);
	    }
	  }
	  else {
	    buffer[bufCnt++] = c;
	  }
	}
	else { // too many chars
	  reset();
	}
      }
    }
  }

  return rc;
}

<<<<<<< HEAD
void EvseRapiProcessor::sendEvseState()
{
  sprintf(g_sTmp,"%cST %02x%c",ESRAPI_SOC,g_EvseController.GetState(),ESRAPI_EOC);
  write(g_sTmp);
=======

void EvseRapiProcessor::sendEvseState()
{
#ifdef RAPI_RESPONSE_CHK
  sprintf(g_sTmp,"%cST %02x",ESRAPI_SOC,g_EvseController.GetState());
  appendChk(g_sTmp);
#else
  sprintf(g_sTmp,"%cST %02x%c",ESRAPI_SOC,g_EvseController.GetState(),ESRAPI_EOC);
#endif //RAPI_RESPONSE_CHK
  writeStart();
  write(g_sTmp);
  writeEnd();
>>>>>>> upstream/stable
}

void EvseRapiProcessor::setWifiMode(uint8_t mode)
{
<<<<<<< HEAD
  sprintf(g_sTmp,"%cWF %02x%c",ESRAPI_SOC,(int)mode,ESRAPI_EOC);
  write(g_sTmp);
}

int EvseRapiProcessor::tokenize()
{
  tokens[0] = &buffer[1];
  char *s = &buffer[2];
  tokenCnt = 1;
  uint8_t achkSum = ESRAPI_SOC + buffer[1];
  uint8_t xchkSum = ESRAPI_SOC ^ buffer[1];
=======
#ifdef RAPI_RESPONSE_CHK
  sprintf(g_sTmp,"%cWF %02x",ESRAPI_SOC,(int)mode);
  appendChk(g_sTmp);
#else
  sprintf(g_sTmp,"%cWF %02x%c",ESRAPI_SOC,(int)mode,ESRAPI_EOC);
#endif //RAPI_RESPONSE_CHK
  writeStart();
  write(g_sTmp);
  writeEnd();
}

int EvseRapiProcessor::tokenize(char *buf)
{
  tokens[0] = &buf[1];
  char *s = &buf[2];
  tokenCnt = 1;
  uint8_t achkSum = ESRAPI_SOC + buf[1];
  uint8_t xchkSum = ESRAPI_SOC ^ buf[1];
>>>>>>> upstream/stable
  uint8_t hchkSum;
  uint8_t chktype=0; // 0=none,1=additive,2=xor
  while (*s) {
    if (*s == ' ') {
<<<<<<< HEAD
      achkSum += *s;
      xchkSum ^= *s;
      *s = '\0';
      tokens[tokenCnt++] = ++s;
=======
      if (tokenCnt >= ESRAPI_MAX_ARGS) {
	chktype = 255;
	break;
      }
      else {
	achkSum += *s;
	xchkSum ^= *s;
	*s = '\0';
	tokens[tokenCnt++] = ++s;
      }
>>>>>>> upstream/stable
    }
    else if ((*s == '*') ||// additive checksum
	     (*s == '^')) { // XOR checksum
      if (*s == '*') chktype = 1;
      else if (*s == '^') chktype = 2;
      *(s++) = '\0';
      hchkSum = htou8(s);
      break;
    }
    else {
      achkSum += *s;
      xchkSum ^= *(s++);
    }
  }
<<<<<<< HEAD
  
  return ((chktype == 0) ||
	  ((chktype == 1) && (hchkSum == achkSum)) ||
	  ((chktype == 2) && (hchkSum == xchkSum))) ? 0 : 1;
=======

  int rc = ((chktype == 0) ||
	   ((chktype == 1) && (hchkSum == achkSum)) ||
 	   ((chktype == 2) && (hchkSum == xchkSum))) ? 0 : 1;
  if (rc) tokenCnt = 0;
  //  sprintf(g_sTmp,"trc: %d",rc);
  //  g_EIRP.writeStr(g_sTmp);

  return rc;
>>>>>>> upstream/stable
}

int EvseRapiProcessor::processCmd()
{
<<<<<<< HEAD
  UNION4B u1,u2,u3;
  int rc = 0;
=======
  UNION4B u1,u2,u3,u4;
  int rc = -1;

#ifdef RAPI_SENDER
  // throw away extraneous responses that we weren't expecting
  // these could be from commands that we already timed out
  if (isRespToken()) {
    return rc;
  }
#endif // RAPI_SENDER

#ifdef RAPI_SEQUENCE_ID
  curReceivedSeqId = INVALID_SEQUENCE_ID;
  const char *seqtoken = tokens[tokenCnt-1];
  if ((tokenCnt > 1) && (*seqtoken == ESRAPI_SOS)) {
    curReceivedSeqId = htou8(++seqtoken);
    tokenCnt--;
  }
#endif // RAPI_SEQUENCE_ID
>>>>>>> upstream/stable

  // we use bufCnt as a flag in response() to signify data to write
  bufCnt = 0;

  char *s = tokens[0];
  switch(*(s++)) { 
  case 'F': // function
    switch(*s) {
<<<<<<< HEAD
#ifdef LCD16X2
    case 'B': // LCD backlight
      g_OBD.LcdSetBacklightColor(dtou32(tokens[1]));
=======
    case '0': // enable/disable LCD update
      if (tokenCnt == 2) {
	g_OBD.DisableUpdate((*tokens[1] == '0') ? 1 : 0);
	if (*tokens[1] != '0') g_OBD.Update(OBD_UPD_FORCE);
	rc = 0;
      }
      break;
 #ifdef BTN_MENU
    case '1': // simulate front panel short press
      g_BtnHandler.DoShortPress(g_EvseController.InFaultState());
      g_OBD.Update(OBD_UPD_FORCE);
      rc = 0;
      break;
#endif // BTN_MENU
#ifdef LCD16X2
    case 'B': // LCD backlight
      if (tokenCnt == 2) {
	g_OBD.LcdSetBacklightColor(dtou32(tokens[1]));
	rc = 0;
      }
>>>>>>> upstream/stable
      break;
#endif // LCD16X2      
    case 'D': // disable EVSE
      g_EvseController.Disable();
<<<<<<< HEAD
      break;
    case 'E': // enable EVSE
      g_EvseController.Enable();
      break;
#ifdef LCD16X2
    case 'P': // print to LCD
      {
=======
      rc = 0;
      break;
    case 'E': // enable EVSE
      g_EvseController.Enable();
      rc = 0;
      break;
#ifdef RAPI_FF
    case 'F': // enable/disable feature
      if (tokenCnt == 3) {
	u1.u8 = (uint8_t)(*tokens[2] - '0');
	if (u1.u8 <= 1) {
	  rc = 0;
	  switch(*tokens[1]) {
	  case 'D': // diode check
	    g_EvseController.EnableDiodeCheck(u1.u8);
	    break;
	  case 'E': // command echo
	    echo = ((u1.u8 == '0') ? 0 : 1);	      
	    break;
#ifdef ADVPWR
	  case 'F': // GFI self test
	    g_EvseController.EnableGfiSelfTest(u1.u8);
	    break;
	  case 'G': // ground check
	    g_EvseController.EnableGndChk(u1.u8);
	    break;
	  case 'R': // stuck relay check
	    g_EvseController.EnableStuckRelayChk(u1.u8);
	    break;
#endif // ADVPWR
#ifdef TEMPERATURE_MONITORING
	  case 'T': // temperature monitoring
	    g_EvseController.EnableTempChk(u1.u8);
	    break;
#endif // TEMPERATURE_MONITORING
	  case 'V': // vent required check
	    g_EvseController.EnableVentReq(u1.u8);
	    break;
	  default: // unknown
	    rc = -1;
	  }
	}
      }
      break;
#endif // RAPI_FF
#ifdef LCD16X2
    case 'P': // print to LCD
      if (tokenCnt >= 4) {
>>>>>>> upstream/stable
	u1.u = dtou32(tokens[1]); // x
	u2.u = dtou32(tokens[2]); // y
	// now restore the spaces that were replaced w/ nulls by tokenizing
	for (u3.i=4;u3.i < tokenCnt;u3.i++) {
	  *(tokens[u3.i]-1) = ' ';
	}
	g_OBD.LcdPrint(u1.u,u2.u,tokens[3]);
<<<<<<< HEAD
=======
	rc = 0;
>>>>>>> upstream/stable
      }
      break;
#endif // LCD16X2      
    case 'R': // reset EVSE
      g_EvseController.Reboot();
<<<<<<< HEAD
      break;
    case 'S': // sleep
      g_EvseController.Sleep();
      break;
    default:
      rc = -1; // unknown
=======
      rc = 0;
      break;
    case 'S': // sleep
      g_EvseController.Sleep();
      rc = 0;
      break;
>>>>>>> upstream/stable
    }
    break;

  case 'S': // set parameter
    switch(*s) {
#ifdef LCD16X2
    case '0': // set LCD type
      if (tokenCnt == 2) {
#ifdef RGBLCD
	rc = g_EvseController.SetBacklightType((*tokens[1] == '0') ? BKL_TYPE_MONO : BKL_TYPE_RGB);
#endif // RGBLCD
      }
      break;
#endif // LCD16X2      
#ifdef RTC      
    case '1': // set RTC
      if (tokenCnt == 7) {
	extern void SetRTC(uint8_t y,uint8_t m,uint8_t d,uint8_t h,uint8_t mn,uint8_t s);
	SetRTC(dtou32(tokens[1]),dtou32(tokens[2]),dtou32(tokens[3]),
	       dtou32(tokens[4]),dtou32(tokens[5]),dtou32(tokens[6]));
<<<<<<< HEAD
=======
	rc = 0;
>>>>>>> upstream/stable
      }
      break;
#endif // RTC      
#ifdef AMMETER
    case '2': // ammeter calibration mode
      if (tokenCnt == 2) {
	g_EvseController.EnableAmmeterCal((*tokens[1] == '1') ? 1 : 0);
<<<<<<< HEAD
      }
      break;
#ifdef TIME_LIMIT
    case '3': // set time limit
      if (tokenCnt == 2) {
	g_EvseController.SetTimeLimit(dtou32(tokens[1]));
      }
      break;
#endif // TIME_LIMIT
=======
	rc = 0;
      }
      break;
#endif // AMMETER
#ifdef TIME_LIMIT
    case '3': // set time limit
      if (tokenCnt == 2) {
	if (g_EvseController.LimitsAllowed()) {
	  g_EvseController.SetTimeLimit15(dtou32(tokens[1]));
	  if (!g_OBD.UpdatesDisabled()) g_OBD.Update(OBD_UPD_FORCE);
	  rc = 0;
	}
      }
      break;
#endif // TIME_LIMIT
#if defined(AUTH_LOCK) && !defined(AUTH_LOCK_REG)
    case '4': // auth lock
      if (tokenCnt == 2) {
	g_EvseController.AuthLock((int8_t)dtou32(tokens[1]));
	rc = 0;
      }
      break;
#endif // AUTH_LOCK && !AUTH_LOCK_REG
#ifdef AMMETER
>>>>>>> upstream/stable
    case 'A':
      if (tokenCnt == 3) {
	g_EvseController.SetCurrentScaleFactor(dtou32(tokens[1]));
	g_EvseController.SetAmmeterCurrentOffset(dtou32(tokens[2]));
<<<<<<< HEAD
=======
	rc = 0;
>>>>>>> upstream/stable
      }
      break;
#endif // AMMETER
    case 'C': // current capacity
<<<<<<< HEAD
      if (tokenCnt == 2) {
	rc = g_EvseController.SetCurrentCapacity(dtou32(tokens[1]),1);
      }
      break;
    case 'D': // diode check
      if (tokenCnt == 2) {
	g_EvseController.EnableDiodeCheck((*tokens[1] == '0') ? 0 : 1);
=======
      if ((tokenCnt == 2) || (tokenCnt == 3)) {
	if (tokenCnt == 3) {
	  // just make volatile no matter what character specified
	  u1.u8 = 1; // nosave = 1
	}
	else {
	  u1.u8 = 0; // nosave = 0
	}

	u2.u8 = dtou32(tokens[1]);
  
#ifdef TEMPERATURE_MONITORING
	if (g_TempMonitor.OverTemperature() &&
	    (u2.u8 > g_EvseController.GetCurrentCapacity())) {
	  // don't allow raising current capacity during
	  // overtemperature event
	  rc = 1;
	}
	else {
	  rc = g_EvseController.SetCurrentCapacity(u2.u8,1,u1.u8);
	}
#else // !TEMPERATURE_MONITORING
	rc = g_EvseController.SetCurrentCapacity(u2.u8,1,u1.u8);
#endif // TEMPERATURE_MONITORING


	sprintf(buffer,"%d",(int)g_EvseController.GetCurrentCapacity());
	bufCnt = 1; // flag response text output
      }
      break;
#ifndef RAPI_FF
    case 'D': // diode check
      if (tokenCnt == 2) {
	g_EvseController.EnableDiodeCheck((*tokens[1] == '0') ? 0 : 1);
	rc = 0;
>>>>>>> upstream/stable
      }
      break;
    case 'E': // echo
      if (tokenCnt == 2) {
	echo = ((*tokens[1] == '0') ? 0 : 1);
<<<<<<< HEAD
=======
	rc = 0;
>>>>>>> upstream/stable
      }
      break;
#ifdef GFI_SELFTEST
    case 'F': // GFI self test
      if (tokenCnt == 2) {
	g_EvseController.EnableGfiSelfTest(*tokens[1] == '0' ? 0 : 1);
<<<<<<< HEAD
=======
	rc = 0;
>>>>>>> upstream/stable
      }
      break;
#endif // GFI_SELFTEST
#ifdef ADVPWR
    case 'G': // ground check
      if (tokenCnt == 2) {
	g_EvseController.EnableGndChk(*tokens[1] == '0' ? 0 : 1);
<<<<<<< HEAD
      }
      break;
#endif // ADVPWR
#ifdef CHARGE_LIMIT
    case 'H': // cHarge limit
      if (tokenCnt == 2) {
	g_EvseController.SetChargeLimit(dtou32(tokens[1]));
=======
	rc = 0;
      }
      break;
#endif // ADVPWR
#endif // !RAPI_FF
#ifdef CHARGE_LIMIT
    case 'H': // cHarge limit
      if (tokenCnt == 2) {
	if (g_EvseController.LimitsAllowed()) {
	  g_EvseController.SetChargeLimitkWh(dtou32(tokens[1]));
	  if (!g_OBD.UpdatesDisabled()) g_OBD.Update(OBD_UPD_FORCE);
	  rc = 0;
	}
>>>>>>> upstream/stable
      }
      break;
#endif // CHARGE_LIMIT
#ifdef KWH_RECORDING
    case 'K': // set accumulated kwh
<<<<<<< HEAD
      g_WattHours_accumulated = dtou32(tokens[1]);
      eeprom_write_dword((uint32_t*)EOFS_KWH_ACCUMULATED,g_WattHours_accumulated); 
=======
      g_EnergyMeter.SetTotkWh(dtou32(tokens[1]));
      g_EnergyMeter.SaveTotkWh();
      rc = 0;
>>>>>>> upstream/stable
      break;
#endif //KWH_RECORDING
    case 'L': // service level
      if (tokenCnt == 2) {
      switch(*tokens[1]) {
	case '1':
	case '2':
	  g_EvseController.SetSvcLevel(*tokens[1] - '0',1);
<<<<<<< HEAD
#ifdef ADVPWR
	  g_EvseController.EnableAutoSvcLevel(0);
#endif
	  break;
#ifdef ADVPWR
	case 'A':
	  g_EvseController.EnableAutoSvcLevel(1);
	  break;
#endif // ADVPWR
	default:
	  rc = -1; // unknown
=======
#if defined(ADVPWR) && defined(AUTOSVCLEVEL)
	  g_EvseController.EnableAutoSvcLevel(0);
#endif
	  rc = 0;
	  break;
#if defined(ADVPWR) && defined(AUTOSVCLEVEL)
	case 'A':
	  g_EvseController.EnableAutoSvcLevel(1);
	  rc = 0;
	  break;
#endif // ADVPWR && AUTOSVCLEVEL
>>>>>>> upstream/stable
	}
      }
      break;
#ifdef VOLTMETER
    case 'M':
      if (tokenCnt == 3) {
        g_EvseController.SetVoltmeter(dtou32(tokens[1]),dtou32(tokens[2]));
<<<<<<< HEAD
      }
      break;
#endif // VOLTMETER
#ifdef TEMPERATURE_MONITORING_NY
    case 'O':
      if (tokenCnt == 3) {
        g_TempMonitor.m_ambient_thresh = dtou32(tokens[1]);
        g_TempMonitor.m_ir_thresh = dtou32(tokens[2]);
	g_TempMonitor.SaveThresh();
      }
      break;
#endif // TEMPERATURE_MONITORING
#ifdef ADVPWR      
    case 'R': // stuck relay check
      if (tokenCnt == 2) {
	g_EvseController.EnableStuckRelayChk(*tokens[1] == '0' ? 0 : 1);
      }
      break;
#endif // ADVPWR      
#ifdef GFI_SELFTEST
    case 'S': // GFI self-test
      if (tokenCnt == 2) {
	g_EvseController.EnableGfiSelfTest(*tokens[1] == '0' ? 0 : 1);
      }
      break;
#endif // GFI_SELFTEST   
=======
	rc = 0;
      }
      break;
#endif // VOLTMETER
#if defined(ADVPWR) && !defined(RAPI_FF)
    case 'R': // stuck relay check
      if (tokenCnt == 2) {
	g_EvseController.EnableStuckRelayChk(*tokens[1] == '0' ? 0 : 1);
	rc = 0;
      }
      break;
#endif // ADVPWR && !RAPI_FF
>>>>>>> upstream/stable
#ifdef DELAYTIMER     
    case 'T': // timer
      if (tokenCnt == 5) {
	extern DelayTimer g_DelayTimer;
<<<<<<< HEAD
	if ((*tokens[1] == '0') && (*tokens[2] == '0') && (*tokens[3] == '0') && (*tokens[4] == '0')) {
	  g_DelayTimer.Disable();
	}
	else {
	  g_DelayTimer.SetStartTimer(dtou32(tokens[1]),dtou32(tokens[2]));
	  g_DelayTimer.SetStopTimer(dtou32(tokens[3]),dtou32(tokens[4]));
	  g_DelayTimer.Enable();
	}
      }
      break;
#endif // DELAYTIMER      
    case 'V': // vent required
      if (tokenCnt == 2) {
	g_EvseController.EnableVentReq(*tokens[1] == '0' ? 0 : 1);
      }
      break;
=======
	u1.u8 = (uint8_t)dtou32(tokens[1]);
	u2.u8 = (uint8_t)dtou32(tokens[2]);
	u3.u8 = (uint8_t)dtou32(tokens[3]);
	u4.u8 = (uint8_t)dtou32(tokens[4]);
	if ((u1.u8 == 0) && (u2.u8 == 0) && (u3.u8 == 0) && (u4.u8 == 0)) {
	  g_DelayTimer.Disable();
	}
	else {
	  g_DelayTimer.SetStartTimer(u1.u8,u2.u8);
	  g_DelayTimer.SetStopTimer(u3.u8,u4.u8);
	  g_DelayTimer.Enable();
	}
	rc = 0;
      }
      break;
#endif // DELAYTIMER      
#ifndef RAPI_FF
    case 'V': // vent required
      if (tokenCnt == 2) {
	g_EvseController.EnableVentReq(*tokens[1] == '0' ? 0 : 1);
	rc = 0;
      }
      break;
#endif // !RAPI_FF
>>>>>>> upstream/stable
    }
    break;

  case 'G': // get parameter
    switch(*s) {
<<<<<<< HEAD
#ifdef TIME_LIMIT
    case '3': // get time limit
      sprintf(buffer,"%d",(int)g_EvseController.GetTimeLimit());
      bufCnt = 1; // flag response text output
      break;
#endif // TIME_LIMIT
=======
    case '0': // get EV connect state
      {
	uint8_t connstate;
	if (g_EvseController.GetPilot()->GetState() == PILOT_STATE_N12) {
	  connstate = 2; // unknown
	}
	else {
	  if (g_EvseController.EvConnected()) connstate = 1;
	  else connstate = 0;
	}
	sprintf(buffer,"%d",(int)connstate);
      }
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#ifdef TIME_LIMIT
    case '3': // get time limit
      sprintf(buffer,"%d",(int)g_EvseController.GetTimeLimit15());
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // TIME_LIMIT
#if defined(AUTH_LOCK) && !defined(AUTH_LOCK_REG)
    case '4': // get auth lock
      sprintf(buffer,"%d",(int)g_EvseController.AuthLockIsOn() ? 1 : 0);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // AUTH_LOCK && !AUTH_LOCK_REG
>>>>>>> upstream/stable
#ifdef AMMETER
    case 'A':
      u1.i = g_EvseController.GetCurrentScaleFactor();
      u2.i = g_EvseController.GetAmmeterCurrentOffset();
      sprintf(buffer,"%d %d",u1.i,u2.i);
      bufCnt = 1; // flag response text output
<<<<<<< HEAD
      break;
#endif // AMMETER
    case 'C': // get current capacity range
      if (g_EvseController.GetCurSvcLevel() == 2) {
	u1.i = MIN_CURRENT_CAPACITY_L2;
	u2.i = MAX_CURRENT_CAPACITY_L2;
      }
      else {
	u1.i = MIN_CURRENT_CAPACITY_L1;
=======
      rc = 0;
      break;
#endif // AMMETER
    case 'C': // get current capacity range
      u1.i = MIN_CURRENT_CAPACITY_J1772;
      if (g_EvseController.GetCurSvcLevel() == 2) {
	u2.i = MAX_CURRENT_CAPACITY_L2;
      }
      else {
>>>>>>> upstream/stable
	u2.i = MAX_CURRENT_CAPACITY_L1;
      }
      sprintf(buffer,"%d %d",u1.i,u2.i);
      bufCnt = 1; // flag response text output
<<<<<<< HEAD
      break;
=======
      rc = 0;
      break;
#ifdef DELAYTIMER
    case 'D': // get delay timer
      extern DelayTimer g_DelayTimer;
      if (g_DelayTimer.IsTimerEnabled()) {
	u1.i = g_DelayTimer.GetStartTimerHour();
	u2.i = g_DelayTimer.GetStartTimerMin();
	u3.i = g_DelayTimer.GetStopTimerHour();
	u4.i = g_DelayTimer.GetStopTimerMin();
      }
      else {
	u1.i = 0;
	u2.i = 0;
	u3.i = 0;
	u4.i = 0;
      }
      sprintf(buffer,"%d %d %d %d",u1.i,u2.i,u3.i,u4.i);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // DELAYTIMER
>>>>>>> upstream/stable
    case 'E': // get settings
      u1.u = g_EvseController.GetCurrentCapacity();
      u2.u = g_EvseController.GetFlags();
      sprintf(buffer,"%d %04x",u1.u,u2.u);
      bufCnt = 1; // flag response text output
<<<<<<< HEAD
      break;
    case 'F': // get fault counters
//      u1.u = g_EvseController.GetGfiTripCnt();
//      u2.u = g_EvseController.GetNoGndTripCnt();
//      u3.u = g_EvseController.GetStuckRelayTripCnt();
      sprintf(buffer,"%x %x %x",u1.u,u2.u,u3.u);
      bufCnt = 1; // flag response text output
=======
      rc = 0;
      break;
    case 'F': // get fault counters
#ifdef GFI
      u1.u = g_EvseController.GetGfiTripCnt();
#else
      u1.u = 0;
#endif // GFI
#ifdef ADVPWR
      u2.u = g_EvseController.GetNoGndTripCnt();
      u3.u = g_EvseController.GetStuckRelayTripCnt();
#else
	  u2.u = 0;
	  u3.u = 0;
#endif // ADVPWR
      sprintf(buffer,"%x %x %x",u1.u,u2.u,u3.u);
      bufCnt = 1; // flag response text output
      rc = 0;
>>>>>>> upstream/stable
      break;
#if defined(AMMETER)||defined(VOLTMETER)
    case 'G':
      u1.i32 = g_EvseController.GetChargingCurrent();
      u2.i32 = (int32_t)g_EvseController.GetVoltage();
      sprintf(buffer,"%ld %ld",u1.i32,u2.i32);
      bufCnt = 1; // flag response text output
<<<<<<< HEAD
=======
      rc = 0;
>>>>>>> upstream/stable
      break;
#endif // AMMETER || VOLTMETER
#ifdef CHARGE_LIMIT
    case 'H': // get cHarge limit
<<<<<<< HEAD
      sprintf(buffer,"%d",(int)g_EvseController.GetChargeLimit());
      bufCnt = 1; // flag response text output
=======
      sprintf(buffer,"%d",(int)g_EvseController.GetChargeLimitkWh());
      bufCnt = 1; // flag response text output
      rc = 0;
>>>>>>> upstream/stable
      break;
#endif // CHARGE_LIMIT
#ifdef VOLTMETER
    case 'M':
      u1.i = g_EvseController.GetVoltScaleFactor();
      u2.i32 = g_EvseController.GetVoltOffset();
      sprintf(buffer,"%d %ld",u1.i,u2.i32);
      bufCnt = 1; // flag response text output
<<<<<<< HEAD
=======
      rc = 0;
>>>>>>> upstream/stable
      break;
#endif // VOLTMETER
#ifdef TEMPERATURE_MONITORING
#ifdef TEMPERATURE_MONITORING_NY
    case 'O':
      u1.i = g_TempMonitor.m_ambient_thresh;
      u2.i = g_TempMonitor.m_ir_thresh;
      sprintf(buffer,"%d %d",u1.i,u2.i);
      bufCnt = 1; // flag response text output
<<<<<<< HEAD
=======
      rc = 0;
>>>>>>> upstream/stable
      break;
#endif // TEMPERATURE_MONITORING_NY
    case 'P':
      sprintf(buffer,"%d %d %d",(int)g_TempMonitor.m_DS3231_temperature,
	      (int)g_TempMonitor.m_MCP9808_temperature,
	      (int)g_TempMonitor.m_TMP007_temperature);
      /* this is bigger than using sprintf
      strcpy(buffer,u2a(g_TempMonitor.m_DS3231_temperature));
      strcat(buffer,g_sSpace);
      strcat(buffer,u2a(g_TempMonitor.m_MCP9808_temperature));
      strcat(buffer,g_sSpace);
      strcat(buffer,u2a(g_TempMonitor.m_TMP007_temperature));
      */
      bufCnt = 1; // flag response text output
<<<<<<< HEAD
=======
      rc = 0;
>>>>>>> upstream/stable
      break;
#endif // TEMPERATURE_MONITORING
    case 'S': // get state
      sprintf(buffer,"%d %ld",g_EvseController.GetState(),g_EvseController.GetElapsedChargeTime());
      bufCnt = 1; // flag response text output
<<<<<<< HEAD

=======
      rc = 0;
>>>>>>> upstream/stable
      break;
#ifdef RTC
    case 'T': // get time
      extern void GetRTC(char *buf);
      GetRTC(buffer);
      bufCnt = 1; // flag response text output
<<<<<<< HEAD

=======
      rc = 0;
>>>>>>> upstream/stable
      break;
#endif // RTC
#ifdef KWH_RECORDING
    case 'U':
<<<<<<< HEAD
      sprintf(buffer,"%lu %lu",g_WattSeconds,g_WattHours_accumulated);
      bufCnt = 1;
=======
      sprintf(buffer,"%lu %lu",g_EnergyMeter.GetSessionWs(),g_EnergyMeter.GetTotkWh());
      bufCnt = 1;
      rc = 0;
>>>>>>> upstream/stable
      break;
#endif // KWH_RECORDING
    case 'V': // get version
      GetVerStr(buffer);
      strcat(buffer," ");
      strcat_P(buffer,RAPI_VER);
      bufCnt = 1; // flag response text output
<<<<<<< HEAD
      break;
    default:
      rc = -1; // unknown
    }
    break;

  default:
    rc = -1; // unknown
  }

  response((rc == 0) ? 1 : 0);
=======
      rc = 0;
      break;
    }
    break;

#ifdef RAPI_T_COMMANDS
  case 'T': // testing op
    switch(*s) {
#ifdef FAKE_CHARGING_CURRENT
    case '0': // set fake charging current
      if (tokenCnt == 2) {
	g_EvseController.SetChargingCurrent(dtou32(tokens[1])*1000);
	g_OBD.SetAmmeterDirty(1);
	g_OBD.Update(OBD_UPD_FORCE);
	rc = 0;
      }
      break;
#endif // FAKE_CHARGING_CURRENT
    }
    break;
#endif //RAPI_T_COMMANDS
#if defined(RELAY_HOLD_DELAY_TUNING)
  case 'Z': // reserved op
    switch(*s) {
    case '1': // set relayCloseMs
      if (tokenCnt == 2) {
	u1.u32 = dtou32(tokens[1]);
	g_EvseController.setRelayHoldDelay(u1.u32);
	sprintf(g_sTmp,"\nZ1 %ld",u1.u32);
	Serial.println(g_sTmp);
	eeprom_write_dword((uint32_t*)EOFS_RELAY_HOLD_DELAY,u1.u32);
      }
      rc = 0;
      break;

    }
    break;
#endif // RELAY_AUTO_PWM_PIN_TESTING

  default:
    ; // do nothing
  }

  if (bufCnt != -1){
  response((rc == 0) ? 1 : 0);
  }
>>>>>>> upstream/stable

  reset();

  return rc;
}

<<<<<<< HEAD
void EvseRapiProcessor::response(uint8_t ok)
{
  write(ESRAPI_SOC);
  write(ok ? "OK " : "NK ");

  if (bufCnt) {
    write(buffer);
  }
  write(ESRAPI_EOC);
  if (echo) write('\n');
}


EvseRapiProcessor g_ERP;
=======
// append
void EvseRapiProcessor::appendChk(char *buf)
{
  char *s = buf;
  uint8_t chk = 0;
  while (*s) {
    chk ^= *(s++);
  }
  sprintf(s,"^%02X",(unsigned)chk);
  s[3] = ESRAPI_EOC;
  s[4] = '\0';
}


void EvseRapiProcessor::response(uint8_t ok)
{
  writeStart();

#ifdef RAPI_RESPONSE_CHK
  sprintf(g_sTmp,"%c%s",ESRAPI_SOC,ok ? "OK" : "NK");
  if (bufCnt) {
    strcat(g_sTmp," ");
    strcat(g_sTmp,buffer);
  }
#ifdef RAPI_SEQUENCE_ID
  if (curReceivedSeqId != INVALID_SEQUENCE_ID) {
    appendSequenceId(g_sTmp,curReceivedSeqId);
  }
#endif // RAPI_SEQUENCE_ID
  appendChk(g_sTmp);
  write(g_sTmp);
#else // !RAPI_RESPONSE_CHK
  write(ESRAPI_SOC);
  write(ok ? "OK" : "NK");

  if (bufCnt) {
    write(" ");
    write(buffer);
  }
  write(ESRAPI_EOC);
#endif // RAPI_RESPONSE_CHK
  if (echo) write('\n');

  writeEnd();
}

#ifdef RAPI_SEQUENCE_ID
void EvseRapiProcessor::appendSequenceId(char *s,uint8_t seqId)
{
  sprintf(s+strlen(s)," %c%02X",ESRAPI_SOS,seqId);
}
#endif // RAPI_SEQUENCE_ID

#ifdef RAPI_SENDER
#ifdef RAPI_SEQUENCE_ID
uint8_t EvseRapiProcessor::getSendSequenceId()
{
  if (++curSentSeqId == INVALID_SEQUENCE_ID) ++curSentSeqId;
  return curSentSeqId;
}

int8_t EvseRapiProcessor::isAsyncToken()
{
  if ((*tokens[0] == 'A') ||
      !strcmp(tokens[0],"WF") ||
      (!strcmp(tokens[0],"ST") && (tokenCnt == 2))) {
    return 1;
  }
  else {
    return 0;
  }
}

// OK or NK
int8_t EvseRapiProcessor::isRespToken()
{
  const char *token = tokens[0];
  if ((strlen(token) == 2) && (token[1] == 'K') &&
      ((*token == 'O') || (*token == 'N'))) {
    return 1;
  }
  else {
    return 0;
  }
}
#endif // RAPI_SEQUENCE_ID


void EvseRapiProcessor::_sendCmd(const char *cmdstr)
{
  *sendbuf = ESRAPI_SOC;
  strcpy(sendbuf+1,cmdstr);
#ifdef RAPI_SEQUENCE_ID
  appendSequenceId(sendbuf,getSendSequenceId());
#endif
  appendChk(sendbuf);
  writeStart();
  write(sendbuf);
  writeEnd();
}

int8_t EvseRapiProcessor::receiveResp(unsigned long msstart)
{
  tokenCnt = 0;
  *sendbuf = 0;
  int bufpos = 0;

  // wait for response
  do {
    WDT_RESET();
    int bytesavail = available();
    if (bytesavail) {
      for (int i=0;i < bytesavail;i++) {
	char c = read();

	if (!bufpos && c != ESRAPI_SOC) {
	  // wait for start character
	  continue;
	}
	else if (c == ESRAPI_EOC) {
	  sendbuf[bufpos] = '\0';
	  if (!tokenize(sendbuf)) return 0;
	  else return 1;
	  
	}
	else {
	  sendbuf[bufpos++] = c;
	  if (bufpos >= (RAPIS_BUFLEN-1)) return 2;
	}
      }
    }
  } while (!tokenCnt && ((millis() - msstart) < RAPIS_TIMEOUT_MS));

  return -1;
}

int8_t EvseRapiProcessor::sendCmd(const char *cmdstr)
{
  _sendCmd(cmdstr);

  unsigned long msstart = millis();
 start:
  while (receiveResp(msstart) > 0) WDT_RESET();
  if (tokenCnt) {
#ifdef RAPI_SEQUENCE_ID
    uint8_t seqId = INVALID_SEQUENCE_ID;
    const char *seqtoken = tokens[tokenCnt-1];
    if ((tokenCnt > 1) && isRespToken() && (*seqtoken == ESRAPI_SOS)) {
      seqId = htou8(++seqtoken);
      tokenCnt--;
    }
#endif // RAPI_SEQUENCE_ID
    if (!strcmp(tokens[0],"OK")
#ifdef RAPI_SEQUENCE_ID
	&& (seqId == curSentSeqId)
#endif // RAPI_SEQUENCE_ID
	) {
      return 0;
    }
    else if (!strcmp(tokens[0],"NK")
#ifdef RAPI_SEQUENCE_ID
	     && (seqId == curSentSeqId)
#endif // RAPI_SEQUENCE_ID
	     ) {
      return 1;
    }
    else { // command or async notification received - process it
      processCmd();
      msstart = millis();
      goto start;
    }
  }
  else {
    return -1;
  }
}

#endif // RAPI_SENDER


#ifdef RAPI_SERIAL
EvseSerialRapiProcessor::EvseSerialRapiProcessor()
{
}

void EvseSerialRapiProcessor::init()
{
  EvseRapiProcessor::init();
}
#endif // RAPI_SERIAL


#ifdef RAPI_I2C

EvseI2cRapiProcessor::EvseI2cRapiProcessor()
{
}

void EvseI2cRapiProcessor::init()
{
  Wire.begin(RAPI_I2C_LOCAL_ADDR);
  Wire.onReceive(receiveEvent);   // define the receive function for receiving data from master

  EvseRapiProcessor::init();
}


#endif // RAPI_I2C

#ifdef RAPI_SERIAL
EvseSerialRapiProcessor g_ESRP;
#endif
#ifdef RAPI_I2C
EvseI2cRapiProcessor g_EIRP;
#endif

void RapiInit()
{
#ifdef RAPI_SERIAL
  g_ESRP.init();
#ifdef GPPBUGKLUDGE
  static char g_rapiSerialBuffer[ESRAPI_BUFLEN];
  g_ESRP.setBuffer(g_rapiSerialBuffer);
#endif // GPPBUGKLUDGE
#endif // RAPI_SERIAL
#ifdef RAPI_I2C
  g_EIRP.init();
#ifdef GPPBUGKLUDGE
  static char g_rapiI2ClBuffer[ESRAPI_BUFLEN];
  g_ESRP.setBuffer(g_rapiI2CBuffer);
#endif // GPPBUGKLUDGE
#endif // RAPI_I2C
}

void RapiDoCmd()
{
#ifdef RAPI_SERIAL
  g_ESRP.doCmd();
#endif
#ifdef RAPI_I2C
  // kludge - delay below is needed when RapiDoCmd() is running in a tight loop
  // such as during a hard fault. without the delay, I2C can't receive characters
#define RDCDELAY 6
#ifdef RDCDELAY
  static unsigned long lastdocmd;
  unsigned long msnow = millis();
  if ((msnow-lastdocmd) < RDCDELAY) {
    delay(RDCDELAY - (msnow-lastdocmd));
  }
  lastdocmd = msnow;
#endif // RDCDELAY

  g_EIRP.doCmd();
#endif // RAPI_I2C
}

void RapiSendEvseState(uint8_t nodupe)
{
  static uint8_t evseStateSent = EVSE_STATE_UNKNOWN;
  uint8_t es = g_EvseController.GetState();

  if (!nodupe || (evseStateSent != es)) {
#ifdef RAPI_SERIAL
    g_ESRP.sendEvseState();
#endif
#ifdef RAPI_I2C
    g_EIRP.sendEvseState();
#endif
    evseStateSent = es;
  }
}

void RapiSetWifiMode(uint8_t mode)
{
#ifdef RAPI_SERIAL
  g_ESRP.setWifiMode(mode);
#endif
#ifdef RAPI_I2C
  g_EIRP.setWifiMode(mode);
#endif
}

>>>>>>> upstream/stable
#endif // RAPI
