/*
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

#include "open_evse.h"

OnboardDisplay::OnboardDisplay()
#if defined(I2CLCD) || defined(RGBLCD)
#ifdef I2CLCD_PCF8574
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
  : m_Lcd(LCD_I2C_ADDR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE)
#else
  : m_Lcd(LCD_I2C_ADDR,1)
#endif // I2CLCD_PCF8574
#endif // defined(I2CLCD) || defined(RGBLCD)
{
}

#if defined(DELAYTIMER)||defined(TIME_LIMIT)
const char CustomChar_0[8] PROGMEM = {0x0,0xe,0x15,0x17,0x11,0xe,0x0,0x0}; // clock
#endif
#ifdef DELAYTIMER
const char CustomChar_1[8] PROGMEM = {0x0,0x0,0xe,0xe,0xe,0x0,0x0,0x0}; // stop (cube)
const char CustomChar_2[8] PROGMEM = {0x0,0x8,0xc,0xe,0xc,0x8,0x0,0x0}; // play
#endif // DELAYTIMER
#if defined(DELAYTIMER)||defined(CHARGE_LIMIT)
const char CustomChar_3[8] PROGMEM = {0x0,0xe,0xc,0x1f,0x3,0x6,0xc,0x8}; // lightning
#endif
#ifdef AUTH_LOCK
const char CustomChar_4[8] PROGMEM = { // padlock
  0b00000,
  0b01110,
  0b01010,
  0b11111,
  0b11011,
  0b11011,
  0b01110,
  0b00000
};
#endif // AUTH_LOCK

void OnboardDisplay::MakeChar(uint8_t n, PGM_P bytes)
{
  memcpy_P(g_sTmp, bytes, 8);
  m_Lcd.createChar(n, (uint8_t*)g_sTmp);
}

void OnboardDisplay::Init()
{
  WDT_RESET();

#ifdef RGBLCD
  m_bFlags = 0;
#else
  m_bFlags = OBDF_MONO_BACKLIGHT;
#endif // RGBLCD

#ifdef GREEN_LED_REG
  pinGreenLed.init(GREEN_LED_REG,GREEN_LED_IDX,DigitalPin::OUT);
  SetGreenLed(0);
#endif
#ifdef RED_LED_REG
  pinRedLed.init(RED_LED_REG,RED_LED_IDX,DigitalPin::OUT);
  SetRedLed(0);
#endif

#ifdef LCD16X2
  LcdBegin(LCD_MAX_CHARS_PER_LINE, 2);
  LcdSetBacklightColor(WHITE);

#if defined(DELAYTIMER)||defined(TIME_LIMIT)
  MakeChar(0,CustomChar_0);
#endif
#ifdef DELAYTIMER
  MakeChar(1,CustomChar_1);
  MakeChar(2,CustomChar_2);
#endif //#ifdef DELAYTIMER
#if defined(DELAYTIMER)||defined(CHARGE_LIMIT)
  MakeChar(3,CustomChar_3);
#endif
  m_Lcd.clear();

#ifdef OPENEVSE_2
  LcdPrint_P(0,PSTR("Open EVSE II"));
#else
  LcdPrint_P(0,PSTR("OpenEVSE Limiter"));
#endif
  LcdPrint_P(0,1,PSTR("Ver. "));
  LcdPrint_P(VERSTR);
  delay(1500);
  WDT_RESET();
#endif //#ifdef LCD16X2
}

#ifdef LCD16X2
void OnboardDisplay::LcdPrint(int x,int y,const char *s)
{ 
  m_Lcd.setCursor(x,y);
  m_Lcd.print(s); 
}

void OnboardDisplay::LcdPrint_P(PGM_P s)
{
  strncpy_P(m_strBuf,s,LCD_MAX_CHARS_PER_LINE);
  m_strBuf[LCD_MAX_CHARS_PER_LINE] = 0;
  m_Lcd.print(m_strBuf);
}

void OnboardDisplay::LcdPrint_P(int y,PGM_P s)
{
  strncpy_P(m_strBuf,s,LCD_MAX_CHARS_PER_LINE);
  m_strBuf[LCD_MAX_CHARS_PER_LINE] = 0;
  LcdPrint(y,m_strBuf);
}

void OnboardDisplay::LcdPrint_P(int x,int y,PGM_P s)
{
  strncpy_P(m_strBuf,s,LCD_MAX_CHARS_PER_LINE);
  m_strBuf[LCD_MAX_CHARS_PER_LINE] = 0;
  m_Lcd.setCursor(x,y);
  m_Lcd.print(m_strBuf);
}

void OnboardDisplay::LcdMsg_P(PGM_P l1,PGM_P l2)
{
  LcdPrint_P(0,l1);
  LcdPrint_P(1,l2);
}


// print at (0,y), filling out the line with trailing spaces
void OnboardDisplay::LcdPrint(int y,const char *s)
{
  m_Lcd.setCursor(0,y);
  uint8_t i,len = strlen(s);
  if (len > LCD_MAX_CHARS_PER_LINE)
    len = LCD_MAX_CHARS_PER_LINE;
  for (i=0;i < len;i++) {
    m_Lcd.write(s[i]);
  }
  for (i=len;i < LCD_MAX_CHARS_PER_LINE;i++) {
    m_Lcd.write(' ');
  }
}

void OnboardDisplay::LcdMsg(const char *l1,const char *l2)
{
  LcdPrint(0,l1);
  LcdPrint(1,l2);
}
#endif // LCD16X2


void OnboardDisplay::Update(int8_t updmode)
{
  if (updateDisabled()) return;

  uint8_t curstate = g_EvseController.GetState();
  uint8_t svclvl = g_EvseController.GetCurSvcLevel();
  int currentcap = g_EvseController.GetCurrentCapacity();
  unsigned long curms = millis();

  if (g_EvseController.StateTransition() || (updmode != OBD_UPD_NORMAL)) {
    curms += 1000; // trigger periodic update code below

    sprintf(g_sTmp,g_sRdyLAstr,(int)svclvl,currentcap);
    switch(curstate) {
    case EVSE_STATE_A: // not connected
      SetGreenLed(1);
      SetRedLed(0);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(GREEN);
      // Display Timer and Stop Icon - GoldServe
      LcdClear();
      LcdSetCursor(0,0);
#ifdef DELAYTIMER
      g_DelayTimer.PrintTimerIcon();
#endif //#ifdef DELAYTIMER
      LcdPrint_P(g_psReady);
      LcdPrint(10,0,g_sTmp);
      
#ifdef KWH_RECORDING 
      sprintf(g_sTmp,"%5luWh",(g_WattSeconds / 3600) );
      LcdPrint(0,1,g_sTmp);
      
      sprintf(g_sTmp,"%6lukWh",(g_WattHours_accumulated / 1000));  // display accumulated kWh
      LcdPrint(7,1,g_sTmp);
#endif // KWH_RECORDING
      
#endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_B: // connected/not charging
      SetGreenLed(1);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(YELLOW);
      LcdClear();
      LcdSetCursor(0,0);
#ifdef CHARGE_LIMIT
      if (g_EvseController.GetChargeLimit()) {
  LcdWrite(3); // lightning
      }
#endif
#ifdef TIME_LIMIT
      if (g_EvseController.GetTimeLimit()) {
  LcdWrite(0); // clock
      }
#endif
      // Display Timer and Stop Icon - GoldServe
#ifdef DELAYTIMER
      g_DelayTimer.PrintTimerIcon();
#endif //#ifdef DELAYTIMER
      LcdPrint_P(g_psEvConnected);
      LcdPrint(10,0,g_sTmp);
      
#ifdef KWH_RECORDING
      sprintf(g_sTmp,"%5luWh",(g_WattSeconds / 3600) );
      LcdPrint(0,1,g_sTmp);
      
      sprintf(g_sTmp,"%6lukWh",(g_WattHours_accumulated / 1000));  // Display accumulated kWh
      LcdPrint(7,1,g_sTmp);
#endif // KWH_RECORDING
      
#endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_C: // charging
      SetGreenLed(0);
      SetRedLed(0);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(TEAL);
      LcdClear();
      LcdSetCursor(0,0);
      // Display Timer and Stop Icon - GoldServe
#ifdef CHARGE_LIMIT
      if (g_EvseController.GetChargeLimit()) {
  LcdWrite(3); // lightning
      }
#endif
#ifdef TIME_LIMIT
      if (g_EvseController.GetTimeLimit()) {
  LcdWrite(0); // clock
      }
#endif
#ifdef DELAYTIMER
      g_DelayTimer.PrintTimerIcon();
#endif //#ifdef DELAYTIMER
      LcdPrint_P(g_psCharging);
#endif //Adafruit RGB LCD
      // n.b. blue LED is on
      break;
    case EVSE_STATE_D: // vent required
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg_P(g_psEvseError,g_psVentReq);
#endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_DIODE_CHK_FAILED:
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg_P(g_psEvseError,g_psDiodeChkFailed);
#endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_GFCI_FAULT:
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      if (updmode == OBD_UPD_HARDFAULT) {
        LcdMsg_P(g_psEvseError,g_psGfciFault);
      }
      else {
  // 2nd line will be updated below with auto retry count
        LcdPrint_P(0,g_psGfciFault);
      }
#endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
#ifdef TEMPERATURE_MONITORING      
    case EVSE_STATE_OVER_TEMPERATURE:    // overtemp message in Red on the RGB LCD
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg_P(g_psSvcReq,g_psTemperatureFault);  //  SERVICE REQUIRED     OVER TEMPERATURE 
#endif
      break;
#endif //TEMPERATURE_MONITORING        
    case EVSE_STATE_NO_GROUND:
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      if (updmode == OBD_UPD_HARDFAULT) {
        LcdMsg_P(g_psEvseError,g_psNoGround);
      }
      else {
  // 2nd line will be updated below with auto retry count
        LcdPrint_P(0,g_psNoGround);
      }
#endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_STUCK_RELAY:
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg_P(updmode == OBD_UPD_HARDFAULT ? g_psSvcReq : g_psEvseError,g_psStuckRelay);
#endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_DISABLED:
      SetGreenLed(0);
      SetRedLed(1);
#ifdef LCD16X2
      LcdSetBacklightColor(VIOLET);
      LcdClear();
      LcdSetCursor(0,0);
      LcdPrint_P(g_psDisabled);
      LcdPrint(10,0,g_sTmp);
#endif // LCD16X2
      break;
    case EVSE_STATE_GFI_TEST_FAILED:
      SetGreenLed(0);
      SetRedLed(1);
      LcdSetBacklightColor(RED);
//      LcdMsg_P(g_psTestFailed,g_psGfci);
      break;
    case EVSE_STATE_SLEEPING:
      SetGreenLed(1);
      SetRedLed(1);
#ifdef LCD16X2
      LcdSetBacklightColor(VIOLET);
      LcdClear();
      LcdSetCursor(0,0);
      LcdPrint_P(g_psSleeping);
      LcdPrint(10,0,g_sTmp);
#endif // LCD16X2
      break;
    default:
      SetGreenLed(0);
      SetRedLed(1);
      // n.b. blue LED is off
    }
  }

  //
  // put anything that needs to be updated periodically here
  // the code below will only run once per second
  //
  if ((curms-m_LastUpdateMs) >= 1000) {
    m_LastUpdateMs = curms;
    
    if (!g_EvseController.InHardFault() &&
  ((curstate == EVSE_STATE_GFCI_FAULT) || (curstate == EVSE_STATE_NO_GROUND))) {
      strcpy(g_sTmp,g_sRetryIn);
      int resetsec = (int)(g_EvseController.GetResetMs() / 1000ul);
      if (resetsec >= 0) {
  sprintf(g_sTmp+sizeof(g_sTmp)-6,g_sHHMMfmt,resetsec / 60,resetsec % 60);
  strcat(g_sTmp,g_sTmp+sizeof(g_sTmp)-6);
  LcdPrint(1,g_sTmp);
      }
      return;
    }

#ifdef RTC
    g_CurrTime = g_RTC.now();
#endif

#ifdef LCD16X2
#if defined(AMMETER)
    if (((curstate == EVSE_STATE_C) || g_EvseController.AmmeterCalEnabled()) && AmmeterIsDirty()) {
      SetAmmeterDirty(0);

      uint32_t current = g_EvseController.GetChargingCurrent();

      if (current >= 1000) { // display only if > 1000
  int a = current / 1000;
  int ma = (current % 1000) / 100;
  if (ma > 9) {
    ma = 0;
    a++;
  }
  sprintf(g_sTmp,"%3d.%dA",a,ma);
      }
      else {
  strcpy_P(g_sTmp,PSTR("    0A"));
      }
      LcdPrint(10,0,g_sTmp);
    }
#endif // AMMETER

    if (curstate == EVSE_STATE_C) {
#ifndef KWH_RECORDING
      time_t elapsedTime = g_EvseController.GetElapsedChargeTime();
#endif
   
#ifdef KWH_RECORDING
      uint32_t current = g_EvseController.GetChargingCurrent();     
      sprintf(g_sTmp,"%5luWh",(g_WattSeconds / 3600) );
      LcdPrint(0,1,g_sTmp);

#ifdef VOLTMETER
      sprintf(g_sTmp," %3luV",(g_EvseController.GetVoltage() / 1000));  // Display voltage from OpenEVSE II
      LcdPrint(11,1,g_sTmp);
#else
      sprintf(g_sTmp,"%6lukWh",(g_WattHours_accumulated / 1000));  // display accumulated kWh
      LcdPrint(7,1,g_sTmp);
#endif // VOLTMETER
#endif // KWH_RECORDING

#ifdef TEMPERATURE_MONITORING
      if ((g_TempMonitor.OverTemperature()) || TEMPERATURE_DISPLAY_ALWAYS)  {
  g_OBD.LcdClearLine(1);
  const char *tempfmt = "%2d.%1dC";
#ifdef MCP9808_IS_ON_I2C
  if ( g_TempMonitor.m_MCP9808_temperature != 0 ) {   // it returns 0 if it is not present
    sprintf(g_sTmp,tempfmt,g_TempMonitor.m_MCP9808_temperature/10, g_TempMonitor.m_MCP9808_temperature % 10);  //  Ambient sensor near or on the LCD
    LcdPrint(0,1,g_sTmp);
  }
#endif

#ifdef RTC  
  if ( g_TempMonitor.m_DS3231_temperature != 0) {   // it returns 0 if it is not present
    sprintf(g_sTmp,tempfmt,g_TempMonitor.m_DS3231_temperature/10, g_TempMonitor.m_DS3231_temperature % 10);      //  sensor built into the DS3231 RTC Chip
    LcdPrint(5,1,g_sTmp);
  }
#endif
  
#ifdef TMP007_IS_ON_I2C
  if ( g_TempMonitor.m_TMP007_temperature != 0 ) {    // it returns 0 if it is not present
    sprintf(g_sTmp,tempfmt,g_TempMonitor.m_TMP007_temperature/10, g_TempMonitor.m_TMP007_temperature % 10);  //  Infrared sensor probably looking at 30A fuses
    LcdPrint(11,1,g_sTmp);
  }
#endif

  if (g_TempMonitor.BlinkAlarm() && g_TempMonitor.OverTemperatureShutdown()) { // Blink Red off-and-on while over the temperature shutdown limit, zero current should flow to the EV
    g_TempMonitor.SetBlinkAlarm(0);                                            // toggle the alarm flag so we can blink
    SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
    LcdSetBacklightColor(RED);
#endif //Adafruit RGB LCD            
  }
  else  {
    g_TempMonitor.SetBlinkAlarm(1);        // toggle the alarm flag so we can blink
    SetRedLed(0);
#ifdef LCD16X2 //Adafruit RGB LCD
    LcdSetBacklightColor(TEAL);
#endif
  }           
      }  // (g_TempMonitor.OverTemperature()) || TEMPERATURE_DISPLAY_ALWAYS) 
      else  {
  SetRedLed(0);          // restore the normal TEAL backlight in case it was left RED while last blinking
#ifdef LCD16X2 //Adafruit RGB LCD
  LcdSetBacklightColor(TEAL);
#endif
      }
      if (!(g_TempMonitor.OverTemperature() || TEMPERATURE_DISPLAY_ALWAYS)) { 
#endif // TEMPERATURE_MONITORING
#ifndef KWH_RECORDING
      int h = hour(elapsedTime);          // display the elapsed charge time
      int m = minute(elapsedTime);
      int s = second(elapsedTime);
      sprintf(g_sTmp,"%02d:%02d:%02d",h,m,s);
#ifdef RTC
      g_sTmp[8]=' ';
      g_sTmp[9]=' ';
      g_sTmp[10]=' ';
      sprintf(g_sTmp+11,g_sHHMMfmt,(int)g_CurrTime.hour(),(int)g_CurrTime.minute());
#endif //RTC
      LcdPrint(1,g_sTmp);
#endif // KWH_RECORDING
#ifdef TEMPERATURE_MONITORING
      }
#endif // TEMPERATURE_MONITORING
    } // curstate == EVSE_STATE_C
    // Display a new stopped LCD screen with Delay Timers enabled - GoldServe
#ifdef DELAYTIMER
    else if (curstate == EVSE_STATE_SLEEPING) {
      LcdSetCursor(0,0);
      g_DelayTimer.PrintTimerIcon();
      //     LcdPrint_P(g_DelayTimer.IsTimerEnabled() ? g_psWaiting : g_psSleeping);
      LcdPrint_P(g_psSleeping);
      sprintf(g_sTmp,"%02d:%02d:%02d",g_CurrTime.hour(),g_CurrTime.minute(),g_CurrTime.second());
      LcdPrint(0,1,g_sTmp);
      if (g_DelayTimer.IsTimerEnabled()){
  LcdSetCursor(9,0);
  LcdWrite(0x2);
  LcdWrite(0x0);
  sprintf(g_sTmp,g_sHHMMfmt,g_DelayTimer.GetStartTimerHour(),g_DelayTimer.GetStartTimerMin());
  LcdPrint(11,0,g_sTmp);
  LcdSetCursor(9,1);
  LcdWrite(0x1);
  LcdWrite(0x0);
  sprintf(g_sTmp,g_sHHMMfmt,g_DelayTimer.GetStopTimerHour(),g_DelayTimer.GetStopTimerMin());
  LcdPrint(11,1,g_sTmp);
      } else {
  sprintf(g_sTmp,g_sRdyLAstr,(int)svclvl,currentcap);
  LcdPrint(10,0,g_sTmp);
      }
    }
#endif // DELAYTIMER
#endif // LCD16X2
  }

#ifdef FT_ENDURANCE
  LcdSetCursor(0,0);
  sprintf(g_sTmp,"%d %d",g_CycleCnt,(int)g_EvseController.ReadACPins());
  LcdPrint(g_sTmp);
#endif // FT_ENDURANCE
}

