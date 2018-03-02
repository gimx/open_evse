#define LIBCALL_ENABLEINTERRUPT
#include <EnableInterrupt.h>

#include "open_evse.h"

//S0 pulse counter
volatile unsigned long pulseCount = 0;
volatile unsigned long pulsetime=0;                                    // Record time of interrupt pulse

// Instantiate RTC and Delay Timer - GoldServe
#ifdef RTC
RTC_DS1307 g_RTC;
DateTime g_CurrTime;

#if defined(RAPI)
void SetRTC(uint8_t y,uint8_t m,uint8_t d,uint8_t h,uint8_t mn,uint8_t s) {
  g_RTC.adjust(DateTime(y,m,d,h,mn,s));
}
void GetRTC(char *buf) {
  g_CurrTime = g_RTC.now();
  sprintf(buf,"%d %d %d %d %d %d",g_CurrTime.year()-2000,g_CurrTime.month(),g_CurrTime.day(),g_CurrTime.hour(),g_CurrTime.minute(),g_CurrTime.second());
}
#endif // RAPI
#endif // RTC
#ifdef DELAYTIMER
DelayTimer g_DelayTimer;
#ifdef DELAYTIMER_MENU
// Start variables to support RTC and Delay Timer - GoldServe
uint16_t g_year;
uint8_t g_month;
uint8_t g_day;
uint8_t g_hour;
uint8_t g_min;
uint8_t sec = 0;
#endif // DELAYTIMER_MENU
#endif // DELAYTIMER

char g_sTmp[TMP_BUF_SIZE];

unsigned long g_WattHours_accumulated;
unsigned long g_WattSeconds;

void EvseReset()
{
  Wire.begin();

#ifdef RTC
  g_RTC.begin();
#ifdef DELAYTIMER
  g_DelayTimer.Init();
#endif  // DELAYTIMER
#endif // RTC

#ifdef SERIALCLI
  g_CLI.Init();
#endif // SERIALCLI

  g_OBD.Init();

  g_EvseController.Init();
}


void setup(){
  Serial.begin(115200);

  EvseReset(); 

 //S0 pulse counter
  pinMode(S0_PULSE_PIN, INPUT_PULLUP);
  enableInterrupt(S0_PULSE_PIN | PINCHANGEINTERRUPT, onS0Pulse, FALLING);     // Attach pulse counting interrupt pulse counting

  //Enable Digital Communication pin
  pinMode(DCOM_ENAB_PIN, INPUT_PULLUP);

  WDT_ENABLE();
  
#ifdef KWH_RECORDING
  if (eeprom_read_dword((uint32_t*)EOFS_KWH_ACCUMULATED) == 0xffffffff) { // Check for unitialized eeprom condition so it can begin at 0kWh
      eeprom_write_dword((uint32_t*)EOFS_KWH_ACCUMULATED,0); //  Set the four bytes to zero just once in the case of unitialized eeprom
  }

  g_WattHours_accumulated = eeprom_read_dword((uint32_t*)EOFS_KWH_ACCUMULATED);        // get the stored value for the kWh from eeprom
#endif // KWH_RECORDING

}


// ISR runs each time a falling edge of S0 pulse is detected
void onS0Pulse(){
  unsigned long now = millis();
  
  pulseCount++;      
  g_WattSeconds =  pulseCount*3600;  // accumulate Watt Seconds for charging (scaled for 1000imp/kWh = 1 imp/Wh = 3600imp/Ws)

  //calculate current from time between pulses (assumption 1000imp/kWh)
  g_EvseController.SetChargingCurrent(3600000/((now-pulsetime)*VOLTS_FOR_L2)); //adjust your average grid voltage here, this is good enough for current display, energy is measured directly anyway 
  pulsetime=now;
}

// Buffer to store incoming commands from serial port
String inData;

// Start Delay Timer Functions - GoldServe
#ifdef DELAYTIMER
void DelayTimer::Init() {
  //Read EEPROM settings
  uint8_t rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_FLAGS);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_DelayTimerEnabled = 0x00;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  }
  else {
    m_DelayTimerEnabled = rtmp;
  }
  rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_START_HOUR);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_StartTimerHour = DEFAULT_START_HOUR;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_START_HOUR, m_StartTimerHour);
  }
  else {
    m_StartTimerHour = rtmp;
  }
  rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_START_MIN);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_StartTimerMin = DEFAULT_START_MIN;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_START_MIN, m_StartTimerMin);
  }
  else {
    m_StartTimerMin = rtmp;
  }
  rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_STOP_HOUR);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_StopTimerHour = DEFAULT_STOP_HOUR;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_STOP_HOUR, m_StopTimerHour);
  }
  else {
    m_StopTimerHour = rtmp;
  }
  rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_STOP_MIN);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_StopTimerMin = DEFAULT_STOP_MIN;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_STOP_MIN, m_StopTimerMin);
  }
  else {
    m_StopTimerMin = rtmp;
  }
}

void DelayTimer::CheckTime()
{
  if (!g_EvseController.InFaultState() &&
      IsTimerEnabled() &&
      IsTimerValid()) {
    unsigned long curms = millis();
    if ((curms - m_LastCheck) > 1000ul) {
      g_CurrTime = g_RTC.now();
      m_CurrHour = g_CurrTime.hour();
      m_CurrMin = g_CurrTime.minute();

      uint16_t startTimerMinutes = m_StartTimerHour * 60 + m_StartTimerMin;
      uint16_t stopTimerMinutes = m_StopTimerHour * 60 + m_StopTimerMin;
      uint16_t currTimeMinutes = m_CurrHour * 60 + m_CurrMin;

      if (stopTimerMinutes < startTimerMinutes) {
  //End time is for next day
  if ( ( (currTimeMinutes >= startTimerMinutes) && (currTimeMinutes > stopTimerMinutes) ) ||
             ( (currTimeMinutes <= startTimerMinutes) && (currTimeMinutes < stopTimerMinutes) ) ){
    // Within time interval
          if (g_EvseController.GetState() == EVSE_STATE_SLEEPING) {
      g_EvseController.Enable();
    }
  }
  else {
    // S.Low 3/12/14 Added check at T+1 minute in case interrupt is late
    if ((currTimeMinutes >= stopTimerMinutes)&&(currTimeMinutes <= stopTimerMinutes+1)) {
      // Not in time interval
      g_EvseController.Sleep();
    }
  }
      }
      else { // not crossing midnite
  if ((currTimeMinutes >= startTimerMinutes) && (currTimeMinutes < stopTimerMinutes)) {
    // Within time interval
    if (g_EvseController.GetState() == EVSE_STATE_SLEEPING) {
      g_EvseController.Enable();
    }
  }
  else {
    // S.Low 3/12/14 Added check at T+1 minute in case interrupt is late
    if ((currTimeMinutes >= stopTimerMinutes)&&(currTimeMinutes <= stopTimerMinutes+1)) {
      // Not in time interval
      g_EvseController.Sleep();
    }
  }
      }
      m_LastCheck = curms;
    }
  }
}
void DelayTimer::Enable(){
  m_DelayTimerEnabled = 0x01;
  eeprom_write_byte((uint8_t*)EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  g_EvseController.SaveSettings();
  CheckTime();
  g_OBD.Update(OBD_UPD_FORCE);
}
void DelayTimer::Disable(){
  m_DelayTimerEnabled = 0x00;
  eeprom_write_byte((uint8_t*)EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  g_EvseController.SaveSettings();
  g_OBD.Update(OBD_UPD_FORCE);
}
void DelayTimer::PrintTimerIcon(){
#ifdef LCD16X2
  if (IsTimerEnabled() && IsTimerValid()){
    g_OBD.LcdWrite(0x0);
  }
#endif // LCD16X2
}
// End Delay Timer Functions - GoldServe
#endif //#ifdef DELAYTIMER


void ProcessInputs()
{
#ifdef RAPI
  g_ERP.doCmd();
#endif
#ifdef SERIALCLI
  g_CLI.getInput();
#endif // SERIALCLI
#ifdef BTN_MENU
  g_BtnHandler.ChkBtn();
#endif
#ifdef TEMPERATURE_MONITORING
  g_TempMonitor.Read();  //   update temperatures once per second
#endif
}

void loop(){

  WDT_ENABLE();
    
  g_EvseController.Update();

  g_OBD.Update();

  ProcessInputs();
}

OnboardDisplay g_OBD;



