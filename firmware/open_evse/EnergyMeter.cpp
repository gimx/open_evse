#include <EnableInterrupt.h>

#include "open_evse.h"

#ifdef KWH_RECORDING

//S0 pulse counter
volatile unsigned long pulseCount = 0, pulseCountSession = 0;
volatile unsigned long pulsetime=0;     // holds time of occurence of interrupt 
//S0 pulse counter input pin, connect first S0 meter ouput to this pin, second S0 meter output goes to ground
#define S0_PULSE_PIN 6

EnergyMeter g_EnergyMeter;

// ISR runs each time S0 pulse is detected
void onS0Pulse(){
  unsigned long now = millis();

  //only count energy while charging
  if(g_EvseController.GetState() == EVSE_STATE_C){   
    pulseCount++;  
    
    //calculate current from time between pulses (assumption 1000imp/kWh)
    g_EvseController.SetChargingCurrent(3600000/((now-pulsetime)) * 1000/VOLTS_FOR_L2); 
  }
  
  pulsetime=now;
}

EnergyMeter::EnergyMeter()
{
  // S0 pulse counter output is usually open connector, hence pull-up
  pinMode(S0_PULSE_PIN, INPUT_PULLUP);
  // trigger on falling edge of pulse
  enableInterrupt(S0_PULSE_PIN | PINCHANGEINTERRUPT, onS0Pulse, FALLING);     

  m_bFlags = 0;
  m_wattSeconds = 0;
  m_lastUpdateMs = millis();
  m_wattHoursTot = eeprom_read_dword((uint32_t*)EOFS_KWH_ACCUMULATED);        // get the stored value for the kWh from eeprom

  if (eeprom_read_dword((uint32_t*)EOFS_KWH_ACCUMULATED) == 0xffffffff) { // Check for unitialized eeprom condition so it can begin at 0kWh
    eeprom_write_dword((uint32_t*)EOFS_KWH_ACCUMULATED,0); //  Set the four bytes to zero just once in the case of unitialized eeprom
  }
}

void EnergyMeter::Update()
{
  // 1. charging session begins when EV is connected, ends when EV disconnected
  // 2. we record only when the relay is closed
  // 3. total kWh updated only when session ends
  uint8_t evconnected = g_EvseController.EvConnected();

  if (!evConnected() && evconnected) {
    startSession();
  }
  else if (!evconnected && evConnected()) {
    endSession();
  }

//  if (inSession()) {
//	  m_lastUpdateMs = millis();
	  calcUsage();
//  }

  if (evconnected) setEvConnected();
  else clrEvConnected();
}

void EnergyMeter::calcUsage()
{
  unsigned long curms = millis();
  unsigned long dms = curms - m_lastUpdateMs;
  if (dms > KWH_CALC_INTERVAL_MS) {
      // no need to bother with voltage, we have energy measured directly
      // accumulate Watt Seconds for charging (scaled for 1000imp/kWh = 1 imp/Wh = 3600imp/Ws), adjust if necessary
      m_wattSeconds =  pulseCount*3600;  
      m_lastUpdateMs = curms;
  }
  if ((g_EvseController.GetState() != EVSE_STATE_C) && pulseCount) {
    m_wattHoursTot += pulseCount; //scaled for 1000imp/kWh = 1 imp/Wh = 3600imp/Ws, adjust if necessary
    eeprom_write_dword((uint32_t*)EOFS_KWH_ACCUMULATED,m_wattHoursTot);
    pulseCount = 0;    
    g_EvseController.SetChargingCurrent(0);
  }
}

void EnergyMeter::startSession()
{
  endSession();
  m_wattSeconds = 0;
  m_lastUpdateMs = millis();
  setInSession();
}

void EnergyMeter::endSession()
{
  if (inSession()) {
    clrInSession();
    
  }
}

void EnergyMeter::SaveTotkWh()
{
  eeprom_write_dword((uint32_t*)EOFS_KWH_ACCUMULATED,m_wattHoursTot);
}

#endif // KWH_RECORDING
