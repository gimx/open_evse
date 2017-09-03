#include "open_evse.h"

#define S0_PULSE_IDX 6


//S0 pulse counter
const byte min_pulsewidth= 110;                                // minimum width of interrupt pulse (default pulse output meters = 100ms)
volatile byte pulseCount = 0;
unsigned long pulsetime=0;                                    // Record time of interrupt pulse
unsigned long start=0;

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
  pinMode(S0_PULSE_IDX, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(S0_PULSE_IDX), onPulse, FALLING);     // Attach pulse counting interrupt pulse counting
  
#ifdef KWH_RECORDING
  if (eeprom_read_dword((uint32_t*)EOFS_KWH_ACCUMULATED) == 0xffffffff) { // Check for unitialized eeprom condition so it can begin at 0kWh
      eeprom_write_dword((uint32_t*)EOFS_KWH_ACCUMULATED,0); //  Set the four bytes to zero just once in the case of unitialized eeprom
  }

  g_WattHours_accumulated = eeprom_read_dword((uint32_t*)EOFS_KWH_ACCUMULATED);        // get the stored value for the kWh from eeprom
#endif // KWH_RECORDING

}

 
// ISR runs each time a falling edge of a pulse is detected
void onPulse(){
  if ( (millis() - pulsetime) > min_pulsewidth) {
    pulseCount++;          //calculate wh elapsed from time between pulses
  }
  pulsetime=millis();
  //Watt = 1/1000 Wh / pulsetime
}

// Buffer to store incoming commands from serial port
String inData;

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

  g_EvseController.Update();

  g_OBD.Update();

  ProcessInputs();
    
//    Serial.println(pulseCount);
    g_WattSeconds = pulseCount * 1.2;

}

OnboardDisplay g_OBD;




