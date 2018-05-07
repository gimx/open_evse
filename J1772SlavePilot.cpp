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
#include <EnableInterrupt.h>

volatile uint32_t t0=0,t1=0,t2=0, tLow, tHigh;
volatile uint16_t uSecPulsewidth = 200; //init to 6A


void pilotHigh(){
  pinMode(PILOT_PIN,INPUT_PULLUP);
//  digitalWrite(PILOT_PIN,HIGH);
}

void pilotLow() {
  digitalWrite(PILOT_PIN,LOW);
  pinMode(PILOT_PIN,OUTPUT);
}

void onMasterPilotChange() {
  int state = digitalRead(MASTER_PILOT_PIN); 
  
  if (state == HIGH) { //rising MASTER
    t0=  micros();
    
    // phase delay to trick MASTER charger electronic checking output signal, i.e. testing whether high after X us, 
    // if the shortened pulse would be low already then, the charger reports a failure 
    //delayMicroseconds(MASTER_SLAVE_PHASE_DELAY_US);
    
    //rise
    pilotHigh();
    
    //drive low after set pulse width, to shorten high phase of MASTER_PILOT
    delayMicroseconds(uSecPulsewidth);
    pilotLow(); 
  
    tLow = t0-t1;
  } 
  else{//falling MASTER
    t1 = micros();
    tHigh = t1-t0;
    
    //slave pulse shall not extend beyond MASTER pulse, i.e. chargers limit, 
    //this has to be ensured with the external diodes and the open collector(INPUT_PULLUP) output of pilot 
    pilotHigh();

  }
}

void J1772SlavePilot::Init()
{
  pinMode(MASTER_PILOT_PIN, INPUT_PULLUP);
  
  digitalWrite(PILOT_PIN, digitalRead(MASTER_PILOT_PIN));
  pinMode(PILOT_PIN, OUTPUT);
 
  //master pilot sensing
  enableInterrupt(MASTER_PILOT_PIN, onMasterPilotChange, CHANGE);   
}


// no PWM pilot signal - steady state, dont do anything, but follow the master
// PILOT has to be externally NORed with MASTER pilot signal
// two diodes with cathode to signal sources, i.e. MASTER PILOT and PILOT pins, joined anodes and pull-up
// this will ensure that 
// 1. duty cycle stays below MASTER regardless of commanding,
// 2. MASTER failure overwrites any PWM or positive output on pilot

void J1772SlavePilot::SetState(PILOT_STATE state)
{
  pilotHigh(); // do not interfere with master, high Z
  
  if (state == PILOT_STATE_P12) {
//    pilotHigh();
  }
  else{
//    pilotLow();
  }
  
  m_State = state;
}


// set EVSE current capacity in Amperes
// duty cycle 
int J1772SlavePilot::SetPWM(int amps)
{
  int16_t limit = SenseMaster(); 
  amps = min( max(0,limit), amps); // limit commanding by amps value sensed on master pilot

  uSecPulsewidth = 0;
  if ((amps >= 6) && (amps <= 51)) {
    uSecPulsewidth = 100 * amps / 6;  // J1772 states "Available current = (duty cycle %) X 0.6"
  } else if ((amps > 51) && (amps <= 80)) {
    uSecPulsewidth = ((amps/2.5)+64)*10;  // J1772 states "Available current = (duty cycle % - 64) X 2.5"
  }
  else {
    return 1; // error
  }

//overwrite setting 5% duty cycle to signal digital communication
  if (digitalRead(DCOM_ENAB_PIN) == LOW){
    uSecPulsewidth = 50;
  }
  
#ifdef SERDBG
  Serial.print(uSecPulsewidth);Serial.print(" usec, i.e. amps:");Serial.println(amps);
#endif  
  if (uSecPulsewidth) {    
    pinMode(PILOT_PIN, OUTPUT);
    m_State = PILOT_STATE_PWM;
    return 0;
  }
  else { // !duty
    // invalid amps
    return 1;
  }
}


//returns amps and state of incoming master PWM signal 
int J1772SlavePilot::SenseMaster()
{
  uint32_t tPeriod = tLow+tHigh;
  uint16_t duty = (uint16_t)((double)tHigh*100/tPeriod);
  uint16_t amps = 0;

  if (tPeriod == 0){//steady master, nothing measured yet
    return 0;
  }
  
  if(tPeriod<950 || tPeriod>1050){
    //check if the master PWM signal is standard compliant (within measurement tolerances of the Arduino)
  #ifdef SERDBG
    Serial.print("Pilot PWM outside of 5% tolerance of 1kHz. tP[ms]="); Serial.println(tPeriod);
  #endif
    return -1;
  } 
  if (duty <=5){
  #ifdef SERDBG
    Serial.println("Digital communication request sensed on master.");
  #endif
    return -2;
  }
  if (duty <= 85){
       amps = (uint16_t)(duty*0.6);
  }
  else{
     amps = (uint16_t)((duty-64)*2.5);
  }
  #ifdef SERDBG
    Serial.print(tHigh);Serial.print(", ");Serial.print(tLow);Serial.println(", ");
    Serial.print(duty);Serial.print(", ");Serial.print(amps);Serial.println(", ");
  #endif //#ifdef SERDBG
  return amps;  
}

void J1772SlavePilot::ReadPilot(uint16_t *plow, uint16_t *phigh)
{
  uint16_t pl = 1023;
  uint16_t ph = 0;

  // 1x = 114us 20x = 2.3ms 100x = 11.3ms
  for (int i = 0; i < PILOT_LOOP_CNT; i++) {
    uint16_t reading = adcPilot.read();  // measures pilot voltage

    if (reading > ph) {
      ph = reading;
    }
    else if (reading < pl) {
      pl = reading;
    }
  } 
  
  if (plow!=NULL && phigh!=NULL) {
    *plow = pl;
    *phigh = ph;
  }
}


PILOT_STATE J1772SlavePilot::GetState()  { 
  PILOT_STATE state = PILOT_STATE_PWM;
  uint16_t pl = 1023;
  uint16_t ph = 0;
  int16_t deltap = 0;

  ReadPilot(&pl, &ph);

  deltap = ph-pl;
 
  if (deltap<10){ //pilot steady 
    if (ph >= 500){
      m_State = PILOT_STATE_P12;
      pilotHigh();
    }
    if (pl < 500){
      m_State = PILOT_STATE_N12;
    }
  }else{
    PILOT_STATE_PWM;
  }

  return state; 
}
