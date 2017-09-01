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

#define OSP_SET_WIDTH(cycles) (OCR2B = 0xff-(cycles-1))
//#define OSP_SET_WIDTH(cycles) (OCR2A = 0xff-(cycles-1))

// Setup the one-shot pulse generator and initialize with a pulse width that is (cycles) clock counts long
void osp_setup(uint8_t cycles) {
  TCCR2B =  0;      // Halt counter by setting clock select bits to 0 (No clock source).
//  TCCR2A =  0;      // Halt counter by setting clock select bits to 0 (No clock source).
                    // This keeps anyhting from happeneing while we get set up

  TCNT2 = 0x00;     // Start counting at bottom. 
  OCR2B = 0;      // Set TOP to 0. This effectively keeps us from counting becuase the counter just keeps reseting back to 0.
          // We break out of this by manually setting the TCNT higher than 0, in which case it will count all the way up to MAX and then overflow back to 0 and get locked up again.
  OSP_SET_WIDTH(cycles);    // This also makes new OCR values get loaded frm the buffer on every clock cycle. 

  TCCR2A = _BV(COM2B0) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); // OC2B=Set on Match, clear on BOTTOM. Mode 7 Fast PWM.
  //TCCR2A = _BV(COM2A0) | _BV(COM2A1) | _BV(WGM20) | _BV(WGM21); // OC2B=Set on Match, clear on BOTTOM. Mode 7 Fast PWM.
  TCCR2B = _BV(WGM22)| _BV(CS22);         // Start counting now. WGM22=1 to select Fast PWM mode 7
}


// Fire a one-shot pulse. Use the most recently set width. 
#define OSP_FIRE() (TCNT2 = OCR2B - 1)
//#define OSP_FIRE() (TCNT2 = OCR2A - 1)

// Test there is currently a pulse still in progress
#define OSP_INPROGRESS() (TCNT2>0)

// Fire a one-shot pusle with the specififed width. 
// Order of operations in calculating m must avoid overflow of the unint8_t.
// TCNT2 starts one count lower than the match value becuase the chip will block any compare on the cycle after setting a TCNT. 
#define OSP_SET_AND_FIRE(cycles) {uint8_t m=0xff-(cycles-1); OCR2B=m; TCNT2=m-1;}
//#define OSP_SET_AND_FIRE(cycles) {uint8_t m=0xff-(cycles-1); OCR2A=m; TCNT2=m-1;}


#define TOP ((F_CPU / 2000000) * 1000) // for 1KHz (=1000us period)

volatile uint32_t t0=0,t1=0,t2=0, tLow, tHigh;
volatile uint8_t cycles = 10, dutyCycleChanged=true;

void onMasterPilotChange() {
  int state = digitalRead(MASTER_PILOT_PIN);
  
  if (state == HIGH) { //rising 
    if (dutyCycleChanged){ 
      OSP_SET_AND_FIRE(cycles);
      dutyCycleChanged = false;
    }
    else 
      OSP_FIRE();
  
    t0=  micros();
    tLow = t0-t1;
  } 
  else{//falling
    t1 = micros();
    tHigh = t1-t0;
  }

}

void J1772Pilot::Init()
{
  pinMode(MASTER_PILOT_PIN, INPUT);
  
  pinMode(PILOT_PIN, OUTPUT);
  digitalWrite(PILOT_PIN, HIGH);
    
  //master pilot sensing
  attachInterrupt(digitalPinToInterrupt(MASTER_PILOT_PIN), onMasterPilotChange, CHANGE); 
  SenseMaster(); //TODO set sensed max amps
}


// no PWM pilot signal - steady state
// PILOT_STATE_P12 = steady +12V (EVSE_STATE_A - VEHICLE NOT CONNECTED)
// PILOT_STATE_N12 = steady -12V (EVSE_STATE_F - FAULT) 
void J1772Pilot::SetState(PILOT_STATE state)
{
  
  if (state == PILOT_STATE_P12) {
    TCCR2A &= ~(_BV(COM2B0)|_BV(COM2B1));
//    digitalWrite(PILOT_PIN,HIGH);
  }
  else{
    TCCR2A &= ~(_BV(COM2B0)|_BV(COM2B1));
//    digitalWrite(PILOT_PIN,LOW);
  }
  
  m_State = state;
}


// set EVSE current capacity in Amperes
// duty cycle 
// outputting a 1KHz square wave to digital pin 10 via Timer 1
//
int J1772Pilot::SetPWM(int amps)
{

  uint8_t ocr1b = 0;
  if ((amps >= 6) && (amps <= 51)) {
    ocr1b = 25 * amps / 6 - 1;  // J1772 states "Available current = (duty cycle %) X 0.6"
  } else if ((amps > 51) && (amps <= 80)) {
    ocr1b = amps + 159;  // J1772 states "Available current = (duty cycle % - 64) X 2.5"
  }
  else {
    return 1; // error
  }
#ifdef SERDBG
  Serial.print(ocr1b);Serial.print("cycles. amps:");Serial.println(amps);
#endif  
  if (ocr1b) {
    AutoCriticalSection asc;
    
    // 10% = 24 , 96% = 239
    cycles = ocr1b;
    osp_setup(cycles); 
    dutyCycleChanged = true;
    
    m_State = PILOT_STATE_PWM;
    return 0;
  }
  else { // !duty
    // invalid amps
    return 1;
  }
}


//returns amps and state of incoming master PWM signal as defined by 
int J1772Pilot::SenseMaster()
{
  uint32_t tPeriod = tLow+tHigh;
  uint16_t duty = (uint16_t)((double)tHigh*100/tPeriod);
  uint16_t amps = 0;
  
  if(tPeriod<950 || tPeriod>1050){
    //check if the master PWM signal is standard compliant (within measurement tolerances of the Arduino)
  #ifdef SERDBG
    Serial.print("Pilot PWM outside of 5% tolerance of 1kHz."); Serial.println(tPeriod);
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


