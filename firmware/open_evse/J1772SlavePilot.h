#pragma once
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
#include "J1772Pilot.h"

class J1772SlavePilot:public J1772Pilot{
public:
  PILOT_STATE GetState();
  void Init();
  void SetState(PILOT_STATE pstate); // P12/N12 
  int SetPWM(int amps); // 12V 1KHz PWM
  int SenseMaster(); 
  void ReadPilot(uint16_t *plow=NULL,uint16_t *phigh=NULL);
};
