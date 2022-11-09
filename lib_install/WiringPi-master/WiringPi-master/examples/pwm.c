/*
 * pwm.c:
 *	This tests the hardware PWM channel.
 *
 * Copyright (c) 2012-2013 Gordon Henderson.
 ***********************************************************************
 * This file is part of wiringPi:
 *      https://github.com/WiringPi/WiringPi
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <wiringPi.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

int main (void)
{
  int bright ;

  printf ("Raspberry Pi wiringPi PWM test program\n") ;

  if (wiringPiSetup () == -1)
    exit (1) ;

  pinMode (18, PWM_OUTPUT) ;

  pwmSetMode(PWM_MODE_MS);
  pwmSetClock(19);
  pwmSetRange(1024);

  
  pwmWrite (18, 512) ;
  for (;;)
  {

      
  }

  return 0 ;
}
