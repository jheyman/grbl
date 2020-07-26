/*
  sleep.c - determines and executes sleep procedures
  Part of Grbl
  
  Copyright (c) 2016 Sungeun K. Jeon  

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h" 
#include <avr/wdt.h>

#define SLEEP_COUNT_MAX (SLEEP_DURATION/0.032) // WDT set to expire every 32ms

volatile uint8_t sleep_counter;

// Initialize sleep counters and enable timer.
static void sleep_enable() { 
  sleep_counter = 0; // Reset sleep counter
  if (!(WDTCSR & (1<<WDIE))) WDTCSR |= (1<<WDIE); // enable watchdog timer interrupt
} 

// Disable sleep timer.
static void sleep_disable() 
{
  WDTCSR &= ~(1<<WDIE); // Disable watchdog timer interrupt. 
}

// Initialization routine for sleep timer.
void sleep_init()
{
  // configure the watchdog timer 
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = (1<<WDP0); // Set time-out at ~32msec.
  sleep_disable();
}

// Increment sleep counter each time the watchdog expires
ISR(WDT_vect) { sleep_counter++; };

// Starts sleep timer if running conditions are satified. When elaped, sleep mode is executed.
static void sleep_execute()
{
  // Fetch current number of buffered characters in serial RX buffer.
  uint8_t rx_initial = serial_get_rx_buffer_count();

  // Enable sleep counter
  sleep_enable();

  do {          
    // Monitor for any new RX serial data or external events (queries, buttons, alarms) to exit.
    if ( (serial_get_rx_buffer_count() > rx_initial) || sys_rt_exec_state || sys_rt_exec_alarm ) {
      // Disable sleep timer and return to normal operation.
      sleep_disable();  
      return;
    }
  } while(sleep_counter <= SLEEP_COUNT_MAX);
  
  // If reached, sleep counter has expired. Execute sleep procedures.
  // Notify user that Grbl has timed out and will be parking. 
  // To exit sleep, resume or reset. Either way, the job will not be recoverable. 
  report_feedback_message(MESSAGE_SLEEP_MODE);
  system_set_exec_state_flag(EXEC_SLEEP);
}


// Checks running conditions for sleep. If satisfied, enables sleep countdown and executes
// sleep mode upon elapse.
// NOTE: Sleep procedures can be blocking, since Grbl isn't receiving any commands, nor moving.
// Hence, make sure any valid running state that executes the sleep timer is not one that is moving.
void sleep_check()
{
  // The sleep execution feature will continue only if the machine is in an IDLE or HOLD state and
  // has any powered components enabled.
  // NOTE: With overrides or in laser mode, modal spindle and coolant state are not guaranteed. Need 
  // to directly monitor and record running state during parking to ensure proper function.
  if (gc_state.modal.spindle || gc_state.modal.coolant) {
    if (sys.state == STATE_IDLE) { 
      sleep_execute();
    } else if ((sys.state & STATE_HOLD) && (sys.suspend & SUSPEND_HOLD_COMPLETE)) {
      sleep_execute();
    } else if (sys.state == STATE_SAFETY_DOOR && (sys.suspend & SUSPEND_RETRACT_COMPLETE)) {
      sleep_execute();
    }
  }
}  
