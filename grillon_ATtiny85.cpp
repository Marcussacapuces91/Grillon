/**
 *  Copyright (c) 2019 Marc Sibert
 *
 *  Permission is hereby granted, free of charge, to any person obtaining
 *  a copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without limitation
 *  the rights to use, copy, modify, merge, publish, distribute, sublicense,
 *  and/or sell copies of the Software, and to permit persons to whom the Software
 *  is furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 *  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 *  OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * @file grillon.h
 * @brief Declare & define Grillon class.
 * @licence https://opensource.org/licenses/mit-license.php
 * @author Marc Sibert
 */
 
#ifdef __AVR_ATtiny85__
#include <arduino.h>
#include "grillon.h"
  
/**
 * Setup, spécialisation ATtiny85.
 */
template<>
void Grillon<AVR_ATtiny85>::setup() {
// turn off brown-out enable in software
  MCUCR = _BV(BODS) | _BV(BODSE);
  MCUCR = _BV(BODS); 
  
  ADCSRA &= ~_BV(ADEN);             // Disable ADC
  ACSR = _BV(ACD);                  // Disable the analog comparator
  DIDR0 = 0x3F;                     // Disable digital input buffers on all ADC0-ADC3 pins & AIN1/0

  power_adc_disable();
  
// Power Reduction Register
//    PRR |= /* _BV(PRTIM1) | _BV(PRTIM0) | */ _BV(PRUSI) | _BV(PRADC); // Timer1, USI & ADC shutted down

  power_usi_disable();
//    power_timer0_disable();
//    power_timer1_disable();         // Needed for delay and millis()

// Toutes sorties actives
  DDRB = 0x1F;    // PB0 à PB5
  PORTB &= ~0x1F;
};

/**
 * Active le Watchdog & met la CPU en sommeil.
 */
template<>
void Grillon<AVR_ATtiny85>::sleeping(void) const {

/*
#ifdef DEBUG
  power_usart0_enable(); // Needed for serial.print
  Serial.println(F("Sleeping 8S!"));
  Serial.flush();
#endif

  cli();                           // disable interrupts for changing the registers

  MCUSR = 0;                       // reset status register flags

                                   // Put timer in interrupt-only mode:                                       
  WDTCSR = _BV(WDCE) | _BV(WDE);   // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
  WDTCSR = _BV(WDIE) | 0b100001;   // set WDIE: interrupt enabled
                                   // clr WDE: reset disabled
                                   // and set delay interval (right side of bar) to 8 seconds

  sei();                           // re-enable interrupts

  // reminder of the definitions for the time before firing
  // delay interval patterns:
  //  16 ms:     0b000000
  //  500 ms:    0b000101
  //  1 second:  0b000110
  //  2 seconds: 0b000111
  //  4 seconds: 0b100000
  //  8 seconds: 0b100001

  wdt_enable();
  wdt_reset();  // start watchdog timer
  set_sleep_mode (SLEEP_MODE_PWR_DOWN); // prepare for powerdown  
  sleep_enable(); 

#ifdef DEBUG
  power_usart0_disable(); // Needed for serial.print
#endif
  power_timer0_disable();
  power_timer1_disable();

  sleep_cpu ();   // power down !
*/  

  delay(8000);
}

/**
 * Règle et active le Timer0 en Fast PWM.
 */
template<> 
void Grillon<AVR_ATtiny85>::timerOn() const {
  cli();
  
// Timer0 set reggisters
  TCNT0 = 0;
  TIMSK |= _BV(TOIE0);  // Overflow interrupt

// Timer0 : Fast PWM 8 bits ; 16 MHz => f = 16 MHz / 256 = 62.5 KHz
  TCCR0A = _BV(COM0A1) | /* _BV(COM0A0) | */              // Clear OC0A on compare, set at TOP
           _BV(COM0B1) | _BV(COM0B0) |                    // Set OC0B on compare, set at TOP
           _BV(WGM01)  | _BV(WGM00);                      // Fast PWM (3)
  TCCR0B = /* _BV(WGM02) | */                             // Fast PWM (3)
           /* _BV(CS02) | _BV(CS01) | */ _BV(CS00);       // No Prescaling

  sei();
};

/**
 * Arrête le Timer0.
 */
template<>
void Grillon<AVR_ATtiny85>::timerOff() const {
  cli();  // disable interrupts

// Timer0 set registers
  TIMSK &= ~_BV(TOIE0);  // Disable Overflow Interrupt

// Timer0 : off
  TCCR0A = 0;
  TCCR0B = 0;

  sei();  // enable interrupts
}

/**
 * Émet un échantillon à la valeur indiquée sur les sorties PWM OC0A & OC0B (PB1 & PB2)
 * OCR0A & OCR0B en opposition de phase.
 * @param value Signal entier non signé sur 8 bits
 */
template<>
void Grillon<AVR_ATtiny85>::output(const byte value) {
  OCR0A = OCR0B = value;
}

/**
 * Timer0 overflow call Grillon::overflow()
 */
ISR(TIMER0_OVF_vect) {  
  Grillon<AVR_ATtiny85>::overflow();
}

#endif
