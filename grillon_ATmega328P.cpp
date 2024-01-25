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
 
#ifdef __AVR_ATmega328P__
#include <arduino.h>
#include "grillon.h"

/**
 * Setup, spécialisation ATmega328P.
 */
template<>
void Grillon<AVR_ATmega328P>::setup() {
// turn off brown-out enable in software
  MCUCR = _BV(BODS) | _BV(BODSE);
  MCUCR = _BV(BODS); 
  
  ADCSRA &= ~_BV(ADEN);             // Disable ADC
  ACSR = _BV(ACD);                  // Disable the analog comparator
  DIDR0 = 0x3F;                     // Disable digital input buffers on all ADC0-ADC5 pins
  DIDR1 = _BV(AIN1D) | _BV(AIN0D);  // Disable digital input buffer on AIN1/0

  power_adc_disable();
  power_twi_disable();
  power_spi_disable();
  #ifndef DEBUG
  power_usart0_disable();           // Needed for serial.print
  #endif
//    power_timer0_disable();         // Needed for delay and millis()
//    power_timer1_disable();
  power_timer2_disable();           // Needed for asynchronous 32kHz operation
      
// Toutes sorties actives
  DDRB = 0x3F;    // sauf 6 & 7 -> xtal
  PORTB &= ~0x3F;
  DDRC = 0xFF;
  PORTC = 0x00;
  DDRD = 0xFF;
  PORTD = 0x00;
  
  #ifdef DEBUG
  DDRB |= _BV(5); // PB5 (D13) -> led
  #endif
};

/**
 * Active le Watchdog & met la CPU en sommeil.
 */
template<>
void Grillon<AVR_ATmega328P>::sleeping(void) const {
 
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

  wdt_reset();  // start watchdog timer
  wdt_enable(WDTO_8S);

  set_sleep_mode (SLEEP_MODE_PWR_DOWN); // prepare for powerdown  
  sleep_enable(); 

  power_usart0_disable();
  power_timer0_disable();
  power_timer1_disable();

  sleep_cpu ();   // power down !
}

  
/**
 * Règle et active le Timer1 en Fast PWM.
 */
template<> 
void Grillon<AVR_ATmega328P>::timerOn() const {
  cli();  // stop interrupts

// Timer1 set registers
  TCNT1 = 0;    // reset the value of Timer1 counter
  TIMSK1 |= _BV(TOIE1);  // Overflow Interrupt Enable
#ifdef DEBUG    
//    TIMSK1 |= _BV(OCIE1A);  
#endif

// Timer1 : Fast PWM 8 bits ; 16 MHz => f = fclk / [N . (1+top)] = 16 MHz / 256 = 62,5 KHz
  TCCR1A = _BV(COM1A1) + /* _BV(COM1A0) + */            // Clear OC1A on compare, set at BOTTOM
           _BV(COM1B1) + _BV(COM1B0) +                  // Set OC1B on compare, clear at BOTTOM
           _BV(WGM10) /* _BV(WGM11) */ ;                // Fast PWM 8 bits
  TCCR1B = _BV(WGM12) + /* _BV(WGM13) + */              // Fast PWM 8 bits
           /* _BV(CS12) + _BV(CS11) + */ _BV(CS10);     // No Prescaling
 
  sei();  //enable interrupts
};

/**
 * Arrête le Timer1.
 */
template<>
void Grillon<AVR_ATmega328P>::timerOff() const {
  cli();  // disable interrupts

// Timer1 set registers
  TIMSK1 &= ~_BV(TOIE1);  // Disable Overflow Interrupt

// Timer1 : off
  TCCR1A = 0;
  TCCR1B = 0;

  sei();  // enable interrupts
}

/**
 * Règle le PWM à la valeur indiquée, en opposition de phase.
 * OCR1A & OCR1B en opposition de phase.
 * @param value Signal entier non signé sur 8 bits --> PWM
 */
template<>
void Grillon<AVR_ATmega328P>::output(const byte value) {
  OCR1A = OCR1B = value; 
}

/**
 * Timer1 comparator A.
 */
#ifdef DEBUG
ISR(TIMER1_COMPA_vect){  
  PORTB &= ~_BV(0);
}
#endif

/**
 * Timer1 overflow call Grillon::overflow()
 */
ISR(TIMER1_OVF_vect) {  
#ifdef DEBUG
  PORTB |= _BV(0);
#endif
  Grillon<AVR_ATmega328P>::overflow();
}

#endif
