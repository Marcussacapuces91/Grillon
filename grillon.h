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
 
#pragma once

#include "Documents_grillon_16khz_8bits_snd.h"
#include <avr/wdt.h>            // library for default watchdog functions
#include <avr/sleep.h>          // library for sleep
#include <avr/power.h>          // library for power control

/**
 * Classe principale de l'application.
 */
class Grillon {

public:
/**
 * Méthode statique devant être appelée par l'interruption overflow.
 */
  inline static void overflow() {
// 62.5 kHz * num / den ==> 16 kHz 
    enum { num = 32, den = 125 };

    t += num;
    if (t > den) { 
      t -= den;
      flag = true;
    }
  };

  inline static void watchdog() {
    wdt_disable();     
  }

/**
 * Initialise le contrôleur.
 * - Les I/Os en sortie ;
 * - Le timer1 en FAST PWM 8bits.
 */
  void setup() {
// turn off brown-out enable in software
    MCUCR = _BV(BODS) | _BV(BODSE);
    MCUCR = _BV(BODS); 
    
    ADCSRA &= ~_BV(ADEN);             // Disable ADC
    ACSR = _BV(ACD);                  // Disable the analog comparator
    DIDR0 = 0x3F;                     // Disable digital input buffers on all ADC0-ADC5 pins
    DIDR1 = _BV(AIN1D) | _BV(AIN0D);  // Disable digital input buffer on AIN1/0

    power_twi_disable();
    power_spi_disable();
#ifndef DEBUG
    power_usart0_disable(); // Needed for serial.print
#endif
//    power_timer0_disable(); // Needed for delay and millis()
//    power_timer1_disable();
    power_timer2_disable(); //Needed for asynchronous 32kHz operation
        
/*
    DDRB = _BV(1) | // PB1 (D9) -> OC1A
           _BV(2);  // PB2 (D10) -> OC1B
*/           
// Toutes sorties actives
    DDRB = 0x3F;    // sauf 6 & 7 -> xtal
    DDRC = 0xFF;
    DDRD = 0xFF;
#ifdef DEBUG
    DDRB |= _BV(5); // PB5 (D13) -> led
#endif
  };

/**
 * Exécute une séquence d'émission du signal puis attend.
 */
  void loop() {
    const int vol = _BV((rnd() & 0x07) + 1); // Calcul du vol entre [2..256].

    setTimer1On();
    crisser(vol / 8);
    delay(120);
    crisser(vol / 4);
    delay(110);
    crisser(vol / 2);
    delay(100);

    while (rnd() > 0x8000000) {
      const int v = normale(vol, vol/16);
      crisser(v > 256 ? 256 : v); // pas de saturation.
      delay(normale(160, 20));
    }
    crisser(vol / 2);
    setTimer1Off();

    for (int i = normale(4, 2); i > 0; --i) {
      startSleeping();
      wakeup();
    }
  }

protected:
/**
 * Émet une séquence de cri (env. 3 amplitudes : --o--O--O--)
 * @param vol Volume sonore maxi [0..256]
 */
  void crisser(const int vol = 256) {
#ifdef DEBUG
    PORTB |= _BV(5); // PB5 (D13) -> led
#endif
    const unsigned char *p = ___Documents_grillon_16khz_8bits_snd;
    for (unsigned i = ___Documents_grillon_16khz_8bits_snd_size; i > 0; --i) {
      OCR1A = OCR1B = (int(pgm_read_byte(p++)) - 0x80) * vol / 256 + 0x80; 
      while (!flag) ; 
      flag = false;
    }
#ifdef DEBUG
    PORTB &= ~_BV(5); // PB5 (D13) -> led
#endif
  }

/**
 * Retourne un entier pseudo-aléatoire sur 32 bits.
 * @see https://en.wikipedia.org/wiki/Linear_congruential_generator
 * @return Un entier pseudo-aléatoire.
 */
  long unsigned rnd() const {
    return (seed = (seed * 69069LLU + 1) % 0x100000000);
  }

/**
 * Retourne un entier pseudo-aléatoire selon une distribution normale, centrée sur mean et avec un écart-type de dev.
 * @param meam Valeur moyenne.
 * @param dev Ecart-type.
 * @return un entier pseudo-aléatoire sur 32 bits suivant une distribution normale.
 */
  long unsigned normale(const int mean, const unsigned dev) const {
    const long long unsigned r[] = { rnd(), rnd(), rnd(), rnd(), rnd(), rnd() };
    const long long unsigned i = (r[0] + r[1] + r[2] + r[3] + r[4] + r[5]) * dev / 0x100000000LLU - 3 * dev + mean;
    return i;
  }

/**
 * Règle et active le Timer1 en Fast PWM @ 62,5 kHz.
 */
  void setTimer1On() const {
    cli();  // stop interrupts

// Timer1 set registers
    TCNT1 = 0;    // reset the value of Timer1 counter
    TIMSK1 = _BV(TOIE1);  // Overflow Interrupt Enable
    t = 0;
  
// Timer1 : Fast PWM 8 bits ; 16 MHz => f = fclk / [N . (1+top)] = 16 MHz / 256 = 62,5 KHz
    TCCR1A = _BV(COM1A1) + /* _BV(COM1A0) + */            // Clear OC1A on compare, set at BOTTOM
             _BV(COM1B1) + _BV(COM1B0) +                  // Set OC1B on compare, clear at BOTTOM
             _BV(WGM10) /* _BV(WGM11) */ ;                // Fast PWM 8 bits
    TCCR1B = _BV(WGM12) + /* _BV(WGM13) + */              // Fast PWM 8 bits
             /* _BV(CS12) + _BV(CS11) + */ _BV(CS10);     // No Prescaling
   
    sei();  //enable interrupts
/*
    for (byte v = 0; v < 0x80; ++v) {
      OCR1A = v;
      OCR1B = 255 - v;
      delayMicroseconds(10);
    }
*/
  }

/**
 * Arrête le Timer1.
 */
  void setTimer1Off() const {
/*
    for (byte v = OCR1A; v > 0; --v) {
      OCR1A = v;
      OCR1B = 255 - v;
      delayMicroseconds(10);
    }
*/    
    cli();  // disable interrupts

// Timer1 set registers
    TIMSK1 = 0;  // Disable all Timer1's interrupts

// Timer1 : off
    TCCR1A = 0;
    TCCR1B = 0;

    sei();  // enable interrupts
  }

  void startSleeping(void) const {
   
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
    set_sleep_mode (SLEEP_MODE_PWR_DOWN); // prepare for powerdown  
    sleep_enable(); 
  
#ifdef DEBUG
    power_usart0_disable(); // Needed for serial.print
#endif
    power_timer0_disable(); // Needed for delay and millis()
    power_timer1_disable();
  
    sleep_cpu ();   // power down !
  }  

  void wakeup() const {
    sleep_disable();
#ifdef DEBUG
    power_usart0_enable(); // Needed for serial.print
#endif
    power_timer0_enable();
    power_timer1_enable();
// BOD is automatically restarted at wakeup
  }

private:
/// Variable temps utilisée dans l'interruption d'overflow du timer1
  static volatile unsigned t;
  static volatile bool flag;

/// Graine du générateur de nombres pseudo-aléatoires.
  static long unsigned seed;
};

volatile unsigned Grillon::t = 0;
volatile bool Grillon::flag = false;

long unsigned Grillon::seed = 12345;

/**
 * Timer1 overflow call Grillon::overflow()
 */
ISR(TIMER1_OVF_vect){  
  Grillon::overflow();
}

ISR(WDT_vect){  
  Grillon::watchdog();
}
