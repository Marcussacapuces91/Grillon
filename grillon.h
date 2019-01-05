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

class Grillon {

public:
  inline static void overflow() {
    t += 8;
  };

/**
 * Initialise le contrôleur.
 * - Les I/Os en sortie ;
 * - Le timer1 en FAST PWM 8bits.
 */
  void setup() {
    DDRB |= _BV(5); // PB5 (D13) -> led
    DDRB |= _BV(1); // PB1 (D9) -> OC1A
    DDRB |= _BV(2); // PB2 (D10) -> OC1B
  
    cli();  // stop interrupts
  
  // Timer1 reset all reg
    TCCR1A = 0;   // reset the value
    TCCR1B = 0;   // reset the value
    TCNT1 = 0;    // reset the value
    OCR1A = 0;    // reset compare match value
    OCR1B = 0;    // reset compare match value
    TIMSK1 = 0;   // reset mask reg
  
  // Timer1 : Fast PWM 8 bits ; 16 MHz => f = fclk / [N . (1+top)] = 16 MHz / 256 = 62,5 KHz
    TCCR1A = _BV(COM1A1) + /* _BV(COM1A0) + */            // Clear OC1A on compare, set at BOTTOM
             _BV(COM1B1) + _BV(COM1B0) +                  // Set OC1B on compare, clear at BOTTOM
             _BV(WGM10) /* _BV(WGM11) */ ;                // Fast PWM 8 bits
    TCCR1B = _BV(WGM12) + /* _BV(WGM13) + */              // Fast PWM 8 bits
             /* _BV(CS12) + _BV(CS11) + */ _BV(CS10);     // No Prescaling
   
    sei();  //enable interrupts
  };

/**
 * Exécute une séquence d'émission du signal puis attend.
 */
  void loop() {
    const int vol = _BV((rnd() & 0x07) + 1); // Calcul du vol entre [2..256].
    
    crisser(vol / 8);
    delay(120);
    crisser(vol / 4);
    delay(110);
    crisser(vol / 2);
    delay(100);
  
    while (rnd() > 0x8000000) {
      const int v = normale(vol, vol/16);
      crisser(v > 256 ? 256 : v); // pas de saturation.
      delay(normale(100, 20));
    }
    crisser(vol / 2);
  
    delay(normale(30000, 10000));
  }
    
protected:
/**
 * Émet une séquence de cri (env. 3 amplitudes : --o--O--O--)
 * @param vol Volume sonore maxi [0..256]
 */
  void crisser(const int vol = 256) {
    PORTB |= _BV(5); // PB5 (D13) -> led
    t = 0;
    TIMSK1 |= _BV(TOIE1);  // Overflow Interrupt Enable
    for (unsigned p = 0; p < ___Documents_grillon_16khz_8bits_snd_size; ++p) {
      const byte e1 = unsigned(pgm_read_byte(___Documents_grillon_16khz_8bits_snd + p)) * vol / 256; 
      OCR1A = OCR1B = e1;
      while (t % 25 >= 8) ; 
    }
    TIMSK1 &= !_BV(TOIE1);  // Overflow Interrupt Disable
    PORTB &= !_BV(5); // PB5 (D13) -> led
  }

/**
 * Retourne un entier pseudo-aléatoire sur 32 bits.
 * @see https://en.wikipedia.org/wiki/Linear_congruential_generator
 * @return Un entier pseudo-aléatoire.
 */
  inline long unsigned rnd() {
    return (seed = (seed * 69069LLU + 1) % 0x100000000);
  }

/**
 * Retourne un entier pseudo-aléatoire selon une distribution normale, centrée sur mean et avec un écart-type de dev.
 * @param meam Valeur moyenne.
 * @param dev Ecart-type.
 * @return un entier pseudo-aléatoire sur 32 bits suivant une distribution normale.
 */
  long unsigned normale(const int mean, const unsigned dev) {
    const long long unsigned r[] = { rnd(), rnd(), rnd(), rnd(), rnd(), rnd() };
    const long long unsigned i = (r[0] + r[1] + r[2] + r[3] + r[4] + r[5]) * dev / 0x100000000LLU - 3 * dev + mean;
    return i;
  }  

private:
/// Variable temps utilisée dans l'interruption d'overflow du timer1
  static volatile unsigned t;

/// Graine du générateur de nombres pseudo-aléatoires.
  static long unsigned seed;
};

volatile unsigned Grillon::t;
long unsigned Grillon::seed = 0;
