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

enum cpu_t { AVR_ATtiny85, AVR_ATmega328P };

/**
 * Classe principale de l'application.
 * @see https://www.disk91.com/2014/technology/hardware/arduino-atmega328p-low-power-consumption/
 */
#if defined(__AVR_ATmega328P__)
template<const cpu_t CPU = AVR_ATmega328P>
#elif defined(__AVR_ATtiny85__)
template<const cpu_t CPU = AVR_ATtiny85>
#else
  #error "neither __AVR_ATmega328P__, nor __AVR_ATtiny85__"
#endif
class Grillon {

public:

/**
 * Constructeur avec paramètres.
 */
  Grillon(const unsigned volMax = 256) :
    VOL_MAX(volMax)
    {}

/**
 * Méthode statique devant être appelée par l'interruption overflow.
 * Lève le flag à la fréquence de 16 KHz
 * f kHz * num / den ==> 16 kHz 
 */
  inline
  static void overflow() {
    enum { NUM = 32, DEN = 125 }; // f = 62.5 KHz
    static byte t = 0;
  
    t += NUM;
    if (t > DEN) { 
      t -= DEN;
      flag = true;
    }
  }  

/**
 * Initialise le contrôleur.
 * - Les I/Os en sortie ;
 * - Le timer en FAST PWM 8bits.
 */
  void setup();

/**
 * Exécute une séquence d'émission du signal puis attend.
 */
  inline
  void loop() {
    const int vol = (rnd() % (VOL_MAX - 16)) + 16; // Calcul du vol entre [16..256].

#if defined(__AVR_ATmega328P__) && defined(DEBUG)
    Serial.print(F("Vol: "));
    Serial.println(vol);
#endif

    timerOn();

    // 3 crissements progressifs
    crisser(vol / 8);
    delay(120);
    crisser(vol / 4);
    delay(110);
    crisser(vol / 2);
    delay(100);

    // Puis une série de longueur aléatoire et de volume aléatoire aussi
    while (rnd() < 0x0FE000000) { // si P < 0x0FE000000 / 0x0FFFFFFFF
      const auto v = normale(vol, vol/16);
      crisser(v > 256 ? 256 : v); // pas de saturation.
      delay(normale(160, 20));
    }
    crisser(vol / 2);

    timerOff();

    for (auto i = normale(16, 8); i > 0; --i) {
      sleeping();
    }
    wakeup();
  }

protected:
/**
 * Émet un cri à l'amplitude indiquée sur la sortie.
 * @param vol Volume sonore maxi [0..256]
 */
  void crisser(const unsigned vol = 256) {
    const unsigned char* p = ___Documents_grillon_16khz_8bits_snd;
    const unsigned char* const q = p + ___Documents_grillon_16khz_8bits_snd_size;
  
#if defined(__AVR_ATmega328P) && defined(DEBUG)
    PORTB |= _BV(5); // PB5 (D13) -> led
#endif
    
    while (p < q) {
      output((int(pgm_read_byte(p++)) - 0x80) * vol / 256 + 0x80);
      while (!flag) ;
      flag = false;
    }

#if defined(__AVR_ATmega328P) && defined(DEBUG)
    PORTB &= ~_BV(5); // PB5 (D13) -> led
    PORTB &= ~_BV(0);
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
 * @see https://fr.wikipedia.org/wiki/Loi_normale
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
 * Emet un échantillon en sortie.
 * @param aValue valeur à appliquer sur la sortie.
 */
  void output(const byte aValue);

/**
 * Initialise & Demarre le (bon) timer.
 */
  void timerOn() const;

/**
 * Arrête le Timer.
 */
  void timerOff() const;

/**
 * Active le Watchdog & met la CPU en sommeil.
 */
  void sleeping(void) const;
  
/**
 * Réveille le CPU après appel du Watchdog.
 */
  void wakeup() const {
    
#if defined(__AVR_ATmega328P__)
    wdt_disable();
    sleep_disable();

  #ifdef DEBUG
    power_usart0_enable(); // Needed for serial.print
    Serial.println(F("Wakeup!"));
    Serial.flush();
  #endif

#elif defined(__AVR_ATtiny85__)
    wdt_disable();
    sleep_disable();
#endif
    power_timer0_enable();
    power_timer1_enable();
// BOD is automatically restarted at wakeup
  }

private:
  const unsigned VOL_MAX;

/// Flag utilisé dans l'interruption d'overflow du timer1
  static volatile bool flag;

/// Graine du générateur de nombres pseudo-aléatoires.
  static long unsigned seed;
};

/// Initialisation du flag à false au départ.
template<const cpu_t CPU>
volatile bool Grillon<CPU>::flag = false;

/// Initialisation de graine du générateur de nombres pseudo-aléatoires à une valeur arbitraire.
template<const cpu_t CPU>
long unsigned Grillon<CPU>::seed = 12345;
