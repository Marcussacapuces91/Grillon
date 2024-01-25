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
 * @file Grillon.ino
 * @brief Main program
 * @licence https://opensource.org/licenses/mit-license.php
 * @author Marc Sibert
 */

#define DEBUG 1

#include "grillon.h"

// Grillon<> grillon(64);
Grillon<> grillon;

void setup() {
#if defined(DEBUG) && defined(__AVR_ATmega328P__ )
  Serial.begin(115200);
  while (!Serial) ;
  Serial.println();
  Serial.println(F("Starting Grillon!"));
#endif
/*  
  pinMode(1, OUTPUT);
 */
  grillon.setup();
}

void loop() {
/*  
  digitalWrite(1, HIGH);
  delay(100);
  digitalWrite(1, LOW);
  delay(1000);
*/  
  grillon.loop();
}
