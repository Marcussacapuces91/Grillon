# Grillon

Au départ, c'est un développement pour [Arduino](https://www.arduino.cc/) et son processeur [ATmega328](https://en.wikipedia.org/wiki/ATmega328) qui consiste a émettre le son d'un grillon à intervalle irrégulier.

Le système est conçu pour avoir une faible consommation électrique et fonctionner avec des piles ou des batteries pour être caché et rester longtemps sans intervention.

Au départ prévu pour l'[ATmega328](https://en.wikipedia.org/wiki/ATmega328), il se décline aussi pour l'[ATtiny85](https://www.microchip.com/en-us/product/attiny85) et la carte de développement Digispark.

## ATmega328

Il utilise principalement le `Timer1` afin de contrôler les broches 9 & 10 (9 -> `PB1` -> `OC1A` & 10 -> `PB2` -> `OC1B`) en mode "push-pull" afin d'activer un haut-parleur piézo à haute impédance, évitant l'utilisation d'un amplificateur.

## ATtiny85

Cette fois, on utilise le `Timer0` et les broches `PB1` -> `OCR0A` et `PB2` -> `OCR0B` pour attaquer le haut-parleur piezo.
