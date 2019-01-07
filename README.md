# Grillon

C'est un développement pour Arduino (https://en.wikipedia.org/wiki/ATmega328) qui consiste a émettre le son d'un grillon.
Le système est conçu pour avoir une faible consommation électrique et fonctionner avec des piles ou des batteries qui assurent une tension entre 4 et 5v env. Il fonctionne à une fréquence de 16MHz.

Il utilise principalement le Timer1 afin de contrôler les broches 9 & 10 (9 -> PB1 -> OC1A & 10 -> PB2 -> OC1B) en mode "push-pull" afin d'activer un haut-parleur piézo à haute impédance, évitant l'utilisation d'un amplificateur.

L'objectif étant de déployer cette application sur Arduino Pro Mini, des aménagements particuliers seront peut être à prévoir pour cette carte (flasher sans bootloader, désactiver le quartz 16 MHz au profit de l'oscillateur interne à 8 MHz, etc.)

