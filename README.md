# NIST-Arduino-Uno-Time-System

This is a repository that holds all of the code for the NIST Arduino Uno based time system that collects WWV AM, WWV BPSK, and GPS signals and distributes them via LoRan to child Arduinos.

Credit to Bruce E. Hall for the code used in decoding the WWV AM data and error checking. His tutorial on this code is linked here: [WWVB Clock tutorial](https://universal-solder.ca/downloads/WWVB%20Clock%20Project%20by%20Bruce%20Hall.pdf)

There are multiple 3rd party libraries used, which are linked here:
[Timelib.h](https://github.com/PaulStoffregen/Time)
[RH_RF95.h](https://github.com/epsilonrt/RadioHead)
[Adafruit_GPS.h](https://github.com/adafruit/Adafruit_GPS)

All other libraries are pre-installed on the Arduino IDE, if an error ocurrs, a quick internet search of the library name will allow the user to download them off of GitHub.
