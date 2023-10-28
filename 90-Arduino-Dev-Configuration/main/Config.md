# Arduino IDE configuration notes
## IDE setup
- Set up the Arduino IDE with all warnings on. This might help catch some logic errors or issues in the code.
- Make sure that the Serial Monitor is running at 9600 Baud
## Program notes
Code was written mainly following this tutorial - https://learn.adafruit.com/adafruit-bmp280-barometric-pressure-plus-temperature-sensor-breakout/overview
Code used the following libraries:
- Adafruit_BMP280.h (provided for pressure sensor)
- Wire.h (Arduino reference libary)
- SPI.h (Arduino reference library)

The Adafruit library also comes with a bunch of examples on how to integrate their sensors in a program (bmp280test.ino), which was extremely useful.
