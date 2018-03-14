# Bela integration of the Adafruit FXOS8700 and FXAS21002C 9-DOF breakout board

Reading values on a Bela (http://bela.io) from the I2C board from Adafruit (https://learn.adafruit.com/nxp-precision-9dof-breakout/overview)

## Getting Started

Connect the 9-DOF sensor to the I2C pins on the Bela. Put the code in a project (I use the project name Adafruit_IMU) on your Bela and run via IDE or Bela script (may require the better-i2c branch for the moment 2018-03-12) to compile. Then while logged in on the Bela through ssh into the bela using
```ssh root@192.168.7.2```
you have to manually stop the process to free the IDE
`pkill -9 Adafruit_IMU` (or whatever you call your project)

subsequently run the program by going to the folder it is in (e.g. ~/Bela/projects/Adafruit_IMU) and run it
```./Adafruit_IMU```
Then run your other code through the IDE. Press `CTRL+C` to stop the program.

## Built With

I used code from the Adafruit Arduino libraries:
* https://github.com/adafruit/Adafruit_FXOS8700
* https://github.com/adafruit/Adafruit_FXAS21002C

There is also a slightly adapted version of the Madgwick sensor fusion algorithm available here:
* http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/


## License

The code in Adafruit_FXOS8700.cpp/.h and Adafruit_FXAS21002C.cpp/.h is licensed under the MIT License in accordance with the Adafruit original license - see the [LICENSE.md](LICENSE.md) file for details.
The code in render.cpp is licensed under LGPL 3.0 in accordance with the Bela examples code it was based on.
