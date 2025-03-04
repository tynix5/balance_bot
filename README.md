# Balance Bot #
- 2 wheeled balancing robot centered around the STM32F401RE microcontroller
- Utilizes BNO055 IMU to determine orientation in space

## Design ##
At a top level, this project is fairly straightforward: determine the bot's orientation, then correct it via motors. The STM32 reads euler angles from the IMU using I2C at a rate of 100 Hz. The euler angles are computed by the sensor itself, configured in the "sensor fusion" mode.
Based on the physical orientation of the sensor itself, the pitch is the angle we are focused on. At equilibrium (in an upright, balanced position), the pitch will read exactly 0 degrees. If the bot were to tilt forwards or backwards, the pitch would read a nonzero number,
which is interpreted as "error". It is an error in the sense that any we are attempting to stabilize the bot at exactly 0 degrees pitch, so the larger the magnitude of pitch, the larger the angle from equilibirum, the larger the error. This error is fed into a PID control loop
whose output is then mapped to a range of PWM values, used to drive the motors. The internal clock speed of the STM32 is configured to 84 MHz for the fastest response.

## Construction ##
- Initial build is shown below
- Hand cut and drilled plywood used for construction of frame
- The two bases are separated by spacers, and connected using hobby screws and nuts
![image](https://github.com/user-attachments/assets/075075a5-caa8-4c1e-b2ab-431e65b59f97)
![image](https://github.com/user-attachments/assets/7bc06242-e929-4888-a3ab-802111ed4c80)
![image](https://github.com/user-attachments/assets/86369cb9-baa2-450e-8b83-71f13cdc9d10)
- Future design will be 3D printed

## Power ##
The design is powered by a 9V battery for now. In the future, a LiPo battery will be substituted. The STM32 is powered through the VIN input, and it supplies 5V to the L293D motor driver through the onboard regulator. 
The BNO055 is also powered using the onboard 5V, even though the I2C lines are 3.3V tolerant. At max load, each motor draws up to 250 mA. The onboard 5V voltage regulator is a L1117S50 with a max draw of 800 mA, 
so under maximum load, the entire system will draw less than 550 mA.

## Control Loop ##
### Initial Design ###
Initially, a linear PID controller was used to map the error to its corresponding PWM value. Due to lower quality manufacturing, each of the motors begins to spin at different PWM values, and the speed of the motors does not directly
correlate to its PWM value. Even though the design was not perfect, it functioned okay on carpet, but it could not balance itself on hard, smooth surfaces.
### First Revision ###
In the first revision, the PWM values were quadratically mapped to the system output. In other words, smaller errors would not result in the same motor changes as larger errors would. The Integral component of the PID 
controller was also removed, as the system would never be fully stable. Using this configuration, the bot was better at balancing, but it still had trouble on hard, smooth surfaces.
### Second Revision ###
Moved back to using a linear controller, this time only PD. Added a multiplier for errors greater than 5 degrees, balances much better on hard surfaces. 
### Third Revision ###
Removed multiplier. Decided multiplier's effect was due to low battery voltage. Need to improve physical design before changing any more software.
