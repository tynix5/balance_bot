# Balance Bot #
- 2 wheeled balancing robot centered around the STM32F401RE microcontroller
- Utilizes BNO055 IMU to read euler angles using I2C interface
- 5V DC Motors are driven by L293D IC

## Design ##
The premise of this bot is actually simple: read a sensor, then adjust motor speed based on the error. The STM32 will read euler angles from the IMU using its inertial measurement mode at a rate of 100 Hz.
Based on the placement of my IMU, the pitch determines the angle of error. At equilibrium, the pitch will be exactly 0 degrees, and by reading the pitch, the exact error can be determined. This error is fed into a PID control loop
whose output is then mapped to a range of PWM values, used to drive the motors. To control the direction of the motors, the PWM signal is inverted and the second motor input line is active. The internal clock speed
of the STM32 is configured to 84 MHz.

## Construction ##
- Initial build is shown below
- Hand cut and drilled plywood was used for both bases
- These bases are separated by spacers, and connected using hobby screws and nuts
![image](https://github.com/user-attachments/assets/075075a5-caa8-4c1e-b2ab-431e65b59f97)
![image](https://github.com/user-attachments/assets/7bc06242-e929-4888-a3ab-802111ed4c80)
![image](https://github.com/user-attachments/assets/86369cb9-baa2-450e-8b83-71f13cdc9d10)
- Future designs will be 3D printed

## Power ##
The design is powered by a 9V battery for now. In the future, a LiPo battery will be substituted. The STM32 is powered through the VIN input, and it supplies 5V to the L293D through the onboard regulator. 
The BNO055 is also powered using the onboard 5V, but the I2C lines are 3.3V tolerant. At max load, each motor draws up to 250 mA. The onboard 5V voltage regulator is a L1117S50 with a max draw of 800 mA, 
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
