# Balance Bot #
- 2 wheeled balancing robot centered around the STM32F401RE microcontroller
- Utilizes BNO055 IMU to determine orientation in space
- WIP: Utilize motor encoders to maintain position

## Design ##
At a top level, this project is fairly straightforward: determine the bot's orientation relative to equilibrium, then correct it via motors. The STM32 reads euler angles, computed using the sensor's "sensor fusion" mode, from the IMU using I2C at a rate of 100 Hz.
Based on the physical orientation of the sensor itself, pitch is the angle we are focused on. At equilibrium (in an upright, balanced position), the pitch will read exactly 0 degrees. The bot's tilt from the zero point is the error used in the PID algorithms. This error
is mapped to a range of PWM values, used to drive the motors. The internal clock speed of the STM32 is configured to 84 MHz.

## Construction ##
### Initial Design ###
- Initial build is shown below
- Hand cut and drilled plywood used for construction of frame
- The two bases are separated by spacers, and connected using hobby screws and nuts
![image](https://github.com/user-attachments/assets/075075a5-caa8-4c1e-b2ab-431e65b59f97)
![image](https://github.com/user-attachments/assets/7bc06242-e929-4888-a3ab-802111ed4c80)
![image](https://github.com/user-attachments/assets/86369cb9-baa2-450e-8b83-71f13cdc9d10)
### First Revision ###
- 3D Printed Design
![image](https://github.com/user-attachments/assets/c4c70c7a-f9c3-4ebe-b838-47490d968acc)
![image](https://github.com/user-attachments/assets/eeff3b18-019c-4ae9-9f4b-d152b0c3dbe6)
![image](https://github.com/user-attachments/assets/6cdc571f-60e5-4613-ada9-d4600b4a5d50)
- BNO055, DRV8833, and MPM3160 are mounted below STM32 using double sided tape and screws


## Power ##
### Initial Design ###
The design is powered using a 9V battery through the VIN input, and it supplies 5V to the L293D motor driver through the onboard regulator. 
The BNO055 is also powered using the onboard 5V, even though the I2C lines are 3.3V tolerant. At max load, each motor draws up to 250 mA. The onboard 5V voltage regulator is a L1117S50 with a max draw of 800 mA, 
so under maximum load, the entire system will draw less than 550 mA.
### Second Revision ###
Due to the outdated bipolar technology in the L293D, there was significant voltage drop across the motor outputs, and the PWM response was finicky due to the lower voltage. A DRV8833 H-Bridge driver was substituted to reduce the output voltage drop and improve motor speed and torque. 
A 2-cell Lipo replaced the 9V battery, powering an MPM3160 5V buck converter. The output from the buck converter powers the BNO055, DRV8833 driver, and STM32 (through VIN input) with a maximum supply current of 1.2A.

## Control Loop ##
### Initial Design ###
Initially, a linear PID controller was used to map the error to its corresponding PWM value. Due to lower quality manufacturing, each of the motors begins to spin at different PWM values, and the speed of the motors does not directly
correlate to its PWM value. Even though the design was not perfect, it functioned okay on carpet, but it could not balance itself on hard, smooth surfaces.
### First Revision ###
In the first revision, the PWM values were quadratically mapped to the system output. In other words, smaller errors would not result in the same motor changes as larger errors would. The Integral component of the PID 
controller was also removed, as the system would never be fully stable. Using this configuration, the bot was better at balancing, but it still had trouble on hard, smooth surfaces.
### Second Revision ###
Moved back to a linear controller, adding an integral term to account for linear position. Significant improvement in balance, but drifts out of its original position.
### Third Revision (WIP) ###
Adding 2 Hall Effect sensors on motor shaft to create a quadrature encoder. This will be used to keep the bot from drifting forwards or backwards. A separate PID controller is used to determine the target pitch based on how far the bot has moved (using encoder), 
and the PID controller for the IMU will calculate motor speed and direction based on its current pitch relative to the target pitch.
