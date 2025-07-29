# Balance Bot #
- 2 wheeled balancing robot centered around the STM32F401RE microcontroller
- Utilizes BNO055 IMU to determine orientation in space
- Utilizes motor encoders to maintain position

## Design ##
At a top level, this project is fairly straightforward: determine the bot's orientation relative to equilibrium, then correct it via motors. The STM32 reads euler angles, computed using the sensor's "sensor fusion" mode, from the IMU using I2C at a rate of 100 Hz.
Based on the physical orientation of the sensor itself, pitch is the angle we are focused on. At equilibrium (in an upright, balanced position), the pitch will read exactly 0 degrees. The bot's tilt from the zero point is the error used in the PID algorithms. This error
is mapped to a range of PWM values, used to drive the motors. The internal clock speed of the STM32 is configured to 84 MHz.

## Construction ##
### Initial Design ###
- Initial build is shown below
- Hand cut and drilled plywood used for construction of frame
- The two bases are separated by spacers, and connected using hobby screws and nuts

![IMG_4705](https://github.com/user-attachments/assets/ff6cd8bb-00a0-4934-b249-c4d20f78a818)
![IMG_4707](https://github.com/user-attachments/assets/79d5e728-3fc7-4af4-a859-5b4ae82541d0)
![IMG_4706](https://github.com/user-attachments/assets/18cc2f01-16d4-49ee-a3eb-085e845e61c1)

### First Revision ###
- 3D Printed Design
<img width="1100" height="1149" alt="image" src="https://github.com/user-attachments/assets/bd338e7f-eef7-4729-82c4-5a9378328fe4" />

![IMG_4760](https://github.com/user-attachments/assets/c5746626-72c1-4f61-9e55-ce039aea54a6)

![IMG_4761](https://github.com/user-attachments/assets/63964f8c-9544-4f92-bc71-355c948efdf6)

- BNO055, DRV8833, and MPM3160 are mounted below STM32 using double sided tape and screws

## Schematic ##
<img width="1800" height="1236" alt="balance_bot_schematic_rev1_2" src="https://github.com/user-attachments/assets/baffed60-1463-4d13-aa16-f9a8d333b3b0" />


## PCB Design ##
- After completing the initial project, I decided to design a PCB to clean it up
- Rev 1.1 fixed Rev 1.0 improper pinout for the PMOS and missing vias for the ground plane on the +3.3V regulator
- Rev 1.1 caused a BNO055 sensor to fry due to back EMF; Rev 1.2 moved sensor supply to +3.3V and added 100uF decoupling cap on +5V rails for extra isolation
- Rev 1.2 could be improved using VIN rails for motor power and limiting motor voltage in software using PWM
<img width="2109" height="996" alt="balance_bot_pcb_rev1_2" src="https://github.com/user-attachments/assets/09be0087-69a4-47cf-9654-d05b666cd263" />
- Revision 1.2


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
### Third Revision ###
Adding 2 Hall Effect sensors on motor shaft to create a quadrature encoder. This will be used to keep the bot from drifting forwards or backwards. A separate PID controller is used to determine the target pitch based on how far the bot has moved (using encoder), 
and the PID controller for the IMU will calculate motor speed and direction based on its current pitch relative to the target pitch.
