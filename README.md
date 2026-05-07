# AVA (Autonomous Vision Arm)
AVA is a 5 DOF robot arm that uses computer vision to pick and place objects autonomously. A camera placed above the workspace detects objects by their HSV colour values and stores their centroid coordinates. The robot arm then uses homography and inverse kinematics to locate a cube, pick it up, and place it on a target plate. Here's a sped up video of what it does:

<img width="800" height="450" alt="AVA" src="https://github.com/user-attachments/assets/bd9d4b1d-1c0b-4992-992f-8e90d1dc3390" />

Check the demos folder for more demos!

## README Contents
- [Pipeline](#pipeline)
- [Hardware](#hardware)
- [Wiring](#wiring)
- [Calibration](#calibration)
- [Contact](#contact)

## Pipeline
1. Colour Detection (object_detection_4.py): Camera detects the cube(s) and plate(s) by their HSV values and returns their centroid pixel coordinates.
2. Pixel to World Coordinates (transform_coordinates_3.py): pixel coordinates (px) are transformed to world coordinates (mm) using a homography matrix.
3. Joint angles from Inverse Kinematics (ik_solver_2.py): converts the (x, y) position in mm on the workspace to 4 joint angles (excluding wrist roll and gripper).
4. Communicate to Arduino (serial_comms_comp_1.py): receives the joint angles and sends instructions to the Arduino through serial communication.
5. Arduino Execution (serial_comms.ino): Arduino drives each servo to its target using interpolation.

Requirements for these files are the numpy, opencv, pyserial, and scipy pip packages. Install these, upload serial_comms.ino to the Arduino, and run pipeline.py to orchestrate the pipeline. 

## Hardware

Insert Robot Arm Image Here (Insert)

The robot arm is designed by HowToMechatronics. You can find the .STEP files and how to build it [here](https://howtomechatronics.com/tutorials/arduino/diy-arduino-robot-arm-with-smartphone-control/). As for 3D print settings, I generally used a 60% infill and 5 to 6 walls using this [filament](https://www.amazon.ca/OVERTURE-Filament-Consumables-Dimensional-Accuracy/dp/B07PGZNM34/ref=sr_1_7?crid=32Y6RWAJZ1P58&dib=eyJ2IjoiMSJ9.oENE_oqNuW4ibJTvFRA6tV6PlZP92wlCZInNHlAr0BNb2jDfTTaqZTIXCzrx2JLeKTxwQnpkKTCuHsCiJYJQTrIUSta4BO6iHQ_jCmz1FPFDynscHQc8Sv2tG0Il6_oq4D35H3MQWhMed2pzaSoSnwq-K75yQHHAKeG4J9MJh2YWJ9akym9AysOcIjxp6XfoRiqfyX1EDPeSoQdwuY8pvQiLWgJkpAKrE8egRtiP5-n6g-5x-wK6x-NKak0zJjDzsz_kX38mZQl3ljUAowuroQ3kd8kN3tBLzaJiMMN6D34.gkanKJlYCuA_DmxeTz90W-1bZWYO52E53FQnxGwNwcg&dib_tag=se&keywords=pla%2B1.75mm&qid=1773802071&sprefix=pla%2B1%2B75mm%2Caps%2C117&sr=8-7&th=), but it varied by part.

Insert Cutting board image here (insert)

I used a wooden cutting board to mount the arm and its electronics. Note that the guide does not specify the screws, nuts, and bolts used in the robot arm (except for the ones that come with the servos). So here's a table of what I used:

| Type | Length (mm) | Purpose |
| :------- | :------: | -------: |
| M4 Screws | 15 | Fix robot arm onto cutting board |
| M4 Bolts | 25 | Claw joints |
| M4 Nyloc Nuts | N/A | For claw bolts |

Although the guide doesn't mention them, appropriate heatset inserts can help keep the fasteners in place and improve the structural integrity of the arm. Note that a power screwdriver/drill can be very helpful in assembling the robot arm as well.

A major issue I experienced with the robot arm was with the SG90 servo that controlled the claw pitch. The part of the servo horn that fixed onto the servo was too short and rubbed against another part of the robot arm, introducing enough friction to burn out one of the SG90s. I have attached a modified, 3D-printable servo horn (SG90ModifiedHorn.sldprt) that addresses this problem.

However, I would recommend using another robot arm (such as the SO 101) to pursue the same or similar projects altogether. This robot arm struggles with accuracy and has many flaws with its claw (3D printed gears, weight, etc.). An upside to this was that I learnt more about what makes a reliable robot arm by fixing its problems rather than mindlessly building it. Feel free to do the same, but beware of the frustration that comes along!

## Wiring

Image of Wiring Setup (Insert)

The waist, shoulder, and elbow are controlled by MG996R servos. The other three (wrist roll, wrist pitch, and gripper) are controlled by SG90 servos. 

Image of Servo Driver (Insert)

Image of Arduino (Insert)

Unlike the guide, I connected the servos to a PCA9685 servo driver. An Arduino UNO R3 controls the servos through 5V, GND, SDA, and SCL connections to the servo driver. The servos were powered using a 5V 6A wall adapter, which was connected to a DC Barrel Jack to Terminal block adapter, which was wired to the servo driver. To prevent voltage dips, I also installed a 2200 uF, 16 V capacitor between the block adapter and the servo driver. Note that the Arduino cannot provide enough current for 6 servos since it can only provide around 500 mA of current (MG996R servos can draw up to 2.5A of current, SG90 servos up to 600 mA). 

Before mounting the servo horns, make sure to find their minimum and maximum PWM values, then set it to the PWM value associated with 90 degrees. This calibration prevents inaccuracies later on in the build. Many servos don't use their full range, but it's useful to know what they are for potential adjustments you want to make. Although every servo has its own range, here's a table of servo information that I found was relevant during calibration:

| Servo Name | Purpose | MIN VAL | REST | MAX VAL | Tick at Horizontal |
| :--- | :--- | :--- | :--- | :--- | :--- |
| B | Base | 120 | 325 | 530 | 325 |
| A | Shoulder | 150 | 470 | 530 | 209 |
| C | Elbow | 120 | 400 | 409 | 81 |
| a | Wrist (Claw Roll) | 140 (Upside down) | 519 | 520 | |
| b | Claw (Claw Pitch) | 120 | 200 | 480 | 385 |
| c | Gripper | 230 | 300 | 370 | |

Additionally, in order to run the computer vision required to detect the cubes and plates, the laptop has to be constantly plugged in to the Arduino's USB cable. It is entirely possible to run the robot arm independently if the laptop was replaced by a single board computer like a Raspberry Pi.  

## Calibration

Insert Camera Positioning Image (Insert)

Rulers, protractors, tape measures, and the human eye are inaccurate methods of measurement for something as precise as a robotic arm, but they were all I had access to. I used software to minimize the error as much as possible. Here's a rundown of each file involved in calibration.
- calibrate_camera_3a.py: input the coordinates of measured, marked spots on the workspace. The user is instructed to click on each one of these measured spots to generate a homography matrix that converts pixel (px) to world (mm) coordinates, which is outputted to a calibration.json file. Note that the camera does not have to be placed directly above the workspace for this to work; the matrix takes care of any slants/inaccuracies in the camera positioning. 
- arm_calibration_3b.py: instructs the robot arm to move to listed world coordinate. The user is instructed to click on the actual position of the claw using the camera feed to calculate the error between the instructed position and the actual position. The error across multiple reachable points and the Radial Basis Function from the Scipy package are used to calculate the offsets for the robot arm, which is outputted to an arm_corrections.json file imported by pipeline.py.
- colour_picker_4a.py: instructs the user to click on multiple points of an object through the camera feed to detect its HSV values. These HSV values are then added to object_detection_4.py by the user to detect said object.

Additionally, ik_test_2a.py was used to test if the equations I wrote in the ik_solver file were plausible and realistic. The workspace is limited by the arm's reach, so objects placed too close to the base or too far from it will not be detected or reached reliably. One other thing to note is that AVA is sensitive to lighting conditions. Significant changes to the lighting used to find the HSV values will throw off the HSV detection and require retuning. 

Calibration is one of the most important steps to building a reliable and accurate robot arm. Acquiring a few tools to measure distances and angles accurately is part of the equation that differentiates hobbyist and industrial grade mechanisms. Unfortunately I didn't have access to any, but I would definitely procure some if I was to reiterate the project. 

## Contact
If you have any questions or comments regarding the project, feel free to contact me at presleyprince007 at gmail dot com. Visit my website at [presleyprince.com](https://presleyprince.com/) to learn more about me!
