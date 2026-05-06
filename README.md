# AVA (Autonomous Vision Arm)
AVA is a robot arm that uses computer vision to pick and place objects autonomously. A camera placed above the workspace detects objects by their HSV colour values and stores their centroid coordinates. The robot arm then uses homography and inverse kinematics to locate a cube, pick it up, and place it on a target plate. Here's a video of what it does:

Insert GIF here (Insert)

## Pipeline

1. Colour Detection (object_detection_4.py): Camera detects the cube(s) and plate(s) by their HSV values and returns their centroid pixel coordinates.
2. Pixel to World Coordinates (transform_coordinates_3.py): pixel coordinates (px) are transformed to world coordinates (mm) using a matrix (homography).
3. Joint angles from Inverse Kinematics (ik_solver_2.py): converts the (x, y) position in mm on the workspace using a bunch of cosine law to joint angles.
4. Communicate to Arduino (serial_comms_comp_1.py): receives the joint angles and sends instructions to the Arduino through serial communication.
5. Arduino Execution (serial_comms.ino): Arduino drives each servo to its target using interpolation.

pipeline.py orchestrates the pipeline. 

## Hardware

Insert Robot Arm Image Here (Insert)

The robot arm is designed by HowToMechatronics. You can find the guide to build it [here](https://howtomechatronics.com/tutorials/arduino/diy-arduino-robot-arm-with-smartphone-control/). As for 3D print settings, I generally used a 60% infil and 5 to 6 walls, but it varied by part.

Insert Cutting board image here (insert)

I used a wooden cutting board to mount the arm and its electronics. Note that the guide does not specify the screws, nuts, and bolts used in the robot arm (except for the ones that come with the servos). So here's a table of what I used:

| Type | Length (mm) | Purpose |
| :------- | :------: | -------: |
| M4 Screws | 15 | Fix robot arm onto cutting board |
| M4 Bolts | 25 | Claw joints |
| M4 Nyloc Nuts | N/A | For claw bolts |

Although the guide doesn't meantion them, appropriate heatset inserts can help keep the fasteners in place and improve the structural integrity of the arm. Note that a power screwdriver/drill can be very helpful in assembling the robot arm as well.

A major issue I experienced with the robot arm was with the SG90 servo that controlled the claw pitch. The part of the servo horn that fixed onto the servo was too short and rubbed against another part of the robot arm, introducing enough friction to burn out one of the SG90s. I have attached a modified, 3D-printable servo horn (SG90ModifiedHorn.sldprt) that addresses this problem.

However, I would recommend using another robot arm (such as the SO 101) to pursue the same or similar projects altogether. This robot arm struggles with accuracy and has many flaws with its claw (3D printed gears, weight, etc.). An upside to this was that I learnt more about what makes a reliable robot arm by fixing its problems rather than mindlessly building it. Feel free to do the same, but beware of the frustration that comes along!

## Electrical
I decided on a different power source and did not use the Bluetooth control module.
