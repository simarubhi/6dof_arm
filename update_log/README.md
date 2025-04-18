# Update Log

# April 17, 2025

-   After a long break I worked on some redesigns of the gearbox that should reduce the size and increase performance.
-   In the coming days I'll be working on the CAD model.

# March 20, 2025

-   Currently working on designing a torque and longevity testing rig for the gearbox.
-   Had to order new stepper drivers as the old ones were accidentally shorted (will not be using breadboards moving forward).

# March 15, 2025

-   After a couple days of test fitting the components, the current design is working well with the motor. Last component to optimize is the flex spline. I am currently trying a higher infill density, more walls, and also changed the pattern to concentric. I wasn't able to test the new flex spline design today.
-   I also added 3D printed washers to the bearing assemblies when now allow me to tighten the screws and nuts all the way getting rid of unwanted vibrations.
-   I ended up bothering with the STM32 driver for the TMC2209 as I will write the UART version soon. Instead I used a 3 in 1 multimeter function generator which was decent but not a very stable way of testing. The STM32 UART version will be more consistent and allow me to accurately evaluate the gearbox.
-   I'll upload a video of the working drive once I'm happy with the stability. (pre torque limit testing)

# March 12, 2025

-   After inspecting the flex spline closer, I noticed small gaps in some on parallel sides of the ring. This is likely due to the infill pattern and limited area between the teeth and inner diameter. In a recent version I made the flex spline thinner as the wave generator was deforming it too much. Now i will proportionally make the flex spline thicker and wave generator shorter. This will not only decrease the chances of less densely packed infill, but also increase rigidity in the flex spline which is desirable to avoid unwanted twisting.
-   Wrote a simple stepper-direction driver for the TMC2209 and a STM32 producing PWM signals. Testing will begin soon.

# March 11, 2025

-   Over the past couple days I did a lot of test prints and now have tolerances that work.
-   Designed the first output spline with a mounting plate.
-   Now need to work on writing (or temporarily finding) a driver for the TMC 2209 stepper motor with a STM32.

<img src="./docs/march11-2025-firstplate.jpg" style="width: 300px" alt="assembly with mounting plate"/>

## March 9, 2025

-   Remodeled the entire gearbox (keeping the same design) after observing multiple issues in the previous versions.
    -   The stepper CAD model I was using in Fusion 360 was slightly different causing the bearings to be higher than intended.
    -   I didn't leave enough room in the circular spline for the flex spline in stretch enough to mesh with the output spline.
    -   Both of the above issues were causing the flex spline to slowly climb out of the circular spline.
-   Gears should mesh properly now (hopefully). I also adjusted the print overall to be more suited for 3D printing.
-   Ran out of time, not able to test yet.

## March 8, 2025

-   Did some more test fits keeping the same general design for the time being. It seems that the output spline's dimensions are not completely right at the moment.
-   After some research it seems that making the flex or output spline have slight thinner teeth (more gap between teeth) may fix the issue.
-   Currently working on a slightly new test design that will use 3mm bearings instead of 5mm bearings, as well as having a more stable output spline that doesn't try to pop out.

## March 6, 2025

-   First test fit complete. The circular spline and flex spline split was very good. The output spline diameter needs to be smaller. The wave generator should be lower to make proper contact and I'm considering using smaller bearings (3mm instead of 5mm) and having 3 on each side for more tooth contact.

-   Overall the drive is working correctly.

<img src="./docs/march06-2025-firstprint.jpg" style="width: 300px" alt="first picture"/>

## March 5, 2025

-   Adjusted the 3D strain wave gearbox model, optimizing for cost, component capability, and FDM 3D printing.
-   Testing and adjustments of model are ongoing currently.

<img src="./docs/march05-2025-strain-wave-firstprint.jpg" style="width: 300px" alt="first print"/>

Updated initial design for 3D printing tolerance testing. I'm currently planning to make the final version closer to my original design as in theory, it should be more reliable.

## February 22, 2025

-   Worked on defining problem/project definition, general design, and required materials.
-   Looked at getting inspiration and ideas from existing solutions.
-   Started prototyping strain wave gearbox in Fusion 360 which will be used in multiple joints.

### General Structure

<img src="./docs/feb22-2025-general-structure.jpg" style="width: 300px" alt="general structure"/>

This is the general blueprint of the arm. The design decision are are the following:

Steppers motors will be used for their superior accuracy compared to servo motors, and cost compared to BLDC motors. The "yaw" of the arm will be performed by a Nema 17 motor in the base. The vertical rotation of joint 1 will be performed by a larger Nema 23 motor. Directly above the Nema 23, a Nema 17 with a belt will control the vertical rotation of joint 2. A Nema 17 will be mounted to joint 2 and via a belt control will control the horizontal rotation of joint 2. 2 Nema 17's will be mounted on the opposite side of joint 2 to the wrist joint. By mounting the motors here, their mass will be closer to the center, reducing undesied torque and acting as a counter weight. This should allow for a larger payload capacity. The wrist and actuator joints will be controlled by belts, the exact design has not been decided yet.

### First Gearbox Design

<img src="./docs/feb22-2025-strainwave-prototype.jpg" style="width: 300px" alt="first strain wave gearbox design"/>

This design is not optimized for 3D printing and a new iteration of the gearbox suitable for 3D printing prototyping is nearly finished. The strain wave gearbox was chosen for it's quiet operation, minimal backlash, compact form factor, and high gear reduction.

## February 16, 2025 - Stage 2 Started
