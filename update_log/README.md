# Update Log

## Feb 16 - Stage 2 Started

## Feb 22

-   Worked on defining problem/project definition, general design, and required materials.
-   Looked at getting inspiration and ideas from existing solutions.
-   Started prototyping strain wave gearbox in Fusion 360 which will be used in multiple joints.

### General Structure

<img src="./docs/feb22-2025-general-structure.jpg" style="width: 500px" alt="general structure"/>

This is the general blueprint of the arm. The design decision are are the following:

Steppers motors will be used for their superior accuracy compared to servo motors, and cost compared to BLDC motors. The "yaw" of the arm will be performed by a Nema 17 motor in the base. The vertical rotation of joint 1 will be performed by a larger Nema 23 motor. Directly above the Nema 23, a Nema 17 with a belt will control the vertical rotation of joint 2. A Nema 17 will be mounted to joint 2 and via a belt control will control the horizontal rotation of joint 2. 2 Nema 17's will be mounted on the opposite side of joint 2 to the wrist joint. By mounting the motors here, their mass will be closer to the center, reducing undesied torque and acting as a counter weight. This should allow for a larger payload capacity. The wrist and actuator joints will be controlled by belts, the exact design has not been decided yet.

### First Gearbox Design

<img src="./docs/feb22-2025-strainwave-prototype.jpg" style="width: 300px" alt="first strain wave gearbox design"/>

This design is not optimized for 3D printing and a new iteration of the gearbox suitable for 3D printing prototyping is nearly finished. The strain wave gearbox was chosen for it's quiet operation, minimal backlash, compact form factor, and high gear reduction.
