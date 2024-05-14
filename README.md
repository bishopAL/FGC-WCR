# Feedforward Gravity Compensation for a Wall Climbing Robot

![](https://github.com/bishopAL/FeedforwardGravityCompensation-WCR/blob/main/videos/s1.gif)

## Structure

*Script* folder contains a MATLAB function and a script to calculate the gravity compensation leg position.

*Data* folder contains the position and posture data during the ceiling walking. The video can be downloaded as the supplement materials.

## Your own compensation

You can obtain your own compensation value by the MATLAB script.

These are the parameters you needs to modify:

- dataGenerator.m line 20: the gravity and its direction of the whole robot `Fg`

- dataGenerator.m line 25: the size of the link `L1, L2, L3`; the half of the length and width of the robot body `lengthHF, widthHF`

- dataGenerator.m line 26: the positions of the foot `Pfc` in the body coordinate

- dataGenerator.m line 150: the stiffness of the joints `K_tq`
