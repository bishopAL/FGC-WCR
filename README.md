# Feedforward Gravity Compensation for a Wall Climbing Robot

## Structure

*Script* folder contains a MATLAB function and a script to calculate the gravity compensation leg position.

*Data* folder contains the position and posture data during the ceiling walking. The video can be downloaded as the supplement materials.

## Your own compensation

You can obtain your own compensation value by the MATLAB script.

These are the parameters you needs to modify:

--comp.m line 43: the gravity and its direction of the whole robot
--comp.m line 44: opsite value of the line 43
--comp.m line 47: L1, L2, L3: the size of the link; lengthHF, widthHF: the half of the length and width of the robot body
--comp.m line 51-54: the stiffness of the joints
--comp.m line 73: the positions of the foot in the body coordinate. You may also modify it in *dataGenerator.m*
