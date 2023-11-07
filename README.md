# Matlab_ndt2_control

There is 3 dependencies to launch the NDT2 MAV:
 
 1. mav_description
 2. arm_description
 3. ndt2_control

First of all clone all these repositories on the current workspace.

How to run the code:
- Write "matlab" in the terminal to open matlab software.
- In matlab go to the file location "<path to>/Matlab_ndt2_control/NMPC for NDT2/Multirotor_NMPC_Updated 27.9.22/Ndt2_States" to get the states of the model.
- Run the file in the above same directory "ndt2_control_NMPC".
- Open the Matlab Simulink file with NMPC applied to the system with blocks.
- Run the ndt2_control_NMPC code first.
- Then run the Matlab/Simulink model to get the results.

Requirments for NMPC:
- The scripts to define the NMPC object (Here it will be ndt2_control_NMPC).
- System states.
- Jocabian of the system if possible
- Reference Trajectory.
- Plots if needed.

Note: All these file should remain in the same directory.

