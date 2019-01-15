# Quadcopter_Simulation

Code to simulate a quadcopter's dynamics (from Bresciani, 2008) along with PID controllers to control the quadcopter's X, Y, Z, and Yaw positions.

Needs neatening up with a lot being put into functions etc. Also, PID controllers are currently wrong as U2, U3, and U4 all use Ix, but U3 should use Iy, and U4 should use Iz. Changing this will likely require retuning of PID parameters however. 
