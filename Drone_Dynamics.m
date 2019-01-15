close all

timestep = 0.4; % timestep of 0.01s
time = 100; % Simulation time
time = timestep:timestep:time;
sim_itr = 1:1:size(time,2);


x_double_dot = sim_itr*0; % intialise x acceleration
y_double_dot = sim_itr*0; % intialise y acceleration
z_double_dot = sim_itr*0; % intialise z acceleration

x_dot = sim_itr*0; % intialise x velocity
y_dot = sim_itr*0; % intialise y velocity
z_dot = sim_itr*0; % intialise z velocity

x = sim_itr*0; % intialise x position
y = sim_itr*0; % intialise y position
z = sim_itr*0; % intialise z position

phi_double_dot = sim_itr*0; % intialise roll acceleration
theta_double_dot = sim_itr*0; % intialise pitch acceleration
psi_double_dot = sim_itr*0; % intialise yaw acceleration

phi_dot = sim_itr*0; % intialise roll velocity
theta_dot = sim_itr*0; % intialise pitch velocity
psi_dot = sim_itr*0; % intialise yaw velocity

phi = sim_itr*0; % initialise roll
theta = sim_itr*0; % initialise pitch
psi = sim_itr*0; % initialise yaw

m = 0.4734; % drone's mass in kg
Ix = 8.1E-3; % drone's moment of inertia about x axis
Iy = 8.1E-3; % drone's moment of inertia about y axis
Iz = 14.2E-3; % drone's moment of inertia about z axis
Jr = 104E-6; % rotor moment of inertia
b = 54.2E-6; % thrust factor
l = 0.24; % distance from drone's centre to centre of propellor
d = 1.1E-6; % drag factor

g = 9.81; % acceleration due to gravity

max_roll = 0.1745; % limit roll to 10^o
max_pitch = 0.1745; % limit pitch to 10^o

%% Initialise Variables
% Initial throttle, roll, pitch, and yaw commands
U1 = 0; % Throttle - For m=0.4734, U1 must be greater than 4.64 to overcome gravity
U2 = 0; % Roll
U3 = 0; % Pitch
U4 = 0; % Yaw

% Desired Positions
phi_des = 0;
phi_des_arr = sim_itr*0;
theta_des = 0;
theta_des_arr = sim_itr*0;
psi_des = 0;
psi_des_arr = sim_itr*0;
z_des = 0;
z_des_arr = sim_itr*0;
x_des = 0;
x_des_arr = sim_itr*0;
y_des = 0;
y_des_arr = sim_itr*0;

% PID Constants
kp_phi = 4; % 
ki_phi = 0; % 
kd_phi = 2; % 
I_e_phi = 0;
D_phi = 0;

kp_theta = 4; % 0.6
ki_theta = 0; % 0.318
kd_theta = 2.2; % 0.785
I_e_theta = 0;
D_theta = 0;

kp_psi = 1.8; % 0.6
ki_psi = 0; % 0.24
kd_psi = 3; % 1.04
I_e_psi = 0;
D_psi = 0;

kp_x = 0.05; % 
ki_x = 0; % 
kd_x = 0.18; % 
I_e_x = 0;
D_x = 0;

kp_y = 0.05; % 
ki_y = 0; % 
kd_y = 0.14; % 
I_e_y = 0;
D_y = 0;

kp_z = 1.8; % 0.6
ki_z = 0; % 0.319
kd_z = 2; % 0.784
I_e_z = 0;
D_z = 0;

% Individual and total propellor speeds
Omega1 = 0;
Omega2 = 0;
Omega3 = 0;
Omega4 = 0;
Omega = 0;

sim_itr(:,1:2) = []; % remove first element of iteration
e_phi_arr = sim_itr*0;
e_x_arr = sim_itr*0;
e_y_arr = sim_itr*0;
U2_arr = sim_itr*0;

z(1:3) = [0,0,0];

for (i=2:3)
    while (psi(i-1) >= 2*pi)
        psi(i-1) = psi(i-1) - 2*pi;
    end
    while (psi(i-1) < 0)
        psi(i-1) = psi(i-1) + 2*pi;
    end
end

phi_des_arr(1, 1:2) = [phi_des, phi_des];
theta_des_arr(1, 1:2) = [theta_des, theta_des];
psi_des_arr(1, 1:2) = [psi_des, psi_des];
z_des_arr(1, 1:2) = [z_des, z_des];

%% Simulation
for i = sim_itr
    %% Desired movement plan
%     if time(i) < 10 
%         z_des = 1;
%         phi_des = 0;
%         theta_des = 0;
%         psi_des = 0;
%     elseif time(i) < 20
%         z_des = 5;
%         phi_des = 3;
%         theta_des = 5;
%         psi_des = pi/4;
%     else 
%         z_des = 5;
%         phi_des = 3;
%         theta_des = 5;
%         psi_des = pi/4;
%     end

    if time(i) < 10
        x_des = 0;
        y_des = 0;
        z_des = 4;
        psi_des = pi/2;
    elseif time(i) < 20
        x_des = 10;
        y_des = 0;
        z_des = 4;
        psi_des = pi/2;
    elseif time(i) < 50
        x_des = 10;
        y_des = 5;
        z_des = 4;
        psi_des = pi/4;
    else
        x_des = 10;
        y_des = 5;
        z_des = 4;
        psi_des = pi/4;
    end
    
    %% Outer X Controller
    e_x = x_des - x(i-1);
    I_e_x = I_e_x + e_x*timestep;
    D_x = (x(i-1) - x(i-2))/timestep;
    theta_des_hat = e_x*kp_x + ki_x*I_e_x - kd_x*D_x;
    e_x_arr(i) = e_x;
    
    %% Outer Y Controller
    e_y = y_des - y(i-1);
    I_e_y = I_e_y + e_y*timestep;
    D_y = (y(i-1) - y(i-2))/timestep;
    phi_des_hat = -(e_y*kp_y + ki_y*I_e_y - kd_y*D_y); 
    e_y_arr(i) = e_y;
    
    if time(i) > 10
        i;
    end
    if time(i) >= 50
        i;
    end
    
    if (phi_des_hat > max_roll)
        phi_des_hat = max_roll;
    elseif (phi_des_hat < -max_roll)
        phi_des_hat = -max_roll;
    end
    if (theta_des_hat > max_pitch)
        theta_des_hat = max_pitch;
    elseif (theta_des_hat < -max_pitch)
        theta_des_hat = -max_pitch;
    end
    
    theta_des = cos(psi(i-1))*theta_des_hat - sin(psi(i-1))*phi_des_hat;
    phi_des = sin(psi(i-1))*theta_des_hat + cos(psi(i-1))*phi_des_hat;
    
    %% Update reference arrays
    phi_des_arr(i) = phi_des;
    theta_des_arr(i) = theta_des;
    psi_des_arr(i) = psi_des;
    z_des_arr(i) = z_des;
    x_des_arr(i) = x_des;
    y_des_arr(i) = y_des;

    %% Roll Controller
    e_phi = phi_des - phi(i-1);
    I_e_phi = I_e_phi + e_phi*timestep;
    D_phi = (phi(i-1) - phi(i-2))/timestep;
    U2 = Ix*(e_phi*kp_phi + ki_phi*I_e_phi - kd_phi*D_phi);
    e_phi_arr(i) = e_phi;
    U2_arr(i) = U2;
    
    %% Pitch Controller
    e_theta = theta_des - theta(i-1);
    I_e_theta = I_e_theta + e_theta*timestep;
    D_theta = (theta(i-1) - theta(i-2))/timestep;
    U3 = Ix*(e_theta*kp_theta + ki_theta*I_e_theta - kd_theta*D_theta);

    %% Yaw Controller
    % Wraparound psi(i-1)
    while (psi(i-1) >= 2*pi)
        psi(i-1) = psi(i-1) - 2*pi;
    end
    while (psi(i-1) < 0)
        psi(i-1) = psi(i-1) + 2*pi;
    end
    % Wraparound psi_des
    while (psi_des >= 2*pi)
        psi_des = psi_des - 2*pi;
    end
    while (psi_des < 0)
        psi_des = psi_des + 2*pi;
    end
    % Compute error
    e_psi = psi_des - psi(i-1);
    
    % Wraparound error so that the smallest magnitude error is used, this
    % is the direction in which the least movement is required to reach
    % target
    if ((e_psi+2*pi)^2 < e_psi^2)
        e_psi = e_psi + 2*pi;
    elseif ((e_psi-2*pi)^2 < e_psi^2)
        e_psi = e_psi - 2*pi;
    end
    
    I_e_psi = I_e_psi + e_psi*timestep;
    
    % Compute potential differentials
    D_psi_1 = (psi(i-1) - psi(i-2))/timestep;
    D_psi_2 = (psi(i-1) - psi(i-2) + 2*pi)/timestep;
    D_psi_3 = (psi(i-1) - psi(i-2) - 2*pi)/timestep;
    
    % Pick the smallest magnitude differential
    if (D_psi_2^2 < D_psi_1^2)
        if (D_psi_3^2 < D_psi_2^2)
            D_psi = D_psi_3;
        else
            D_psi = D_psi_2;
        end
    elseif (D_psi_3^2 < D_psi_1^2)
        D_psi = D_psi_3;
    else
        D_psi = D_psi_1;
    end
    
    U4 = Ix*(e_psi*kp_psi + ki_psi*I_e_psi - kd_psi*D_psi);
    
    %% Z Controller
    e_z = z_des - z(i-1);
    I_e_z = I_e_z + e_z*timestep;
    D_z = (z(i-1) - z(i-2))/timestep;
    U1 = (m/(cos(theta(i-1))*cos(phi(i-1))))*(g + e_z*kp_z + ki_z*I_e_z - kd_z*D_z);
    
    %% Calculate propellor speeds
    if (U1 == 0)
        Omega1 = 0;
        Omega2 = 0;
        Omega3 = 0;
        Omega4 = 0;
    else
        Omega1 = real(sqrt(U1/(4*b) - U3/(2*b*l) - U4/(4*d)));
        Omega2 = real(sqrt(U1/(4*b) - U2/(2*b*l) + U4/(4*d)));
        Omega3 = real(sqrt(U1/(4*b) + U3/(2*b*l) - U4/(4*d)));
        Omega4 = real(sqrt(U1/(4*b) + U2/(2*b*l) + U4/(4*d)));
    end
    
    Omega = - Omega1 + Omega2 - Omega3 + Omega4;
    
    %% Angular components
    % calculate angular accelerations based on commands
    phi_double_dot(i) = theta_dot(i-1)*psi_dot(i-1)*(Iy - Iz)/Ix - ...
                        Jr*theta_dot(i-1)*Omega/Ix + U2/Ix;
    theta_double_dot(i) = phi_dot(i-1)*psi_dot(i-1)*(Iz - Ix)/Iy - ...
                        Jr*phi_dot(i-1)*Omega/Iy + U3/Iy;
    psi_double_dot(i) = theta_dot(i-1)*phi_dot(i-1)*(Ix - Iy)/Iz+ U4/Iz;  
    
    % integrate for angular velocities    
    phi_dot(i) = phi_dot(i-1) + phi_double_dot(i)*timestep;
    theta_dot(i) = theta_dot(i-1) + theta_double_dot(i)*timestep;
    psi_dot(i) = psi_dot(i-1) + psi_double_dot(i)*timestep;
    
    % integrate again for angular positions
    phi(i) = phi(i-1) + phi_dot(i)*timestep;
    theta(i) = theta(i-1) + theta_dot(i)*timestep;
    psi(i) = psi(i-1) + psi_dot(i)*timestep;
        
    %% Linear components
    % calculate linear accelerations based on commands
    x_double_dot(i) = U1*(cos(psi(i))*sin(theta(i))*cos(phi(i)) + sin(psi(i))*sin(phi(i)))/m;
    y_double_dot(i) = U1*(sin(psi(i))*sin(theta(i))*cos(phi(i)) - cos(psi(i))*sin(phi(i)))/m;
    z_double_dot(i) = U1*cos(theta(i))*cos(phi(i))/m - g;

    % integrate for linear velocities 
    x_dot(i) = x_dot(i-1) + x_double_dot(i)*timestep;
    y_dot(i) = y_dot(i-1) + y_double_dot(i)*timestep;
    z_dot(i) = z_dot(i-1) + z_double_dot(i)*timestep;
    
    % integrate for linear positions
    x(i) = x(i-1) + x_dot(i)*timestep;
    y(i) = y(i-1) + y_dot(i)*timestep;
    z(i) = z(i-1) + z_dot(i)*timestep;
    
    % If drone hits ground, linear acceleration and velocity both stop
    if (z(i) <= 0)
        z_double_dot(i) = z_double_dot(i) + g;
        z_dot(i) = 0;   
        z(i) = 0;
        x_dot(i) = 0;
        y_dot(i) = 0;
    end
end

%% Plot accelerations against time
% X
plot_acc_vel_pos(x_double_dot, x_dot, x, x_des_arr, "x", "m", time);
% Y
plot_acc_vel_pos(y_double_dot, y_dot, y, y_des_arr, "y", "m", time);
% Z
plot_acc_vel_pos(z_double_dot, z_dot, z, z_des_arr, "z", "m", time);

% Phi
plot_acc_vel_pos(phi_double_dot, phi_dot, phi, phi_des_arr, "roll", "rad", time);
% Theta
plot_acc_vel_pos(theta_double_dot, theta_dot, theta, theta_des_arr, "pitch", "rad", time);
% Psi
plot_acc_vel_pos(psi_double_dot ,psi_dot, psi, psi_des_arr, "yaw", "rad", time);

figure
q = quiver3(x,y,z,x_dot,y_dot,0.5*z_dot);
hold on;
plot3(x_des_arr, y_des_arr, z_des_arr);
q.AutoScale = 'off';
q.AutoScaleFactor = 1;
q.MaxHeadSize = 0.5;
xlabel("X /m");
ylabel("Y /m");
zlabel("Z /m");

%% Shows larger arrows for direction of travel
% Credit to https://uk.mathworks.com/matlabcentral/fileexchange/29307-plot-with-direction-3d

% rMag = 1;
% % Length of vector
% lenTime = length(x);
% % Indices of tails of arrows
% vSelect0 = 1:(lenTime-1);
% % Indices of tails of arrows
% vSelect1 = vSelect0 + 1;
% % X coordinates of tails of arrows
% vXQ0 = x(1, vSelect0);
% % Y coordinates of tails of arrows
% vYQ0 = y(1, vSelect0);
% % X coordinates of tails of arrows
% vZQ0 = z(1, vSelect0);
% % X coordinates of heads of arrows
% vXQ1 = x(1, vSelect1);
% % Y coordinates of heads of arrows
% vYQ1 = y(1, vSelect1);
% % Z coordinates of heads of arrows
% vZQ1 = z(1, vSelect1);
% % vector difference between heads & tails
% vPx = (vXQ1 - vXQ0) * rMag;
% vPy = (vYQ1 - vYQ0) * rMag;
% vPz = (vZQ1 - vZQ0) * rMag;
% % make plot 
% h1 = plot3 (x, y, z, '.-'); 
% hold on;
% % add arrows 
% h2 = quiver3 (vXQ0, vYQ0, vZQ0, vPx, vPy, vPz, 0, 'r'); grid on; 
% plot3(x_des_arr, y_des_arr, z_des_arr, 'bl');
% hold off
% % axis equal
