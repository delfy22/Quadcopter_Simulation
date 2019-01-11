close all

timestep = 0.2; % timestep of 1s
time = 30;
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
z_des = 1;
z_des_arr = sim_itr*0;

% PID Constants
% kp_phi = 1.2; % 0.6
% ki_phi = 0.8; % 0.318
% kd_phi = 1.8; % 0.785
% I_e_phi = 0;
% D_phi = 0;
% 
% kp_theta = 1.2; % 0.6
% ki_theta = 0.8; % 0.318
% kd_theta = 1.8; % 0.785
% I_e_theta = 0;
% D_theta = 0;

kp_phi = 2.5; % 
ki_phi = 1; % 
kd_phi = 3; % 
I_e_phi = 0;
D_phi = 0;

kp_theta = 2.5; % 0.6
ki_theta = 1; % 0.318
kd_theta = 3; % 0.785
I_e_theta = 0;
D_theta = 0;

kp_psi = 1.2; % 0.6
ki_psi = 0.65; % 0.24
kd_psi = 2; % 1.04
I_e_psi = 0;
D_psi = 0;

kp_z = 0.8; % 0.6
ki_z = 0.319; % 0.319
kd_z = 2.3; % 0.784
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
U2_arr = sim_itr*0;

z(1:3) = [1,1,1];

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

for i = sim_itr
    %% Desired movement plan
    if time(i) < 15 
        z_des = 1;
        phi_des = 0;
        theta_des = 0;
        psi_des = 0;
    elseif time(i) < 20
        z_des = 5;
        phi_des = 0;
        theta_des = 0;
        psi_des = pi/4;
    elseif time(i) < 30
        z_des = 3;
        phi_des = 0;
        theta_des = 0;
        psi_des = pi/4;
    elseif time(i) < 50
        z_des = 5;
        phi_des = 0;
        theta_des = pi/18;
        psi_des = pi/4;
    elseif time(i) < 65
        z_des = 5;
        phi_des = 0;
        theta_des = -pi/18;
        psi_des = pi/4;
    elseif time(i) < 80
        z_des = 5;
        phi_des = 0;
        theta_des = 0;
        psi_des = pi/4;
    else 
        z_des = 3;
        phi_des = 0;
        theta_des = 0;
        psi_des = pi/4;
    end
    
    phi_des_arr(i) = phi_des;
    theta_des_arr(i) = theta_des;
    psi_des_arr(i) = psi_des;
    z_des_arr(i) = z_des;

    %% Roll Controller
    e_phi = phi_des - phi(i-1);
    I_e_phi = I_e_phi + e_phi*timestep;
    D_phi = (phi(i-1) - phi(i-2))/timestep;
    U2 = Ix*(e_phi*(kp_phi + ki_phi*I_e_phi) - kd_phi*D_phi);
    e_phi_arr(i) = e_phi;
    U2_arr(i) = U2;
    
    %% Pitch Controller
    e_theta = theta_des - theta(i-1);
    I_e_theta = I_e_theta + e_theta*timestep;
    D_theta = (theta(i-1) - theta(i-2))/timestep;
    U3 = Ix*(e_theta*(kp_theta + ki_theta*I_e_theta) - kd_theta*D_theta);

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
    
    U4 = Ix*(e_psi*(kp_psi + ki_psi*I_e_psi) - kd_psi*D_psi);
    
    %% Z Controller
    e_z = z_des - z(i-1);
    I_e_z = I_e_z + e_z*timestep;
    D_z = (z(i-1) - z(i-2))/timestep;
    U1 = (m/(cos(theta(i-1))*cos(phi(i-1))))*(g + e_z*(kp_z + ki_z*I_e_z) - kd_z*D_z);
    
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
%     phi_double_dot(i) = U2/Ix;
%     theta_double_dot(i) = U3/Iy;
%     psi_double_dot(i) = U4/Iz;
    
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
% plot_acc_vel_pos(x_double_dot,x_dot,x,"x","m",time);
% Y
% plot_acc_vel_pos(y_double_dot,y_dot,y,"y","m",time);
% Z
plot_acc_vel_pos(z_double_dot, z_dot, z, z_des_arr, "z", "m", time);

% Phi
plot_acc_vel_pos(phi_double_dot, phi_dot, phi, phi_des_arr, "roll", "rad", time);
% Theta
plot_acc_vel_pos(theta_double_dot, theta_dot, theta, theta_des_arr, "pitch", "rad", time);
% Psi
plot_acc_vel_pos(psi_double_dot ,psi_dot, psi, psi_des_arr, "yaw", "rad", time);

figure
q = quiver3(x,y,z,x_dot/10,y_dot/10,z_dot/10);
q.AutoScaleFactor = 0.3;
xlabel("X /m");
ylabel("Y /m");
zlabel("Z /m");
