close all

time = [0:0.01:10];
timestep = 0.01; % timestep of 1s

m = 1; % drone's mass in kg
g = 9.81; % acceleration due to gravity

Td = [0; 0; 9.81]; % Thrust produced by drone
Tw = zeros(3, size(time,2)); 
vw = zeros(3, size(time,2));
pw = zeros(3, size(time,2));

p_ini = [0; 0; 0]; % Initial position
v_ini = [0; 0; 0]; % Initial velocity
del_p = [0; 0; 0]; % Change in position at each time step

for i = 2:size(time,2)
    if time(i)<=4 
        psi = 0; % yaw
        theta = 0; % pitch
        phi = 0; % roll
    elseif time(i)<=6
        psi = 0; % yaw
        theta = pi/16; % pitch
        phi = 0; % roll    
    elseif time(i)<=8
        psi = pi/2; % yaw
        theta = 0; % pitch
        phi = 0; % roll   
    else
        psi = pi/2; % yaw
        theta = -pi/16; % pitch
        phi = 0; % roll        
    end
    
    Td = [0; 0; 9.81/cos(theta)*cos(phi)]; % total thrust produced - make sure z stays constant

    cpsi = cos(psi);
    spsi = sin(psi);
    cphi = cos(phi);
    sphi = sin(phi);
    cthe = cos(theta);
    sthe = sin(theta);
    
    Rzyx = [cpsi*cthe   cpsi*sthe*sphi-spsi*cphi    cpsi*sthe*cphi+spsi*sphi ;
            spsi*cthe   spsi*sthe*sthe+cpsi*cphi    spsi*sthe*cphi-cpsi*sphi ;
            -sthe       cthe*sphi                   cthe*cphi               ];
        
    Tw(:,i) = Rzyx*Td - [0; 0; m*g]; % Total thrust in world frame
    acc = m*Tw(:,i);
    vw(:,i) = vw(:,i-1) + acc*timestep;
    pw(:,i) = pw(:,i-1) + vw(:,i)*timestep;
    
end

figure
subplot(3,1,1);
plot(time, Tw(1,:));
xlabel("Time /s");
ylabel("Thrust in x direction /N");

subplot(3,1,2);
plot(time, Tw(2,:));
xlabel("Time /s");
ylabel("Thrust in y direction /N");

subplot(3,1,3);
plot(time, Tw(3,:));
xlabel("Time /s");
ylabel("Thrust in z direction /N");

figure
subplot(3,1,1);
plot(time, vw(1,:), time, pw(1,:));
xlabel("Time /s");
ylabel("Velocity and position in x direction /N");

subplot(3,1,2);
plot(time, vw(2,:), time, pw(2,:));
xlabel("Time /s");
ylabel("Velocity and position in y direction /N");

subplot(3,1,3);
plot(time, vw(3,:), time, pw(3,:));
% legend("Velocity","Position");
xlabel("Time /s");
ylabel("Velocity and position in z direction /N");
