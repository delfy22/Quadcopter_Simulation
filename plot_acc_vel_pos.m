function [] = plot_acc_vel_pos(acc,vel,pos,ref,axis,unit,time)
    %   Plot acceleration, velocity, and position for one axis on the same
    %   graph. Unit is the unit of measurement for the Y-axis, "m", or
    %   "rad".
    
    figure;
    plot(time, acc);
    hold on;
    plot(time, vel, ":");
    plot(time, pos, "--");
    plot(time, ref);
    hold off;
    titlename = "Dynamics in " + axis + " direction"; 
    title(titlename);
    xlabel("Time /s");
    legend("Acceleration /" + unit + "s^-^2", ...
           "Velocity /" + unit + "s^-^1", ...
           "Position /" + unit, ...
           "Position Reference", ...
           "Location", "best");
end

