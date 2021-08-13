function stop = optim_plot_rover(optimValues,state,wheel_radius)


x = optimValues.bestx;
wheelbase_length = x(1);
suspension_height = x(2);
suspension_trap_len = x(3);

suspension_design_x = [-wheelbase_length/2, -suspension_trap_len/2, suspension_trap_len/2, wheelbase_length/2]; % Distance from center of rover, negative towards rear wheel, positive towards front
suspension_design_y = [0,suspension_height, suspension_height, 0]; % Height above wheel centerline, not above ground

front_wheel_x = wheelbase_length/2;
front_wheel_y = wheel_radius;

rear_wheel_x = -wheelbase_length/2;
rear_wheel_y = wheel_radius;

[wheel_f_x_list,wheel_f_y_list] = create_circle(front_wheel_x, front_wheel_y, wheel_radius, 100);
[wheel_r_x_list,wheel_r_y_list] = create_circle(rear_wheel_x, rear_wheel_y, wheel_radius, 100);
[suspension_x_list,suspension_y_list] = position_relative_to_wheels(front_wheel_x, front_wheel_y, rear_wheel_x, rear_wheel_y, suspension_design_x, suspension_design_y);

plot([-1.2/2,-1.2/2,1.2/2,1.2/2],[1,0,0,1],"k");
title("Best Design")
hold on
axis equal
plot(wheel_f_x_list,wheel_f_y_list,"b");
plot(wheel_r_x_list,wheel_r_y_list,"b");
plot(suspension_x_list,suspension_y_list,"c");
hold off

stop = false;

end

function [circle_x,circle_y] = create_circle(center_x, center_y, radius, segments)
if nargin == 3
    segments = 100;
end

angles = linspace(0, 2*pi, segments);
circle_x = center_x + radius*cos(angles);
circle_y = center_y + radius*sin(angles);
end

function [rel_x,rel_y] = position_relative_to_wheels(front_wheel_x, front_wheel_y, rear_wheel_x, rear_wheel_y, design_x, design_y)

center_x = (front_wheel_x+rear_wheel_x)/2;
center_y = (front_wheel_y+rear_wheel_y)/2;

wheel_distance = sqrt((front_wheel_x-rear_wheel_x)^2 + (front_wheel_y-rear_wheel_y)^2);
wheel_dir_x = (front_wheel_x-rear_wheel_x)/wheel_distance;
wheel_dir_y = (front_wheel_y-rear_wheel_y)/wheel_distance;

normal_x = -wheel_dir_y;
normal_y = wheel_dir_x;

for i = 1:length(design_x)
    rel_x(i) = center_x + wheel_dir_x*design_x(i) + normal_x*design_y(i);
    rel_y(i) = center_y + normal_y*design_y(i) + wheel_dir_y*design_x(i);
end
end