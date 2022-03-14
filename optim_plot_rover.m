function stop = optim_plot_rover(optimValues,state,wheel_radius)


x = optimValues.bestx;
wheelbase_length = x(1);
suspension_height = x(2);
suspension_trap_len = x(3);
base1_len = x(4);

suspension_design_x = [-base1_len/2, -suspension_trap_len/2, suspension_trap_len/2, base1_len/2]; % Distance from center of rover, negative towards rear wheel, positive towards front
suspension_design_y = [0,suspension_height, suspension_height, 0]; % Height above wheel centerline, not above ground

front_wheel_x_b = wheelbase_length/2;
front_wheel_y_b = wheel_radius;

rear_wheel_x_b = -wheelbase_length/2;
rear_wheel_y_b = wheel_radius;

[wheel_f_x_list_b,wheel_f_y_list_b] = create_circle(front_wheel_x_b, front_wheel_y_b, wheel_radius, 100);
[wheel_r_x_list_b,wheel_r_y_list_b] = create_circle(rear_wheel_x_b, rear_wheel_y_b, wheel_radius, 100);
[suspension_x_list_b,suspension_y_list_b] = position_relative_to_wheels(front_wheel_x_b, front_wheel_y_b, rear_wheel_x_b, rear_wheel_y_b, suspension_design_x, suspension_design_y);
[pivot_x_b,pivot_y_b] = position_relative_to_wheels(front_wheel_x_b, front_wheel_y_b, rear_wheel_x_b, rear_wheel_y_b, 0, 0.05+suspension_height);

plot([-1.2/2,-1.2/2,1.2/2,1.2/2],[1,0,0,1],"k");
title("Best Design")
hold on
axis equal
xl = xlim;
yl = ylim;


for x = optimValues.swarm'
    wheelbase_length = x(1);
    suspension_height = x(2);
    suspension_trap_len = x(3);
    base1_len = x(4);
    
    suspension_design_x = [-base1_len/2, -suspension_trap_len/2, suspension_trap_len/2, base1_len/2]; % Distance from center of rover, negative towards rear wheel, positive towards front
    suspension_design_y = [0,suspension_height, suspension_height, 0]; % Height above wheel centerline, not above ground

    front_wheel_x = wheelbase_length/2;
    front_wheel_y = wheel_radius;

    rear_wheel_x = -wheelbase_length/2;
    rear_wheel_y = wheel_radius;

    [wheel_f_x_list,wheel_f_y_list] = create_circle(front_wheel_x, front_wheel_y, wheel_radius, 100);
    [wheel_r_x_list,wheel_r_y_list] = create_circle(rear_wheel_x, rear_wheel_y, wheel_radius, 100);
    [suspension_x_list,suspension_y_list] = position_relative_to_wheels(front_wheel_x, front_wheel_y, rear_wheel_x, rear_wheel_y, suspension_design_x, suspension_design_y);

    plot(wheel_f_x_list,wheel_f_y_list,'Color',[0, 0, 1, 0.2]);
    plot(wheel_r_x_list,wheel_r_y_list,'Color',[0, 0, 1, 0.2]);
    plot(suspension_x_list,suspension_y_list,'Color',[0, 1, 1, 0.2]);
end

wheel_radius_ref = 0.21/2;
wheelbase_length_ref = 0.955;
suspension_height_ref = 0.221;
suspension_trap_len_ref = 0.140;
base1_len_ref = 0.904;

suspension_design_x_ref = [-base1_len_ref/2, -suspension_trap_len_ref/2, suspension_trap_len_ref/2, base1_len_ref/2]; % Distance from center of rover, negative towards rear wheel, positive towards front
suspension_design_y_ref = [0,suspension_height_ref, suspension_height_ref, 0]; % Height above wheel centerline, not above ground

front_wheel_x_ref = wheelbase_length_ref/2;
front_wheel_y_ref = wheel_radius_ref;

rear_wheel_x_ref = -wheelbase_length_ref/2;
rear_wheel_y_ref = wheel_radius_ref;

[wheel_f_x_list_ref,wheel_f_y_list_ref] = create_circle(front_wheel_x_ref, front_wheel_y_ref, wheel_radius_ref, 100);
[wheel_r_x_list_ref,wheel_r_y_list_ref] = create_circle(rear_wheel_x_ref, rear_wheel_y_ref, wheel_radius_ref, 100);
[suspension_x_list_ref,suspension_y_list_ref] = position_relative_to_wheels(front_wheel_x_ref, front_wheel_y_ref, rear_wheel_x_ref, rear_wheel_y_ref, suspension_design_x_ref, suspension_design_y_ref);
[pivot_x_ref,pivot_y_ref] = position_relative_to_wheels(front_wheel_x_ref, front_wheel_y_ref, rear_wheel_x_ref, rear_wheel_y_ref, 0, 0.055+suspension_height_ref);

plot(wheel_f_x_list_ref,wheel_f_y_list_ref,'Color',[1, 0, 0, 1]);
plot(wheel_r_x_list_ref,wheel_r_y_list_ref,'Color',[1, 0, 0, 1]);
plot(suspension_x_list_ref,suspension_y_list_ref,'Color',[1, 0, 0, 1]);
plot(pivot_x_ref,pivot_y_ref,"ro");

plot(wheel_f_x_list_b,wheel_f_y_list_b,'Color',[0, 0, 0.5, 1],'LineWidth',2);
plot(wheel_r_x_list_b,wheel_r_y_list_b,'Color',[0, 0, 0.5, 1],'LineWidth',2);
plot(suspension_x_list_b,suspension_y_list_b,'Color',[0, 0.5, 0.5, 1],'LineWidth',2);
plot(pivot_x_b,pivot_y_b,"ko");
    
xlim(xl)
ylim(yl)

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