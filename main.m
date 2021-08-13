% step_height = 0.215;
% slope_angle = 42.5;
% 
% wheel_radius = 0.228/2; % meters
% wheelbase_length = 0.955; % meters
% suspension_height = 0.258; % meters
% suspension_trap_frac = 0;
% 
% % suspension_design_x = [-wheelbase_length/2, -wheelbase_length/2, wheelbase_length/2, wheelbase_length/2]; % Distance from center of rover, negative towards rear wheel, positive towards front
% % suspension_design_y = [0,suspension_height, suspension_height, 0]; % Height above wheel centerline, not above ground
% 
% center_of_mass_x = 0.064; % Distance from center of rover, negative towards rear wheel, positive towards front
% center_of_mass_y = suspension_height+0.072; % Height above wheel centerline, not above ground
% 
% sim_distance_step = 0.01; % 0.05 is good for coarse sims, 0.01 is good for fine sims
% 
% filename = 'testAnimated.gif';
% gif_fps = 60;
% do_plot = false; % Code is much much faster without plotting! Set this to false when trying to optimize design!
% 
% score_sim(step_height=step_height, slope_angle=slope_angle, wheel_radius=wheel_radius, wheelbase_length=wheelbase_length, suspension_height=suspension_height, suspension_trap_frac=suspension_trap_frac,...
%     center_of_mass_x=center_of_mass_x, center_of_mass_y=center_of_mass_y, sim_distance_step=sim_distance_step, filename=filename, gif_fps=gif_fps, do_plot=do_plot, do_final_plot=true)

wheel_radius = 0.228/2;

slope_angle = 42.5;
step_height = 0.5;

min_wheelbase_length = 0.5;
max_wheelbase_length = 1.2 - 2*wheel_radius;
min_suspension_height = 0.1;
max_suspension_height = 2;
min_suspension_trapeziod_length = 0;
max_suspension_trapeziod_length = 2;

lb = [min_wheelbase_length, min_suspension_height, min_suspension_trapeziod_length];
ub = [max_wheelbase_length, max_suspension_height, max_suspension_trapeziod_length];

options = optimoptions('particleswarm');
options.Display = 'iter';
options.PlotFcn = {@pswplotbestf, @(optimValues,state)optim_plot_rover(optimValues,state,wheel_radius)};
options.UseParallel = true;


x = particleswarm(@(x) score_sim(slope_angle=slope_angle, step_height=step_height, wheelbase_length=x(1), suspension_height=x(2), suspension_trap_len=x(3), wheel_radius=wheel_radius, center_of_mass_y=0.072+x(2)), 3, lb, ub, options);

disp("Wheelbase Length: "+x(1)+" m");
disp("Suspension Height: "+x(2)+" m");
disp("Suspension Trapeziod Top Length: "+x(3)+" m");

do_sim(slope_angle=slope_angle, step_height=step_height, wheelbase_length=x(1), suspension_height=x(2), suspension_trap_len=x(3), center_of_mass_y=0.072+x(2), wheel_radius=wheel_radius, do_plot = true)