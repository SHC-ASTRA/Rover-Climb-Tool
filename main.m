wheel_radius = 0.25/2;

slope_angle = 42;
step_height = 0.4;
spike_angle = 30;
spike_height = 0.2;

min_wheelbase_length = 0.5;
max_wheelbase_length = 1.2;
min_suspension_height = 0.05;
max_suspension_height = 0.5;
min_suspension_trapezoid_length = 0.1;
max_suspension_trapezoid_length = max_wheelbase_length;

lb = [min_wheelbase_length, min_suspension_height, min_suspension_trapezoid_length, min_wheelbase_length];
ub = [max_wheelbase_length, max_suspension_height, max_suspension_trapezoid_length, max_wheelbase_length];

options = optimoptions('particleswarm');
options.Display = 'iter';
options.PlotFcn = {@pswplotbestf, @(optimValues,state)optim_plot_rover(optimValues,state,wheel_radius)};
options.UseParallel = true;

%%

x = particleswarm(@(x) score_sim(slope_angle=slope_angle, step_height=step_height, spike_angle=spike_angle, spike_height=spike_height, wheelbase_length=x(1), base1_len=x(4), suspension_height=x(2), suspension_trap_len=x(3), wheel_radius=wheel_radius, center_of_mass_y=0.072+0.05+x(2)), 4, lb, ub, options);

%%
disp("Wheelbase Length: "+x(1)+" m");
disp("Trapezoid Base One Length: "+x(4)+" m");
disp("Trapezoid Base Two Length: "+x(3)+" m");
disp("Suspension Height: "+x(2)+" m");
disp("Total Height: "+(x(2)+wheel_radius)+" m");

score_sim(slope_angle=slope_angle, step_height=step_height, spike_height=spike_height, spike_angle=spike_angle, wheelbase_length=x(1), base1_len=x(4), suspension_height=x(2), suspension_trap_len=x(3), wheel_radius=wheel_radius, center_of_mass_y=0.072+0.05+x(2), do_plot = true)

%% Reference

% slope_angle = 42;
% step_height = 0.18;
% 
% wheel_radius_ref = 0.21/2;
% wheelbase_length_ref = 0.955;
% suspension_height_ref = 0.221;
% suspension_trap_len_ref = 0.140;
% base1_len_ref = 0.904;
% 
% score_sim(slope_angle=slope_angle, step_height=step_height, spike_height=spike_height, spike_angle=spike_angle, wheelbase_length=wheelbase_length_ref, base1_len=base1_len_ref, suspension_height=suspension_height_ref, suspension_trap_len=suspension_trap_len_ref, wheel_radius=wheel_radius_ref, center_of_mass_y=0.072+0.055+suspension_height_ref, do_plot = false)