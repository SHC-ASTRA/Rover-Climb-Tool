function suspension_design_x = suspension_design_x_func(wheel_radius,x)
suspension_design_x = (x(1)-wheel_radius)*linspace(-0.5,0.5,12);
