function [intersection,tip,intersection_Xs,intersection_Ys,tip_x_list,tip_y_list,suspension_length,suspension_height,wheel_radius,wheelbase_length,suspension_design_x,suspension_design_y] = do_sim(varargin)
default_step_height = 1;
default_spike_height = 1;
default_slope_angle = 30;

default_wheel_radius = 0.21/2; % meters
default_wheelbase_length = 0.955; % meters
default_suspension_height = 0.258; % meters
default_suspension_trap_len = 0.5;
default_suspension_design_x = []; % Distance from center of rover, negative towards rear wheel, positive towards front
default_suspension_design_y = []; % Height above wheel centerline, not above ground

default_center_of_mass_x = 0.064; % Distance from center of rover, negative towards rear wheel, positive towards front
default_center_of_mass_y = default_suspension_height+0.072; % Height above wheel centerline, not above ground
default_tip_safety_factor = 0; 

default_sim_distance_step = 0.01; % 0.05 is good for coarse sims, 0.01 is good for fine sims

default_filename = 'testAnimated.gif';
default_gif_fps = 60;
default_do_plot = false; % Code is much much faster without plotting! Set this to false when trying to optimize design!
default_do_final_plot = false;

p = inputParser;
p.KeepUnmatched = true;
addParameter(p,"step_height",default_step_height)
addParameter(p,"step_up_height",NaN)
addParameter(p,"step_down_height",NaN)
addParameter(p,"spike_height",default_spike_height)
addParameter(p,"slope_angle",default_slope_angle)
addParameter(p,"spike_angle",default_slope_angle)
addParameter(p,"wheel_radius",default_wheel_radius)
addParameter(p,"wheelbase_length",default_wheelbase_length)
addParameter(p,"base1_len",default_wheelbase_length)
addParameter(p,"suspension_height",default_suspension_height)
addParameter(p,"suspension_trap_len",default_suspension_trap_len)
addParameter(p,"suspension_design_x",default_suspension_design_x)
addParameter(p,"suspension_design_y",default_suspension_design_y)
addParameter(p,"center_of_mass_x",default_center_of_mass_x)
addParameter(p,"center_of_mass_y",default_center_of_mass_y)
addParameter(p,"tip_safety_factor",default_tip_safety_factor)
addParameter(p,"sim_distance_step",default_sim_distance_step)
addParameter(p,"filename",default_filename)
addParameter(p,"gif_fps",default_gif_fps)
addParameter(p,"do_plot",default_do_plot)
addParameter(p,"do_final_plot",default_do_final_plot)

parse(p,varargin{:})
step_height = p.Results.step_height;
step_up_height = p.Results.step_up_height;
step_down_height = p.Results.step_down_height;
spike_height = p.Results.spike_height;
slope_angle = p.Results.slope_angle;
spike_angle = p.Results.spike_angle;
wheel_radius = p.Results.wheel_radius;
wheelbase_length = p.Results.wheelbase_length;
suspension_height = p.Results.suspension_height;
suspension_trap_len = p.Results.suspension_trap_len;
base1_len = p.Results.base1_len;
suspension_design_x = p.Results.suspension_design_x;
suspension_design_y = p.Results.suspension_design_y;
center_of_mass_x = p.Results.center_of_mass_x;
tip_safety_factor = p.Results.tip_safety_factor;
center_of_mass_y = p.Results.center_of_mass_y;
sim_distance_step = p.Results.sim_distance_step;
filename = p.Results.filename;
gif_fps = p.Results.gif_fps;
do_plot = p.Results.do_plot;
do_final_plot = p.Results.do_final_plot;

if isnan(step_up_height)
    step_up_height = step_height;
end
if isnan(step_down_height)
    step_down_height = step_height;
end

slope_x = (1)/tand(slope_angle);
spike_x = (spike_height)/tand(spike_angle);
transition_x = (abs(step_up_height-step_down_height))/tand(20);
transition_x = max(transition_x,0.1);

course_dx = [0,2,0,1,transition_x,1,0,2,spike_x,0.5,spike_x,2,slope_x,2,slope_x,2];
course_x = cumsum(course_dx);
course_y = [0,0,step_up_height,step_up_height,step_down_height,step_down_height,0,0,spike_height,spike_height,0,0,1,1,0,0];

if isempty(suspension_design_x) || isempty(suspension_design_y)
    suspension_design_x = [-base1_len/2, -suspension_trap_len/2, suspension_trap_len/2, base1_len/2]; % Distance from center of rover, negative towards rear wheel, positive towards front
    suspension_design_y = [0,suspension_height, suspension_height, 0]; % Height above wheel centerline, not above ground
end

d = diff([suspension_design_x(:) suspension_design_y(:)]);
suspension_length = sum(sqrt(sum(d.^2,2)));

tip_margin = wheelbase_length*tip_safety_factor;

front_wheel_pos = wheelbase_length - wheel_radius;
rear_wheel_pos = front_wheel_pos - wheelbase_length;
center_pos = (front_wheel_pos+rear_wheel_pos)/2;


[dirs_x, dirs_y, lengths] = calc_dirs_and_lengths(course_x, course_y);
adjusted_lengths = calc_adjusted_lengths(course_x, course_y, dirs_x, dirs_y, lengths, wheel_radius);
adjusted_distances = cumsum(adjusted_lengths);




segment_idx = 1;

wheel_f_x_list = [0];
wheel_f_y_list = [0];

wheel_r_x_list = [0];
wheel_r_y_list = [0];

suspension_x_list = [0];
suspension_y_list = [0];

intersection_Xs = [NaN];
intersection_Ys = [NaN];

contact_pts_x = [0];
contact_pts_y = [0];

com_x_list = [0];
com_y_list = [0];

tip_x_list = [NaN];
tip_y_list = [NaN];

pivot_x = 0;
pivot_y = 0;

if do_final_plot || do_plot
    h = figure();
    hold on
    axis equal
    plot(course_x,course_y,"g")
    xlabel("Horizontal Dimension (m)")
    ylabel("Vertical Dimension (m)")
    
    wheel_f_plot = plot(wheel_f_x_list,wheel_f_y_list,"b");
    wheel_f_plot.XDataSource = 'wheel_f_x_list';
    wheel_f_plot.YDataSource = 'wheel_f_y_list';
    
    wheel_r_plot = plot(wheel_r_x_list,wheel_r_y_list,"b");
    wheel_r_plot.XDataSource = 'wheel_r_x_list';
    wheel_r_plot.YDataSource = 'wheel_r_y_list';

    pivot_plot = plot(0,0,"ko");
    pivot_plot.XDataSource = 'pivot_x';
    pivot_plot.YDataSource = 'pivot_y';
    
    suspension_plot = plot(suspension_x_list,suspension_y_list,"c");
    suspension_plot.XDataSource = 'suspension_x_list';
    suspension_plot.YDataSource = 'suspension_y_list';
    
    intersection_plot = plot(intersection_Xs,intersection_Ys,"rx");
    intersection_plot.XDataSource = 'intersection_Xs';
    intersection_plot.YDataSource = 'intersection_Ys';
    
    contact_plot = plot(contact_pts_x,contact_pts_y,"b.");
    contact_plot.XDataSource = 'contact_pts_x';
    contact_plot.YDataSource = 'contact_pts_y';
    
    com_plot = plot(com_x_list,com_y_list,Color=[0.9100 0.4100 0.1700]);
    com_plot.XDataSource = 'com_x_list';
    com_plot.YDataSource = 'com_y_list';
    
    tip_plot = plot(tip_x_list,tip_y_list,"x",Color=[0.9100 0.4100 0.1700]);
    tip_plot.XDataSource = 'tip_x_list';
    tip_plot.YDataSource = 'tip_y_list';
end

intersection = false;
tip = false;

for iter = 1:10000
    
    center_pos = center_pos + sim_distance_step;
    
    pos_dist = fzero(@(pos_dist) wheelbase_length-calc_wheel_distance(pos_dist, center_pos, course_x, course_y, dirs_x, dirs_y, lengths, adjusted_lengths, adjusted_distances, wheel_radius)...
        ,wheelbase_length);
    
    front_wheel_pos = center_pos+pos_dist/2;
    rear_wheel_pos = center_pos-pos_dist/2;
    
    [front_wheel_x,front_wheel_y, front_contact_x,front_contact_y, done] = calc_wheel_position(front_wheel_pos, course_x, course_y, dirs_x, dirs_y, lengths, adjusted_lengths, adjusted_distances, wheel_radius);
    if done
%         disp("all done!")
        break;
    end
    
    [rear_wheel_x,rear_wheel_y, rear_contact_x,rear_contact_y] = calc_wheel_position(rear_wheel_pos, course_x, course_y, dirs_x, dirs_y, lengths, adjusted_lengths, adjusted_distances, wheel_radius);
    
    contact_pts_x = [front_contact_x, rear_contact_x];
    contact_pts_y = [front_contact_y, rear_contact_y];
    
    [com_x,com_y] = position_relative_to_wheels(front_wheel_x, front_wheel_y, rear_wheel_x, rear_wheel_y, center_of_mass_x, center_of_mass_y);
    [pivot_x,pivot_y] = position_relative_to_wheels(front_wheel_x, front_wheel_y, rear_wheel_x, rear_wheel_y, 0, 0.05+suspension_height);
    
    [com_x_list,com_y_list] = create_circle(com_x, com_y, 0.05);
    com_x_list = [com_x_list, NaN, com_x, com_x];
    com_y_list = [com_y_list, NaN, com_y, min(front_contact_y,rear_contact_y)-0.1];
    
    if com_x+tip_margin>front_contact_x || com_x-tip_margin<rear_contact_x
        tip_x_list(end+1) = com_x;
        tip_y_list(end+1) = com_y;
        tip = true;
    end
    
    [suspension_x_list,suspension_y_list] = position_relative_to_wheels(front_wheel_x, front_wheel_y, rear_wheel_x, rear_wheel_y, suspension_design_x, suspension_design_y);
    
    
    for i = 1:length(suspension_x_list)-1
        for j = 1:length(course_x)-1
            [intersect_now, inter_x, inter_y] = check_lines_intersect(suspension_x_list(i:i+1),suspension_y_list(i:i+1),course_x(j:j+1),course_y(j:j+1));
            
            if intersect_now
                intersection_Xs(end+1) = inter_x;
                intersection_Ys(end+1) = inter_y;
                
                intersection = true;
            end
        end
    end
    
    
    [wheel_f_x_list,wheel_f_y_list] = create_circle(front_wheel_x, front_wheel_y, wheel_radius, 100);
    [wheel_r_x_list,wheel_r_y_list] = create_circle(rear_wheel_x, rear_wheel_y, wheel_radius, 100);
    if do_plot

        xlim([com_x-1,com_x+1])
        ylim([com_y-1.1,com_y+0.9])
        refreshdata(h,'caller')
        pause(0.000001)
        
        
        frame = getframe(h);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        % Write to the GIF File
        if iter == 1
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',1/gif_fps);
        else
            imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',1/gif_fps);
        end
    end
    
    
end

if do_final_plot || do_plot
    refreshdata(h,'caller')
    pause(0.000001)
end

% if intersection
%     disp("Suspension collides with terrain!");
% end
% 
% if tip
%     disp("Rover tips over!");
% end
end

function [dirs_x, dirs_y, lengths] = calc_dirs_and_lengths(x_list, y_list)
lengths = zeros(1,length(x_list)-1);

dirs_x = zeros(1,length(x_list)-1);
dirs_y = zeros(1,length(x_list)-1);
for i = 1:length(x_list)-1
    lengths(i) = sqrt((x_list(i+1)-x_list(i))^2 + (y_list(i+1)-y_list(i))^2);
    
    dirs_x(i) = (x_list(i+1)-x_list(i))/lengths(i);
    dirs_y(i) = (y_list(i+1)-y_list(i))/lengths(i);
end
end

function adj_lengths = calc_adjusted_lengths(x_list, y_list, dirs_x, dirs_y, lengths, radius)
adj_lengths = zeros(1,length(x_list)-1);

last_corner_cut_length = 0;

for segment_idx = 1:length(x_list)-1
    
    corner_rotate_length = 0;
    corner_cut_length = 0;
    
    if segment_idx < length(lengths)
        C = cross([dirs_x(segment_idx),dirs_y(segment_idx),0], [dirs_x(segment_idx+1),dirs_y(segment_idx+1),0]);
        angle_to_next = asin(C(3));
        if angle_to_next < 0
            next_junction = -1;
            corner_rotate_length = abs(angle_to_next)*radius;
        elseif angle_to_next > 0
            next_junction = 1;
            corner_cut_length = (1/tan((pi-angle_to_next)/2))*radius;
        end
    end
    
    adj_lengths(segment_idx) = lengths(segment_idx) + corner_rotate_length - corner_cut_length - last_corner_cut_length;
    
    last_corner_cut_length = corner_cut_length;
    
end
end

function [wheel_x,wheel_y,contact_x,contact_y] = position_wheel(pos,radius,linex1,linex2,liney1,liney2)

line_len = sqrt((linex2-linex1)^2 + (liney2-liney1)^2);
line_dir_x = (linex2-linex1)/line_len;
line_dir_y = (liney2-liney1)/line_len;
contact_x = linex1 + line_dir_x*pos;
contact_y = liney1 + line_dir_y*pos;

normal_dir_x = -line_dir_y;
normal_dir_y = line_dir_x;

wheel_x = contact_x+normal_dir_x*radius;
wheel_y = contact_y+normal_dir_y*radius;

end

function [wheel_x,wheel_y, contact_x,contact_y, done] = calc_wheel_position(pos, input_x, input_y, dirs_x, dirs_y, lengths, adjusted_lengths, adjusted_distances, wheel_radius)

adjusted_distances = [0 adjusted_distances];
diff_pos = adjusted_distances-pos;
diff_pos(diff_pos>0) = -inf;
[~, indexOfMax] = max(diff_pos);
start_distance = adjusted_distances(indexOfMax);
wheel_pos = pos-start_distance;
segment_idx = min(indexOfMax,length(lengths));
done = indexOfMax > length(lengths);


next_junction = 0;
corner_cut_length = 0;
corner_rotate_length = 0;

if segment_idx < length(lengths)
    C = cross([dirs_x(segment_idx),dirs_y(segment_idx),0], [dirs_x(segment_idx+1),dirs_y(segment_idx+1),0]);
    angle_to_next = asin(C(3));
    if angle_to_next < 0
        next_junction = -1;
        corner_rotate_length = abs(angle_to_next)*wheel_radius;
    elseif angle_to_next > 0
        next_junction = 1;
        corner_cut_length = (1/tan((pi-angle_to_next)/2))*wheel_radius;
    end
end

if segment_idx-1 > 0
    C = cross([dirs_x(segment_idx-1),dirs_y(segment_idx-1),0], [dirs_x(segment_idx),dirs_y(segment_idx),0]);
    angle_to_next = asin(C(3));
    if angle_to_next > 0
        prev_corner_cut_length = (1/tan((pi-angle_to_next)/2))*wheel_radius;
        wheel_pos = wheel_pos + prev_corner_cut_length;
    end
end


% if next_junction == 0 && pos > lengths(segment_idx)
%     wheel_pos = wheel_pos-lengths(segment_idx);
%     segment_idx = segment_idx + 1;
%
% elseif next_junction == 1 && wheel_pos > lengths(segment_idx)-corner_cut_length
%     wheel_pos = wheel_pos-lengths(segment_idx)+2*corner_cut_length;
%     segment_idx = segment_idx + 1;
%
% elseif next_junction == -1 && wheel_pos > lengths(segment_idx)+corner_rotate_length
%     wheel_pos = wheel_pos-(lengths(segment_idx)+corner_rotate_length);
%     segment_idx = segment_idx + 1;
% end

if next_junction == -1 && wheel_pos > lengths(segment_idx)
    wheel_angle = (wheel_pos-lengths(segment_idx))/wheel_radius;
    init_angle = atan2(dirs_y(segment_idx),dirs_x(segment_idx));
    [wheel_x,wheel_y] = rotate_wheel(wheel_angle,init_angle,wheel_radius,input_x(segment_idx+1),input_y(segment_idx+1));
    contact_x = input_x(segment_idx+1);
    contact_y = input_y(segment_idx+1);
else
    [wheel_x,wheel_y,contact_x,contact_y] = position_wheel(wheel_pos,wheel_radius,input_x(segment_idx),input_x(segment_idx+1),input_y(segment_idx),input_y(segment_idx+1));
end
end

function distance = calc_wheel_distance(wheel_pos_dist, center_pos, input_x, input_y, dirs_x, dirs_y, lengths, adjusted_lengths, adjusted_distances, wheel_radius)
[front_x,front_y] = calc_wheel_position(center_pos+wheel_pos_dist/2, input_x, input_y, dirs_x, dirs_y, lengths, adjusted_lengths, adjusted_distances, wheel_radius);
[rear_x,rear_y] = calc_wheel_position(center_pos-wheel_pos_dist/2, input_x, input_y, dirs_x, dirs_y, lengths, adjusted_lengths, adjusted_distances, wheel_radius);

distance = sqrt((front_x-rear_x)^2 + (front_y-rear_y)^2);
end

function [wheel_x,wheel_y] = rotate_wheel(angle,init_angle,radius,point_x,point_y)


wheel_x = point_x + radius*cos(angle-init_angle-pi/2);
wheel_y = point_y - radius*sin(angle-init_angle-pi/2);

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

function [intersecting,X,Y] = check_lines_intersect(x1,y1,x2,y2)

if x1(2) ~= x1(1) && x2(2) ~= x2(1)
    a1 = (y1(2)-y1(1)) / (x1(2) - x1(1));
    b1 = y1(1) - a1*x1(1);
    a2 = (y2(2)-y2(1)) / (x2(2) - x2(1));
    b2 = y2(1) - a2*x2(1);
    X = (b1-b2)/(a2-a1);
    Y = a1*X + b1;
    cond1 = x1(1) < X && X < x1(2);
    cond2 = x1(1) > X && X > x1(2);
    cond3 = x2(1) < X && X < x2(2);
    cond4 = x2(1) > X && X > x2(2);
else
    if x1(2) == x1(1)
        X = x1(1);
        a2 = (y2(2)-y2(1)) / (x2(2) - x2(1));
        b2 = y2(1) - a2*x2(1);
        Y = a2*X + b2;
    else
        X = x2(1);
        a1 = (y1(2)-y1(1)) / (x1(2) - x1(1));
        b1 = y1(1) - a1*x1(1);
        Y = a1*X + b1;
    end
    cond1 = y1(1) < Y && Y < y1(2);
    cond2 = y1(1) > Y && Y > y1(2);
    cond3 = y2(1) < Y && Y < y2(2);
    cond4 = y2(1) > Y && Y > y2(2);
end


intersecting = ( cond1 || cond2 ) && ( cond3 || cond4 );

end

