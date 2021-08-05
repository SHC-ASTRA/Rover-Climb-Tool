input_x = [0,2,2+1/tand(30),6,6,8];
input_y = [0,0,1,1,0,0];

wheel_radius = 0.12; % meters
wheelbase_length = 1.2 - 2*wheel_radius; % meters
suspension_height = 0.25; % meters

filename = 'testAnimated.gif';

front_wheel_pos = wheelbase_length - wheel_radius;
rear_wheel_pos = front_wheel_pos - wheelbase_length;
center_pos = (front_wheel_pos+rear_wheel_pos)/2;


[dirs_x, dirs_y, lengths] = calc_dirs_and_lengths(input_x, input_y);
adjusted_lengths = calc_adjusted_lengths(input_x, input_y, dirs_x, dirs_y, lengths, wheel_radius);
adjusted_distances = cumsum(adjusted_lengths);


segment_idx = 1;

h = figure(1);
hold on
axis equal
plot(input_x,input_y,"g")
wheel_f_x_list = [0];
wheel_f_y_list = [0];
wheel_f_plot = plot(wheel_f_x_list,wheel_f_y_list,"b");

wheel_f_plot.XDataSource = 'wheel_f_x_list';
wheel_f_plot.YDataSource = 'wheel_f_y_list';

wheel_r_x_list = [0];
wheel_r_y_list = [0];
wheel_r_plot = plot(wheel_r_x_list,wheel_r_y_list,"b");

wheel_r_plot.XDataSource = 'wheel_r_x_list';
wheel_r_plot.YDataSource = 'wheel_r_y_list';

suspension_x_list = [0];
suspension_y_list = [0];
suspension_plot = plot(suspension_x_list,suspension_y_list,"c");

suspension_plot.XDataSource = 'suspension_x_list';
suspension_plot.YDataSource = 'suspension_y_list';




for i = 1:1000
    
    center_pos = center_pos + 0.05;
    
    pos_dist = fzero(@(pos_dist) wheelbase_length-calc_wheel_distance(pos_dist, center_pos, input_x, input_y, dirs_x, dirs_y, lengths, adjusted_lengths, adjusted_distances, wheel_radius)...
        ,wheelbase_length);
    
    front_wheel_pos = center_pos+pos_dist/2;
    rear_wheel_pos = center_pos-pos_dist/2;
    
    [front_wheel_x,front_wheel_y, done] = calc_wheel_position(front_wheel_pos, input_x, input_y, dirs_x, dirs_y, lengths, adjusted_lengths, adjusted_distances, wheel_radius);
    if done
        disp("all done!")
        break;
    end
    
    [rear_wheel_x,rear_wheel_y] = calc_wheel_position(rear_wheel_pos, input_x, input_y, dirs_x, dirs_y, lengths, adjusted_lengths, adjusted_distances, wheel_radius);
    
    [suspension_x_list,suspension_y_list] = create_suspension(front_wheel_x, front_wheel_y, rear_wheel_x, rear_wheel_y, suspension_height);
    
    [wheel_f_x_list,wheel_f_y_list] = create_circle(front_wheel_x, front_wheel_y, wheel_radius, 100);
    [wheel_r_x_list,wheel_r_y_list] = create_circle(rear_wheel_x, rear_wheel_y, wheel_radius, 100);
    refreshdata
    drawnow
    
    frame = getframe(h);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    % Write to the GIF File
    if i == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',1/30);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',1/30);
    end
    

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

function [wheel_x,wheel_y] = position_wheel(pos,radius,linex1,linex2,liney1,liney2)

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

function [wheel_x,wheel_y, done] = calc_wheel_position(pos, input_x, input_y, dirs_x, dirs_y, lengths, adjusted_lengths, adjusted_distances, wheel_radius)

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
else
    [wheel_x,wheel_y] = position_wheel(wheel_pos,wheel_radius,input_x(segment_idx),input_x(segment_idx+1),input_y(segment_idx),input_y(segment_idx+1));
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

function [susp_x,susp_y] = create_suspension(front_wheel_x, front_wheel_y, rear_wheel_x, rear_wheel_y, height)

center_x = (front_wheel_x+rear_wheel_x)/2;
center_y = (front_wheel_y+rear_wheel_y)/2;

wheel_distance = sqrt((front_wheel_x-rear_wheel_x)^2 + (front_wheel_y-rear_wheel_y)^2);
wheel_dir_x = (front_wheel_x-rear_wheel_x)/wheel_distance;
wheel_dir_y = (front_wheel_y-rear_wheel_y)/wheel_distance;

normal_x = -wheel_dir_y;
normal_y = wheel_dir_x;

pivot_x = center_x + normal_x*height;
pivot_y = center_y + normal_y*height;

susp_x = [rear_wheel_x pivot_x front_wheel_x];
susp_y = [rear_wheel_y pivot_y front_wheel_y];

end