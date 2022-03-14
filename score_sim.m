function score = score_sim(varargin)
[intersection,tip,intersection_Xs,intersection_Ys,tip_x_list,tip_y_list,suspension_length,suspension_height,wheel_radius,wheelbase_legnth,suspension_design_x,suspension_design_y] = do_sim(varargin{:});

score = 0;

susp_density = 1;

total_length = wheelbase_legnth + 2*wheel_radius;
total_length_score = max(total_length-1.2,0);
if total_length_score > 0
    total_length_score = total_length_score+1;
end

score = score + susp_density*suspension_length;
score = score + 0.1*wheelbase_legnth;
score = score + (suspension_height+wheel_radius)*3;
score = score + (length(intersection_Xs)-1)*5;
score = score + (length(tip_x_list)-1)*5;
score = score + total_length_score*20;



end