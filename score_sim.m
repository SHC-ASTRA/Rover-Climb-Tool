function score = score_sim(varargin)
[intersection,tip,intersection_Xs,intersection_Ys,tip_x_list,tip_y_list,suspension_length,suspension_height,wheel_radius] = do_sim(varargin{:});

score = 0;

score = score + suspension_length;
%score = score + (suspension_height+wheel_radius)*3;
score = score + (length(intersection_Xs)-1)*5;
score = score + (length(tip_x_list)-1)*5;


end