function score = score_sim(varargin)
[intersection,tip,intersection_Xs,intersection_Ys,tip_severity,suspension_length,suspension_height,wheel_radius] = do_sim(varargin{:});

score = 0;

score = score + suspension_length;
%score = score + (suspension_height+wheel_radius)*3;
score = score + (length(intersection_Xs)-1)*10;
score = score + (sum(tip_severity))*5 + tip*5;


end