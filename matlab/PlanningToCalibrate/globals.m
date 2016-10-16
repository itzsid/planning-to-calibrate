
% definitions of global variables
% values for these are set in ginit.m
global PLAN_FIG;		% handle for main plan figure
global WORLD_SIZE; 	% size of the world (for display purposes only).
global LINC;			% increment along which spline path is evaluated.
global DT;				% Sample interval for controller
global TEND;         % Total maximum run time for simulator
global VVEL;			% vehicle velocity

global KP;				% vehicle position error gain
global KO;				% vehicle orientation error gain

global WHEEL_BASE;	% vehicle wheel base (m)
global WHEEL_RADIUS; % nominal wheel radius
global R_MAX_RANGE;  % maximum radar range
global R_RATE;			% rotation rate of radar

global GSIGMA_RANGE 	 % Range SD (m) used for observation generation
global GSIGMA_BEARING % Bearing SD (rads) used for observation generation
global GSIGMA_WHEEL 	 % wheel variance used for control generation
global GSIGMA_STEER   % steer variance used for control generation
global GSIGMA_X
global GSIGMA_Y
global GSIGMA_THETA 

global BET_GSIGMA_X
global BET_GSIGMA_Y
global BET_GSIGMA_THETA

global SIGMA_Q			% Multiplicative Wheel Noise SD (percent)
global SIGMA_W			% Additive Wheel Noise SD (rads/s)
global SIGMA_S			% Mutiplicative steer noise SD (percent) 
global SIGMA_G 		% Additive steer noise SD (rads)
global SIGMA_R 		% wheel radius SD noise (m)
global SIGMA_RANGE	% Range Variance (m)
global SIGMA_BEARING % bearing variance (rads)

global R1_OFFSET_X;
global R1_OFFSET_Y;
global R1_OFFSET_THETA;
global R1_OFFSET_COV_X;
global R1_OFFSET_COV_Y;
global R1_OFFSET_COV_THETA;


global R2_OFFSET_X;
global R2_OFFSET_Y;
global R2_OFFSET_THETA;
global R2_OFFSET_COV_X;
global R2_OFFSET_COV_Y;
global R2_OFFSET_COV_THETA;

global USER_DATA;

global r1_offset_x;
global r1_offset_y;
global r1_offset_theta;

global r2_offset_x;
global r2_offset_y;
global r2_offset_theta;

global use_sensor1;
global use_sensor2;

global handler;

global step_size;
global steps;
global num_plan_iters;
global num_samples;
global num_monte_carlo_runs;
global angle_discretization;
global num_runs;