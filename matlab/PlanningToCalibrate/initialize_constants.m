function initialize_constants(figure_handle)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% @brief Initialize variables
% @author Siddharth Choudhary
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
globals

%addpath('/usr/local/gtsam_toolbox');
addpath('../IncrementalSLAM');
import gtsam.*

% values for global variables
% these are defined in globals.m
PLAN_FIG=1;			% default handle for plan figure
WORLD_SIZE=500;   % 0-100 meters in each direction
LINC=0.2;			% 0.1m length for spline interpolation
DT=1;				% Sample interval for controller
TEND=5000*DT;     % Total maximum run time for simulator
VVEL=2;				% Vehicle velocity (assumed constant)

KP=1;					% vehicle position error gain
KO=1;					% vehicle orientation error gain

WHEEL_BASE=1;		% vehicle wheel base (m)
WHEEL_RADIUS=0.3;  % nomial wheel radius (m)
R_MAX_RANGE=50.0;% maximum range (m)
R_RATE=12.566;		% rotation rate (rads/s)

GSIGMA_WHEEL=0.1; 	 % wheel variance used for control generation
GSIGMA_STEER=0.035;	 % steer variance used for control generation

SIGMA_Q=0.25;			% Multiplicative Wheel Noise SD (percent)
SIGMA_W=0.1;			% Additive Wheel Noise SD (rads/s)
SIGMA_S=0.01;			% Mutiplicative steer noise SD (percent) 
SIGMA_G=0.0087; 		% Additive steer noise SD (rads)
SIGMA_R=0.005; 		% wheel radius SD noise (m)

SIGMA_RANGE=0.003;		% Range Variance (m)
SIGMA_BEARING=0.0035; % bearing variance (rads)

fact=0.1;
SIGMA_Q=SIGMA_Q*fact;
SIGMA_W=SIGMA_W*fact;
SIGMA_S=SIGMA_S*fact;
SIGMA_G=SIGMA_G*fact;
SIGMA_R=SIGMA_R*fact;

GSIGMA_RANGE=0.25; 	 % Range SD (m) used for observation generation
GSIGMA_BEARING=0.0174;% Bearing SD (rads) used for observation generation

GSIGMA_X = 0.005;
GSIGMA_Y = 0.005;
GSIGMA_THETA = 0.05;

BET_GSIGMA_X = 0.005;
BET_GSIGMA_Y = 0.005;
BET_GSIGMA_THETA = 0.05;



R1_OFFSET_X = 0;
R1_OFFSET_Y=50.0;		% radar offset (m)
R1_OFFSET_THETA = 0;


R2_OFFSET_X = R1_OFFSET_X;
R2_OFFSET_Y=R1_OFFSET_Y;		% radar offset (m)
R2_OFFSET_THETA = R1_OFFSET_THETA;


R1_OFFSET_COV_X = 10;
R1_OFFSET_COV_Y = 10;
R1_OFFSET_COV_THETA = 0.1;

R2_OFFSET_COV_X = R1_OFFSET_COV_X;
R2_OFFSET_COV_Y = R1_OFFSET_COV_Y;
R2_OFFSET_COV_THETA = R1_OFFSET_COV_THETA;

r1_offset_x = R1_OFFSET_X + R1_OFFSET_COV_X*randn(1);
r1_offset_y = R1_OFFSET_Y + R1_OFFSET_COV_Y*randn(1);
r1_offset_theta = R1_OFFSET_THETA + R1_OFFSET_COV_THETA*randn(1);

r2_offset_x = R2_OFFSET_X + R2_OFFSET_COV_X*randn(1);
r2_offset_y = R2_OFFSET_Y + R2_OFFSET_COV_Y*randn(1);
r2_offset_theta = R2_OFFSET_THETA + R2_OFFSET_COV_THETA*randn(1);



use_sensor1 = 1; %Bearing Range
use_sensor2 = 0; %Between Factor

steps = 1;
step_size = 50;
num_plan_iters = 10;
num_samples = 360;
angle_discretization = 360;
num_monte_carlo_runs = 10;

USER_DATA.sensor1_transform_symbol = gtsam.symbol('t',1);
USER_DATA.sensor2_transform_symbol = gtsam.symbol('t',2);

end
