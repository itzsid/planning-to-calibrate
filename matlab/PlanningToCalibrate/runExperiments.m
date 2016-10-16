globals
initialize_constants

experiment_id = 19;
use_sensor1 = 1;
use_sensor2 = 0;
R1_OFFSET_COV_X = 20;
R1_OFFSET_COV_Y = 20;
R1_OFFSET_COV_THETA = 0.5;

% prompt = {'Enter Number of Runs'};
% dlg_title = 'Input';
% num_lines = 1;
% ans = inputdlg(prompt, dlg_title, num_lines);
% num_monte_carlo_runs = str2num(ans{:});

% prompt2 = {'Enter Scenario Number'};
% dlg_title2 = 'Input';
% num_lines2 = 1;
% ans = inputdlg(prompt2, dlg_title2, num_lines2);
% scNum = str2num(ans{:});

for scNum = 1:4
    
    landmarks = [];
    landmark_values = gtsam.Values;
    
    switch scNum
        case 1
            landmarks = [175 250; 275 250; 225 250+(sqrt(3)/2*100)];
            optStart = [225 250+(sqrt(3)/4*100)];
            
        case 2
            landmarks = [150 250; 170 250; 190 250; 210 250; 210 270; 210 290; 210 310];
            optStart = [160 300];
        case 3
            x = [.1:.2:1]*WORLD_SIZE;
            y = [.1:.2:1]*WORLD_SIZE;
            
            [X,Y] = meshgrid(x,y);

            x = reshape(X,numel(X),1);
            y = reshape(Y,numel(Y),1);
            
            landmarks = [x y];
            optStart = [275 275];
        case 4
            landmarks = [250 250; 300 250];
            optStart = [250 200];
            
            
    end
    
    
    
    for i = 1:length(landmarks)
        xs = landmarks(i,1);
        ys = landmarks(i,2);
        point = gtsam.Point2(xs,ys);
        landmark_values.insert(gtsam.symbol('l',i), point);
    end
    
    USER_DATA.landmarks = landmarks;
    USER_DATA.landmark_values = landmark_values;
    
    covT1_1 = zeros(num_monte_carlo_runs,8);
    covT1_2 = zeros(num_monte_carlo_runs,8);
    errT1_1= zeros(num_monte_carlo_runs,8);
    errT1_2 = zeros(num_monte_carlo_runs,8);
    
    covT2_1 = zeros(num_monte_carlo_runs,8);
    covT2_2 = zeros(num_monte_carlo_runs,8);
    errT2_1 = zeros(num_monte_carlo_runs,8);
    errT2_2 = zeros(num_monte_carlo_runs,8);
    
     for n=1:num_monte_carlo_runs
     [path,cov_sensor1, cov_sensor2, err_sensor1, err_sensor2]=randomWalk_withoutGUI(optStart);
     [temp,n_path]=size(path);
     USER_DATA.path = path;
     
     covT1_1(n,1:length(cov_sensor1)) = cov_sensor1;
     covT1_2(n,1:length(cov_sensor2)) = cov_sensor2;
     errT1_1(n,1:length(err_sensor1)) = err_sensor1;
     errT1_2(n,1:length(err_sensor2)) = err_sensor2;
     end
    
  
    fileName = strcat(['Random_cov_Sensor_1_Scene' num2str(scNum) '_' num2str(num_monte_carlo_runs) '_' num2str(experiment_id) '_' 'runs.csv']);
    csvwrite(fileName,covT1_1);
    
    fileName = strcat(['Random_err_Sensor_1_Scene' num2str(scNum) '_' num2str(num_monte_carlo_runs) '_' num2str(experiment_id) '_' 'runs.csv']);
    csvwrite(fileName,errT1_1);

    fileName = strcat(['Random_cov_Sensor_2_Scene' num2str(scNum) '_' num2str(num_monte_carlo_runs) '_' num2str(experiment_id) '_' 'runs.csv']);
    csvwrite(fileName,covT1_2);
    
    fileName = strcat(['Random_err_Sensor_2_Scene' num2str(scNum) '_' num2str(num_monte_carlo_runs) '_' num2str(experiment_id) '_' 'runs.csv']);
    csvwrite(fileName,errT1_2);
       
     
    for n=1:num_monte_carlo_runs
        [path,cov_sensor1, cov_sensor2, err_sensor1, err_sensor2]=planningInBeliefSpace_withoutGUI(optStart);
        [temp,n_path]=size(path);
        USER_DATA.path = path;
        
        covT2_1(n,1:length(cov_sensor1)) = cov_sensor1;
        errT2_1(n,1:length(err_sensor1)) = err_sensor1;
        covT2_2(n,1:length(cov_sensor2)) = cov_sensor2;
        errT2_2(n,1:length(err_sensor2)) = err_sensor2;
    end
    
    fileName = strcat(['Belief_cov_Sensor_1_Scene' num2str(scNum) '_' num2str(num_monte_carlo_runs) '_' num2str(experiment_id) '_' 'runs.csv']);
    csvwrite(fileName,covT2_1);
    
    fileName = strcat(['Belief_err_Sensor_1_Scene' num2str(scNum) '_' num2str(num_monte_carlo_runs) '_' num2str(experiment_id) '_' 'runs.csv']);
    csvwrite(fileName,errT2_1);

    fileName = strcat(['Belief_cov_Sensor_2_Scene' num2str(scNum) '_' num2str(num_monte_carlo_runs) '_' num2str(experiment_id) '_' 'runs.csv']);
    csvwrite(fileName,covT2_2);
    
    fileName = strcat(['Belief_err_Sensor_2_Scene' num2str(scNum) '_' num2str(num_monte_carlo_runs) '_' num2str(experiment_id) '_' 'runs.csv']);
    csvwrite(fileName,errT2_2);
    
end
