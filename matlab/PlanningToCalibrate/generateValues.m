function ground_truth =  generateValues(xtrue)

globals
%obs = data_associate(USER_DATA.xtrue, USER_DATA.landmarks);
%USER_DATA.observations = obs;

ground_truth = gtsam.Values;
landmark_nodes = gtsam.Values;
pose_nodes = gtsam.Values;

% Add Poses

[rows cols] = size(xtrue);
for i = 1:cols
    %pose_symbol = gtsam.symbol('x',i);
    pose = gtsam.Pose2(xtrue(1,i), xtrue(2,i), xtrue(3,i));
    ground_truth.insert(i, pose);
    pose_nodes.insert(i, pose);
    
end

% Add Landmarks
landmarks = USER_DATA.landmarks;
[rows cols] = size(landmarks);
for i = 1:rows
    landmark_symbol = gtsam.symbol('l',i);
    landmark = gtsam.Point2(landmarks(i,1), landmarks(i,2));
    ground_truth.insert(landmark_symbol, landmark);
    landmark_nodes.insert(landmark_symbol, landmark);   
end

% Add Transform
if use_sensor1 == 1
    sensor1_transform_symbol = USER_DATA.sensor1_transform_symbol;
    sensor1_transform = gtsam.Pose2(r1_offset_x, r1_offset_y, r1_offset_theta); %R_OFFSET in Y;
    ground_truth.insert(sensor1_transform_symbol, sensor1_transform);
    USER_DATA.sensor1_transform_symbol = sensor1_transform_symbol;
    USER_DATA.sensor1ActualTransform = gtsam.Pose2(R1_OFFSET_X, R1_OFFSET_Y, R1_OFFSET_THETA);
end

if use_sensor2 == 1
    sensor2_transform_symbol = USER_DATA.sensor2_transform_symbol;
    sensor2_transform = gtsam.Pose2(r2_offset_x, r2_offset_y, r2_offset_theta); %R_OFFSET in Y;
    ground_truth.insert(sensor2_transform_symbol, sensor2_transform);
    USER_DATA.sensor2_transform_symbol = sensor2_transform_symbol;
    USER_DATA.sensor2ActualTransform = gtsam.Pose2(R2_OFFSET_X, R2_OFFSET_Y, R2_OFFSET_THETA);
end


USER_DATA.landmark_nodes = landmark_nodes;
USER_DATA.pose_nodes = pose_nodes;

