function graph = generate_factor_graph(xtrue, observations, add_noise_index)

globals
graph = gtsam.NonlinearFactorGraph;


% Add Odometry
odometry_noise = gtsam.noiseModel.Diagonal.Sigmas([GSIGMA_X; GSIGMA_Y; GSIGMA_THETA]);
gsigma_x = GSIGMA_X;
gsigma_y = GSIGMA_Y;
gsigma_theta = GSIGMA_THETA;

[rows cols] = size(xtrue);
for i = 1:cols -1
    %fprintf('%d of %d\n',i,cols);
    start_loc=xtrue(1:3,i);
    end_loc=xtrue(1:3,i+1);
    pose1_symbol = i;
    pose2_symbol = i+1;
    
    point1 = gtsam.Point2(start_loc(1:2,:));
    rot1 = gtsam.Rot2(start_loc(3,:));
    
    point2 = gtsam.Point2(end_loc(1:2,:));
    rot2  = gtsam.Rot2(end_loc(3,:));
    
    pose1 = gtsam.Pose2(rot1, point1);
    pose2 = gtsam.Pose2(rot2, point2);
    
    dL = pose1.between(pose2);
    
    if (pose2_symbol <= add_noise_index)
        dX = dL.x + gsigma_x*randn(1);
        dY = dL.y + gsigma_y*randn(1);
        dTheta = dL.theta + gsigma_theta*randn(1);
    else
        dX = dL.x;
        dY = dL.y;
        dTheta = dL.theta;
    end
    dPose = gtsam.Pose2(dX, dY, dTheta);
    graph.add(gtsam.BetweenFactorPose2(pose1_symbol, pose2_symbol, dPose,  odometry_noise));
end



if use_sensor1 == 1
    gsigma_range = GSIGMA_RANGE;
    gsigma_bearing = GSIGMA_BEARING;

    % Add Observations
    observation_noise = gtsam.noiseModel.Diagonal.Sigmas([GSIGMA_BEARING; GSIGMA_RANGE]);
    [rows cols] = size(observations);
    transform_symbol = USER_DATA.sensor1_transform_symbol;
    for i = 1:cols
        %fprintf('%d of %d\n',i,cols);
        if observations(3,i) ~= 0
            pose_symbol = observations(4,i);
            landmark_symbol = gtsam.symbol('l',observations(3,i));
            if (pose_symbol <= add_noise_index)
                obs1 = observations(1,i) +  gsigma_range*randn(size(observations(1,i)));
                obs2 = observations(2,i) + gsigma_bearing*randn(size(observations(2,i)));
            else
                obs1 = observations(1,i);
                obs2 = observations(2,i);
            end
            %graph.add(gtsam.BetweenTransformFactorPose2(pose_symbol, landmark_symbol, transform_symbol, gtsam.Pose2 (gtsam.Rot2(obs2), obs1), observation_noise));
            graph.add(gtsam.BearingRangeTransformFactor2D(pose_symbol, landmark_symbol, transform_symbol, gtsam.Rot2(obs2), obs1, observation_noise));
        end
    end
    
end



if use_sensor2 == 1
    % Add between sensor factors
    between_sensor_noise = gtsam.noiseModel.Diagonal.Sigmas([BET_GSIGMA_X; BET_GSIGMA_Y; BET_GSIGMA_THETA]);
    [rows cols] = size(xtrue);
    transform_symbol = USER_DATA.sensor2_transform_symbol;
    bet_gsigma_x = BET_GSIGMA_X;
    bet_gsigma_y = BET_GSIGMA_Y;
    bet_gsigma_theta = BET_GSIGMA_THETA;
    r2_offset_x = R2_OFFSET_X;
    r2_offset_y = R2_OFFSET_Y;
    r2_offset_theta = R2_OFFSET_THETA;
    
    for i = 1:cols -1
        %fprintf('%d of %d\n',i,cols);
        start_loc=xtrue(1:3,i);
        end_loc=xtrue(1:3,i+1);
        pose1_symbol = i;
        pose2_symbol = i+1;
        
        point1 = gtsam.Point2(start_loc(1:2,:));
        rot1 = gtsam.Rot2(start_loc(3,:));
        
        point2 = gtsam.Point2(end_loc(1:2,:));
        rot2  = gtsam.Rot2(end_loc(3,:));
        
        pose1 = gtsam.Pose2(rot1, point1);
        pose2 = gtsam.Pose2(rot2, point2);

        composed_pose1 = pose1.compose(gtsam.Pose2(r2_offset_x, r2_offset_y, r2_offset_theta));
        composed_pose2 = pose2.compose(gtsam.Pose2(r2_offset_x, r2_offset_y, r2_offset_theta));

        
        dL = composed_pose1.between(composed_pose2);
        
        if (pose2_symbol <= add_noise_index)
            dX = dL.x + bet_gsigma_x*randn(1);
            dY = dL.y + bet_gsigma_y*randn(1);
            dTheta = dL.theta + bet_gsigma_theta*randn(1);
        else
            dX = dL.x;
            dY = dL.y;
            dTheta = dL.theta;
        end
        dPose = gtsam.Pose2(dX, dY, dTheta);
        graph.add(gtsam.BetweenTransformFactorPose2(pose1_symbol, pose2_symbol, transform_symbol, dPose,  odometry_noise));
    end
    
end

