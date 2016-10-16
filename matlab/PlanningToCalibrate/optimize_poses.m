function detTransform = optimize_poses(estimated_graph_handle)

globals
path = USER_DATA.path;
[xtrue, utrue, uz] = simulate_controls(path);
USER_DATA.xtrue = xtrue;
USER_DATA.utrue = utrue;
USER_DATA.uz = uz;
USER_DATA.path = path;
offset = size(xtrue,2);

if use_sensor1 == 1
    % Data associate
    obs = data_associate(USER_DATA.xtrue, USER_DATA.landmark_values, 0);
else
    obs = [];
end
USER_DATA.observations = obs;



% Generate factor graph
ground_truth = generateValues(xtrue);
graph = generate_factor_graph(xtrue, obs, offset);
USER_DATA.graph = graph;
USER_DATA.ground_truth = ground_truth;

axes(estimated_graph_handle);
cla;
hold on;
title('PLOT');
%gtsam.plot2DTrajectory(initialC, 'r-'); axis equal
gtsam.plot2DPoints(ground_truth, 'b'); axis equal

for factor = 0:graph.nrFactors -1
    if strcmp(class(graph.at(factor)), 'gtsam.BearingRangeTransformFactor2D')
        pose_symbol=graph.at(factor).keys.at(0); % Pose 1
        landmark_symbol=graph.at(factor).keys.at(1);
        landmark = ground_truth.atPoint2(landmark_symbol);
        pose = ground_truth.atPose2(pose_symbol);
        plot(pose.x, pose.y,'-g');
        line([landmark.x, pose.x],[landmark.y, pose.y])
    elseif strcmp(class(graph.at(factor)), 'gtsam.BetweenFactorPose2')
        pose1_symbol=graph.at(factor).keys.at(0); % Pose 1
        pose2_symbol=graph.at(factor).keys.at(1);
        pose1 = ground_truth.atPose2(pose1_symbol);
        pose2 = ground_truth.atPose2(pose2_symbol);
        line([pose1.x, pose2.x],[pose1.y, pose2.y]);
    end
end

result = Optimize(graph, ground_truth);
marginals = result.marginals;
res = result.res;
USER_DATA.result = res;
USER_DATA.marginals = marginals;
keys = gtsam.KeyVector(res.keys);
gtsam.plot2DTrajectory(res, 'r', marginals);axis equal


variables = gtsam.KeyVector;

if use_sensor1 == 1
variables.push_back(USER_DATA.sensor1_transform_symbol);
end

if use_sensor2 == 1
variables.push_back(USER_DATA.sensor2_transform_symbol);
end

detTransform.Transform.sensor1 = -1;
detTransform.Transform.sensor2 = -1;
detTransform.Marginals.sensor1 = -1;
detTransform.Marginals.sensor2 = -1;

if res.exists(USER_DATA.sensor1_transform_symbol) || res.exists(USER_DATA.sensor2_transform_symbol)
    if use_sensor1 == 1
        estimatedTransform = res.atPose2(USER_DATA.sensor1_transform_symbol);
    elseif use_sensor2 == 1
        estimatedTransform = res.atPose2(USER_DATA.sensor2_transform_symbol);
    end
    
    if use_sensor1 == 1 || use_sensor2 == 1
        for i = 0:size(keys) -1
            try
                if strcmp(class(res.atPose2(keys.at(i))), 'gtsam.Pose2')
                    if gtsam.symbolChr(keys.at(i)) == 0
                        pose= res.atPose2(keys.at(i)); % Pose 1
                        angle = pose.theta;
                        poseTransformed = pose.compose(estimatedTransform);
                        plot(pose.x, pose.y,'or');
                        line([pose.x, poseTransformed.x],[pose.y, poseTransformed.y],'Color','r');
                    end
                end
            catch
                continue;
            end
        end
    end
    
    if use_sensor1 == 1
        transform_sensor1 = res.atPose2(USER_DATA.sensor1_transform_symbol);
        detTransform.Transform.sensor1 = transform_sensor1;
        detTransform.Marginals.sensor1 = det(marginals.marginalCovariance(USER_DATA.sensor1_transform_symbol));

    end

    if use_sensor2 == 1
        transform_sensor2 = res.atPose2(USER_DATA.sensor2_transform_symbol);
        detTransform.Transform.sensor2 = transform_sensor2;
        detTransform.Marginals.sensor2 = det(marginals.marginalCovariance(USER_DATA.sensor2_transform_symbol));
    end


    transformCov = marginals.jointMarginalCovariance(variables).fullMatrix;
    detTransform.det = det(transformCov);
    detTransform.X = transformCov(1,1);
    detTransform.Y = transformCov(2,2);
    detTransform.Theta = transformCov(3,3);
    detTransform.Result = res;
    %fprintf('Determinant of Transform Covariance: %e\n',detTransform.det);
else
    detTransform.det = flintmax;
    detTransform.Result = res;
end
gtsam.plot2DPoints(res, 'g', marginals);



      % disp('Done');