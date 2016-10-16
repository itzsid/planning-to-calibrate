function detTransform = optimize_poses

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


result = Optimize(graph, ground_truth);
marginals = result.marginals;
res = result.res;
USER_DATA.result = res;
USER_DATA.marginals = marginals;
keys = gtsam.KeyVector(res.keys);


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
    detTransform.det = bitmax;
    detTransform.Result = res;
end



      % disp('Done');