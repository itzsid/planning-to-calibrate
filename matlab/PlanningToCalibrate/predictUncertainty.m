function detTransform = predictUncertainty(plan)

tStart = tic;

globals
xtrue = USER_DATA.xtrue;
obs = USER_DATA.observations;
[xnew, unew, uz_new] = simulate_controls(plan);
offset = size(xtrue,2);
plannedResult = USER_DATA.plannedResult;

tDA = tic;
if use_sensor1 == 1
    %ML (Maximum likelihood) observations
    landmarks = gtsam.Values;
    plannedResult_Keys = gtsam.KeyVector(plannedResult.keys);
    for i = 0:size(plannedResult_Keys) -1
        try
            if strcmp(class(plannedResult.atPoint2(plannedResult_Keys.at(i))), 'gtsam.Point2')
                point= plannedResult.atPoint2(plannedResult_Keys.at(i)); % Pose 1
                landmarks.insert(plannedResult_Keys.at(i), point);
            end
        catch
            continue;
        end
    end
    
    obs_ml = data_associate(xnew, landmarks, offset);
    obs_appended = [obs, obs_ml]; % Append new observations
else
    
    obs_ml = [];
    obs_appended = [];
end
tDATime = toc(tDA);

xtrue_appended = [xtrue, xnew];

tValues = tic;
% Generate factor graph
ground_truth = generateValues(xtrue_appended);
graph = generate_factor_graph(xtrue_appended, obs_appended, offset);
tValuesTime = toc(tValues);


tGraph = tic;
% TODO: Remove explicit optimization
initial = gtsam.Values(plannedResult);
for i = 0:graph.size - 1
    factor = graph.at(i);
    if (isa(factor, 'gtsam.BetweenTransformFactorPose2')) || (isa(factor, 'gtsam.BetweenFactorPose2'))
        pose1 = factor.keys.at(0);
        pose2 = factor.keys.at(1);
        if ~initial.exists(pose2)
            initializedPose = initial.atPose2(pose1).compose(factor.measured);
            initial.insert(pose2, initializedPose);
        end
    end
end

res = gtsam.Values(initial);
tGraphTime = toc(tGraph);


%result = Optimize(graph, ground_truth);
%marginals = result.marginals;
%res = result.res;

priorMean = initial.atPose2(1); % add the prior to the first pose
priorNoise = gtsam.noiseModel.Diagonal.Sigmas([0.005; 0.005; 0.005]);
graph.add(gtsam.PriorFactorPose2(1, priorMean, priorNoise)); % add directly to graph



variables = gtsam.KeyVector;

if use_sensor1 == 1
    variables.push_back(USER_DATA.sensor1_transform_symbol);
end

if use_sensor2 == 1
    variables.push_back(USER_DATA.sensor2_transform_symbol);
end


if res.exists(USER_DATA.sensor1_transform_symbol) || res.exists(USER_DATA.sensor2_transform_symbol)
    try
        
        %    [bnt1,gfg] = graph.linearize(estimate).eliminatePartialMultifrontal(rest_variables);
        %    detTransform.det = -2*gfg.eliminateSequential.logDeterminant;
        timeMarginals = tic;
        marginals = gtsam.Marginals(graph, res);  % compute marginals
        detTransform.det = log(det(marginals.jointMarginalCovariance(variables).fullMatrix));
        timeElapsedMarginals = toc(timeMarginals);
    catch IndeterminantLinearSystemException
        detTransform.det = flintmax;
    end
    
    %determinant = det(marginals.marginalCovariance(USER_DATA.transform_symbol));
    %detTransform.det = det; %det(transformCov);
    detTransform.Result = res;
else
    detTransform.det = flintmax;
    detTransform.Result = res;
end


timeElapsed = toc(tStart);


