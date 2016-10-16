%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% @brief Planning in Belief Space
% @author Siddharth Choudhary
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [path,cov_sensor1, cov_sensor2, err_sensor1, err_sensor2]=planningInBeliefSpace_withoutGUI(optStart)



globals;
disp('Adding Poses....');
disp('Press Enter to Finish');

pin=1;
npoints=0;
points=zeros(2,1);

    startX = optStart(1);
    startY = optStart(2);

pin= ~isempty(startX);
if pin
    npoints=npoints+1;
    points(1,npoints)=startX;
    points(2,npoints)=startY;
end


%% Randomized planning in Belief Space
trajectoryX = [startX];
trajectoryY = [startY];
plannedTrajectory = points;
plannedX = [];
plannedY = [];
plannedNpoints = 1;
USER_DATA.plannedResult = gtsam.Values;
tempTransform = -1;


cov_sensor1 = [];
cov_sensor2 = [];
err_sensor1 = [];
err_sensor2 = [];

for plan_iter = 1:num_plan_iters % Number of steps in the plan
    
    
    minTransformCov = bitmax;
    minStartX = -1;
    minStartY = -1;
    tempTrajectory = plannedTrajectory;
    tempX = [];
    tempY = [];
    tempNpoints = 1;
    
    for iter = 1:num_samples
        
        npoints = plannedNpoints;
        points=plannedTrajectory;    
        
        
        planX = [];
        planY = [];
 
        for step = 1:steps
            npoints = npoints + 1;
            
            angle = [0:2*pi/angle_discretization:2*pi];
            
            random_angle = angle(randi([1,numel(angle)]));
            points(1,npoints) = points(1,npoints-1) + step_size*cos(random_angle);
            points(2,npoints) = points(2,npoints-1) + step_size*sin(random_angle);
            
            dx=points(1,npoints)-points(1,npoints-1);
            dy=points(2,npoints)-points(2,npoints-1);
            length=sqrt(dx*dx + dy*dy);
            
            linc = LINC*length;
            xincs=points(1,npoints-1):linc*dx/length:points(1,npoints);
            yincs=points(2,npoints-1):linc*dy/length:points(2,npoints);
            if dy == 0
                yincs = ones(1,numel(xincs))*points(2, npoints-1);
            end
            
            if dx == 0
                xincs = ones(1,numel(yincs))*points(1, npoints-1);
            end            
            
            if numel(xincs) == numel(yincs)
                planX = [planX, xincs];
                planY = [planY, yincs];
            end
            
            plan = [planX; planY];


            
        end
        % Optimize
        if (size(plan,2)>1)
            if(plan_iter == 1)
                USER_DATA.path = plan;
                detTransform = optimize_poses_withoutGUI;
            else
                plan = plan(:,2:end);
                detTransform = predictUncertainty(plan);
            end
        end
        
        transformCov = detTransform.det;
        if transformCov < minTransformCov
            minTransformCov = transformCov;
            minStartX = points(1,npoints);
            minStartY = points(2, npoints);
            
            tempTrajectory = points;
            tempX = planX;
            tempY = planY;
            tempNpoints = npoints;
            tempTransform = detTransform;
        end
        
        if plan_iter == 1
            break
        end
    end
    
    
    
    if numel(tempX) == 0
        minTransformCov = transformCov;
        minStartX = points(1,npoints);
        minStartY = points(2, npoints);
        
        tempTrajectory = points;
        tempX = planX;
        tempY = planY;
        tempNpoints = npoints;
        tempTransform = detTransform;
    end
    
    plannedTrajectory = tempTrajectory;
    plannedX = [plannedX, tempX];
    plannedY = [plannedY, tempY];
    plannedNpoints = tempNpoints;
    minTransformCov;
    % Optimize again
    path = [plannedX; plannedY];
    USER_DATA.path = path;
    if(size(path,2)>1)
        detTransform = optimize_poses_withoutGUI;
    end
    
    minTransformCov
    USER_DATA.plannedResult = detTransform.Result;
    
    startX = minStartX;
    startY = minStartY;
    trajectoryX = [trajectoryX, startX];
    trajectoryY = [trajectoryY, startY];
    detTransform.Transform;
    
    transformError.sensor1 = -1;
    transformError.sensor2 = -1;
    marginalCov.sensor1 = -1;
    marginalCov.sensor2 = -1;

    if detTransform.Transform.sensor1 ~= -1 || detTransform.Transform.sensor2 ~= -1
        
        if use_sensor1 == 1
            transformError.sensor1 = detTransform.Transform.sensor1.between(USER_DATA.sensor1ActualTransform).translation.norm;
            marginalCov.sensor1 = detTransform.Marginals.sensor1;
        end

        if use_sensor2 == 1
            transformError.sensor2 = detTransform.Transform.sensor2.between(USER_DATA.sensor2ActualTransform).translation.norm;
            marginalCov.sensor2 = detTransform.Marginals.sensor2;
        end
        
        dTD(plan_iter) = marginalCov;
        err(plan_iter) = transformError; %sqrt(transformError.x^2+transformError.y^2+transformError.theta^2);
    else
        dTD(plan_iter) = marginalCov;
        err(plan_iter) = transformError;
    end
    
    cov_sensor1 = [cov_sensor1, marginalCov.sensor1];
    cov_sensor2 = [cov_sensor2, marginalCov.sensor2];
    
    err_sensor1 = [err_sensor1, transformError.sensor1];
    err_sensor2 = [err_sensor2, transformError.sensor2];
    %     translation_error = ceil(transformError.translation.norm)
    %     if translation_error > 100
    %         translation_error = 100;
    %     end

    
    
    pause(0.05)
end
