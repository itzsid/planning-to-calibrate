classdef IncrementalOptimization
    % Incremental Optimization
    properties (SetAccess = public)
        
        graph_ % NonLinearFactorGraph:
        groundTruth_ % Ground Truth:
        adjacencyMatrix_; % Adjacency Matrix
        adjacencyArray_;
        optimizerType_; % Optimizer Type
        optimizer_;
        
        
        estimatedGraph_;
        estimatedInitial_;
        estimatedResult_;
        newFactor_;
        newInitial_;
        count_;
        latestPose_;
        currGroundTruth_;
        bearingRangeOffsetNoise_;
        betweenFactorOffsetNoise_;
        landmarks_ = [];
        poses_ = [];
        
    end
    
    methods
        function IO = IncrementalOptimization(graph, groundTruth, adjacencyMatrix, bearingRangeOffsetNoise, betweenFactorOffsetNoise, optimizerType)
            import gtsam.*
          
            IO.graph_ = graph.clone;
            IO.groundTruth_ = gtsam.Values(groundTruth);
            IO.adjacencyMatrix_ = adjacencyMatrix;
            IO.optimizerType_ = optimizerType;
            IO.estimatedGraph_ = gtsam.NonlinearFactorGraph;
            IO.estimatedInitial_ = gtsam.Values;
            IO.estimatedResult_ = gtsam.Values;
            IO.newFactor_ = gtsam.NonlinearFactorGraph;
            IO.newInitial_ = gtsam.Values;            
            IO.adjacencyArray_ = cell2mat(adjacencyMatrix.values);
            IO.currGroundTruth_ = gtsam.Values;
            IO.bearingRangeOffsetNoise_ = bearingRangeOffsetNoise;
            IO.betweenFactorOffsetNoise_ = betweenFactorOffsetNoise;
            IO.newInitial_.insert(1, groundTruth.atPose2(1)); %If the value at Pose 0 is not given, initialize it at origin
            priorMean = IO.newInitial_.atPose2(1); % add the prior to the first pose
            priorNoise = gtsam.noiseModel.Diagonal.Sigmas([0.005; 0.005; 0.005]);
            IO.newFactor_.add(gtsam.PriorFactorPose2(1, priorMean, priorNoise)); % add directly to graph
            IO.currGroundTruth_.insert(1, IO.groundTruth_.atPose2(1));

            IO.poses_ = [1];
            IO.landmarks_ = [];
            IO.count_ = 1;
            switch optimizerType
                case 'ISAM2'
                    IO.optimizer_ = gtsam.ISAM2;
            end
        end
        
        
        function IO = nextMeasurement(IO)
            
            currFactor = IO.graph_.at(IO.adjacencyArray_(IO.count_));
            IO.newFactor_.add(currFactor);
            % Check if the variables are initialized
            pose = currFactor.keys.at(0);
            IO.latestPose_ = pose;
   
                if IO.estimatedResult_.exists(pose)
                    poseEstimate = IO.estimatedResult_.atPose2(pose);
                elseif IO.newInitial_.exists(pose)
                    poseEstimate = IO.newInitial_.atPose2(pose);
                else
                    disp('Error in initialization');
                end
                                              
                                
            if (isa(currFactor, 'gtsam.BearingRangeTransformFactor2D'))
                
                % Add transform initial value if not already present
                transform = currFactor.keys.at(2);
                if ~IO.estimatedInitial_.exists(transform) && ~IO.newInitial_.exists(transform)
                    IO.newInitial_.insert(transform, IO.groundTruth_.atPose2(transform));
                    transformPriorMean = IO.newInitial_.atPose2(transform); % add the prior to the first pose
                    transformPriorNoise = gtsam.noiseModel.Diagonal.Sigmas([IO.bearingRangeOffsetNoise_.X; IO.bearingRangeOffsetNoise_.Y; IO.bearingRangeOffsetNoise_.Theta]);
                    IO.newFactor_.add(gtsam.PriorFactorPose2(transform, transformPriorMean, transformPriorNoise)); % add directly to graph
                end                    
                
                if IO.estimatedResult_.exists(transform)
                    transformEstimate = IO.estimatedResult_.atPose2(transform);
                elseif IO.newInitial_.exists(transform)
                    transformEstimate = IO.newInitial_.atPose2(transform);
                else
                    disp('Error in initialization');
                end
                
                % Add landmark initial value if not already present
                landmark = currFactor.keys.at(1);                
                if ~IO.estimatedInitial_.exists(landmark) && ~IO.newInitial_.exists(landmark)
                    [bearing, range]=currFactor.measured;
                    local = gtsam.Point2(cos(bearing.theta)*range,sin(bearing.theta)*range);
                    globalPoint = poseEstimate.compose(transformEstimate).transform_from(local);
                    IO.newInitial_.insert(landmark, globalPoint);
                    IO.currGroundTruth_.insert(landmark, IO.groundTruth_.atPoint2(landmark));
                    IO.landmarks_ = [IO.landmarks_, landmark];
                end
                
            elseif (isa(currFactor, 'gtsam.BetweenTransformFactorPose2'))
                
                transform = currFactor.keys.atPose2(2);
                if ~IO.estimatedInitial_.exists(transform) && ~IO.newInitial_.exists(transform)
                    IO.newInitial_.insert(transform, IO.groundTruth_.atPose2(transform));
                    transformPriorMean = IO.newInitial_.atPose2(transform); % add the prior to the first pose
                    transformPriorNoise = gtsam.noiseModel.Diagonal.Sigmas([IO.betweenFactorOffsetNoise_.X; IO.betweenFactorOffsetNoise_.Y; IO.betweenFactorOffsetNoise_.Theta]);
                    IO.newFactor_.add(gtsam.PriorFactorPose2(transform, transformPriorMean, transformPriorNoise)); % add directly to graph
                end                    
                
                if IO.estimatedResult_.exists(transform)
                    transformEstimate = IO.estimatedResult_.atPose2(transform);
                elseif IO.newInitial_.exists(transform)
                    transformEstimate = IO.newInitial_.atPose2(transform);
                else
                    disp('Error in initialization');
                end
                
                   pose2 = currFactor.keys.at(1);
                if ~IO.estimatedInitial_.exists(pose2) && ~IO.newInitial_.exists(pose2)
                    initializedPose = poseEstimate.compose(transformEstimate).compose(currFactor.measured).compose(transformEstimate.inverse); 
                    IO.newInitial_.insert(pose2,initializedPose);
                    IO.currGroundTruth_.insert(pose2, IO.groundTruth_.atPose2(pose2));
                    IO.poses_ = [IO.poses_, pose2];
                end
                
                
            elseif (isa(currFactor, 'gtsam.BetweenFactorPose2'))
                pose2 = currFactor.keys.at(1);
                if ~IO.estimatedInitial_.exists(pose2) && ~IO.newInitial_.exists(pose2)
                    initializedPose = poseEstimate.compose(currFactor.measured); 
                    IO.newInitial_.insert(pose2,initializedPose);
                    IO.currGroundTruth_.insert(pose2, IO.groundTruth_.atPose2(pose2));
                    IO.poses_ = [IO.poses_, pose2];
                end
                
            end
            
            IO.count_ = IO.count_ + 1;
            
        end
        
        
        function IO = optimize(IO)
            
            import gtsam.*;
            
            switch IO.optimizerType_
                case 'ISAM2'
                    %disp('Doing ISAM optimization');
                    IO.optimizer_.update(IO.newFactor_, IO.newInitial_);
                    IO.estimatedGraph_.push_back(IO.newFactor_);
                    IO.estimatedInitial_.insert(IO.newInitial_);
                    IO.estimatedResult_ = IO.optimizer_.calculateEstimate();
                case 'SAM'
                    IO.estimatedGraph_.push_back(IO.newFactor_);
                    IO.estimatedInitial_.insert(IO.newInitial_);
                    IO.optimizer_ = gtsam.LevenbergMarquardtOptimizer(IO.estimatedGraph_, IO.estimatedInitial_);
                    IO.estimatedResult_ = IO.optimizer_.optimize;  % optimize
            end
            
             IO.newFactor_ = gtsam.NonlinearFactorGraph;
             IO.newInitial_ = gtsam.Values;  
            
        end
  
        
        function res = result(IO)
            res = gtsam.Values(IO.estimatedResult_);
        end
        
        
        function marginal = marginals(IO)
               marginal= gtsam.Marginals(IO.estimatedGraph_, IO.estimatedResult_);
        end
        
        function estimatedGraph = graph(IO)
            estimatedGraph = IO.estimatedGraph_;
        end
        
        function estimatedInitial = initial(IO)
            estimatedInitial = IO.estimatedInitial_;
        end
        
        function currentGroundTruth = currentGroundTruth(IO)
            currentGroundTruth = IO.currGroundTruth_;
        end
        
        function ATE = computeATE(IO)
            trans_error = [];
            for i = 1:numel(IO.poses_)
                poseSymbol = IO.poses_(i);
                gt_pose = IO.currGroundTruth_.atPose2(poseSymbol);
                est_pose = IO.estimatedResult_.atPose2(poseSymbol);
                trans_val = (gt_pose.between(est_pose).translation.norm)^2;
                trans_error = [trans_error, trans_val];
            end
            ATE = sqrt(mean(trans_error));
        end
        
        
        function ALE = computeALE(IO)
            trans_error = [];
            for i = 1:numel(IO.landmarks_)
                landmarkSymbol = IO.landmarks_(i);
                gt_landmark = IO.currGroundTruth_.atPoint2(landmarkSymbol);
                est_landmark = IO.estimatedResult_.atPoint2(landmarkSymbol);
                trans_val = (gt_landmark.between(est_landmark).norm)^2;
                trans_error = [trans_error, trans_val];
            end
            ALE = sqrt(mean(trans_error));
        end
        
        
                
        function ARE = computeARE(IO)
            rot_error = [];
            for i = 1:numel(IO.poses_)
                poseSymbol = IO.poses_(i);
                gt_pose = IO.currGroundTruth_.atPose2(poseSymbol);
                est_pose = IO.estimatedResult_.atPose2(poseSymbol);
                rot_val = gt_pose.between(est_pose).rotation.degrees;
                rot_error = [rot_error, rot_val];
            end
            ARE = mean(rot_error);
        end
        
        function marginalCovariance = uncertaintyOfLatestPose(IO)
            marginals = IO.marginals;
            marginalCov = marginals.marginalCovariance(IO.latestPose_);
            marginalCovariance = det(marginalCov);
        end
        
    end
    
end


