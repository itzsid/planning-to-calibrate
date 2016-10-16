function observations =  data_associate(xtrue, landmarks, offset)

globals
% state vector, containing x,y,phi,t
[XSIZE,N_STATES]=size(xtrue);
if XSIZE ~= 4
    error('Incorrect state dimension')
end
observations = [];

% Return if no landmarks are present
if landmarks.size == 0
    return;
end

for i=1:N_STATES-1
    start_loc=xtrue(:,i);
    end_loc=xtrue(:,i+1);
    pose = i;
    
    observation = [];
    obs=zeros(5,1);
    
    
    point = gtsam.Point2(start_loc(1:2,:));
    rot = gtsam.Rot2(start_loc(3,:));
    gtsam_pose = gtsam.Pose2(rot, point);

    % The location vectors start_loc and end_loc each consist
    % of a vector of x,y,phi,t, describing the location of the
    % vehicle at a specific time.
    [XSIZE,temp]=size(start_loc);
    if((XSIZE ~= 4)+(temp ~= 1))
        error('Incorrect size for start_loc')
    end
    [XSIZE,temp]=size(end_loc);
    if((XSIZE ~= 4)+(temp ~= 1))
        error('Incorrect size for end_loc')
    end
    
    
    % The beacon map consists of an array of x,y locations
    % for the landmarks
    num_landmarks=landmarks.size;
    keys = gtsam.KeyVector(landmarks.keys);
    %if (temp ~= 2)
    %    error('Incorrect Size for beacon map')
    %end    
       
    % The algorithm now proceeds by finding the two bounding
    % lines from radar to max range. If the beacon lies between these
    % two lines, then it will be observed
    
    % first find location and aim of radar at start 
    radX = gtsam_pose.compose(gtsam.Pose2(R1_OFFSET_X, R1_OFFSET_Y, R1_OFFSET_THETA));
        
    % Transform offset - make it variable instead of constant
    radx=radX.x;
    rady=radX.y;

    radphi1=a_add(start_loc(4)*R_RATE,0.0); % gets normalized angle
        
    % to be observed beacon must lie in R_MAX_RANGE of sensor location
    range=R_MAX_RANGE;
    b=0;
    for i = 0:size(keys) -1
        point= landmarks.atPoint2(keys.at(i)); % Pose 1
        r=sqrt((point.x-radx)^2+(point.y-rady)^2);
        if((r<R_MAX_RANGE)) %removed point lying in between check *(d1>0)*(d2<0)
            range=r;
            b=i;
            obs(1)=sqrt((radx-point.x)^2+(rady-point.y)^2);
            obs(2)=a_sub(atan2(point.y-rady,point.x-radx),start_loc(3)); 
            obs(3)=gtsam.symbolIndex(keys.at(i)); % beacon index
            obs(4)= pose + offset;
            obs(5)=start_loc(4); % time stamp

            % noise model
            obs(1)=obs(1);
            obs(2)=obs(2);
            observation = [observation, obs];
        end
    end
    
    
    observations = [observations, observation];
end

