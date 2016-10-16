function [xtrue, utrue, uz] = simulate_controls(path)


globals

[temp,n_path]=size(path);
time=0:DT:n_path;
[temp,tsize]=size(time);

xtrue=zeros(4,tsize);	% needed for reseting between runs
utrue=zeros(3,tsize);	% and again
xtrue(1,1)=path(1,1);	% initial x
xtrue(2,1)=path(2,1);	% initial y
xtrue(3,1)=atan2(path(2,2)-path(2,1),path(1,2)-path(1,1)); % initial phi
xtrue(4)=0;					% time=0;
utrue(1)=VVEL;				% velocity set at 2 m/s
utrue(2)=0;					% steer is zero
utrue(3)=0;					% time=0

index = 1;
for i=1:(tsize-1)
    % find error
    [perr, oerr, dist, index,d]=get_err(xtrue(:,i), path, index);
    
    % compute next state
    utrue(:,i+1)=zeros(3,1);
    xtrue(:,i+1)=zeros(4,1);
    
    utrue(1,i+1)=utrue(1,i);
    utrue(2,i+1)=KO*(oerr); %+K1*perr; 
    utrue(3,i+1)=utrue(3,i)+DT;
       
    xtrue(1,i+1) = xtrue(1,i) + DT*dist*cos(xtrue(3,i)+utrue(2,i+1));
    xtrue(2,i+1) = xtrue(2,i) + DT*dist*sin(xtrue(3,i)+utrue(2,i+1));
    xtrue(3,i+1) = xtrue(3,i) + utrue(2,i+1)/WHEEL_BASE; %Wheel base = 1 by default
    xtrue(4,i+1)=xtrue(4,i)+DT;
    if d >10  % test for end of path
        break;
    end
end

% shorten vectors to end length
xtrue=xtrue(:,1:i);
utrue=utrue(:,1:i);
utrue(1,:)=utrue(1,:)/WHEEL_RADIUS; % make speed into rads/s

% add noise if required
uz(1,:)=utrue(1,:)+GSIGMA_WHEEL*randn(size(utrue(1,:)));
uz(2,:)=utrue(2,:)+GSIGMA_STEER*randn(size(utrue(2,:)));
uz(3,:)=utrue(3,:);

