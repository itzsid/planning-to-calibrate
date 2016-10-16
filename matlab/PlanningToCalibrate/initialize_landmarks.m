%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% @brief Initialize landmarks
% @author Siddharth Choudhary
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [landmarks, landmark_values] = initialize_landmarks(axes_handle)

% first step is to input beacons
disp('Input landmark locations');
disp('Press Enter to Finish');

% set up the figure

globals;
axes(axes_handle)
%clf
v=[0 WORLD_SIZE 0 WORLD_SIZE];
axis(v);
hold on;
bin=1;
nlandmarks=0;
landmarks=zeros(1,2);
landmark_values = gtsam.Values;
% now get beacons graphically until return 
while bin
   [x,y]=ginput(1);
   bin= ~isempty(x);
   if bin
      nlandmarks=nlandmarks+1;
      plot(x,y,'go')
      landmarks(nlandmarks,1)=x;
      landmarks(nlandmarks,2)=y;
      
      point = gtsam.Point2(x,y);
      landmark_values.insert(gtsam.symbol('l',nlandmarks), point);
   end
end
hold off

[n_landmarks,temp]=size(landmarks);
fprintf('%d Landmarks read\n',n_landmarks);

end