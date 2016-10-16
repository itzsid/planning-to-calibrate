%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% @brief Initialize poses
% @author Siddharth Choudhary
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function path=initialize_poses(axes_handle, estimated_graph_handle)
globals;
disp('Adding Poses....');
disp('Press Enter to Finish');
axes(axes_handle);
v=[0 WORLD_SIZE 0 WORLD_SIZE];
axis(v);
hold on;

pin=1;
npoints=0;
points=zeros(2,1);
xi=[];
yi=[];
transformCov = [];
transformX = [];
transformY = [];
transformTheta = [];
translationError = [];
rotationError = [];
% get input points graphically 
% and then set up basis x values
% interpolation can get confused so be careful !
while pin
    axes(axes_handle);

   [x,y]=ginput(1);
   pin= ~isempty(x);
   if pin
      npoints=npoints+1;
      plot(x,y,'rx')
      points(1,npoints)=x;
      points(2,npoints)=y;
      % now find a basis for x
      if npoints > 1
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
       
       	xi=[xi xincs];
        yi=[yi yincs];
       
      end
   else
       break;
   end
   
   path=[xi;yi];
   USER_DATA.path = path;
   
 
   % Optimize
   if(size(path,2)>1)
      detTransform = optimize_poses(estimated_graph_handle);
      transformCov = [transformCov, detTransform.det];
      transformX = [transformX, detTransform.X];
      transformY = [transformY, detTransform.Y];
      transformTheta = [transformTheta, detTransform.Theta];
      
      transformError = detTransform.Transform.between(USER_DATA.actualTransform);
      translationError = [translationError, transformError.translation.norm];
      rotationError = [rotationError, transformError.rotation.theta];
      
      figure(3);
      cla;
      subplot(6,1,1);
      loglog(transformCov);
      xlabel('#Actions');
      ylabel('Determinant');
      
      subplot(6,1,2);
      plot(transformX);
      ylabel('X');
      
      subplot(6,1,3);
      plot(transformY);
      ylabel('Y');
      
      subplot(6,1,4);
      plot(transformTheta);
      ylabel('Theta');
      
      subplot(6,1,5);
      plot(translationError);
      ylabel('TranslationError');
      
      subplot(6,1,6);
      plot(rotationError);
      ylabel('RotationError');
      
      
      
   end
end


% now we have all the basis points, interpolate the
% path in y. A better method would be to do this along
% the arc length; would need to think how to do this !

%yi=interp1(points(1,:),points(2,:),xi,'spline');
%plot(xi,yi,'r')

hold off
