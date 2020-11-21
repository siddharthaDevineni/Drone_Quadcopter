function [] = control_function(droneObj,distanceToCamera)

if(distanceToCamera>=70)
    distanceToCamera = distanceToCamera/1000;
%     takeoff(droneObj);
    moveforward(droneObj,'Distance',distanceToCamera,'WaitUntilDone',false);
    land(droneObj);
else
%     move(droneObj,[(distanceToCamera/1000) 0 0]);
   RealTimeDistanceFinder;
%    land(droneObj); 
end
%     
end