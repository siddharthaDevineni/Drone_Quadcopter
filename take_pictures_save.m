clear all;
clc;
close all;
%% Connect to the drone and take off
droneObj = ryze();
% takeoff(droneObj);
%% Connect to the camera and take snapshots
cameraObj = camera(droneObj);
tic
previewObj = preview(cameraObj);
for i = 1:50
    snap = snapshot(cameraObj);
    filename = sprintf('Image%d.png',i);
    imwrite(snap, filename)
    pause(0.3);
end