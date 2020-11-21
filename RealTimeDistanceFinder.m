clear all
clc
close all
%% Connect to the Drone and Camera
droneObj = ryze();
takeoff(droneObj);
tic
cameraObj = camera(droneObj);
%% Take snapshot and Undistort the Image
imOrig = snapshot(cameraObj);
magnification = 25;
% figure; imshow(imOrig, 'InitialMagnification', magnification);
% title('Input Image');
%% Load the calibrated camera parameters
cameraParams = load('cameraParams6.mat');
cameraParams = cameraParams.cameraParams6;
[im, newOrigin] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
%% Segment Marker
% Convert the image to the HSV color space.
imHSV = rgb2hsv(im);

% Get the saturation channel.
saturation = imHSV(:, :, 2);

% Threshold the image
t = graythresh(saturation);
imMarker = (saturation > t);
% figure; imshow(imMarker, 'InitialMagnification', magnification);
% title('Segmented Marker');
%% Detect Marker
% Find connected components.
blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,...
    'CentroidOutputPort', false,...
    'BoundingBoxOutputPort', true,...
    'MinimumBlobArea', 2000, 'ExcludeBorderBlobs', true);
[areas, boxes] = step(blobAnalysis, imMarker);

% Sort connected components in descending order by area
[~, idx] = sort(areas, 'Descend');

% Get the largest component.
boxes = double(boxes(idx(1), :));

% Reduce the size of the image for display.
scale = magnification / 100;
imDetectedMarker = imresize(im, scale);

% Insert label for the marker.
imDetectedMarker = insertObjectAnnotation(imDetectedMarker, 'rectangle', ...
    scale * boxes, 'marker');
figure; imshow(imDetectedMarker);
title('Detected MArker');

%% Compute Extrinsics
% Detect the checkerboard.
boardSize = [7,10];
[imagePoints, boardSize] = detectCheckerboardPoints(im);

% Adjust the imagePoints so that they are expressed in the coordinate system
% used in the original image, before it was undistorted.  This adjustment
% makes it compatible with the cameraParameters object computed for the original image.
imagePoints = imagePoints + newOrigin; % adds newOrigin to every row of imagePoints
%% Calibarted World Points
worldPoints = [0,0;0,24;0,48;0,72;0,96;0,120;24,0;24,24;24,48;24,72;24,96;24,120;48,0;48,24;48,48;48,72;48,96;48,120;72,0;72,24;72,48;72,72;72,96;72,120;96,0;96,24;96,48;96,72;96,96;96,120;120,0;120,24;120,48;120,72;120,96;120,120;144,0;144,24;144,48;144,72;144,96;144,120;168,0;168,24;168,48;168,72;168,96;168,120;192,0;192,24;192,48;192,72;192,96;192,120];
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);
%% Measure the Marker
% Adjust upper left corners of bounding boxes for coordinate system shift 
% caused by undistortImage with output view of 'full'. This would not be
% needed if the output was 'same'. The adjustment makes the points compatible
% with the cameraParameters of the original image.
boxes = boxes + [newOrigin, 0, 0]; % zero padding is added for widht and height

% Get the top-left and the top-right corners.
box1 = double(boxes(1, :));
% imagePoints1 = [box1(1:2); box1(1) + box1(3), box1(2)];

% Compute the dimension of the marker in millimeters.
% d = worldPoints1(2, :) - worldPoints1(1, :);
% diameterInMillimeters = hypot(d(1), d(2));
% fprintf('Measured dimension of Square Marker = %0.2f mm\n', diameterInMillimeters);
%% Measure the Distance to The Marker
% Compute the center of the marker in the image.
center1_image = box1(1:2) + box1(3:4)/2;

% Convert to world coordinates.
center1_world  = pointsToWorld(cameraParams, R, t, center1_image);

% Remember to add the 0 z-coordinate.
center1_world = [center1_world 0];

% Compute the distance to the camera.
[~, cameraLocation] = extrinsicsToCameraPose(R, t);
distanceToCamera = norm(center1_world - cameraLocation);
fprintf('Distance from the camera to the Marker = %0.2f mm\n',distanceToCamera);
toc
control_function(droneObj,distanceToCamera);
% land(droneObj);
%% References
% Matlab
% Z. Zhang. A flexible new technique for camera calibration. IEEE Transactions on Pattern Analysis and Machine Intelligence, 22(11):1330-1334, 2000.