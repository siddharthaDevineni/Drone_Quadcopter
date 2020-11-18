clear all
clc
close all
%% Prepare Calibration Images
numImages = 23; 
files = cell(1, numImages);
set_number = '7thset';
for i = 1:numImages
    files{i} = fullfile('c:\','Users', 'Mahadeva', 'Desktop', 'QuadCopter', ...
        'Drone Images', set_number, sprintf('image%d.png', i));
end

% Display one of the calibration images
magnification = 25;
I = imread(files{1});
% figure; imshow(I, 'InitialMagnification', magnification);
% title('One of the Calibration Images');

%% Estimate Camera Parameters
% Detect the checkerboard corners in the images.
[imagePoints, boardSize] = detectCheckerboardPoints(files);

% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).
squareSize = 24.5; % in millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera.
imageSize = [size(I, 1), size(I, 2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
                                     'ImageSize', imageSize);

% Evaluate calibration accuracy.
figure; showReprojectionErrors(cameraParams);
title('Reprojection Errors');
%% Read the Image of Objects to Be Measured
imOrig = imread(fullfile('c:\','Users', 'Mahadeva', 'Desktop', 'QuadCopter', ...
        'Drone Images', set_number, 'image1.png'));
figure; imshow(imOrig, 'InitialMagnification', magnification);
title('Input Image');

%% Undistort the Image
% Since the lens introduced little distortion, use 'full' output view to illustrate that
% the image was undistored. If we used the default 'same' option, it would be difficult
% to notice any difference when compared to the original image. Notice the small black borders.
[im, newOrigin] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
figure; imshow(im, 'InitialMagnification', magnification);
title('Undistorted Image');
%% Segment Marker
% Convert the image to the HSV color space.
imHSV = rgb2hsv(im);

% Get the saturation channel.
saturation = imHSV(:, :, 2);

% Threshold the image
t = graythresh(saturation);
imMarker = (saturation > t);

figure; imshow(imMarker, 'InitialMagnification', magnification);
title('Segmented Marker');

%% Detect Marker
% Find connected components.
blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,...
    'CentroidOutputPort', false,...
    'BoundingBoxOutputPort', true,...
    'MinimumBlobArea', 1000, 'ExcludeBorderBlobs', true);
[areas, boxes] = step(blobAnalysis, imMarker);

% Sort connected components in descending order by area
[~, idx] = sort(areas, 'Descend');

% Get the largest component.
boxes = double(boxes(idx(1), :));

% Reduce the size of the image for display.
scale = magnification / 100;
imDetectedCoins = imresize(im, scale);

% Insert label for the marker.
imDetectedCoins = insertObjectAnnotation(imDetectedCoins, 'rectangle', ...
    scale * boxes, 'marker');
figure; imshow(imDetectedCoins);
title('Detected MArker');

%% Compute Extrinsics
% Detect the checkerboard.
[imagePoints, boardSize] = detectCheckerboardPoints(im);

% Adjust the imagePoints so that they are expressed in the coordinate system
% used in the original image, before it was undistorted.  This adjustment
% makes it compatible with the cameraParameters object computed for the original image.
imagePoints = imagePoints + newOrigin; % adds newOrigin to every row of imagePoints

% Compute rotation and translation of the camera.
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);
%% Measure the Marker
% Adjust upper left corners of bounding boxes for coordinate system shift 
% caused by undistortImage with output view of 'full'. This would not be
% needed if the output was 'same'. The adjustment makes the points compatible
% with the cameraParameters of the original image.
boxes = boxes + [newOrigin, 0, 0]; % zero padding is added for widht and height

% Get the top-left and the top-right corners.
box1 = double(boxes(1, :));
imagePoints1 = [box1(1:2); ...
                box1(1) + box1(3), box1(2)];

% Get the world coordinates of the corners            
worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);

% Compute the dimension of the marker in millimeters.
d = worldPoints1(2, :) - worldPoints1(1, :);
diameterInMillimeters = hypot(d(1), d(2));
fprintf('Measured dimension of Square Marker = %0.2f mm\n', diameterInMillimeters);
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
fprintf('Distance from the camera to the Marker = %0.2f mm\n', ...
    distanceToCamera);

%% References
% Matlab
% Z. Zhang. A flexible new technique for camera calibration. IEEE Transactions on Pattern Analysis and Machine Intelligence, 22(11):1330-1334, 2000.