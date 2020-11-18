clear all
clc
close all
%% Renaming any random file names to image1, image2, image3 and so on....original source: Matlab Answers
files = dir('C:\Users\Mahadeva\Desktop\QuadCopter\Drone Images\7thset');
files = files(3:end);

% Loop through each file 
for id = 1:length(files)
    % Get the file name 
    [~, f,ext] = fileparts(files(id).name);
    rename = sprintf('image%d.png', id) ; 
    movefile(files(id).name, rename); 
end