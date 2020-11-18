files = dir('C:\Users\Mahadeva\Desktop\QuadCopter\Drone Images\6thset');
files = files(3:end);
% Get all text files in the current folder
% files = dir('*.txt');
% Loop through each file 
for id = 1:length(files)
    % Get the file name 
    [~, f,ext] = fileparts(files(id).name);
%     x = str2num(id);
    rename = sprintf('image%d.png', id) ; 
    movefile(files(id).name, rename); 
end