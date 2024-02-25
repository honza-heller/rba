function [camc, exp] = camcalib_exp1

ipath = [fileparts(mfilename('fullpath')) filesep ...
         'data' filesep 'HTC' filesep];

imgs = [dir([ipath '*.jpg'])];

for i = 1:numel(imgs)
    exp.image_paths{i} = [ipath imgs(i).name];
end

% Mandatory
exp.target_width = 9;
exp.target_height = 6;
exp.target_type = 1;

disp('Example of using RBA for camera calibration using images of known calibration target');

camc = camcalib(exp);
