function calib = camcalib_exp2

ipath = [fileparts(mfilename('fullpath')) filesep ...
         'data' filesep 'RV6S' filesep];

imgs = { ...
'DSC_0021.JPG',...
'DSC_0022.JPG',...
'DSC_0023.JPG',...
'DSC_0024.JPG',...
'DSC_0025.JPG',...
'DSC_0026.JPG',...
'DSC_0027.JPG',...
'DSC_0028.JPG',...
'DSC_0029.JPG',...
'DSC_0030.JPG'};

% Loading target detections
load([ipath 'exp1.mat']);

for i = 1:numel(imgs)
    exp.image_paths{i} = [ipath imgs{i}];
end

disp('Example of using RBA for camera calibration using images and');
disp('user supplied detections of a calibration target');

% RBA
calib = camcalib(exp);