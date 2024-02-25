function calib = camcalib_exp3

ipath = [fileparts(mfilename('fullpath')) filesep ...
         'data' filesep 'RV6S' filesep];

% Loading target detections
load([ipath 'exp1.mat']);

exp.image_width = 3872;
exp.image_height = 2592;

disp('Example of using RBA for camera calibration using image dimensions');
disp('and user supplied detections of a calibration target');

% RBA
calib = camcalib(exp);