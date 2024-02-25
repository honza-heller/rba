function [cals, expsB, expsJ] = heccalib_exp1(nlevel)

exp_path = [fileparts(mfilename('fullpath')) filesep ...
         'data' filesep 'synth' filesep];
     
if (nargin == 0)
    nlevel = 4;
end

load([exp_path 'm1400_irnoise.mat']);
expB = expsB{nlevel}; %#ok<USENS>
expJ = expsJ{nlevel}; %#ok<USENS>

fprintf('Synthetic hand-eye calibration experiment with both image and joint noise synthetic data\n\n');

cals = hecsynth(expB, expJ);

