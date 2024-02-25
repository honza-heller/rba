function [cals, expsB, expsJ] = heccalib_exp1(nlevel)

exp_path = [fileparts(mfilename('fullpath')) filesep ...
         'data' filesep 'synth' filesep];
     
if (nargin == 0)
    nlevel = 1;
end

load([exp_path 'm1400_inoise.mat']);
expB = expsB{nlevel}; %#ok<USENS>
expJ = expsJ{nlevel}; %#ok<USENS>

fprintf('Synthetic hand-eye calibration experiment with noiseless synthetic data\n\n');

cals = hecsynth(expB, expJ);

