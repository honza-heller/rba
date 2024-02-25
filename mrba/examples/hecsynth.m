function cals = hecsynth(expB, expJ)

cals = cell(0,0);

%--------------------------------------------------------------------------

fprintf('Calibration using B matrices, linear solution\n\n');

exp = expB;
exp.ba_hec = 0;
exp.ba_wbc = 0;
exp.ba_scale = 0;

cals{end+1} = mrba(exp);

dispcal(cals{end}, 2, 4, 1, 'B matrices, linear solution', 'Robot residual error in pixels');
fprintf('\n\n');

%--------------------------------------------------------------------------

fprintf('Calibration using B matrices and BA solution\n\n');

exp = expB;
exp.ba_hec = 1;
exp.ba_wbc = 1;
exp.ba_scale = 1;

cals{end+1} = mrba(exp);

dispcal(cals{end}, 2, 4, 2, 'B matrices, BA solution, image space error', 'Robot residual error in pixels');
fprintf('\n\n');

%--------------------------------------------------------------------------

fprintf('Calibration using B matrices and BA solution, object space\n\n');

exp = expB;
exp.ba_hec = 1;
exp.ba_wbc = 1;
exp.ba_scale = 1;
exp.calib_ospace = 1;

cals{end+1} = mrba(exp);

dispcal(cals{end}, 2, 4, 3, 'B matrices, BA solution, object space error', 'Robot residual error in robot units', 1);
fprintf('\n\n');

%--------------------------------------------------------------------------

fprintf('Calibration using joint parameters and linear solution\n\n');

exp = expJ;
exp.ba_hec = 0;
exp.ba_wbc = 0;
exp.ba_scale = 0;

cals{end+1} = mrba(exp);

dispcal(cals{end}, 2, 4, 5, 'Joint parameters, linear solution', 'Robot residual error in pixels');
fprintf('\n\n');

%--------------------------------------------------------------------------

fprintf('Calibration using joint parameters and BA solution, image space error\n\n');

exp = expJ;
exp.ba_hec = 1;
exp.ba_wbc = 1;
exp.ba_scale = 1;

cals{end+1} = mrba(exp);

dispcal(cals{end}, 2, 4, 6, 'Joint parameters, BA solution, image space error', 'Robot residual error in pixels');
fprintf('\n\n');

%--------------------------------------------------------------------------

fprintf('Calibration using joint parameters and BA solution, object space error\n\n');

exp = expJ;
exp.ba_hec = 1;
exp.ba_wbc = 1;
exp.ba_scale = 1;
exp.calib_ospace = 1;

cals{end+1} = mrba(exp);

dispcal(cals{end}, 2, 4, 7, 'Joint parameters, BA solution, object space', 'Robot residual error in robot units', 1);
fprintf('\n\n');

end

function dispcal(cal, w, h, i, ttl, xlbl, ospace)
    if (nargin == 6)
        res = cal.rierrs;
    else
        res = cal.roerrs;
    end
    
    fprintf('Mean robot error: %g\n', mean(res(:)));
    fprintf('Max robot error: %g\n', max(res(:)));

    subfig(w, h, i);
    hist(res(res>0), 100);
    title(sprintf('%s (mean: %0.2f max: %0.2f)', ttl, mean(res(:)), max(res(:))));
    xlabel(xlbl);
end