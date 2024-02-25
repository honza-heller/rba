function camc = camcalib(exp, no_show)

% RBA
if (isfield(exp, 'A'))
    % probably just results, do not run calibration
    camc = exp;
else
    camc = mrba(exp);
    
    if (isfield(exp, 'target_noup') && (exp.target_noup == -1))
        camc.cmodel = 'fisheye_p8p';
    end
    
    if (isfield(exp, 'cmodel') && strcmp(exp.cmodel, 'fisheye_p8p'))
        camc.cmodel = 'fisheye_p8p';
    end
    
    if (isfield(exp, 'cmodel_type') && (exp.cmodel_type == 3))
        camc.cmodel = 'fisheye_p8p';
    end    
    
    if (isfield(exp, 'cmodel') && strcmp(exp.cmodel, 'fisheye_e7p'))
        camc.cmodel = 'fisheye_e7p';
    end
    
    if (isfield(exp, 'cmodel_type') && (exp.cmodel_type == 4))
        camc.cmodel = 'fisheye_e7p';
    end        
end

% Resuts ------------------------------------------------------------------

disp('RBA Calibration results');
disp('Calibration matrix K');
disp(camc.K);

disp('camera model distortion parameters');
disp(camc.dist);

res = camc.crerrs;
res0 = res(res>0);
disp('Mean residual error');
disp(mean(res0));
disp('Maximal residual error');
disp(max(res0));

if (isfield(exp, 'image_paths'))
    no_cameras = numel(exp.image_paths);
elseif (isfield(exp, 'u'))
    no_cameras = size(exp.u, 1);
end


if ((nargin == 2) && (no_show ~= 0))
    no_figs = 4;
else
    no_figs = no_cameras + 4;
end

screen = get(0, 'ScreenSize');
rows = 1;
while(1 == 1)
    h = floor(screen(4) / rows);
    cols = floor(screen(3) / h);
    if (rows * cols >= no_figs)
        break;
    end
    rows = rows + 1;
end

fgs = 1;


cflag = 0;
if (isfield(exp, 'cmodel_cubic') && exp.cmodel_cubic ~= 0)
    cflag = 1;
elseif (isfield(camc, 'cmodel'))
    cflag = camc.cmodel;
end

% Histogram of residual errors
subfig(rows, cols, fgs);
fgs = fgs + 1;
hist(res0, 100);
title(sprintf('Histogram of residual errors (mean: %0.2f max: %0.2f)', mean(res0), max(res0)));
xlabel('Residual error in pixels');

% Camera positions vs. calibration target
subfig(rows, cols, fgs);
fgs = fgs + 1;
hold on;
plot3(camc.target(1,:), camc.target(2,:), camc.target(3,:),'.b'); 

csize = 0.1 * max(max(camc.target,[],2) - min(camc.target,[],2));

for i = 1:no_cameras
    if (~isempty(camc.A{i}) && any(camc.A{i}(:)))
        plotframe(camc.A{i}, csize, sprintf('%d', i));
    end
end
title('Camera positions vs. calibration target');
axis equal;

% Calibration target positions vs. camera position
subfig(rows, cols, fgs);
fgs = fgs + 1;
hold on;
plotframe(eye(4), csize, 'camera');
for i = 1:no_cameras
    if (~isempty(camc.A{i}) && any(camc.A{i}(:))) 
        target = camc.A{i} * [camc.target; ones(1, size(camc.target, 2))];
        plot3(target(1,:), target(2,:), target(3,:),'.b');
    end
end
title('Calibration target positions vs. camera position');
axis equal;

% End if no_show
if ((nargin == 2) && (no_show ~= 0))
    return;
end

% End if cannot determine image size
if (~isfield(exp, 'image_paths') && ...
        (~isfield(exp, 'image_width') || ~isfield(exp, 'image_height')))
    warning('Cannot determine image size');
    return;
end

% Field of view coverage
if (isfield(exp, 'image_paths'))
    im = imread(exp.image_paths{1});
elseif (isfield(exp, 'image_width') && isfield(exp, 'image_height'))
    im = uint8(255 * ones(exp.image_height, exp.image_width));
else
    error('Cannot determine image size');
end

im = (255 - ((255 - im) / 2));

f = figure('Visible','off');
w = warning('off', 'all');
imshow(im);
warning(w);
hold on;
subfig(rows, cols, fgs, f);
fgs = fgs + 1;

title('Field of view (FOV) coverage');

% Cameras
for c = 1:no_cameras    
    if (isfield(exp, 'image_paths'))
        im = imread(exp.image_paths{c});
    elseif (isfield(exp, 'image_width') && isfield(exp, 'image_height'))
        im = ones(exp.image_height, exp.image_width);
    else
        error('Cannot determine image size');
    end
        
    idx = squeeze(logical(camc.u(c, 3, :)));
    pts = squeeze(camc.u(c, 1:2, idx));
    res = camc.crerrs(c, idx);
    cal = camc.target(:, idx);

    % Target detections for selected image
    f = figure('Visible','off');
    w = warning('off', 'all');
    imshow(im);
    warning(w);
    subfig(rows, cols, fgs, f);
    fgs = fgs + 1;
    hold on;
            
    if (~isempty(camc.A{c}) && any(camc.A{c}(:)))
        v = X2u(cal, camc.A{c}, camc.K, camc.dist, cflag);
        plot(v(1,:) + 1, v(2,:) + 1, 'gx');        
        plot(pts(1,:) + 1, pts(2,:) + 1, 'rx');        
        
        u = v + (v - pts) * 20;
        for i = 1:size(pts,2)
            %u = v(:, i) + (v - pts) * 5;
           plot([u(1,i), v(1,i)] + 1, [u(2,i), v(2,i)] + 1, '-c');        
            %text(pts(1,i), pts(2,i), sprintf('\\color{red}[%d]', i));
            %text(pts(1,i), pts(2,i), sprintf('\\color{red}[%d, %d, %d, %0.1f]', cal(1,i), cal(2,i), i, res(i)));
        end
        title(sprintf('Target detection in image no. %d (err mean: %0.1f max: %0.1f)', c, mean(res), max(res)));
    else
        title(sprintf('Target detection in image no. %d (pose not recovered)', c)); 
    end
end;


