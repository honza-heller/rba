function res = mrba(exp)
% This is for RBA v0.4.0
mrba_version = '0.4.0';

% rba executable ----------------------------------------------------------

if (isunix)
    rba_paths = strsplit(getenv('PATH'), ':');
elseif (ispc)
    rba_paths = strsplit(getenv('PATH'), ':');
else
    rba_paths = {};
end
    
rba_paths = {
    rba_paths{:}, ...
    fileparts(mfilename('fullpath')), ...
    [fileparts(mfilename('fullpath')) filesep '..' filesep 'bin'], ...
    [fileparts(mfilename('fullpath')) filesep '..' filesep 'build' filesep 'src'], ...
    '/usr/bin', ...
    '/usr/local/bin'};

rba_command = '';

if (nargin == 1)
    if (isfield(exp, 'rba_path'))
        rba_command = exp.rba_path;
    end
end

if (isempty(rba_command))
    for i = 1:numel(rba_paths)
      if (exist([rba_paths{i} filesep 'rba'], 'file'))
          rba_path = rba_paths{i};
          rba_command = [rba_paths{i} filesep 'rba'];
          break;
      end
      if (exist([rba_paths{i} filesep 'rba.exe'], 'file'))
          rba_path = rba_paths{i};
          rba_command = [rba_paths{i} filesep 'rba.exe'];
          break;
      end
    end
end

if (~exist(rba_command, 'file'))
  error('Cannot find rba executable. Please try to set ''rba_path'' field');
end

if (nargin == 0)    
    [dummy,rba_version]=system([rba_command ' --version']);
    if (findstr(mrba_version, rba_version))
        disp(['rba executable found: ' rba_command]);
        disp(['version: ' rba_version]);
    end
    return;
end

% Use system libraries instead of the Matlab defaults 
command = 'ldconfig -v 2>/dev/null | sed "/^[^/].*/d" | sed "s/\\([^:]*\\):.*/\\1/" | xargs | sed "s/ /:/g"';
[retval, lib_paths] = system(command);
lib_paths = lib_paths(1:end-1);
rba_command =['LD_LIBRARY_PATH=' rba_path ':' lib_paths ' ' rba_command];

% Init --------------------------------------------------------------------
    
rba_exp = '';
tmp = '';

% Determine camera calibration type and set default parameters ------------

calibdev_type = -1;

if (isfield(exp,'target_type'))
    if (((exp.target_type > 0) && (exp.target_type < 4)) || (exp.target_type == 6))
        if (~isfield(exp,'target_height'))
            error('The field "target_height" must be set for target_type 1--3,6');
        end
        if (~isfield(exp,'target_height'))
            error('The field "target_width" must be set for target_type 1--3,6');
        end
    end
    
    if (~isfield(exp,'image_paths'))
      error('The field "image_paths" must be set for target detection');
    end
   
    no_poses = numel(exp.image_paths);
    no_points = 0;
    
    if (isfield(exp, 'K') && isfield(exp, 'dist'))
        calibdev_type = 4;
    else        
        calibdev_type = 3;
    end
    
    % Input params
    exp = default_param(exp, 'target_xstride', 1);
    exp = default_param(exp, 'target_ystride', 1);
    exp = default_param(exp, 'target_width', 1);
    exp = default_param(exp, 'target_height', 1);
    exp = default_param(exp, 'target_mindets', 30);    
    exp = default_param(exp, 'ba_cposes', 1);
    exp = default_param(exp, 'ba_scene', 1);
    exp = default_param(exp, 'cmodel_mask', [1,1,0,0,0,0,1,0]);
    exp = default_param(exp, 'xvalid_cameras', []);
    exp = default_param(exp, 'report_dir', '');
    
    % ElMark detector specific parameters
    if (exp.target_type == 4 || exp.target_type == 5 || exp.target_type == 6)
        exp = default_param(exp, 'target_maxe', -1);
        exp = default_param(exp, 'target_mina', -1);
        exp = default_param(exp, 'target_maxa', -1);
        exp = default_param(exp, 'target_delta', 30);
        exp = default_param(exp, 'target_maxd', -1);
        exp = default_param(exp, 'target_noup', 0);        
        exp = default_param(exp, 'target_detdir', '');
    end
    
    % Results
    exp = default_param(exp, 'cmodel_res', 1);
    exp = default_param(exp, 'cposes_res', 1);
    exp = default_param(exp, 'target_res', 1);
    exp = default_param(exp, 'crerrs_res', 1);
    exp = default_param(exp, 'tidets_res', 1);
    exp = default_param(exp, 'log', 1); 
end

if (isfield(exp, 'target') && ...
    isfield(exp, 'u') && ...
    (isfield(exp, 'image_paths') || ...
    (isfield(exp, 'image_width') && isfield(exp, 'image_height'))))

    if (~isfield(exp, 'K') && ~isfield(exp, 'dist'))
        calibdev_type = 2;
    elseif (isfield(exp, 'K') && isfield(exp, 'dist'))
        if (~isequal(size(exp.K), [3, 3]))
            error('The field "K" must be a 3x3 matrix');
        end
        if (~(isequal(size(exp.dist), [1, 8]) || ...
              isequal(size(exp.dist), [8, 1])))
            error('The field "dist" must be a 1x8 vector');
        end
        calibdev_type = 1;
    end
        
    ts = size(exp.target);
    if ((numel(ts) ~= 2) || (ts(1) ~= 3))
        error('The dimensions of the field "target" must be [3, no. points]');
    end
    
    us = size(exp.u);
    if ((numel(us) ~= 3) || (us(2) ~= 3))
        error('The dimensions of the field "u" must be [no. poses, 3, no. points]');
    end
        
    if (isfield(exp, 'image_paths'))
        no_poses = [us(1), numel(exp.image_paths)];
        no_poses = unique(no_poses);
        if (numel(no_poses) ~= 1)
            error('Incompatible number of poses in the fields "u" and "image_paths"');
        end
    else
        no_poses = us(1);
    end
   
    no_points = [ts(2), us(3)];
    no_points = unique(no_points);
    if (numel(no_points) ~= 1)
        error('Incompatible numer of points in the fields "target" and "u"');
    end;
    
    % Input params
    if (calibdev_type == 2)
        exp = default_param(exp, 'ba_cposes', 1);
        exp = default_param(exp, 'ba_scene', 1);
    elseif (calibdev_type == 1)
        exp = default_param(exp, 'ba_scene', 0);
    end
    
    exp = default_param(exp, 'cmodel_mask', [1,1,0,0,0,0,1,0]);
    exp = default_param(exp, 'xvalid_cameras', []);
    exp = default_param(exp, 'report_dir', '');

    % Results
    exp = default_param(exp, 'cmodel_res', 1);
    exp = default_param(exp, 'cposes_res', 1);
    exp = default_param(exp, 'target_res', 1);
    exp = default_param(exp, 'crerrs_res', 1);
    exp = default_param(exp, 'tidets_res', 1);
    exp = default_param(exp, 'log', 1);
end

if (calibdev_type == -1)
    error('Cannot determine camera calibration type from parameters');
end

% We don't want to touch user supplied camera poses (unless told otherwise)
if (isfield(exp, 'A_0'))
    exp = default_param(exp, 'ba_cposes', 0);
end

% Determine robot calibration type and set default parameters ------------

rcalib_type = 0;
rcalib_has_mask = 0;
if (isfield(exp, 'B'))
    rcalib_type = 1;
    
    if (numel(exp.B) ~= no_poses)
        error('The number of robot poses "B" does not match the number of camera poses');
    end
    
    for i = 1:no_poses
      if (~isequal(size(exp.B{i}),[4,4]))
         error('Fields of "B" must be 4x4 matrices');
      end
    end
elseif (isfield(exp, 'J') && isfield(exp, 'mdh_table'))
    rcalib_type = 2;

    if (numel(exp.J) ~= no_poses)
        error('The number of robot poses "J" does not match the number of camera poses');
    end
    
    tsize = size(exp.mdh_table);
    no_joints = tsize(1);
    for i = 1:no_poses
        if (no_joints ~= numel(exp.J{i})) 
            error('The number of parameters of robot poses "J" does not match the mdh_table');
        end
    end
    
    if (isfield(exp, 'mdh_mask'))
        tsize = size(exp.mdh_table);
        msize = size(exp.mdh_mask);
        if ((tsize(1) ~= msize(1)) || (tsize(2) ~= (msize(2) + 3)))
            error('If "mdh_table" size is m x n then "mdh_mask" size must be m x (n-3)');
        end
        rcalib_has_mask = 1;
    end
end

if ((rcalib_type == 0) && isfield(exp, 'J')) 
    error('Field "J" present, roboto description missing');
end
    
if (rcalib_type > 0)
    exp = default_param(exp, 'calib_res', 1);
    exp = default_param(exp, 'ba_hec', 1);
    exp = default_param(exp, 'ba_wbc', 1);
    exp = default_param(exp, 'ba_scale', 1);
    
    exp = default_param(exp, 'calib_ospace', 0);
    exp = default_param(exp, 'calib_relpose', 0);
    exp = default_param(exp, 'rierrs_res', 1);
    exp = default_param(exp, 'roerrs_res', 1);

    if (rcalib_has_mask)
        exp = default_param(exp, 'rdesc_res', 1);
    end
    
    if (isfield(exp, 'mdh_mask_is_penalty'))
        if (exp.mdh_mask_is_penalty)
          exp = default_param(exp, 'rdmask_is_penalty', 1);
        else
          exp = default_param(exp, 'rdmask_is_penalty', 0);
        end            
    end
        
    if (isfield(exp, 'calib_relpose'))
        if (exp.calib_relpose == 1)
            exp = default_param(exp, 'rierrs_res', 0);
            exp = default_param(exp, 'roerrs_res', 0);
        end
    end
end

% Set command line parameters ---------------------------------------------

exp = default_param(exp, 'verbose', 1);
rba_exp = set_bool_param(exp, rba_exp, 'verbose');

[exp, rba_exp] = set_save_param(exp, rba_exp, 'cmodel_res');
[exp, rba_exp] = set_save_param(exp, rba_exp, 'cposes_res');
[exp, rba_exp] = set_save_param(exp, rba_exp, 'target_res');
[exp, rba_exp] = set_save_param(exp, rba_exp, 'crerrs_res');
[exp, rba_exp] = set_save_param(exp, rba_exp, 'rierrs_res');
[exp, rba_exp] = set_save_param(exp, rba_exp, 'roerrs_res');
[exp, rba_exp] = set_save_param(exp, rba_exp, 'tidets_res');
[exp, rba_exp] = set_save_param(exp, rba_exp, 'calib_res');
[exp, rba_exp] = set_save_param(exp, rba_exp, 'rdesc_res');

rba_exp = set_bool_param(exp, rba_exp, 'ba_cposes');
rba_exp = set_bool_param(exp, rba_exp, 'ba_scene');
rba_exp = set_bool_param(exp, rba_exp, 'ba_hec');
rba_exp = set_bool_param(exp, rba_exp, 'ba_wbc');
rba_exp = set_bool_param(exp, rba_exp, 'ba_scale');
rba_exp = set_bool_param(exp, rba_exp, 'calib_ospace');
rba_exp = set_bool_param(exp, rba_exp, 'calib_relpose');
rba_exp = set_bool_param(exp, rba_exp, 'rdmask_is_penalty');

rba_exp = set_vint_param(exp, rba_exp, 'xvalid_cameras', 1);
rba_exp = set_vint_param(exp, rba_exp, 'cmodel_mask', 0);
rba_exp = set_bool_param(exp, rba_exp, 'ocvcalib_linear');
rba_exp = set_bool_param(exp, rba_exp, 'ocvcalib');
rba_exp = set_int_param(exp, rba_exp, 'cmodel_type');
rba_exp = set_int_param(exp, rba_exp, 'target_mindets');

rba_exp = set_string_param(exp, rba_exp, 'report_dir');
rba_exp = set_int_param(exp, rba_exp, 'crecoef');
rba_exp = set_int_param(exp, rba_exp, 'rrecoef');
rba_exp = set_int_param(exp, rba_exp, 'no_threads');

if (isfield(exp, 'target_type') && (exp.target_type == 4 || exp.target_type == 5 || exp.target_type == 6))
    rba_exp = set_int_param(exp, rba_exp, 'target_mina');
    rba_exp = set_int_param(exp, rba_exp, 'target_maxa');
    rba_exp = set_int_param(exp, rba_exp, 'target_maxe');
    rba_exp = set_int_param(exp, rba_exp, 'target_maxd');
    rba_exp = set_int_param(exp, rba_exp, 'target_delta');
    rba_exp = set_int_param(exp, rba_exp, 'target_noup');
    rba_exp = set_string_param(exp, rba_exp, 'target_detdir');
end

% Camera data -------------------------------------------------------------

exp.calibdev_txt = [tempname '_calibdev_txt'];
rba_exp = set_string_param(exp, rba_exp, 'calibdev_txt');

fid = fopen(exp.calibdev_txt, 'w');

if (calibdev_type == 4)
    fprintf(fid, '%d 0 4\n', no_poses);
    fprintf(fid, '%d %d %d %d %d\n', exp.target_type, ...
      exp.target_width, exp.target_height, exp.target_xstride, exp.target_ystride);
    K = exp.K;
    fprintf(fid, '%.20e %.20e %.20e\n%.20e %.20e %.20e\n%.20e %.20e %.20e\n', ...
      K(1,1), K(1,2), K(1,3), K(2,1), K(2,2), K(2,3), K(3,1), K(3,2), K(3,3));
    d = exp.dist;
    fprintf(fid, '%.20e %.20e %.20e %.20e %.20e %.20e %.20e %.20e\n', ...
      d(1), d(2), d(3), d(4), d(5), d(6), d(7), d(8));  
elseif (calibdev_type == 3)
    fprintf(fid, '%d 0 3\n', no_poses);
    fprintf(fid, '%d %d %d %d %d\n', exp.target_type, ...
      exp.target_width, exp.target_height, exp.target_xstride, exp.target_ystride);
elseif (calibdev_type == 2)
    fprintf(fid, '%d %d 2\n', no_poses, no_points);
elseif (calibdev_type == 1)
    fprintf(fid, '%d %d 1\n', no_poses, no_points);
    K = exp.K;
    fprintf(fid, '%.20e %.20e %.20e\n%.20e %.20e %.20e\n%.20e %.20e %.20e\n', ...
      K(1,1), K(1,2), K(1,3), K(2,1), K(2,2), K(2,3), K(3,1), K(3,2), K(3,3));
    d = exp.dist;
    fprintf(fid, '%.20e %.20e %.20e %.20e %.20e %.20e %.20e %.20e\n', ...
      d(1), d(2), d(3), d(4), d(5), d(6), d(7), d(8));
end    

if (isfield(exp, 'image_paths'))
    for i = 1:no_poses
        fprintf(fid, '%s\n', exp.image_paths{i});
    end
else
    fprintf(fid, '@ %d %d\n', exp.image_width, exp.image_height);
end

for j = 1:no_points
    ppos = squeeze(exp.u(:, 3, j));
    pos = find(ppos > 0);

    if (~isempty(pos))
        p = exp.target(:, j);            
        fprintf(fid, '%.20e %.20e %.20e %d ', p(1), p(2), p(3), numel(pos));

        for i = pos'
            p = exp.u(i, 1:2, j);
            idx = i - 1;
            fprintf(fid, '%d %.20e %.20e ', idx, p(1), p(2));
        end

        fprintf(fid, '\n');
    end
end

fclose(fid);

if (isfield(exp, 'A_0'))
    if (numel(exp.A_0) ~= no_poses)
        error('The number of initial camera poses "A_0" does not match the number of camera poses');
    end
    
    for i = 1:no_poses
      if (~isequal(size(exp.A_0{i}),[4,4]))
         error('Fields of "A_0" must be 4x4 matrices');
      end
    end
    
    exp.cposes_txt = [tempname '_cposes_txt'];
    rba_exp = set_string_param(exp, rba_exp, 'cposes_txt');
    fid = fopen(exp.cposes_txt, 'w');   
    fprintf(fid, '%d\n', no_poses);
    
    for i = 1:no_poses
        fprint_trans(fid, exp.A_0{i});
    end
    
    fclose(fid);
end    

% Robot Data --------------------------------------------------------------

if (rcalib_type == 1)
    exp.rposes_txt = [tempname '_rposes_txt'];
    rba_exp = set_string_param(exp, rba_exp, 'rposes_txt');
    fid = fopen(exp.rposes_txt, 'w');
     
    fprintf(fid, '%d\n1\n', no_poses);    
    for i = 1:no_poses
        fprint_trans(fid, exp.B{i});
    end
    
    fclose(fid);
elseif (rcalib_type == 2)
    exp.rposes_txt = [tempname '_rposes_txt'];
    rba_exp = set_string_param(exp, rba_exp, 'rposes_txt');
    fid = fopen(exp.rposes_txt, 'w');
    fprintf(fid, '%d\n2\n', no_poses);
    for i = 1:no_poses
        fprint_vector(fid, exp.J{i});
    end    
    fclose(fid);
    
    exp.rdesc_txt = [tempname '_rdesc_txt'];
    rba_exp = set_string_param(exp, rba_exp, 'rdesc_txt');
    fid = fopen(exp.rdesc_txt, 'w');
    
    fprintf(fid, '1\n%d\n', no_joints);
    for i = 1:no_joints
        fprint_vector(fid, exp.mdh_table(i,:));
    end
    
    fclose(fid);
    
    if (isfield(exp, 'mdh_mask'))
        exp.rdmask_txt = [tempname '_rdmask_txt'];
        rba_exp = set_string_param(exp, rba_exp, 'rdmask_txt');
        fid = fopen(exp.rdmask_txt, 'w');

        fprintf(fid, '1\n%d\n', no_joints);
        for i = 1:no_joints
            fprint_vector(fid, exp.mdh_mask(i,:));
        end
        
        fclose(fid);
    end        
end

% Initial calibration -----------------------------------------------------

cinit = 0;
if (isfield(exp, 'X_0'))
    if (~isequal(size(exp.X_0),[4,4]))
         error('Field "X_0" must be a 4x4 matrix');
    end
    cinit = cinit + 1;
end

if (isfield(exp, 'Z_0'))
    if (~isequal(size(exp.Z_0),[4,4]))
         error('Field "Z_0" must be a 4x4 matrix');
    end
    cinit = cinit + 1;
end

if (isfield(exp, 'scale_0'))
    if (~isequal(size(exp.scale_0),[1,1]))
         error('Field "scale_0" must be a scalar');
    end
    cinit = cinit + 1;
end

if (cinit > 0)
    exp.cinit_txt = [tempname '_cinit_txt'];
    rba_exp = set_string_param(exp, rba_exp, 'cinit_txt');
    fid = fopen(exp.cinit_txt, 'w');
    fprintf(fid, '%d\n', cinit);
    
    if (isfield(exp, 'X_0'))
        fprintf(fid, '1\n');
        fprint_trans(fid, exp.X_0);
    end
    
    if (isfield(exp, 'Z_0'))
        fprintf(fid, '2\n');
        fprint_trans(fid, exp.Z_0);
    end        
    
    if (isfield(exp, 'scale_0'))
        fprintf(fid, '3\n%.20e', exp.scale_0);
    end        
    
    fclose(fid);
end

% Run RBA -----------------------------------------------------------------

[status, log] = system([rba_command rba_exp]);

res = struct;
if (isfield(exp, 'log'))
    if (exp.log == 1)
        res.log = log;
        res.cmd = [rba_command rba_exp];
    end
end

if (status ~= 0)
    cleanup(exp);
    disp(log);
    error('RBA ERR');
end

% Parse RBA results -------------------------------------------------------

res = parse_cposes(exp, res);
res = parse_cmodel(exp, res);
res = parse_target(exp, res);
res = parse_tidets(exp, res);
res = parse_rerrs(exp, res, 'crerrs');
res = parse_rerrs(exp, res, 'rierrs');
res = parse_rerrs(exp, res, 'roerrs');
res = parse_calib(exp, res);
res = parse_rdesc(exp, res);

cleanup(exp);
end

% Functions ---------------------------------------------------------------

function res = parse_cposes(exp, res)
    if (~isfield(exp, 'cposes_res_txt'))
        return;
    end
    if(exist(exp.cposes_res_txt, 'file'))
        cameras = load(exp.cposes_res_txt);
        for i = 1:(size(cameras, 1)/4)
            pos = 4 * (i - 1) + 1; 
            res.A{i} = rt2hom(cameras(pos:(pos+2),:), cameras(pos+3,:));
            if (cameras(pos:(pos+2),:) == zeros(3,3))
                res.A{i} = zeros(4,4);
            end
        end
    end
end

function res = parse_cmodel(exp, res)
    if (~isfield(exp, 'cmodel_res_txt'))
        return;
    end
    if (exist(exp.cmodel_res_txt, 'file'))
        cmodel = load(exp.cmodel_res_txt);
        res.K = cmodel(1:3,1:3);
        res.dist = [cmodel(4,:), cmodel(5,:), cmodel(6,1:2)];
    end
end

function res = parse_target(exp, res)
    if (~isfield(exp, 'target_res_txt'))
        return;
    end
    if (exist(exp.target_res_txt, 'file'))
        res.target = load(exp.target_res_txt)';
    end
end

function res = parse_tidets(exp, res)
    if (~isfield(exp, 'tidets_res_txt'))
        return;
    end
    if (exist(exp.tidets_res_txt, 'file'))
        detections = load(exp.tidets_res_txt);

        no_cams = detections(1,1);
        no_points = detections(1,2);
        res.u = zeros(no_cams, 2, no_points);
        uidx = zeros(no_cams, no_points);

        cidx = detections(2:end, 1) + 1;
        pidx = detections(2:end, 2) + 1;    
        inds_x = sub2ind(size(res.u), cidx, ones(size(pidx)), pidx);
        inds_y = sub2ind(size(res.u), cidx, 2 * ones(size(pidx)), pidx);
        inds_i = sub2ind(size(uidx), cidx, pidx);

        res.u(inds_x) = detections(2:end, 3);
        res.u(inds_y) = detections(2:end, 4);
        uidx(inds_i) = 1;
        res.u(:,3,:) = uidx;
    end
end

function res = parse_rerrs(exp, res, rtype)
    if (~isfield(exp, [rtype '_res_txt']))
        return;
    end
    rfile = getfield(exp, [rtype '_res_txt']); %#ok<GFLD>
    if(exist(rfile, 'file'))
        residuals = load(rfile);

        if (isfield(exp, 'target'))
            no_points = size(exp.target, 2);
        else
            no_points = residuals(1, 2);
        end

        idx = (1:no_points)';
        if (isfield(exp, 'u_index'))
            idx = idx(find(sum(exp.u_index)>0));
        end    

        rsize = [residuals(1,1), no_points];
        rerrs = -1 * ones(rsize);
        cidx = residuals(2:end, 1) + 1;
        pidx = residuals(2:end, 2) + 1;
        inds = sub2ind(rsize, cidx, idx(pidx));
        rerrs(inds) = residuals(2:end,3);
        res = setfield(res, rtype, rerrs); %#ok<SFLD>
    end
end

function res = parse_calib(exp, res)
    if (~isfield(exp, 'calib_res_txt'))
        return;
    end
    if (exist(exp.calib_res_txt, 'file'))
        calib = load(exp.calib_res_txt);
        res.X = rt2hom(calib(1:3,1:3), calib(4,1:3));
        res.Z = rt2hom(calib(5:7,1:3), calib(8,1:3));
        res.scale = calib(9,1);
    end
end

function res = parse_rdesc(exp, res)
    if (~isfield(exp, 'rdesc_res_txt'))
        return;
    end
    if (exist(exp.rdesc_res_txt, 'file'))
        rdesc = load(exp.rdesc_res_txt);
        if (isfield(exp, 'mdh_table'))
            res.mdh_table = rdesc;
        else
            res.rdesc = rdesc;
            warning('Cannot determine robot description type, saved as "rdesc" field');
        end
    end
end


function cleanup(exp)
    delete_tmp_file(exp, 'calibdev_txt');
    delete_tmp_file(exp, 'rposes_txt');
    delete_tmp_file(exp, 'rdesc_txt');
    delete_tmp_file(exp, 'rdmask_txt');
    delete_tmp_file(exp, 'cposes_txt');
    delete_tmp_file(exp, 'cinit_txt');    

    delete_tmp_file(exp, 'cmodel_res_txt');
    delete_tmp_file(exp, 'cposes_res_txt');
    delete_tmp_file(exp, 'target_res_txt');
    delete_tmp_file(exp, 'crerrs_res_txt');
    delete_tmp_file(exp, 'rrerrs_res_txt');    
    delete_tmp_file(exp, 'rierrs_res_txt');    
    delete_tmp_file(exp, 'tidets_res_txt'); 
    delete_tmp_file(exp, 'calib_res_txt'); 
    delete_tmp_file(exp, 'rdesc_res_txt');     
end

function delete_tmp_file(exp, param)
    if (isfield(exp, param))
        fname = getfield(exp, param);
        if (exist(fname, 'file'))
            delete(fname);
        end
    end
end

function exp = default_param(exp, param_name, param_value)
    if (~isfield(exp, param_name))
        exp = setfield(exp, param_name, param_value);
    end
end

function cmd = set_bool_param(exp, cmd, param)
    if (isfield(exp, param))
        if (getfield(exp, param) == 1);
            cmd = [cmd ' --' param ''];
        else
            cmd = [cmd ' --' param '=0'];
        end
    end
end

function cmd = set_int_param(exp, cmd, param)
    if (isfield(exp, param))
        pval = getfield(exp, param); 
        cmd = [cmd ' --' param '=' sprintf('%d', pval)];
    end
end

function cmd = set_string_param(exp, cmd, param)
    if (isfield(exp, param))
        if (numel(getfield(exp, param)) > 0)
            pval = getfield(exp, param); 
            cmd = [cmd ' --' param '=' pval];
        end
    end
end

function cmd = set_vint_param(exp, cmd, param, subone)
    if (isfield(exp, param))
        pval = getfield(exp, param);
        if (isempty(pval))
            return;
        end

        xc = '"';
        for i = 1:numel(pval)
            val = pval(i);
            if (subone == 1)
                val = val - 1;
            end
            xc = [xc sprintf('%d,', val)];  
        end
        xc(end) = '"';
        cmd = [cmd ' --' param '=' xc];
    end
end

function [exp, cmd] = set_save_param(exp, cmd, param)
    if (isfield(exp, param))
        if (getfield(exp, param) == 1)
            fname = [tempname '_' param];
            exp = setfield(exp, [param '_txt'], fname);
            cmd = [cmd ' --' param '_txt=' fname];
        end
    end
end

function H = rt2hom(R, t)
    if (size(t, 1) == 1)
        t = t';
    end
    H = [R, t; 0, 0, 0 1];
end

function fprint_trans(fid, T)
    fprintf(fid, '%.20e %.20e %.20e\n%.20e %.20e %.20e\n%.20e %.20e %.20e\n', ... 
      T(1,1), T(1,2), T(1,3), T(2,1), T(2,2), T(2,3), T(3,1), T(3,2), T(3,3)); 
    fprintf(fid, '%.20e %.20e %.20e\n', T(1,4), T(2,4), T(3,4));
end

function fprint_vector(fid, v)
    for i = 1:(numel(v)-1)
        fprintf(fid, '%.20e ', v(i));
    end
    fprintf(fid, '%.20e\n', v(end));
end
