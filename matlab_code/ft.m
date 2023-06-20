function ft(chain_robot,secs,chain_rig,...
    T_roots_robot,q_revs_robot,T_roots_rig,q_revs_rig,mocap_name,varargin)
%
% Fine-tuning of wrist trajectories
%

% Parse options
iP = inputParser;
addParameter(iP,'data_folder_path','../data/ft');
addParameter(iP,'gimbal_threshold',0.1);
addParameter(iP,'joi_p_trgts',{'rs','re','rw','ls','le','lw'});
addParameter(iP,'max_ik_tick',100);
addParameter(iP,'PLOT_IK_INSIDE',1);
addParameter(iP,'PLOT_EACH_TICK',1);
addParameter(iP,'GIMBAL_LOCK_HEURISTICS',1);
addParameter(iP,'VERBOSE',1);
addParameter(iP,'PLOT_TRAJ_AFFINE',0);
addParameter(iP,'SKIP_IF_MAT_EXIST',0);
addParameter(iP,'SAVE_MAT',1);
parse(iP,varargin{:});
data_folder_path       = iP.Results.data_folder_path;
gimbal_threshold       = iP.Results.gimbal_threshold;
joi_p_trgts            = iP.Results.joi_p_trgts;
max_ik_tick            = iP.Results.max_ik_tick;
PLOT_IK_INSIDE         = iP.Results.PLOT_IK_INSIDE;
PLOT_EACH_TICK         = iP.Results.PLOT_EACH_TICK;
GIMBAL_LOCK_HEURISTICS = iP.Results.GIMBAL_LOCK_HEURISTICS;
VERBOSE                = iP.Results.VERBOSE;
PLOT_TRAJ_AFFINE       = iP.Results.PLOT_TRAJ_AFFINE;
SKIP_IF_MAT_EXIST      = iP.Results.SKIP_IF_MAT_EXIST;
SAVE_MAT               = iP.Results.SAVE_MAT;
D2R = pi/180;

% Mat path to save
robot_name = chain_robot.name; % robot name
mat_path = sprintf('%s/%s_%s.mat',data_folder_path,robot_name,mocap_name);
if SKIP_IF_MAT_EXIST && exist(mat_path,'file')
    fprintf(2,'Skip this as [%s] exists.\n',mat_path);
    return
elseif SAVE_MAT
    fprintf('The result will be saved at [%s].\n',mat_path);
end

% Close all
ca;

% Get T-pose
chain_robot        = get_chain_t_pose(chain_robot);
chain_robot        = move_chain_two_feet_on_ground(chain_robot);
chain_robot_t_pose = chain_robot;
root_idx_robot     = get_topmost_idx(chain_robot);

% Get trajectories
L             = length(secs);
rs_robot_traj = zeros(L,3);
ls_robot_traj = zeros(L,3);
re_robot_traj = zeros(L,3);
le_robot_traj = zeros(L,3);
rw_robot_traj = zeros(L,3);
lw_robot_traj = zeros(L,3);
rw_rig_traj   = zeros(L,3);
lw_rig_traj   = zeros(L,3);

for tick = 1:L
    % Update robot and rig poses
    q_robot      = q_revs_robot(tick,:);
    T_root_robot = T_roots_robot{tick};
    chain_robot  = update_chain_q_root_T(chain_robot,q_robot,T_root_robot);
    q_rig        = q_revs_rig(tick,:);
    T_root_rig   = T_roots_rig{tick};
    chain_rig    = update_chain_q_root_T(chain_rig,q_rig,T_root_rig);
    % Get wrist trajectories
    T_joi_robot = get_t_joi(chain_robot,chain_robot.joi);
    T_joi_rig   = get_t_joi(chain_rig,chain_rig.joi);
    rs_robot_traj(tick,:) = rv(t2p(T_joi_robot.rs));
    ls_robot_traj(tick,:) = rv(t2p(T_joi_robot.ls));
    re_robot_traj(tick,:) = rv(t2p(T_joi_robot.re));
    le_robot_traj(tick,:) = rv(t2p(T_joi_robot.le));
    rw_robot_traj(tick,:) = rv(t2p(T_joi_robot.rw));
    lw_robot_traj(tick,:) = rv(t2p(T_joi_robot.lw));
    rw_rig_traj(tick,:)   = rv(t2p(T_joi_rig.rw));
    lw_rig_traj(tick,:)   = rv(t2p(T_joi_rig.lw));
end

% Find the affine transformation that best transforms rig trajs to robot trajs
traj_weights = ones(2*L,1);
transform_cost = @(x)(...
    mean( ...
    traj_weights.*...
    sum( ...
    abs(affine_transform([rw_rig_traj;lw_rig_traj],x(1:3),x(4:6),x(7:9)) - ...
    [rw_robot_traj;lw_robot_traj]), ...
    2))...
    ); % weighted mean absolute error (WMAE)
x0 = [0,0,0,0,0,0,1,1,1]; % initial affine parameters
opt = optimset('MaxFunEvals',10000,'MaxIter',10000,'TolFun',1e-5);
x = fminsearch(transform_cost,x0,opt); % optimize the affine transformation
rw_rig_traj_affine = affine_transform(rw_rig_traj,x(1:3),x(4:6),x(7:9));
lw_rig_traj_affine = affine_transform(lw_rig_traj,x(1:3),x(4:6),x(7:9));
% Temporal interpolation of 'rw_rig_traj_affine' and 'lw_rig_traj_affine'
L1 = round(L/4); L2 = L-2*L1;
alphas = repmat(cv([linspace(0,1,L1),ones(1,L2),linspace(1,0,L1)]),1,3);
rw_rig_traj_affine = (1-alphas).*rw_robot_traj + alphas.*rw_rig_traj_affine;
lw_rig_traj_affine = (1-alphas).*lw_robot_traj + alphas.*lw_rig_traj_affine;

% Plot wrist trajectories of robot and rig
if PLOT_TRAJ_AFFINE
    fig_idx = 13; view_info = [60,25];
    plot_chain(chain_robot,'fig_idx',fig_idx,'subfig_idx',1,'view_info',view_info,...
        'fig_pos',[0.0,0.5,0.3,0.4],'mfa',0.02,'bafa',0.02,...
        'PLOT_LINK',0,'PLOT_ROTATE_AXIS',0);
    plot_traj(rw_robot_traj,'fig_idx',fig_idx,'subfig_idx',1,'tlc','r','tlw',2,'tls','-');
    plot_traj(lw_robot_traj,'fig_idx',fig_idx,'subfig_idx',2,'tlc','b','tlw',2,'tls','-');
    plot_traj(rw_rig_traj,'fig_idx',fig_idx,'subfig_idx',3,'tlc','r','tlw',2,'tls',':');
    plot_traj(lw_rig_traj,'fig_idx',fig_idx,'subfig_idx',4,'tlc','b','tlw',2,'tls',':');
    plot_traj(rw_rig_traj_affine,'fig_idx',fig_idx,'subfig_idx',5,'tlc','r','tlw',2,'tls',':');
    plot_traj(lw_rig_traj_affine,'fig_idx',fig_idx,'subfig_idx',6,'tlc','b','tlw',2,'tls',':');
    drawnow;
end

% Upper-body matching
q_revs_robot_ft  = zeros(L,chain_robot.n_rev_joint);
T_roots_robot_ft = cell(L,1);
tk = init_tk('ft'); % start time keeping
for tick = 1:L % for each tick
    tk = print_tk(tk,tick,L,5); % time-keeping
    if tick == 1, RESET = 1; else, RESET = 0; end

    % Update robot
    sec          = secs(tick);
    q_rev_robot  = q_revs_robot(tick,:);
    T_root_robot = T_roots_robot{tick};
    chain_robot  = update_chain_q_root_T(chain_robot,q_rev_robot,T_root_robot);

    % Robot Gimbal lock handling
    [n_gimbal,gimbal_q1q2s,gimbal_pairs] = check_gimbal_lock(chain_robot,...
        'gimbal_threshold',gimbal_threshold,'VERBOSE',VERBOSE,'RESET',RESET);

    % Append IK targets
    p_trgts = struct(); R_trgts = struct();
    p_trgts.rs = rs_robot_traj(tick,:);
    p_trgts.re = re_robot_traj(tick,:);
    p_trgts.rw = rw_rig_traj_affine(tick,:); % rw_robot_traj(tick,:);
    p_trgts.ls = ls_robot_traj(tick,:);
    p_trgts.le = le_robot_traj(tick,:);
    p_trgts.lw = lw_rig_traj_affine(tick,:); % lw_robot_traj(tick,:);
    
    % Gimbal lock handling heuristics
    joint_names_to_ctrl = get_rev_joint_names_route_to_jois(chain_robot,{'rh','lh'});
    if (n_gimbal > 0) && GIMBAL_LOCK_HEURISTICS
        joint_names_to_remove = handle_gimbal_lock(chain_robot,gimbal_q1q2s,gimbal_pairs,...
            'chain_robot_t_pose',chain_robot_t_pose);
        idxs = idxs_cell(joint_names_to_ctrl,joint_names_to_remove);
        joint_names_to_ctrl(idxs) = []; % remove joints
    end
    ik_info_upper_p = get_ik_info_from_joi_targets(chain_robot,p_trgts,R_trgts,...
        'ik_err_th',10.0,'dq_th',10*D2R,'step_size',1.0,...
        'joi_p_trgts',joi_p_trgts,...
        'joint_names_to_ctrl',joint_names_to_ctrl);

    % Solve IK for upper body
    for ik_tick = 1:max_ik_tick % for each IK tick (phase 2)
        zero_to_one     = ik_tick/max_ik_tick;
        one_to_zero     = 1 - zero_to_one;
        unit_dq_rad_max = 10;
        unit_dq_rad_min = 1;
        unit_dq_rad     = (unit_dq_rad_max*one_to_zero+unit_dq_rad_min)*D2R;

        % Heuristics (GIMBAL_LOCK_HEURISTICS)
        if (zero_to_one < 1.0) && GIMBAL_LOCK_HEURISTICS && (tick >= 2)
            % rev_joints = chain_robot.rev_joint_names;
            rev_joints = get_rev_joint_names_route_to_jois(chain_robot,{'rh','lh'});
            % Find intersection betwwen 'joint_names_to_ctrl' and 'rev_joints'
            rev_joints = intersect(joint_names_to_ctrl,rev_joints);
            heuristic_type = 'hybrid'; % 't-pose' / 'prev-pose' / 'hybrid'
            switch heuristic_type
                case 't-pose'
                    q_tpose = get_q_chain(chain_robot_t_pose,rev_joints);
                    q_reg = q_tpose;
                case 'prev-pose'
                    q_prev = cv(q_revs_robot(tick-1,:));
                    chain_robot_prev = update_chain_q(...
                        chain_robot,chain_robot.rev_joint_names,q_prev);
                    q_reg = get_q_chain(chain_robot_prev,rev_joints);
                case 'hybrid'
                    q_tpose = get_q_chain(chain_robot_t_pose,rev_joints);
                    q_prev = cv(q_revs_robot(tick-1,:));
                    chain_robot_prev = update_chain_q(...
                        chain_robot,chain_robot.rev_joint_names,q_prev);
                    q_prev = get_q_chain(chain_robot_prev,rev_joints);
                    q_reg = 0.5*(q_tpose + q_prev);
                otherwise
                    fprintf(2,'Unknown heuristic_type:[%s]\n',heuristic_type);
            end
            q_curr = get_q_chain(chain_robot,rev_joints); % current joint position
            q_mix  = q_curr + 5.0*D2R*one_to_zero*sign(q_reg-q_curr); % mix
            chain_robot = update_chain_q(chain_robot,rev_joints,q_mix);
        end

        % IK step for positions
        CONSIDER_JOINT_LIMIT = 1;
        [dq,joint_names_to_ctrl,~,ik_err_upper_p,~] = one_step_ik(...
            chain_robot,ik_info_upper_p,'CONSIDER_JOINT_LIMIT',CONSIDER_JOINT_LIMIT,...
            'UNIT_DQ_HEURISTIC',1,'unit_dq_rad',unit_dq_rad,'limit_margin_rad',0);
        chain_robot = update_chain_q(chain_robot,joint_names_to_ctrl,...
            get_q_chain(chain_robot,joint_names_to_ctrl)+dq,...
            'IGNORE_LIMIT',~CONSIDER_JOINT_LIMIT,'margin_rad',0,'FV',0);

        % IK error
        ik_errs = [ik_err_upper_p];
        ik_err  = max(abs(ik_errs))/chain_robot.sz.xyz_len(3); % normalized IK error
        if PLOT_IK_INSIDE && ((ik_tick<=5) || (mod(ik_tick,20)==0))
            fig_idx = 6; fig_pos = [0.2,0.0,0.2,0.45];
            axis_info = ''; view_info = [60,32]; ral = chain_robot.sz.xyz_len(3)/20;
            fig6 = plot_chain(chain_robot,...
                'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
                'axis_info',axis_info,'view_info',view_info,...
                'PLOT_LINK',1,'PLOT_JOINT_AXIS',0,'jalw',3,'PLOT_JOINT_SPHERE',1,...
                'PLOT_MESH',0,'jsr',0.01,'ral',ral,'bafa',0.01);
            sr  = chain_robot.sz.xyz_len(3)/30; sfa = 0.5;
            all = chain_robot.sz.xyz_len(3)/10;
            plot_ik_targets('chain_robot',chain_robot,...
                'ik_plot_info',get_ik_plot_info_from_ik_info(ik_info_upper_p),...
                'fig_idx',fig_idx,'subfig_idx',1,'sr',sr,'sfa',sfa,'all',all);
            title_str = sprintf('[%d/%d] IK Error:[%.3e]',ik_tick,max_ik_tick,ik_err);
            plot_title(title_str,'fig_idx',fig_idx,'tfs',15,'interpreter','latex');
            drawnow; pause_invalid_handle(fig6)
        end

    end % end of IK

    % Append
    q_rev_robot_ft  = get_q_chain(chain_robot,chain_robot.rev_joint_names);
    T_root_robot_ft = pr2t(chain_robot.joint(root_idx_robot).p,chain_robot.joint(root_idx_robot).R);
    q_revs_robot_ft(tick,:) = rv(q_rev_robot_ft); 
    T_roots_robot_ft{tick}  = T_root_robot_ft;

    % Plot the final motion retaraget results (robot and rig)
    if PLOT_EACH_TICK
        % Plot common-rig
        fig_idx = 1; fig_pos = [0.0,0.5,0.2,0.4]; view_info = [80,12];
        axis_info = [-inf,inf,-inf,inf,-inf,chain_rig.sz.xyz_len(3)*1.4]; AXIS_OFF = 0;
        fig1 = plot_chain(chain_rig,'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
            'view_info',view_info,'axis_info',axis_info,'AXIS_OFF',AXIS_OFF,...
            'PLOT_ROTATE_AXIS',0,'PLOT_CAPSULE',1,'cfc',0.3*[1,1,1],'bafa',0.05);
        title_str = sprintf('[%d/%d][%.2f]sec',tick,L,sec);
        plot_title(title_str,'fig_idx',fig_idx,'tfs',17);
        % Plot robot
        fig_idx = 2; fig_pos = [0.2,0.5,0.2,0.4]; view_info = [80,12];
        axis_info = [-inf,inf,-inf,inf,-inf,chain_robot.sz.xyz_len(3)*1.6]; AXIS_OFF = 0;
        chain_robot_ground = move_chain_two_feet_on_ground(chain_robot,'xy_offset',cv([0,0]));
        fig2 = plot_chain(chain_robot_ground,'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
            'view_info',view_info,'axis_info',axis_info,'AXIS_OFF',AXIS_OFF,...
            'PLOT_ROTATE_AXIS',1,'PLOT_CAPSULE',0,'mfa',0.05,'bafa',0.05);
        title_str = sprintf('[%d/%d][%.2f]sec',tick,L,sec);
        plot_title(title_str,'fig_idx',fig_idx,'tfs',17);
        drawnow;
        if ((~ishandle(fig1)) || (~ishandle(fig2))), fprintf(2,'figure closed\n'); pause; end
    end
end

% Save 'q_revs_robot_ft' / 'T_roots_robot'
if SAVE_MAT
    [p,~,~] = fileparts(mat_path); make_dir_if_not_exist(p);
    save(mat_path,...
        'robot_name','mocap_name','secs',...
        'chain_rig','q_revs_rig','T_roots_rig',...
        'chain_robot','q_revs_robot','T_roots_robot',...
        'q_revs_robot_ft','T_roots_robot_ft',...
        'GIMBAL_LOCK_HEURISTICS');
    fprintf(2,'[%s] saved.\n',mat_path);
end
