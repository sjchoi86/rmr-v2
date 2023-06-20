function mr(chain_robot,secs,chain_rig,T_roots_rig,q_revs_rig,mocap_name,varargin)
%
% Run motion retargeting
%

% Parse options
iP = inputParser;
addParameter(iP,'data_folder_path','../data/mr');
addParameter(iP,'PLOT_INITITAL_T_POSE_QUAT_OFFSET',0);
addParameter(iP,'APPLY_ROOT_TO_NECK_OFFSET',1);
addParameter(iP,'GIMBAL_LOCK_HEURISTICS',1);
addParameter(iP,'gimbal_threshold',0.1);
addParameter(iP,'VERBOSE',1);
addParameter(iP,'PLOT_IK_INSIDE',1);
addParameter(iP,'PLOT_EACH_TICK',1);
addParameter(iP,'SKIP_IF_MAT_EXIST',0);
addParameter(iP,'SAVE_MAT',1);
addParameter(iP,'max_ik_tick',100);
parse(iP,varargin{:});
data_folder_path            = iP.Results.data_folder_path;
PLOT_INITITAL_T_POSE_QUAT_OFFSET = iP.Results.PLOT_INITITAL_T_POSE_QUAT_OFFSET;
APPLY_ROOT_TO_NECK_OFFSET   = iP.Results.APPLY_ROOT_TO_NECK_OFFSET;
GIMBAL_LOCK_HEURISTICS      = iP.Results.GIMBAL_LOCK_HEURISTICS;
gimbal_threshold            = iP.Results.gimbal_threshold;
VERBOSE                     = iP.Results.VERBOSE;
PLOT_IK_INSIDE              = iP.Results.PLOT_IK_INSIDE;
PLOT_EACH_TICK              = iP.Results.PLOT_EACH_TICK;
SKIP_IF_MAT_EXIST           = iP.Results.SKIP_IF_MAT_EXIST;
SAVE_MAT                    = iP.Results.SAVE_MAT;
max_ik_tick                 = iP.Results.max_ik_tick;
D2R = pi/180;

% Mat path to save
robot_name = chain_robot.name; % robot name
mat_path = sprintf('%s/%s_%s.mat',data_folder_path,robot_name,mocap_name);

% Skip this if mat file already exists
if SKIP_IF_MAT_EXIST && exist(mat_path,'file')
    fprintf(2,'Skip this as [%s] exists.\n',mat_path);
    return
elseif SAVE_MAT
    fprintf('The result will be saved at [%s].\n',mat_path);
end

% Close all
ca;

% T-pose matching via qauternion action
L                  = length(secs);
HZ                 = round(L/(secs(end)-secs(1)));
root_idx_robot     = get_topmost_idx(chain_robot);
root_idx_rig       = get_topmost_idx(chain_rig);
chain_robot        = get_chain_t_pose(chain_robot);
chain_robot        = move_chain_two_feet_on_ground(chain_robot);
chain_robot_t_pose = chain_robot;
chain_rig          = get_chain_t_pose(chain_rig);
chain_rig          = move_chain_two_feet_on_ground(chain_rig);
chain_rig_t_pose   = chain_rig;
T_joi_robot        = get_t_joi(chain_robot,chain_robot.joi);
T_joi_rig          = get_t_joi(chain_rig,chain_rig.joi);
uv_robot_t_pose    = get_uv_chain(chain_robot_t_pose);
uv_rig_t_pose      = get_uv_chain(chain_rig_t_pose);
root2neck_quat_offset = get_q_uv1_to_uv2(...
    uv_rig_t_pose.root2neck,uv_robot_t_pose.root2neck); % root2neck quaternion offset
root2neck_rot_axis = uv(root2neck_quat_offset(2:4));
uv_root2neck_rig_offset = quat_action(uv_rig_t_pose.root2neck,root2neck_quat_offset);

% Plot T-poses of both mocap and robot and quaternion offset
if PLOT_INITITAL_T_POSE_QUAT_OFFSET
    fig_idx = 11; fig_pos = [0.0,0.5,0.25,0.45]; view_info = [33,21];
    plot_chain(chain_rig_t_pose,'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
        'view_info',view_info,'PLOT_LINK',0,'PLOT_ROTATE_AXIS',0,'PLOT_CAPSULE',1,'cfa',0.1,...
        'PLOT_BOX_ADDED',1,'bafa',0.1,'DISREGARD_JOI_GUIDE',1);
    plot_chain(chain_robot_t_pose,'fig_idx',fig_idx,'subfig_idx',2,'PLOT_LINK',0,...
        'PLOT_ROTATE_AXIS',0,'PLOT_CAPSULE',0,'cfa',0.1,'PLOT_BOX_ADDED',1,'bafa',0.1,'mfa',0.05,...
        'DISREGARD_JOI_GUIDE',1);
    p_torso_rig = t2p(T_joi_robot.torso); p_torso_robot = t2p(T_joi_rig.torso);
    sw = 0.025; tw = 0.05; text_fs = 13; text_p2_offset = 0.01;
    plot_plane('fig_idx',fig_idx,'subfig_idx',1,...
        'xmin',-0.3,'xmax',0.3,'xres',0.1,'ymin',-0.3,'ymax',0.3,'yres',0.1,...
        'plane_normal',root2neck_rot_axis,'plane_center',p_torso_rig,'pfc',0.5*[1,1,1],'pfa',0.2);
    plot_arrow_3d(p_torso_rig,p_torso_rig+root2neck_rot_axis*0.5,...
        'fig_idx',fig_idx,'subfig_idx',1,'alpha',0.5,'color','k','sw',sw,'tw',tw,...
        'text_str','Rotation Axis','text_fs',text_fs,'text_p2_offset',text_p2_offset,...
        'interpreter','latex');
    plot_arrow_3d(p_torso_rig,p_torso_rig+uv_rig_t_pose.root2neck*0.5,...
        'fig_idx',fig_idx,'subfig_idx',2,'alpha',0.5,'color','b','sw',sw,'tw',tw,...
        'text_str','Rig Offset','text_fs',text_fs,'text_p2_offset',text_p2_offset,...
        'interpreter','latex');
    plot_arrow_3d(p_torso_rig,p_torso_rig+uv_root2neck_rig_offset*0.5,...
        'fig_idx',fig_idx,'subfig_idx',3,'alpha',0.5,'color','m','sw',sw,'tw',tw,...
        'text_str','Rig','text_fs',text_fs,'text_p2_offset',text_p2_offset,'interpreter','latex');
    plot_arrow_3d(p_torso_robot,p_torso_robot+uv_robot_t_pose.root2neck*0.5,...
        'fig_idx',fig_idx,'subfig_idx',4,'alpha',0.5,'color','r','sw',sw,'tw',tw,...
        'text_str','Robot','text_fs',text_fs,'text_p2_offset',text_p2_offset,'interpreter','latex');
    plot_title('T-poses and Torso-to-Neck Offset','fig_idx',fig_idx,'tfs',20);
    drawnow;
    fprintf(2,'Paused.\n');
    pause; % pause
    ca; % close all
end

% Loop
q_revs_robot  = zeros(L,chain_robot.n_rev_joint);
T_roots_robot = cell(L,1);
tk = init_tk('mr'); % start time keeping
for tick = 1:L % for each tick
    tk = print_tk(tk,tick,L,5); % time-keeping
    if tick == 1, RESET = 1; else, RESET = 0; end

    % Update common rig
    sec       = secs(tick);
    q_rev     = q_revs_rig(tick,:);
    T_root    = T_roots_rig{tick};
    chain_rig = update_chain_q_root_T(chain_rig,q_rev,T_root,'FV',1,'RESET',RESET);
    chain_rig = move_chain_two_feet_on_ground(chain_rig); % to ground

    % Robot Gimbal lock handling
    [n_gimbal,gimbal_q1q2s,gimbal_pairs] = check_gimbal_lock(chain_robot,...
        'gimbal_threshold',gimbal_threshold,'VERBOSE',VERBOSE,'RESET',RESET);

    % Motion Retarget Phase 1: lower-body (+two feet) matching
    chain_robot = move_chain_two_feet_on_ground(chain_robot); % robot to ground
    if isequal(robot_name,'ambidex')
        T_joi_rig          = get_t_joi(chain_rig,chain_rig.joi);
        T_joi_robot        = get_t_joi(chain_robot,chain_robot.joi);
        T_joi_robot_t_pose = get_t_joi(chain_robot_t_pose,chain_robot_t_pose.joi);
        len_rig            = get_len_chain(chain_rig);
        len_leg_robot_t_pose = norm(t2p(T_joi_robot_t_pose.torso)-t2p(T_joi_robot_t_pose.ra));
        len_leg_rig        = len_rig.rk2ra + len_rig.rp2rk;
        z_torso2ankle_rig  = (t2p(T_joi_rig.torso)-t2p(T_joi_rig.ra))'*cv([0,0,1]);
        p_torso_robot      = 0.5*(t2p(T_joi_robot.ra)+t2p(T_joi_robot.la));
        p_torso_robot(3)   = len_leg_robot_t_pose*z_torso2ankle_rig/len_leg_rig; % height
        % Append IK targets
        p_trgts = struct(); R_trgts = struct();
        p_trgts.torso = p_torso_robot;
        ik_info_lower_p = get_ik_info_from_joi_targets(chain_robot,p_trgts,R_trgts,...
            'ik_err_th',10.0,'dq_th',10*D2R,'step_size',1.0,...
            'joi_p_trgts',{'torso'});
        ik_info_lower_R = get_ik_info_from_joi_targets(chain_robot,p_trgts,R_trgts,...
            'ik_err_th',10.0,'dq_th',10*D2R,'step_size',1.0,...
            'joi_R_trgts',{});
    else
        % Besides Ambidex, we consider lower-body joints
        T_joi_robot = get_t_joi(chain_robot,chain_robot.joi);
        T_joi_robot_t_pose = get_t_joi(chain_robot_t_pose,chain_robot_t_pose.joi);
        uv_rig = get_uv_chain(chain_rig);
        len_robot = get_len_chain(chain_robot);
        p_rp_robot = t2p(T_joi_robot.torso) + len_robot.hip2rp*uv_rig.hip2rp;
        p_rk_robot = p_rp_robot + len_robot.rp2rk*uv_rig.rp2rk;
        p_ra_robot = p_rk_robot + len_robot.rk2ra*uv_rig.rk2ra;
        p_lp_robot = t2p(T_joi_robot.torso) + len_robot.hip2lp*uv_rig.hip2lp;
        p_lk_robot = p_lp_robot + len_robot.lp2lk*uv_rig.lp2lk;
        p_la_robot = p_lk_robot + len_robot.lk2la*uv_rig.lk2la;
        % Two feet fine-tuning
        p_ca_robot = 0.5*(p_ra_robot+p_la_robot);
        feet_dist_robot = norm(t2p(T_joi_robot_t_pose.ra)-t2p(T_joi_robot_t_pose.la));
        p_ra_robot = p_ca_robot; p_la_robot = p_ca_robot;
        p_ra_robot(2) = p_ra_robot(2) - 0.5*feet_dist_robot;
        p_la_robot(2) = p_la_robot(2) + 0.5*feet_dist_robot;
        % Append IK targets
        p_trgts = struct(); R_trgts = struct();
        p_trgts.rp = p_rp_robot; p_trgts.rk = p_rk_robot; p_trgts.ra = p_ra_robot;
        p_trgts.lp = p_lp_robot; p_trgts.lk = p_lk_robot; p_trgts.la = p_la_robot;
        R_trgts.rf = t2r(T_joi_robot_t_pose.rf); R_trgts.lf = t2r(T_joi_robot_t_pose.lf);
        ik_info_lower_p = get_ik_info_from_joi_targets(chain_robot,p_trgts,R_trgts,...
            'ik_err_th',10.0,'dq_th',10*D2R,'step_size',1.0,...
            'joi_p_trgts',{'rp','rk','ra','lp','lk','la'});
        ik_info_lower_R = get_ik_info_from_joi_targets(chain_robot,p_trgts,R_trgts,...
            'ik_err_th',10.0,'dq_th',10*D2R,'step_size',1.0,...
            'joi_R_trgts',{'rf','lf'});
    end

    % Solve IK for lower body
    for ik_tick = 1:max_ik_tick % for each IK tick (phase 1)
        zero_to_one = ik_tick/max_ik_tick;
        one_to_zero = 1 - zero_to_one;
        unit_dq_rad_max  = 20; unit_dq_rad_min  = 1;
        unit_dq_rad = (unit_dq_rad_max*one_to_zero+unit_dq_rad_min)*D2R;
        % IK step for positions
        [dq,joint_names_to_ctrl,~,ik_err_lower_p,~] = one_step_ik(...
            chain_robot,ik_info_lower_p,'CONSIDER_JOINT_LIMIT',1,...
            'UNIT_DQ_HEURISTIC',1,'unit_dq_rad',unit_dq_rad,...
            'limit_margin_rad',0);
        chain_robot = update_chain_q(chain_robot,joint_names_to_ctrl,...
            get_q_chain(chain_robot,joint_names_to_ctrl)+dq,...
            'IGNORE_LIMIT',0,'margin_rad',0,'FV',0);
        % IK step for rotations
        [dq,joint_names_to_ctrl,~,~,~] = one_step_ik(...
            chain_robot,ik_info_lower_R,'CONSIDER_JOINT_LIMIT',1,...
            'UNIT_DQ_HEURISTIC',1,'unit_dq_rad',unit_dq_rad,...
            'limit_margin_rad',0);
        chain_robot = update_chain_q(chain_robot,joint_names_to_ctrl,...
            get_q_chain(chain_robot,joint_names_to_ctrl)+dq,...
            'IGNORE_LIMIT',0,'margin_rad',0,'FV',0);
        % IK error
        ik_errs = [ik_err_lower_p];
        ik_err = max(abs(ik_errs))/chain_robot.sz.xyz_len(3); % normalized IK error
        if PLOT_IK_INSIDE && ((ik_tick<=5) || (mod(ik_tick,20)==0))
            fig_idx = 5; fig_pos = [0.0,0.0,0.2,0.45];
            axis_info = ''; view_info = [60,32]; ral = chain_robot.sz.xyz_len(3)/20;
            fig5 = plot_chain(chain_robot,...
                'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
                'axis_info',axis_info,'view_info',view_info,...
                'PLOT_LINK',1,'PLOT_JOINT_AXIS',0,'jalw',3,'PLOT_JOINT_SPHERE',1,...
                'PLOT_MESH',0,'jsr',0.01,'ral',ral,'bafa',0.01);
            sr = chain_robot.sz.xyz_len(3)/30; sfa = 0.5;
            all = chain_robot.sz.xyz_len(3)/10;
            plot_ik_targets('chain_robot',chain_robot,...
                'ik_plot_info',get_ik_plot_info_from_ik_info(ik_info_lower_p),...
                'fig_idx',fig_idx,'subfig_idx',1,'sr',sr,'sfa',sfa,'all',all);
            plot_ik_targets('chain_robot',chain_robot,...
                'ik_plot_info',get_ik_plot_info_from_ik_info(ik_info_lower_R),...
                'fig_idx',fig_idx,'subfig_idx',2,'sr',sr,'sfa',sfa,'all',all);
            title_str = sprintf('[%d/%d] A.Waist IK Error:[%.3e]',ik_tick,max_ik_tick,ik_err);
            plot_title(title_str,'fig_idx',fig_idx,'tfs',15,'interpreter','latex');
            drawnow; pause_invalid_handle(fig5);
        end
    end % for ik_tick = 1:max_ik_tick % for each IK tick (phase 1)

    % Motion Retarget Phase 2: upper-body matching
    T_joi_rig        = get_t_joi(chain_rig,chain_rig.joi);
    chain_robot      = move_chain_two_feet_on_ground(chain_robot); % to ground
    T_joi_robot      = get_t_joi(chain_robot,chain_robot.joi);
    len_robot        = get_len_chain(chain_robot);
    uv_rig           = get_uv_chain(chain_rig);
    p_torso_robot    = t2p(T_joi_robot.torso);
    uv_rig_root2neck = uv_rig.root2neck;

    % Apply root-to-neck rotation offset
    if APPLY_ROOT_TO_NECK_OFFSET
        uv_rig_root2neck_prev = uv_rig_root2neck;
        uv_rig_root2neck = quat_action(uv_rig_root2neck_prev,root2neck_quat_offset);
        uv_rig_root2neck = 0.5*(uv_rig_root2neck + uv_rig_root2neck_prev);
        % Plot
        if PLOT_IK_INSIDE
            fig_idx = 7; fig_pos = [0.4,0.0,0.2,0.45];
            axis_info = ''; view_info = [60,32]; ral = chain_robot.sz.xyz_len(3)/20;
            fig7 = plot_chain(chain_robot,...
                'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
                'axis_info',axis_info,'view_info',view_info,...
                'PLOT_LINK',1,'PLOT_JOINT_AXIS',0,'jalw',3,'PLOT_JOINT_SPHERE',1,...
                'PLOT_MESH',0,'jsr',0.01,'ral',ral,'bafa',0.01);
            p_torso_robot = t2p(T_joi_robot.torso);
            sw = 0.025; tw = 0.05; text_fs = 13; text_p2_offset = 0.01;
            plot_arrow_3d(p_torso_robot,p_torso_robot+uv_rig.root2neck*0.5,...
                'fig_idx',fig_idx,'subfig_idx',1,'alpha',0.5,'color','k','sw',sw,'tw',tw,...
                'text_str','Original root2neck','text_fs',text_fs,'text_p2_offset',text_p2_offset,...
                'interpreter','latex');
            plot_arrow_3d(p_torso_robot,p_torso_robot+uv_rig_root2neck*0.5,...
                'fig_idx',fig_idx,'subfig_idx',2,'alpha',0.5,'color','b','sw',sw,'tw',tw,...
                'text_str','Modified root2neck','text_fs',text_fs,'text_p2_offset',text_p2_offset,...
                'interpreter','latex');
            drawnow; pause_invalid_handle(fig7);
        end
    end
    p_neck_robot     = p_torso_robot + len_robot.root2neck*uv_rig_root2neck;
    p_rs_robot       = p_neck_robot + len_robot.neck2rs*uv_rig.neck2rs;
    p_re_robot       = p_rs_robot + len_robot.rs2re*uv_rig.rs2re;
    p_re_guide_robot = p_re_robot + len_robot.re2re_guide*uv_rig.re2re_guide;
    p_rw_robot       = p_re_robot + len_robot.re2rw*uv_rig.re2rw;
    p_ls_robot       = p_neck_robot + len_robot.neck2ls*uv_rig.neck2ls;
    if isfield(len_robot,'rs2rs_guide') && isfield(uv_rig,'rs2rs_guide')
        p_rs_guide_robot = p_rs_robot + len_robot.rs2rs_guide*uv_rig.rs2rs_guide;
        p_ls_guide_robot = p_ls_robot + len_robot.ls2ls_guide*uv_rig.ls2ls_guide;
    end
    p_le_robot = p_ls_robot + len_robot.ls2le*uv_rig.ls2le;
    p_le_guide_robot = p_le_robot + len_robot.le2le_guide*uv_rig.le2le_guide;
    p_lw_robot = p_le_robot + len_robot.le2lw*uv_rig.le2lw;
    R_rh_robot = t2r(T_joi_rig.rh); R_lh_robot = t2r(T_joi_rig.lh);

    % Append IK targets
    p_trgts = struct(); R_trgts = struct();
    p_trgts.torso = p_torso_robot; p_trgts.neck = p_neck_robot;
    p_trgts.rs = p_rs_robot; p_trgts.re = p_re_robot; p_trgts.re_guide = p_re_guide_robot;
    p_trgts.ls = p_ls_robot; p_trgts.le = p_le_robot; p_trgts.le_guide = p_le_guide_robot;
    if isfield(len_robot,'rs2rs_guide') && isfield(uv_rig,'rs2rs_guide')
        p_trgts.rs_guide = p_rs_guide_robot;
        p_trgts.ls_guide = p_ls_guide_robot;
    end
    p_trgts.rw = p_rw_robot; p_trgts.lw = p_lw_robot;
    R_trgts.rh = R_rh_robot; R_trgts.lh = R_lh_robot;

    % Gimbal lock handling heuristics
    joint_names_to_ctrl = chain_robot.rev_joint_names;
    if (n_gimbal > 0) && GIMBAL_LOCK_HEURISTICS
        joint_names_to_remove = handle_gimbal_lock(chain_robot,gimbal_q1q2s,gimbal_pairs,...
            'chain_robot_t_pose',chain_robot_t_pose);
        idxs = idxs_cell(joint_names_to_ctrl,joint_names_to_remove);
        joint_names_to_ctrl(idxs) = []; % remove joints
    end
    ik_info_upper_p = get_ik_info_from_joi_targets(chain_robot,p_trgts,R_trgts,...
        'ik_err_th',10.0,'dq_th',10*D2R,'step_size',1.0,...
        'joi_p_trgts',{'torso','neck','rs','rs_guide','re','re_guide','rw',...
        'ls','ls_guide','le','le_guide','lw'},...
        'joint_names_to_ctrl',joint_names_to_ctrl);
    ik_info_upper_R = get_ik_info_from_joi_targets(chain_robot,p_trgts,R_trgts,...
        'ik_err_th',10.0,'dq_th',10*D2R,'step_size',1.0,...
        'joi_R_trgts',{'rh','lh'},...
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

        % Start from the median pose (suitable for IK)
        if (ik_tick == 1) && 0
            rev_joints_names_median = get_rev_joint_names_route_to_jois(chain_robot,{'rh','lh'});
            q_median = zeros(1,length(rev_joints_names_median));
            for r_idx = 1:length(rev_joints_names_median)
                joint_idx       = idx_cell(chain_robot.joint_names,rev_joints_names_median{r_idx});
                q_median(r_idx) = mean(chain_robot.joint(joint_idx).limit);
            end
            chain_robot = update_chain_q(chain_robot,rev_joints_names_median,q_median);
        end

        % IK step for positions
        CONSIDER_JOINT_LIMIT = 1;
        [dq,joint_names_to_ctrl,~,ik_err_upper_p,~] = one_step_ik(...
            chain_robot,ik_info_upper_p,'CONSIDER_JOINT_LIMIT',CONSIDER_JOINT_LIMIT,...
            'UNIT_DQ_HEURISTIC',1,'unit_dq_rad',unit_dq_rad,'limit_margin_rad',0);
        chain_robot = update_chain_q(chain_robot,joint_names_to_ctrl,...
            get_q_chain(chain_robot,joint_names_to_ctrl)+dq,...
            'IGNORE_LIMIT',~CONSIDER_JOINT_LIMIT,'margin_rad',0,'FV',0);

        % IK step for rotations
        [dq,joint_names_to_ctrl,~,~,~] = one_step_ik(...
            chain_robot,ik_info_upper_R,'CONSIDER_JOINT_LIMIT',CONSIDER_JOINT_LIMIT,...
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
            plot_ik_targets('chain_robot',chain_robot,...
                'ik_plot_info',get_ik_plot_info_from_ik_info(ik_info_upper_R),...
                'fig_idx',fig_idx,'subfig_idx',2,'sr',sr,'sfa',sfa,'all',all);
            title_str = sprintf('[%d/%d] A.Waist IK Error:[%.3e]',ik_tick,max_ik_tick,ik_err);
            plot_title(title_str,'fig_idx',fig_idx,'tfs',15,'interpreter','latex');
            drawnow; pause_invalid_handle(fig6)
        end
    end % for ik_tick = 1:max_ik_tick % for each IK tick (phase 2)

    % Append
    q_rev_robot = get_q_chain(chain_robot,chain_robot.rev_joint_names);
    T_root_robot = pr2t(chain_robot.joint(root_idx_robot).p,chain_robot.joint(root_idx_robot).R);
    q_revs_robot(tick,:) = rv(q_rev_robot); T_roots_robot{tick} = T_root_robot;

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

end % for tick = 1:L % for each tick

% Save 'q_revs_robot' / 'T_roots_robot'
if SAVE_MAT
    [p,~,~] = fileparts(mat_path); make_dir_if_not_exist(p);
    save(mat_path,...
        'robot_name','mocap_name','secs',...
        'chain_rig','q_revs_rig','T_roots_rig',...
        'chain_robot','q_revs_robot','T_roots_robot',...
        'GIMBAL_LOCK_HEURISTICS');
    fprintf(2,'[%s] saved.\n',mat_path);
end
