function pre_rigging(secs,chains,mocap_name,varargin)
%
% Run pre-rigging process
%

% Parse options
iP = inputParser;
addParameter(iP,'mocap_type','');
addParameter(iP,'IK_FROM_ZERO_POSE',1);
addParameter(iP,'MIX_WITH_PREV_Q',1);
addParameter(iP,'PLOT_EACH_TICK',1);
addParameter(iP,'PLOT_IK_INSIDE',1);
addParameter(iP,'SAVE_MAT',1);
addParameter(iP,'SKIP_IF_MAT_EXIST',1);
addParameter(iP,'data_folder','../data/pre_rig');
addParameter(iP,'max_sec',inf);
addParameter(iP,'max_ik_tick',100);
parse(iP,varargin{:});
mocap_type        = iP.Results.mocap_type;
IK_FROM_ZERO_POSE = iP.Results.IK_FROM_ZERO_POSE;
MIX_WITH_PREV_Q   = iP.Results.MIX_WITH_PREV_Q;
PLOT_EACH_TICK    = iP.Results.PLOT_EACH_TICK;
PLOT_IK_INSIDE    = iP.Results.PLOT_IK_INSIDE;
SAVE_MAT          = iP.Results.SAVE_MAT;
SKIP_IF_MAT_EXIST = iP.Results.SKIP_IF_MAT_EXIST;
data_folder       = iP.Results.data_folder;
max_sec           = iP.Results.max_sec;
max_ik_tick       = iP.Results.max_ik_tick;

% Skip if necessary
mat_path = sprintf('%s/%s.mat',data_folder,mocap_name);
if exist(mat_path,'file') && SKIP_IF_MAT_EXIST
    fprintf(2,'[pre_rigging] Skip this as [%s] exists.\n',mat_path);
    return;
end

% Numbers
D2R = pi/180;
HZ  = floor(length(secs)/(secs(end)-secs(1)));
L   = length(secs);

if ~isinf(max_sec)
    L_bu   = L;
    L      = round(max_sec*HZ);
    secs   = secs(1:L);
    chains = chains(1:L);
    fprintf('[pre_rigging] Length reduced from [%d] to [%d]\n',L_bu,L)
end

% Set IK position and rotation targets
if PLOT_EACH_TICK
    ca; % close all
end
p_trgts_list = cell(1,L); R_trgts_list = cell(1,L); T_mocap_roots = cell(1,L);
for tick = 1:L % for each tick 
    sec          = secs(tick);
    chain_mocap  = chains{tick};
    T_joi        = get_t_joi(chain_mocap);
    switch mocap_type
        case 'mhformer' % MHFORMER torso
            T_mocap_root = get_t_torso_mhformer(chain_mocap);
        case 'frank' % frank mocap
            T_mocap_root = get_t_root_frank_chain(chain_mocap);
        otherwise
            T_mocap_root = T_joi.torso;
    end
    if tick == 1 % get a humanoid robot whose link lengths match with mocap skeleton
        len_mocap   = get_len_chain(chain_mocap);
        cap_r_rate  = len_mocap.root2neck/0.5;
        head_r      = 0.1*cap_r_rate;
        torso_r     = 0.08*cap_r_rate;
        link_r      = 0.05*cap_r_rate;
        chain_rig = get_common_rig_from_mocap(chain_mocap,...
            'head_r',head_r,'torso_r',torso_r,'link_r',link_r,...
            'ADD_ELBOW_GUIDE',1,'ADD_SHOULDER_GUIDE',0);
        chain_rig.dt = 1/HZ; % set sim. time difference
        chain_rig = update_chain_mass_inertia_com(chain_rig,'RE',1,'density',50);
        chain_rig_init = chain_rig; % save the initial humanoid
        % Save the initial rotations
        R_rig_root_init = chain_rig.joint(get_topmost_idx(chain_rig_init)).R;
        R_mocap_root_init = eye(3,3);
    end
    % Set IK position and rotation targets
    uv_mocap   = get_uv_chain(chain_mocap,...
        'FORCE_NECK_MID_SHOULDER',1,'USE_HEAD_INSTEAD_OF_NECK',1);
    len_rig    = get_len_chain(chain_rig);
    % p_root_rig = chain_rig.joint(get_joi_joint_idx(chain_rig.joi,'torso')).p;
    p_root_rig  = t2p(T_mocap_root);
    uv_mocap.root2spine = uv_mocap.root2neck;
    uv_mocap.spine2neck = uv_mocap.root2neck;
    % Set positional targets
    p_trgts.spine   = p_root_rig    + len_rig.root2spine*uv_mocap.root2spine;
    p_trgts.neck    = p_trgts.spine + len_rig.spine2neck*uv_mocap.spine2neck;
    p_trgts.rs      = p_trgts.neck  + len_rig.neck2rs*uv_mocap.neck2rs;
    p_trgts.re      = p_trgts.rs    + len_rig.rs2re*uv_mocap.rs2re;
    p_trgts.rw      = p_trgts.re    + len_rig.re2rw*uv_mocap.re2rw;
    p_trgts.ls      = p_trgts.neck  + len_rig.n2ls*uv_mocap.n2ls;
    p_trgts.le      = p_trgts.ls    + len_rig.rs2re*uv_mocap.ls2le;
    p_trgts.lw      = p_trgts.le    + len_rig.re2rw*uv_mocap.le2lw;
    p_trgts.hip     = p_root_rig    + len_rig.root2hip*uv_mocap.root2hip;
    p_trgts.rp      = p_trgts.hip   + len_rig.hip2rp*uv_mocap.hip2rp;
    p_trgts.rk      = p_trgts.rp    + len_rig.rp2rk*uv_mocap.rp2rk;
    p_trgts.ra      = p_trgts.rk    + len_rig.rk2ra*uv_mocap.rk2ra;
    p_trgts.lp      = p_trgts.hip   + len_rig.hip2lp*uv_mocap.hip2lp;
    p_trgts.lk      = p_trgts.lp    + len_rig.lp2lk*uv_mocap.lp2lk;
    p_trgts.la      = p_trgts.lk    + len_rig.lk2la*uv_mocap.lk2la;

    % Set rotational targets (both hands and feet)
    if isfield(T_joi,'rh') && isfield(T_joi,'lh')
        R_trgts.rh = t2r(T_joi.rh);
        R_trgts.lh = t2r(T_joi.lh);
    end
    T_mocap_roots{tick} = T_mocap_root; % append
    R_mocap_root = t2r(T_mocap_root);
    R_trgts.rf = R_mocap_root*[0,0,1; 1,0,0; 0,1,0];
    R_trgts.lf = R_mocap_root*[0,0,1; 1,0,0; 0,1,0];
    cost_rf = @(rpy) (...
        norm(rpy2r(rpy)*R_trgts.rf*cv([0,1,0])-cv([0,0,1])) ... % original objective
        + 1e-1*(abs(rpy(1))+abs(rpy(2))+20*abs(rpy(3))) ... % regularizer
        );
    rpy_hat = fminsearch(cost_rf,cv([0,0,0]));
    R_trgts.rf = rpy2r(rpy_hat)*R_trgts.rf;
    cost_lf = @(rpy) (...
        norm(rpy2r(rpy)*R_trgts.lf*cv([0,1,0])-cv([0,0,1])) ... % original objective
        + 1e-1*(abs(rpy(1))+abs(rpy(2))+20*abs(rpy(3))) ... % regularizer
        );
    rpy_hat = fminsearch(cost_lf,cv([0,0,0]));
    R_trgts.lf = rpy2r(rpy_hat)*R_trgts.lf;
    % Append position and rotation targets
    p_trgts_list{tick} = p_trgts;
    R_trgts_list{tick} = R_trgts;
end

% Actual rigging with solving IK starts here
q_revs_pre = zeros(L,35); T_roots_pre = cell(1,L); % buffers to save
tk = init_tk('pre_rigging');
for tick = 1:L % for each tick
    tk = print_tk(tk,tick,L,5);
    chain_mocap = chains{tick}; 
    % chain_mocap.joi = get_joi_chain(chain_mocap,'APPEND_ALL',1); % JOI already exists
    % Translate and rotate the humanoid to match with skeleton's root
    T_mocap_root = T_mocap_roots{tick};
    chain_rig = move_chain(chain_rig,t2p(T_mocap_root));
    chain_rig = rotate_chain_robot_to_match_mocap(...
        chain_rig,t2r(T_mocap_root),R_rig_root_init,R_mocap_root_init);
    % IK from the scratch
    if IK_FROM_ZERO_POSE
        chain_rig = update_chain_q(chain_rig,...
            chain_rig.rev_joint_names,zeros(1,chain_rig.n_rev_joint));
        chain_rig = update_chain_q(chain_rig,...
            {'rs1','rs2','rs3','ls1','ls2','ls3','re2','le2'},...
            [70,25,-5,-70,-25,-5,35,-35]*D2R,'FK',1,'FV',0); % good initial pose
    end
    % Set IK targets
    p_trgts = p_trgts_list{tick};
    R_trgts = R_trgts_list{tick};
    % Define IK information structures
    ik_err_th = 1.0; dq_th = 10*D2R; step_size = 1.0; 
    UNIT_DQ_HEURISTIC             = 1; 
    DISREGARD_UNINFLUENTIAL_JOINT = 1;
    ik_info_upper_p = get_ik_info_from_joi_targets(chain_rig,p_trgts,R_trgts,...
        'ik_err_th',ik_err_th,'dq_th',dq_th,'step_size',step_size,...
        'joi_p_trgts',{'rw','re','rs','lw','le','ls','spine'},'joi_R_trgts',{},...
        'joi_p_weights',[1,1,1,1,1,1,1]);
    ik_info_upper_R = get_ik_info_from_joi_targets(chain_rig,p_trgts,R_trgts,...
        'ik_err_th',ik_err_th,'dq_th',dq_th,'step_size',step_size,...
        'joi_p_trgts',{},'joi_R_trgts',{'rh','lh','head'},...
        'joi_R_weights',[1,1,1]);
    ik_info_lower_p = get_ik_info_from_joi_targets(chain_rig,p_trgts,R_trgts,...
        'ik_err_th',ik_err_th,'dq_th',dq_th,'step_size',step_size,...
        'joi_p_trgts',{'rp','rk','ra','lp','lk','la'},'joi_R_trgts',{},...
        'joi_p_weights',[1,1,1,1,1,1]);
    ik_info_lower_R = get_ik_info_from_joi_targets(chain_rig,p_trgts,R_trgts,...
        'ik_err_th',ik_err_th,'dq_th',dq_th,'step_size',step_size,...
        'joi_R_trgts',{},'joi_R_trgts',{'rf','lf'},...
        'joi_R_weights',[1,1]);
    if tick == 1
        max_ik_tick_use = 100;
    else
        max_ik_tick_use = max_ik_tick;
    end
    for ik_tick = 1:max_ik_tick_use % for each IK tick
        zero_to_one = ik_tick/max_ik_tick_use;
        unit_dq_rad = 5*D2R*(1-ik_tick/max_ik_tick_use);
        % Little mixing with the previous joint position
        if MIX_WITH_PREV_Q && (tick >= 2) && (zero_to_one < 0.5)  % skip the first tick
            q_rev_prev = cv(q_revs_pre(tick-1,:));
            q_rev_curr = get_q_chain(chain_rig,chain_rig.rev_joint_names);
            alpha = 0.1; % the bigger, the more mixing
            q_rev_update = alpha*q_rev_prev + (1-alpha)*q_rev_curr;
            chain_rig = update_chain_q(chain_rig,chain_rig.rev_joint_names,q_rev_update);
        end
        % IK step for upper body positions
        [dq,joint_names_to_ctrl,~,ik_err_upper_p,~] = one_step_ik(...
            chain_rig,ik_info_upper_p,'CONSIDER_JOINT_LIMIT',1,...
            'UNIT_DQ_HEURISTIC',UNIT_DQ_HEURISTIC,'unit_dq_rad',unit_dq_rad,...
            'DISREGARD_UNINFLUENTIAL_JOINT',DISREGARD_UNINFLUENTIAL_JOINT);
        chain_rig = update_chain_q(chain_rig,joint_names_to_ctrl,...
            get_q_chain(chain_rig,joint_names_to_ctrl)+dq,...
            'IGNORE_LIMIT',0,'FV',0);
        % IK step for upper body rotations (both hands and head orientation)
        [dq,joint_names_to_ctrl,~,ik_err_upper_R,~] = one_step_ik(...
            chain_rig,ik_info_upper_R,'CONSIDER_JOINT_LIMIT',1,...
            'UNIT_DQ_HEURISTIC',UNIT_DQ_HEURISTIC,'unit_dq_rad',unit_dq_rad,...
            'DISREGARD_UNINFLUENTIAL_JOINT',DISREGARD_UNINFLUENTIAL_JOINT);
        chain_rig = update_chain_q(chain_rig,joint_names_to_ctrl,...
            get_q_chain(chain_rig,joint_names_to_ctrl)+dq,...
            'IGNORE_LIMIT',0,'FV',0);
        % IK step for lower body positions
        [dq,joint_names_to_ctrl,~,ik_err_lower_p,~] = one_step_ik(...
            chain_rig,ik_info_lower_p,'CONSIDER_JOINT_LIMIT',1,...
            'UNIT_DQ_HEURISTIC',UNIT_DQ_HEURISTIC,'unit_dq_rad',unit_dq_rad,...
            'DISREGARD_UNINFLUENTIAL_JOINT',DISREGARD_UNINFLUENTIAL_JOINT);
        chain_rig = update_chain_q(chain_rig,joint_names_to_ctrl,...
            get_q_chain(chain_rig,joint_names_to_ctrl)+dq,...
            'IGNORE_LIMIT',0,'FV',0);
        % IK step for lower body rotations
        [dq,joint_names_to_ctrl,~,ik_err_lower_R,~] = one_step_ik(...
            chain_rig,ik_info_lower_R,'CONSIDER_JOINT_LIMIT',1,...
            'UNIT_DQ_HEURISTIC',UNIT_DQ_HEURISTIC,'unit_dq_rad',unit_dq_rad,...
            'DISREGARD_UNINFLUENTIAL_JOINT',DISREGARD_UNINFLUENTIAL_JOINT);
        chain_rig = update_chain_q(chain_rig,joint_names_to_ctrl,...
            get_q_chain(chain_rig,joint_names_to_ctrl)+dq,...
            'IGNORE_LIMIT',0,'FV',0);
        % Accumulate IK errors
        % ik_err = [ik_err_upper_p; ik_err_upper_R; ik_err_lower_p; ik_err_lower_R];
        ik_err = [ik_err_upper_p; ik_err_lower_p] / chain_rig.sz.xyz_len(3);
        err_nzd_max = max(abs(ik_err));
        % Debug IK by plotting
        if PLOT_IK_INSIDE && ((ik_tick<=10) || (mod(ik_tick,20)==0))
            fig_idx = 13; view_info = [90,5]; ral = chain_rig.sz.xyz_len(3)/20;
            fig_pos = [0.3,0.65,0.2,0.3]; axis_info = '';
            plot_chain(chain_rig,'fig_idx',fig_idx,'fig_pos',fig_pos,'axis_info',axis_info,...
                'view_info',view_info,'PLOT_ROTATE_AXIS',1,'ral',ral);
            sr = chain_rig.sz.xyz_len(3)/30; sfa = 0.8; all = chain_rig.sz.xyz_len(3)/5;
            plot_ik_targets('chain_robot',chain_rig,...
                'ik_plot_info',get_ik_plot_info_from_ik_info(ik_info_upper_p),...
                'fig_idx',fig_idx,'subfig_idx',1,'sr',sr,'sfa',sfa,'all',all);
            plot_ik_targets('chain_robot',chain_rig,...
                'ik_plot_info',get_ik_plot_info_from_ik_info(ik_info_upper_R),...
                'fig_idx',fig_idx,'subfig_idx',2,'sr',sr,'sfa',sfa,'all',all);
            plot_ik_targets('chain_robot',chain_rig,...
                'ik_plot_info',get_ik_plot_info_from_ik_info(ik_info_lower_p),...
                'fig_idx',fig_idx,'subfig_idx',3,'sr',sr,'sfa',sfa,'all',all);
            plot_ik_targets('chain_robot',chain_rig,...
                'ik_plot_info',get_ik_plot_info_from_ik_info(ik_info_lower_R),...
                'fig_idx',fig_idx,'subfig_idx',4,'sr',sr,'sfa',sfa,'all',all);
            title_str = sprintf('IK:[%d/%d] err_rmse:[%.4f]',ik_tick,max_ik_tick_use,err_nzd_max);
            plot_title(title_str,'fig_idx',fig_idx,'interpreter','latex');
            drawnow;
        end
    end % for ik_tick = 1:max_ik_tick % for each IK tick
    if err_nzd_max >= 0.1 % notify if the IK error is too big
        fprintf(2, '   [%d/%d] err_nzd_max:[%.3f] is too big.\n',tick,L,err_nzd_max);
    end
    % Update q again for computing the joint velocity
    q_rev = get_q_chain(chain_rig,chain_rig.rev_joint_names);
    if tick == 1, RESET = 1; else, RESET = 0; end % reset internal variables for the first tick
    chain_rig = update_chain_q(...
        chain_rig,chain_rig.rev_joint_names,q_rev,'FV',1,'RESET',RESET);
    % Compute ZMP
    com = get_chain_com(chain_rig);
    com_ground = get_com_ground(chain_rig);
    z_ground = com_ground(3);
    zmp_ground = get_zmp_ground(chain_rig,com,z_ground,'fig_idx',2);
    % Append
    root_idx = get_topmost_idx(chain_rig);
    q_revs_pre(tick,:) = rv(get_q_chain(chain_rig,chain_rig.rev_joint_names));
    T_roots_pre{tick} = pr2t(chain_rig.joint(root_idx).p,chain_rig.joint(root_idx).R);
    % Animate each tick of humanoid pre-rigging
    if PLOT_EACH_TICK
        % Plot MoCap skeleton
        fig_idx = 11; fig_pos = [0.0,0.65,0.15,0.3];
        axis_info = get_axis_info_from_chains(chains,'margin',0,'INIT_AT_CENTER',0) + ...
            [-0.1,+0.1,-0.1,+0.1,-0.05,0.3];
        view_info = [70,15];
        fig11 = plot_chain(chain_mocap,'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
            'axis_info',axis_info,'view_info',view_info,...
            'PLOT_LINK',1,'llw',1,'PLOT_JOINT_AXIS',0,'jalw',3,...
            'PLOT_JOINT_SPHERE',1,'jsr',0.02,'bafc','','cfa',0.2,'bafa',0.2);
        plot_T(T_mocap_root,'fig_idx',fig_idx,'subfig_idx',1,'all',0.3,'alw',3,'PLOT_AXIS_TIP',1);
        title_str = sprintf('[%d/%d] Skeleton\n[%s]',tick,L,mocap_name);
        plot_title(title_str,'fig_idx',fig_idx,'tfs',15);
        drawnow;
        % Plot pre-rigging result
        fig_idx = 12; fig_pos = [0.15,0.65,0.15,0.3];
        fig12 = plot_chain(chain_rig,'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
            'axis_info',axis_info,'view_info',view_info,...
            'PLOT_LINK',1,'llw',1,'PLOT_CAPSULE',1,'cfc',0.5*[1,1,1],'cfa',0.2,'bafa',0.2,...
            'PLOT_ROTATE_AXIS',1,'ral',0.05,...
            'PLOT_LINK_V',0,'PLOT_LINK_W',0);
        T_root_rig = pr2t(chain_rig.joint(get_topmost_idx(chain_rig)).p,...
            chain_rig.joint(get_topmost_idx(chain_rig)).R);
        plot_T(T_root_rig,'fig_idx',fig_idx,'subfig_idx',3,'all',0.3,'alw',3,'PLOT_AXIS_TIP',1);
        plot_T(p2t(com_ground),'fig_idx',fig_idx,'subfig_idx',1,... % com (gray)
            'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.025,'sfc',0.5*[1,1,1],'sfa',0.5);
        plot_T(p2t(zmp_ground),'fig_idx',fig_idx,'subfig_idx',2,... % zmp (red)
            'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.03,'sfc','r','sfa',0.5);
        title_str = sprintf('[%d/%d] Pre-Rig\n[%s]',tick,L,mocap_name);
        plot_title(title_str,'fig_idx',fig_idx,'tfs',15);
        drawnow; pause_invalid_handle([fig11,fig12]);
    end
end

% Save mat file
if SAVE_MAT
    [p,~,~] = fileparts(mat_path); make_dir_if_not_exist(p);
    save(mat_path,'chain_rig','q_revs_pre','T_roots_pre','chains','secs','mocap_name');
    fprintf(2,'[pre_rigging] [%s] saved.\n',mat_path);
end
