addpath_yart
%% Motion Retargeting from Video (using MHFormer)
ccc
% Check available motions
info = get_mhformer_info('folder_path','../../mhformer-for-rmr/demo/result','VERBOSE',1);

%% Run motion retargeting with a specific motion 
% Specify which motion and robots to use
mocap_type  = 'mhformer';
motion_name = 'dr_strange2'; % <= modify this part
robot_names = {'ambidex','atlas','coman','thormang'};

mocap_names = cell(1,length(info));
for m_idx = 1:length(info), mocap_names{m_idx} = info(m_idx).name; end
mocap_idxs = idxs_cell(mocap_names,{motion_name}); % select 'idle_motion'

% Run configuration
SKIP_IF_MAT_EXIST  = 0;
SKIP_IF_VID_EXISTS = 0;
fig_w              = 0.15;
fig_h              = 0.3;
AXIS_OFF           = 1;
SET_AXISLABEL      = 0;
axis_info          = [-1,+1,-1,+1,-0.02,2];
axes_info          = [0,0,1,1];
view_info          = [65,12]; % [85,12];

% Run Common-rigging
for m_idx = 1:length(mocap_idxs) % for different mocaps
    mocap_idx  = mocap_idxs(m_idx);
    mocap_name = [mocap_type,'_',info(mocap_idx).name];
    L          = info(mocap_idx).L;
    mat_path   = info(mocap_idx).mat_path;
    fprintf('[%d/%d] name:[%s] L:[%d] \n',m_idx,length(mocap_idxs),mocap_name,L);

    % 1. Outlier filtering of MHFormer results and construct chains
    l           = load(mat_path);
    HZ          = double(l.fps{1});
    data        = l.data; % [L x 17 x 3] where 17 is #joint
    joint_names = l.joint_names;
    secs        = linspace(0,(L-1)/HZ,L)';
    ca; % close all
    data_smt = filter_mhformer_data_with_caching(mocap_name,data,joint_names,secs,...
        'RE',~SKIP_IF_MAT_EXIST,'cache_folder','../cache',...
        'joint_names2examine',{'RWrist','LWrist','RShoulder','LShoulder'},...
        'joint_vel_threshold',3.0,'hyp',[1,0.2],'meas_noise_std',1e-4,'max_iter',1000,...
        'PLOT_EACH_ITER',0,'PLOT_FINAL_RES',0,'VERBOSE',1);

    % Get chains
    chains_org = get_chains_from_mhformer_data(data,joint_names);
    chains_smt = get_chains_from_mhformer_data(data_smt,joint_names);

    % Animate original and filtered motion
    ca; % close all
    animate_org_and_smt_mhformer_results(secs,chains_org,chains_smt,mocap_name,...
        'folder_path','../vid/mocap','view_info',[50,21],...
        'PLOT_JOI_TRAJ',1,'SAVE_VID',1,'SKIP_IF_MP4_EXIST',SKIP_IF_VID_EXISTS);

    % 2. Pre-rigging
    ca; % close all
    chains = chains_smt; % use smoothed
    pre_rigging(secs,chains,mocap_name,...
        'mocap_type',mocap_type,'IK_FROM_ZERO_POSE',1,'MIX_WITH_PREV_Q',1,...
        'PLOT_EACH_TICK',0,'PLOT_IK_INSIDE',0,...
        'SAVE_MAT',1,'SKIP_IF_MAT_EXIST',SKIP_IF_MAT_EXIST,...
        'data_folder','../data/pre_rig','max_sec',inf,'max_ik_tick',100);

    % 3. Post-rigging
    mat_path    = sprintf('../data/pre_rig/%s.mat',mocap_name);
    l           = load(mat_path);
    secs        = l.secs;
    chains      = l.chains;
    chain_rig   = l.chain_rig;
    T_roots_pre = l.T_roots_pre;
    q_revs_pre  = l.q_revs_pre;
    ca; % close all
    post_rigging(secs,chains,chain_rig,T_roots_pre,q_revs_pre,mocap_name,...
        'folder_path','../data/post_rig','SMOOTH_TRAJ',0,... % no smoothing
        'root_yaw_range_rad',[-30,+30]*D2R,...
        'PLOT_EACH_TICK',0,'PLOT_IK_INSIDE',0,...
        'SAVE_MAT',1,'SKIP_IF_MAT_EXIST',SKIP_IF_MAT_EXIST,...
        'max_ik_tick',50,'knee_unbend_rate',0.5);

    % Animate pre- and post-rigging results
    mat_path = sprintf('../data/post_rig/%s.mat',mocap_name); l = load(mat_path);
    ca; % close all
    T_roots_pre     = l.T_roots_pre;
    q_revs_pre      = l.q_revs_pre;
    T_roots_upright = l.T_roots_upright;
    q_revs_upright  = l.q_revs_upright;
    T_roots_post    = l.T_roots_post;
    q_revs_post     = l.q_revs_post;
    animate_post_rigging_results(mocap_name,secs,chains,chain_rig,...
        T_roots_pre,q_revs_pre,T_roots_upright,q_revs_upright,...
        T_roots_post,q_revs_post,'folder_path','../vid/post_rig',...
        'SMOOTH_TRAJ',0,'PLOT_EACH_TICK',1,...
        'SAVE_VID',1,'SKIP_IF_MP4_EXIST',SKIP_IF_VID_EXISTS,...
        'fig_w',fig_w,'fig_h',fig_h,'AXIS_OFF',AXIS_OFF,'SET_AXISLABEL',SET_AXISLABEL,...
        'axis_info',axis_info,'axes_info',axes_info,'view_info',view_info);

    % 4. Smoothing and collision-handling
    ca; % close all
    T_roots             = T_roots_post;
    q_revs              = q_revs_post;
    joint_names_to_ctrl = get_rev_joint_names_route_to_jois(chain_rig,{'rh','lh'});
    smoothing_and_collision_handling(chain_rig,secs,T_roots,q_revs,...
        'data_folder_path','../data/post_rig_cf','robot_name','','mocap_name',mocap_name,...
        'joint_names_to_ctrl',joint_names_to_ctrl,...
        'PLOT_CHAIN_ZERO_POSE',0,'ANIMATE_SC_CHECK',0,'ANIMATE_SC_HANDLING',0,...
        'PLOT_CH_SMT_TRAJ',0,'VERBOSE',1,'SAVE_MAT',1,'SKIP_IF_MAT_EXIST',SKIP_IF_MAT_EXIST,...
        'smoothing_method','grp','hyp_mu',[1,0.2],'meas_noise_std',1e-2,....
        'sc_checks_margin_offset',0.1); % 10cm margin

    % Animate smooting+collision handling results
    mat_path = sprintf('../data/post_rig_cf/%s.mat',mocap_name); l = load(mat_path);
    T_roots_cf = l.T_roots_cf;
    q_revs_cf  = l.q_revs_cf;
    ca; % close all
    collision_margin = 0.10; % 5cm collision margin
    chain_rig.sc_checks = get_sc_checks(chain_rig,...
        'collision_margin',collision_margin,'UPDATE_WITH_ZERO_POSE',1);
    animate_sch_results(mocap_name,secs,chain_rig,T_roots,q_revs,T_roots_cf,q_revs_cf,...
        'folder_path','../vid/post_rig_cf','PLOT_EACH_TICK',1,...
        'SAVE_VID',1,'SKIP_IF_VID_EXISTS',SKIP_IF_VID_EXISTS,...
        'fig_w',fig_w,'fig_h',fig_h,'AXIS_OFF',AXIS_OFF,'SET_AXISLABEL',SET_AXISLABEL,...
        'axis_info',axis_info,'axes_info',axes_info,'view_info',view_info);

    % 5. Run robot motion retargeting
    ca;
    d = dir_compact(sprintf('../data/post_rig_cf/%s_%s.mat',mocap_type,motion_name),'VERBOSE',1);
    n_robot = length(robot_names);
    for robot_idx = 1:n_robot % for different robots
        robot_name  = robot_names{robot_idx};
        chain_robot = get_chain(robot_name,'T_POSE',1,'RE',0);
        l           = load([d(1).folder,'/',d(1).name]);
        secs        = l.secs;
        chain_rig   = l.chain;
        T_roots_rig = l.T_roots_cf;
        q_revs_rig  = l.q_revs_cf;
        chain_rig   = get_common_rig_from_mocap(chain_rig,...
            'ADD_ELBOW_GUIDE',1,'ADD_SHOULDER_GUIDE',0);
        mocap_name  = strrep(d(1).name,'.mat','');

        % Motion retargeting
        ca; % close all
        fprintf('[%d/%d][%d/%d] mocap:[%s] robot:[%s] .\n',...
            m_idx,length(mocap_idxs),robot_idx,n_robot,mocap_name,robot_name);
        mr(chain_robot,secs,chain_rig,T_roots_rig,q_revs_rig,mocap_name,...
            'data_folder_path','../data/mr',...
            'PLOT_INITITAL_T_POSE_QUAT_OFFSET',0,'APPLY_ROOT_TO_NECK_OFFSET',1,...
            'GIMBAL_LOCK_HEURISTICS',1,'gimbal_threshold',0.05,'VERBOSE',1,...
            'PLOT_IK_INSIDE',0,'PLOT_EACH_TICK',0,...
            'SKIP_IF_MAT_EXIST',SKIP_IF_MAT_EXIST,'SAVE_MAT',1,...
            'max_ik_tick',50);

        % Playback motion retargeting results
        ca; % close all
        mat_name = sprintf('../data/mr/%s_%s.mat',robot_name,mocap_name);
        l = load(mat_name);
        mocap_name    = l.mocap_name;
        robot_name    = l.robot_name;
        secs          = l.secs;
        chain_rig     = l.chain_rig;
        T_roots_rig   = l.T_roots_rig;
        q_revs_rig    = l.q_revs_rig;
        chain_robot   = l.chain_robot;
        T_roots_robot = l.T_roots_robot;
        q_revs_robot  = l.q_revs_robot;
        playback_mr_results(mocap_name,robot_name,...
            secs,chain_rig,T_roots_rig,q_revs_rig,chain_robot,T_roots_robot,q_revs_robot,...
            'vid_folder_path','../vid/mr','SAVE_VID',1,'SKIP_IF_VID_EXISTS',SKIP_IF_VID_EXISTS,...
            'fig_w',fig_w,'fig_h',fig_h,'AXIS_OFF',AXIS_OFF,'SET_AXISLABEL',SET_AXISLABEL,...
            'axis_info',axis_info,'axes_info',axes_info,'view_info',view_info);

        % Fine-tuning of wrist trajectories
        ft(chain_robot,secs,chain_rig,T_roots_robot,q_revs_robot,T_roots_rig,q_revs_rig,...
            mocap_name,'data_folder_path','../data/mr_ft','PLOT_TRAJ_AFFINE',0,...
            'GIMBAL_LOCK_HEURISTICS',1,'gimbal_threshold',0.05,'VERBOSE',1,...
            'PLOT_IK_INSIDE',0,'PLOT_EACH_TICK',0,...
            'SKIP_IF_MAT_EXIST',SKIP_IF_MAT_EXIST,'SAVE_MAT',1,...
            'max_ik_tick',50);

        % Smoothing and collision handling of motion retargㅏeting results
        ca; % close all
        mat_name            = sprintf('../data/mr_ft/%s_%s.mat',robot_name,mocap_name);
        l                   = load(mat_name);
        chain_robot         = l.chain_robot;
        secs                = l.secs;
        T_roots_robot_ft    = l.T_roots_robot_ft;
        q_revs_robot_ft     = l.q_revs_robot_ft;
        joint_names_to_ctrl = get_rev_joint_names_route_to_jois(chain_robot,{'rh','lh'});
        smoothing_and_collision_handling(chain_robot,secs,T_roots_robot_ft,q_revs_robot_ft,...
            'data_folder_path','../data/mr_ft_cf',...
            'robot_name',robot_name,'mocap_name',mocap_name,...
            'joint_names_to_ctrl',joint_names_to_ctrl,...
            'PLOT_CHAIN_ZERO_POSE',0,'ANIMATE_SC_CHECK',0,'ANIMATE_SC_HANDLING',0,...
            'PLOT_CH_SMT_TRAJ',0,'VERBOSE',1,'SAVE_MAT',1,'SKIP_IF_MAT_EXIST',SKIP_IF_MAT_EXIST,...
            'smoothing_method','grp','hyp_mu',[1,0.25],'meas_noise_std',1e-2,....
            'sc_checks_margin_offset',0.1); % 10cm margin

        % Playback collision-free motion retargeting results
        ca; % close all
        chain_name  = sprintf('%s_%s',robot_name,mocap_name);
        mat_name    = sprintf('../data/mr_ft_cf/%s.mat',chain_name);
        l           = load(mat_name);
        secs        = l.secs;
        chain_robot = l.chain;
        T_roots     = l.T_roots;
        q_revs      = l.q_revs;
        T_roots_cf  = l.T_roots_cf;
        q_revs_cf   = l.q_revs_cf;
        animate_sch_results(chain_name,secs,chain_robot,T_roots,q_revs,T_roots_cf,q_revs_cf,...
            'mfa',0.5,'cfa',0.05,'folder_path','../vid/mr_ft_cf','PLOT_EACH_TICK',1,...
            'SAVE_VID',1,'SKIP_IF_VID_EXISTS',SKIP_IF_VID_EXISTS,...
            'fig_w',fig_w,'fig_h',fig_h,'AXIS_OFF',AXIS_OFF,'SET_AXISLABEL',SET_AXISLABEL,...
            'axis_info',axis_info,'axes_info',axes_info,'view_info',view_info);

    end % for robot_idx = 1:n_robot % for different robots

    % Playback mocap with motion retargeted robot motions
    % Plot configuration
    fig_w_result = 0.2;
    joi_traj_org = get_joi_traj(chains_org); % mocap JOI trajectories
    chain_robots = cell(1,n_robot);
    T_roots_cfs  = cell(1,n_robot);
    q_revs_cfs   = cell(1,n_robot);
    for robot_idx = 1:n_robot % for different robots
        % Motion retargeting results
        robot_name  = robot_names{robot_idx};
        chain_name  = sprintf('%s_%s',robot_name,mocap_name);
        mat_name    = sprintf('../data/mr_ft_cf/%s.mat',chain_name);
        l           = load(mat_name);
        secs        = l.secs;
        chain_robot = l.chain;
        T_roots     = l.T_roots;
        q_revs      = l.q_revs;
        T_roots_cf  = l.T_roots_cf;
        q_revs_cf   = l.q_revs_cf;
        % Append
        chain_robots{robot_idx} = chain_robot;
        T_roots_cfs{robot_idx}  = T_roots_cf;
        q_revs_cfs{robot_idx}   = q_revs_cf;
    end

    % Animate
    ca;
    vid_path = sprintf('../vid/demo/mr_%s.mp4',mocap_name);
    vobj = init_vid_record(vid_path,'HZ',HZ,'SAVE_VID',1);
    for tick = 1:L % for each tick
        % Animate mocap
        sec     = secs(tick);
        chain   = chains_org{tick};
        T_torso = get_t_torso_mhformer(chain);
        fig_idx = 1;
        fig = plot_chain(chain,'fig_idx',fig_idx,'fig_pos',[0.0,0.4,0.9,0.55],...
            'view_info',[88,6],'axis_info',[-1.0,1.0,-1.0,5.0,-0.05,2.0],...
            'axes_info',[0,0,1,0.92],'AXIS_OFF',1,...
            'PLOT_JOINT_SPHERE',0,'jsr',0.025,'jsfa',0.1,'PLOT_JOINT_NAME',1);
        plot_T(T_torso,'fig_idx',fig_idx,'subfig_idx',1,'all',0.3,'alw',3,'PLOT_AXIS_TIP',1);
        plot_traj(joi_traj_org.rw,'fig_idx',fig_idx,'subfig_idx',1,'tlc','r','tlw',1/3);
        plot_traj(joi_traj_org.lw,'fig_idx',fig_idx,'subfig_idx',2,'tlc','b','tlw',1/3);
        plot_title(sprintf('[%d/%d][%.2f]s [%s]',tick,L,sec,mocap_name),...
            'fig_idx',fig_idx,'tfs',25);
        
        % Animate robots
        y_offset = 0.0;
        for robot_idx = 1:n_robot
            chain_robot = chain_robots{robot_idx};
            q_revs_cf   = q_revs_cfs{robot_idx};
            T_roots_cf  = T_roots_cfs{robot_idx};
            q_rev_cf    = q_revs_cf(tick,:);
            T_root_cf   = T_roots_cf{tick};
            chain_robot = update_chain_q_root_T(chain_robot,q_rev_cf,T_root_cf);
            % Move
            idx_top = get_topmost_idx(chain_robot);
            y_offset = y_offset + 1.0;
            chain_robot = move_chain(chain_robot,chain_robot.joint(idx_top).p + ...
                cv([0.0,y_offset,0.0]));
            % Plot each robot
            fig_idx = 1; subfig_idx = 1+robot_idx;
            plot_chain(chain_robot,'fig_idx',fig_idx,'subfig_idx',subfig_idx,...
                'PLOT_LINK',0,'PLOT_JOINT_SPHERE',0,'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_NAME',0,...
                'DISREGARD_JOI_GUIDE',1);
        end
        drawnow; record_vid(vobj,'fig',fig);
    end % for tick = 1:L % for each tick
    end_vid_record(vobj);

end % for m_idx = 1:length(mocap_idxs) % for different mocaps

%%