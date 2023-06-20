addpath_yart
%% Parse CMU-mocap data
ccc
% Plot configuration
SKIP_IF_VID_EXISTS = 0;
fig_w              = 0.15;
fig_h              = 0.3;
AXIS_OFF           = 1;
SET_AXISLABEL      = 0;
axis_info          = [-1,+1,-1,+1,-0.02,2];
axes_info          = [0,0,1,1];
view_info          = [65,12];

% Loop
mocap_type = 'cmu';
bvh_folder = '../../yet-another-robotics-toolbox/bvh/cmu-mocap';
[~,mocap_names,mocap_infos] = printout_cmu_mocap_info(...
    'bvh_folder',bvh_folder,'PRINTOUT',1,'USE_FIRST_WORD',0);
mocap_idxs = [436]; % 436:dog / 437:cat / 438:fish / 2413:idle
for m_idx = 1:length(mocap_idxs) % for different mocap files
    mocap_idx  = mocap_idxs(m_idx);
    mocap_name = [mocap_type,'_',strrep(mocap_names{mocap_idx},' ','_')];
    mocap_info = mocap_infos(mocap_idx);
    bvh_path   = mocap_info.full_path;
    fprintf('[%d/%d] name:[%s] type:[%s] path:[%s] \n',...
        m_idx,length(mocap_idxs),mocap_name,mocap_type,bvh_path);

    % 1. Get chains from bvf format
    [secs,chains,secs_org,chains_org] = ...
        get_chains_with_joi_from_bvh_with_caching(mocap_name,bvh_path,...
        'RE',0,'cache_folder','../cache','mocap_type',mocap_type,'sec_max',10,'HZ_intp',10);

    % Animate parsed chains (parsed from bvh)
    ca; % close all
    animate_chains(secs,chains,'mocap_name',mocap_name,'PLOT_WRIST_TRAJ',1,...
        'SAVE_VID',1,'SKIP_IF_VID_EXISTS',1);

    % 2. Pre-rigging
    ca; % close all
    pre_rigging(secs,chains,mocap_name,...
        'IK_FROM_ZERO_POSE',1,'MIX_WITH_PREV_Q',1,...
        'PLOT_EACH_TICK',0,'PLOT_IK_INSIDE',0,'SAVE_MAT',1,'SKIP_IF_MAT_EXIST',1,...
        'data_folder','../data/pre_rig','max_sec',inf,'max_ik_tick',100);

    % 3. Post-rigging
    mat_path = sprintf('../data/pre_rig/%s.mat',mocap_name); l = load(mat_path);
    secs = l.secs; chains = l.chains; chain_rig = l.chain_rig;
    T_roots_pre = l.T_roots_pre; q_revs_pre = l.q_revs_pre;
    ca; % close all
    post_rigging(secs,chains,chain_rig,T_roots_pre,q_revs_pre,mocap_name,...
        'folder_path','../data/post_rig','SMOOTH_TRAJ',0,... % no smoothing
        'root_yaw_range_rad',[-30,+30]*D2R,...
        'PLOT_EACH_TICK',0,'PLOT_IK_INSIDE',0,'SAVE_MAT',1,'SKIP_IF_MAT_EXIST',1,...
        'max_ik_tick',50,'knee_unbend_rate',0.5);

    % Animate pre- and post-rigging results
    mat_path = sprintf('../data/post_rig/%s.mat',mocap_name); l = load(mat_path);
    ca; % close all
    T_roots_pre = l.T_roots_pre; q_revs_pre = l.q_revs_pre;
    T_roots_upright = l.T_roots_upright; q_revs_upright = l.q_revs_upright;
    T_roots_post = l.T_roots_post; q_revs_post = l.q_revs_post;
    animate_post_rigging_results(mocap_name,secs,chains,chain_rig,...
        T_roots_pre,q_revs_pre,T_roots_upright,q_revs_upright,...
        T_roots_post,q_revs_post,'folder_path','../vid/post_rig',...
        'SMOOTH_TRAJ',0,'PLOT_EACH_TICK',1,'SAVE_VID',1,'SKIP_IF_MP4_EXIST',SKIP_IF_VID_EXISTS,...
        'fig_w',fig_w,'fig_h',fig_h,'AXIS_OFF',AXIS_OFF,'SET_AXISLABEL',SET_AXISLABEL,...
        'axis_info',axis_info,'axes_info',axes_info,'view_info',view_info);

    % 4. Smoothing and collision-handling
    ca; % close all
    T_roots = T_roots_post; q_revs = q_revs_post;
    joint_names_to_ctrl = get_rev_joint_names_route_to_jois(chain_rig,{'rh','lh'});
    joint_names_to_ctrl(idx_cell(joint_names_to_ctrl,'root1')) = [];
    joint_names_to_ctrl(idx_cell(joint_names_to_ctrl,'root2')) = [];
    joint_names_to_ctrl(idx_cell(joint_names_to_ctrl,'root3')) = [];
    smoothing_and_collision_handling(chain_rig,secs,T_roots,q_revs,...
        'data_folder_path','../data/post_rig_cf','robot_name','','mocap_name',mocap_name,...
        'joint_names_to_ctrl',joint_names_to_ctrl,...
        'PLOT_CHAIN_ZERO_POSE',0,'ANIMATE_SC_CHECK',0,'ANIMATE_SC_HANDLING',0,...
        'PLOT_CH_SMT_TRAJ',0,'VERBOSE',1,'SAVE_MAT',1,'SKIP_IF_MAT_EXIST',1,...
        'smoothing_method','grp','hyp_mu',[1,0.25],'meas_noise_std',1e-4,....
        'sc_checks_margin_offset',0.05);
    % Animate smooting+collision handling results
    mat_path = sprintf('../data/post_rig_cf/%s.mat',mocap_name); l = load(mat_path);
    T_roots_cf = l.T_roots_cf; q_revs_cf = l.q_revs_cf;
    ca; % close all
    chain_rig.sc_checks = get_sc_checks(chain_rig,'collision_margin',0,'UPDATE_WITH_ZERO_POSE',1);
    animate_sch_results(mocap_name,secs,chain_rig,T_roots,q_revs,T_roots_cf,q_revs_cf,...
        'folder_path','../vid/post_rig_cf','PLOT_EACH_TICK',1,...
        'SAVE_VID',1,'SKIP_IF_VID_EXISTS',SKIP_IF_VID_EXISTS,...
        'fig_w',fig_w,'fig_h',fig_h,'AXIS_OFF',AXIS_OFF,'SET_AXISLABEL',SET_AXISLABEL,...
        'axis_info',axis_info,'axes_info',axes_info,'view_info',view_info);
end

%% Parse Emotion-mocap data
ccc
% Plot configuration
SKIP_IF_VID_EXISTS = 0;
fig_w              = 0.15;
fig_h              = 0.3;
AXIS_OFF           = 1;
SET_AXISLABEL      = 0;
axis_info          = [-1,+1,-1,+1,-0.02,2];
axes_info          = [0,0,1,1];
view_info          = [65,12];

% Loop
mocap_type = 'emotion';
bvh_folder = '../../yet-another-robotics-toolbox/bvh/emotion-mocap';
[~,mocap_names,mocap_infos] = printout_emotion_mocap_info(...
    'bvh_folder',bvh_folder,'PRINTOUT',1);
mocap_idxs = [26]; 
for m_idx = 1:length(mocap_idxs)
    mocap_idx  = mocap_idxs(m_idx);
    mocap_name = [mocap_type,'_',strrep(mocap_names{mocap_idx},' ','_')];
    mocap_info = mocap_infos(mocap_idx);
    bvh_path   = mocap_info.full_path;
    fprintf('[%d/%d] name:[%s] type:[%s] path:[%s] \n',...
        m_idx,length(mocap_idxs),mocap_name,mocap_type,bvh_path);

    % 1. Get chains from bvf format
    [secs,chains,secs_org,chains_org] = ...
        get_chains_with_joi_from_bvh_with_caching(mocap_name,bvh_path,...
        'RE',0,'cache_folder','../cache','mocap_type',mocap_type,'sec_max',10,'HZ_intp',10);


    % Animate parsed chains (parsed from bvh)
    ca; % close all
    animate_chains(secs,chains,'mocap_name',mocap_name,'PLOT_WRIST_TRAJ',1,...
        'SAVE_VID',1,'SKIP_IF_VID_EXISTS',1);

    % 2. Pre-rigging
    ca; % close all
    pre_rigging(secs,chains,mocap_name,...
        'IK_FROM_ZERO_POSE',1,'MIX_WITH_PREV_Q',1,...
        'PLOT_EACH_TICK',0,'PLOT_IK_INSIDE',0,'SAVE_MAT',1,'SKIP_IF_MAT_EXIST',1,...
        'data_folder','../data/pre_rig','max_sec',inf,'max_ik_tick',100);

    % 3. Post-rigging
    mat_path = sprintf('../data/pre_rig/%s.mat',mocap_name); l = load(mat_path);
    secs = l.secs; chains = l.chains; chain_rig = l.chain_rig;
    T_roots_pre = l.T_roots_pre; q_revs_pre = l.q_revs_pre;
    ca; % close all
    post_rigging(secs,chains,chain_rig,T_roots_pre,q_revs_pre,mocap_name,...
        'folder_path','../data/post_rig','SMOOTH_TRAJ',0,... % no smoothing
        'root_yaw_range_rad',[-30,+30]*D2R,...
        'PLOT_EACH_TICK',0,'PLOT_IK_INSIDE',0,'SAVE_MAT',1,'SKIP_IF_MAT_EXIST',1,...
        'max_ik_tick',50,'knee_unbend_rate',0.5);

    % Animate pre- and post-rigging results
    mat_path = sprintf('../data/post_rig/%s.mat',mocap_name); l = load(mat_path);
    ca; % close all
    T_roots_pre = l.T_roots_pre; q_revs_pre = l.q_revs_pre;
    T_roots_upright = l.T_roots_upright; q_revs_upright = l.q_revs_upright;
    T_roots_post = l.T_roots_post; q_revs_post = l.q_revs_post;
    animate_post_rigging_results(mocap_name,secs,chains,chain_rig,...
        T_roots_pre,q_revs_pre,T_roots_upright,q_revs_upright,...
        T_roots_post,q_revs_post,'folder_path','../vid/post_rig',...
        'SMOOTH_TRAJ',0,'PLOT_EACH_TICK',1,'SAVE_VID',1,'SKIP_IF_MP4_EXIST',SKIP_IF_VID_EXISTS,...
        'fig_w',fig_w,'fig_h',fig_h,'AXIS_OFF',AXIS_OFF,'SET_AXISLABEL',SET_AXISLABEL,...
        'axis_info',axis_info,'axes_info',axes_info,'view_info',view_info);

    % 4. Smoothing and collision-handling
    ca; % close all
    T_roots = T_roots_post; q_revs = q_revs_post;
    joint_names_to_ctrl = get_rev_joint_names_route_to_jois(chain_rig,{'rh','lh'});
    smoothing_and_collision_handling(chain_rig,secs,T_roots,q_revs,...
        'data_folder_path','../data/post_rig_cf','robot_name','','mocap_name',mocap_name,...
        'joint_names_to_ctrl',joint_names_to_ctrl,...
        'PLOT_CHAIN_ZERO_POSE',0,'ANIMATE_SC_CHECK',0,'ANIMATE_SC_HANDLING',0,...
        'PLOT_CH_SMT_TRAJ',0,'VERBOSE',1,'SAVE_MAT',1,'SKIP_IF_MAT_EXIST',1,...
        'smoothing_method','grp','hyp_mu',[1,0.2],'meas_noise_std',1e-2,....
        'sc_checks_margin_offset',0.1);

    % Animate smooting+collision handling results
    mat_path = sprintf('../data/post_rig_cf/%s.mat',mocap_name); l = load(mat_path);
    T_roots_cf = l.T_roots_cf; q_revs_cf = l.q_revs_cf;
    ca; % close all
    chain_rig.sc_checks = get_sc_checks(chain_rig,'collision_margin',0,'UPDATE_WITH_ZERO_POSE',1);
    animate_sch_results(mocap_name,secs,chain_rig,T_roots,q_revs,T_roots_cf,q_revs_cf,...
        'folder_path','../vid/post_rig_cf','PLOT_EACH_TICK',1,...
        'SAVE_VID',1,'SKIP_IF_VID_EXISTS',SKIP_IF_VID_EXISTS,...
        'fig_w',fig_w,'fig_h',fig_h,'AXIS_OFF',AXIS_OFF,'SET_AXISLABEL',SET_AXISLABEL,...
        'axis_info',axis_info,'axes_info',axes_info,'view_info',view_info);
end

%% Parse MHFormer data
ccc
% Plot configuration
SKIP_IF_VID_EXISTS = 0;
fig_w              = 0.15;
fig_h              = 0.3;
AXIS_OFF           = 1;
SET_AXISLABEL      = 0;
axis_info          = [-1,+1,-1,+1,-0.02,2];
axes_info          = [0,0,1,1];
view_info          = [65,12]; % [85,12]; 

% Loop
mocap_type = 'mhformer';
info = get_mhformer_info('folder_path','../../mhformer_results/','VERBOSE',1);
mocap_names = cell(1,length(info));
for m_idx = 1:length(info)
    mocap_names{m_idx} = info(m_idx).name;
end
mocap_idxs = idxs_cell(mocap_names,{'annoyed_taemoon'});
for m_idx = 1:length(mocap_idxs)
    mocap_idx = mocap_idxs(m_idx);
    mocap_name = [mocap_type,'_',info(mocap_idx).name];
    L = info(mocap_idx).L;
    mat_path = info(mocap_idx).mat_path;
    fprintf('[%d/%d] name:[%s] L:[%d] \n',m_idx,length(mocap_idxs),mocap_name,L);

    % 1. Outlier filtering of MHFormer results and construct chains
    l           = load(mat_path); 
    HZ          = double(l.fps{1});
    data        = l.data; % [L x 17 x 3] where 17 is #joint
    joint_names = l.joint_names; 
    secs        = linspace(0,(L-1)/HZ,L)';
    ca; % close all
    data_smt = filter_mhformer_data_with_caching(mocap_name,data,joint_names,secs,...
        'RE',0,'cache_folder','../cache',...
        'joint_names2examine',{'RWrist','LWrist','RShoulder','LShoulder'},...
        'joint_vel_threshold',3.0,'hyp',[1,0.2],'meas_noise_std',1e-4,'max_iter',1000,...
        'PLOT_EACH_ITER',0,'PLOT_FINAL_RES',0,'VERBOSE',1);
    
    % Interpolate `data` and `data_smt` with `secs`?

    % Get chains
    chains_org = get_chains_from_mhformer_data(data,joint_names);
    chains_smt = get_chains_from_mhformer_data(data_smt,joint_names);

    % Animate original and filtered motion
    ca; % close all
    animate_org_and_smt_mhformer_results(secs,chains_org,chains_smt,mocap_name,...
        'folder_path','../vid/mocap','view_info',[50,21],...
        'PLOT_JOI_TRAJ',1,'SAVE_VID',1,'SKIP_IF_MP4_EXIST',1);
    
    % 2. Pre-rigging
    ca; % close all
    chains = chains_smt; % use smoothed
    pre_rigging(secs,chains,mocap_name,...
        'mocap_type',mocap_type,'IK_FROM_ZERO_POSE',1,'MIX_WITH_PREV_Q',1,...
        'PLOT_EACH_TICK',0,'PLOT_IK_INSIDE',0,'SAVE_MAT',1,'SKIP_IF_MAT_EXIST',1,...
        'data_folder','../data/pre_rig','max_sec',inf,'max_ik_tick',100);

    % 3. Post-rigging
    mat_path = sprintf('../data/pre_rig/%s.mat',mocap_name); l = load(mat_path);
    secs = l.secs; chains = l.chains; chain_rig = l.chain_rig;
    T_roots_pre = l.T_roots_pre; q_revs_pre = l.q_revs_pre;
    ca; % close all
    post_rigging(secs,chains,chain_rig,T_roots_pre,q_revs_pre,mocap_name,...
        'folder_path','../data/post_rig','SMOOTH_TRAJ',0,... % no smoothing
        'root_yaw_range_rad',[-30,+30]*D2R,...
        'PLOT_EACH_TICK',0,'PLOT_IK_INSIDE',0,'SAVE_MAT',1,'SKIP_IF_MAT_EXIST',1,...
        'max_ik_tick',50,'knee_unbend_rate',0.5);

    % Animate pre- and post-rigging results
    mat_path = sprintf('../data/post_rig/%s.mat',mocap_name); l = load(mat_path);
    ca; % close all
    T_roots_pre = l.T_roots_pre; q_revs_pre = l.q_revs_pre;
    T_roots_upright = l.T_roots_upright; q_revs_upright = l.q_revs_upright;
    T_roots_post = l.T_roots_post; q_revs_post = l.q_revs_post;
    animate_post_rigging_results(mocap_name,secs,chains,chain_rig,...
        T_roots_pre,q_revs_pre,T_roots_upright,q_revs_upright,...
        T_roots_post,q_revs_post,'folder_path','../vid/post_rig',...
        'SMOOTH_TRAJ',0,'PLOT_EACH_TICK',1,'SAVE_VID',1,'SKIP_IF_MP4_EXIST',SKIP_IF_VID_EXISTS,...
        'fig_w',fig_w,'fig_h',fig_h,'AXIS_OFF',AXIS_OFF,'SET_AXISLABEL',SET_AXISLABEL,...
        'axis_info',axis_info,'axes_info',axes_info,'view_info',view_info);

    % 4. Smoothing and collision-handling
    ca; % close all
    T_roots = T_roots_post; q_revs = q_revs_post;
    joint_names_to_ctrl = get_rev_joint_names_route_to_jois(chain_rig,{'rh','lh'});
    smoothing_and_collision_handling(chain_rig,secs,T_roots,q_revs,...
        'data_folder_path','../data/post_rig_cf','robot_name','','mocap_name',mocap_name,...
        'joint_names_to_ctrl',joint_names_to_ctrl,...
        'PLOT_CHAIN_ZERO_POSE',0,'ANIMATE_SC_CHECK',0,'ANIMATE_SC_HANDLING',0,...
        'PLOT_CH_SMT_TRAJ',0,'VERBOSE',1,'SAVE_MAT',1,'SKIP_IF_MAT_EXIST',1,...
        'smoothing_method','grp','hyp_mu',[1,0.2],'meas_noise_std',1e-2,....
        'sc_checks_margin_offset',0.1); % 10cm margin
    
    % Animate smooting+collision handling results
    mat_path = sprintf('../data/post_rig_cf/%s.mat',mocap_name); l = load(mat_path);
    T_roots_cf = l.T_roots_cf; q_revs_cf = l.q_revs_cf;
    ca; % close all
    collision_margin = 0.10; % 5cm collision margin
    chain_rig.sc_checks = get_sc_checks(chain_rig,...
        'collision_margin',collision_margin,'UPDATE_WITH_ZERO_POSE',1);
    animate_sch_results(mocap_name,secs,chain_rig,T_roots,q_revs,T_roots_cf,q_revs_cf,...
        'folder_path','../vid/post_rig_cf','PLOT_EACH_TICK',1,...
        'SAVE_VID',1,'SKIP_IF_VID_EXISTS',SKIP_IF_VID_EXISTS,...
        'fig_w',fig_w,'fig_h',fig_h,'AXIS_OFF',AXIS_OFF,'SET_AXISLABEL',SET_AXISLABEL,...
        'axis_info',axis_info,'axes_info',axes_info,'view_info',view_info);
end

%%


