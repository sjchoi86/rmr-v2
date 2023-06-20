addpath_yart
%% Motion Retargeting of Post-Rigging Results
ccc
% Check existing mocaps (post-rigging results)
d = dir_compact('../data/post_rig_cf/*.mat','VERBOSE',1);
robot_names = {'ambidex','atlas','coman','thormang'};

% robot_names = {'ambidex'}; 

n_mocap = length(d); n_robot = length(robot_names);
mocap_names = cell(1,n_mocap);
for m_idx = 1:n_mocap
    mocap_names{m_idx} = strrep(d(m_idx).name,'.mat','');
end
fprintf('n_mocap:[%d], n_robot:[%d]\n',n_mocap,n_robot);
% Loop
mocap_idxs = 1:n_mocap;
for m_idx = 1:length(mocap_idxs) % for different mocap
    mocap_idx = mocap_idxs(m_idx);
    for robot_idx = 1:n_robot % for different robots
        % Load robot and common-rigging results
        robot_name  = robot_names{robot_idx};
        chain_robot = get_chain(robot_name,'T_POSE',1,'RE',0);
        l           = load([d(mocap_idx).folder,'/',d(mocap_idx).name]);
        secs = l.secs; chain_rig = l.chain; T_roots_rig = l.T_roots_cf; q_revs_rig = l.q_revs_cf;
        chain_rig = get_common_rig_from_mocap(chain_rig,'ADD_ELBOW_GUIDE',1,'ADD_SHOULDER_GUIDE',0);
        mocap_name = strrep(d(mocap_idx).name,'.mat','');

        % Motion retargeting starts here
        ca; % close all
        fprintf('[%d/%d][%d/%d] mocap:[%s] robot:[%s] .\n',...
            m_idx,length(mocap_idxs),robot_idx,n_robot,mocap_name,robot_name);
        mr(chain_robot,secs,chain_rig,T_roots_rig,q_revs_rig,mocap_name,...
            'data_folder_path','../data/mr',...
            'PLOT_INITITAL_T_POSE_QUAT_OFFSET',0,'APPLY_ROOT_TO_NECK_OFFSET',1,...
            'GIMBAL_LOCK_HEURISTICS',1,'gimbal_threshold',0.05,'VERBOSE',1,...
            'PLOT_IK_INSIDE',0,'PLOT_EACH_TICK',0,'SKIP_IF_MAT_EXIST',1,'SAVE_MAT',1,...
            'max_ik_tick',100);
        
        % Playback motion retargeting results
        ca; % close all
        mat_name = sprintf('../data/mr/%s_%s.mat',robot_name,mocap_name);
        l = load(mat_name);
        mocap_name = l.mocap_name; robot_name = l.robot_name; secs = l.secs;
        chain_rig = l.chain_rig; T_roots_rig = l.T_roots_rig; q_revs_rig = l.q_revs_rig;
        chain_robot = l.chain_robot; T_roots_robot = l.T_roots_robot; q_revs_robot = l.q_revs_robot;
        playback_mr_results(mocap_name,robot_name,...
            secs,chain_rig,T_roots_rig,q_revs_rig,chain_robot,T_roots_robot,q_revs_robot,...
            'vid_folder_path','../vid/mr','SAVE_VID',1,'SKIP_IF_VID_EXISTS',1,'fig_h',0.32);

        % Fine-tuning of wrist trajectories
        ft(chain_robot,secs,chain_rig,T_roots_robot,q_revs_robot,T_roots_rig,q_revs_rig,...
            mocap_name,'data_folder_path','../data/mr_ft','PLOT_TRAJ_AFFINE',0,...
            'GIMBAL_LOCK_HEURISTICS',1,'gimbal_threshold',0.05,'VERBOSE',1,...
            'PLOT_IK_INSIDE',0,'PLOT_EACH_TICK',0,'SKIP_IF_MAT_EXIST',1,'SAVE_MAT',1,...
            'max_ik_tick',100);

        % Smoothing and collision handling of motion retargㅏeting results
        ca; % close all
        mat_name = sprintf('../data/mr_ft/%s_%s.mat',robot_name,mocap_name);
        l = load(mat_name);
        chain_robot = l.chain_robot; secs = l.secs; 
        T_roots_robot_ft = l.T_roots_robot_ft; q_revs_robot_ft = l.q_revs_robot_ft;
        joint_names_to_ctrl = get_rev_joint_names_route_to_jois(chain_robot,{'rh','lh'});
        smoothing_and_collision_handling(chain_robot,secs,T_roots_robot_ft,q_revs_robot_ft,...
            'data_folder_path','../data/mr_ft_cf',...
            'robot_name',robot_name,'mocap_name',mocap_name,...
            'joint_names_to_ctrl',joint_names_to_ctrl,...
            'PLOT_CHAIN_ZERO_POSE',0,'ANIMATE_SC_CHECK',0,'ANIMATE_SC_HANDLING',0,...
            'PLOT_CH_SMT_TRAJ',0,'VERBOSE',1,'SAVE_MAT',1,'SKIP_IF_MAT_EXIST',1,...
            'smoothing_method','grp','hyp_mu',[1,0.25],'meas_noise_std',1e-2,....
            'sc_checks_margin_offset',0.1); % 10cm margin

        % Playback collision-free motion retargeting results
        ca; % close all
        chain_name = sprintf('%s_%s',robot_name,mocap_name);
        mat_name = sprintf('../data/mr_ft_cf/%s.mat',chain_name);
        l = load(mat_name); secs = l.secs; chain_robot = l.chain;
        T_roots = l.T_roots; q_revs = l.q_revs; T_roots_cf = l.T_roots_cf; q_revs_cf = l.q_revs_cf;
        axis_info = [-1.2,1.2,-1.2,1.2,0,2.2]; view_info = [80,12]; AXIS_OFF = 1;
        animate_sch_results(chain_name,secs,chain_robot,T_roots,q_revs,T_roots_cf,q_revs_cf,...
            'mfa',0.5,'cfa',0.05,'axis_info',axis_info,'view_info',view_info,...
            'AXIS_OFF',AXIS_OFF,'folder_path','../vid/mr_ft_cf','PLOT_EACH_TICK',1,...
            'SAVE_VID',1,'SKIP_IF_MP4_EXIST',1);
        
    end
end

%%