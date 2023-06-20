function post_rigging(secs,chains,chain_rig,T_roots_pre,q_revs_pre,mocap_name,varargin)
%
% Run post-rigging
%

% Parse options
iP = inputParser;
addParameter(iP,'folder_path','../data/post_rig');
addParameter(iP,'SMOOTH_TRAJ',0);
addParameter(iP,'root_yaw_range_rad','');
addParameter(iP,'PLOT_EACH_TICK',1);
addParameter(iP,'PLOT_IK_INSIDE',0);
addParameter(iP,'SAVE_MAT',1);
addParameter(iP,'SKIP_IF_MAT_EXIST',1);
addParameter(iP,'max_ik_tick',100);
addParameter(iP,'knee_unbend_rate',0.0); % knee unbend rate
parse(iP,varargin{:});
folder_path         = iP.Results.folder_path;
SMOOTH_TRAJ         = iP.Results.SMOOTH_TRAJ;
root_yaw_range_rad  = iP.Results.root_yaw_range_rad;
PLOT_EACH_TICK      = iP.Results.PLOT_EACH_TICK;
PLOT_IK_INSIDE      = iP.Results.PLOT_IK_INSIDE;
SAVE_MAT            = iP.Results.SAVE_MAT;
SKIP_IF_MAT_EXIST   = iP.Results.SKIP_IF_MAT_EXIST;
max_ik_tick         = iP.Results.max_ik_tick;
knee_unbend_rate    = iP.Results.knee_unbend_rate;
D2R = pi/180;

% Skip if mat file exists
mat_path = sprintf('%s/%s.mat',folder_path,lower(strrep(mocap_name,' ','_')));
if exist(mat_path,'file') && SKIP_IF_MAT_EXIST
    fprintf(2,'[post_rigging] Skip as [%s] exists.\n',mat_path);
    return;
end
% Smooth
L = length(secs); HZ = round(L/(secs(end)-secs(1)));
if SMOOTH_TRAJ
    [q_revs_pre_smt,T_roots_pre_smt] = smooth_q_T(...
        secs,q_revs_pre,T_roots_pre,secs,'intp_type','linear_GRP',...
        'hyp_mu',[1,0.2],'hyp_var',[1,0.1],'meas_noise_std',1e-2,'PLOT_DEBUG',0);
else
    q_revs_pre_smt  = q_revs_pre;
    T_roots_pre_smt = T_roots_pre;
end
% Initialize common-rig
chain_rig = update_chain_q(chain_rig,chain_rig.rev_joint_names,zeros(1,chain_rig.n_rev_joint));
chain_rig_pre     = chain_rig;
chain_rig_upright = chain_rig;
chain_rig_post    = chain_rig;
root_idx_rig      = get_topmost_idx(chain_rig_pre);
% Loop
ca; % close all
% Buffers to save
q_revs_pre = []; T_roots_pre = cell(1,L); q_revs_upright = []; T_roots_upright = cell(1,L);
q_revs_post = []; T_roots_post = cell(1,L);
tk = init_tk('post-rigging');
for tick = 1:L % for each tick
    tk = print_tk(tk,tick,L,5);
    if tick == 1, RESET = 1; else, RESET = 0; end
    
    % Update mocap, pre-rigging, uprighting
    sec = secs(tick); chain_mocap = chains{tick};
    
%     mocap_type = strtok(mocap_name,'_');
%     switch mocap_type
%         case 'cmu'
%             % JOI already exists
%         case 'emotion'
%             chain_mocap = add_joi_to_emotion_mocap(chain_mocap);
%         case 'aihub'
%             chain_mocap = add_joi_to_aihub_mocap(chain_mocap);
%         case {'frank','mhformer'}
%             % JOI already exists
%         otherwise
%             fprintf(2,'[post_rigging] Unknown type:[%s].\n',mocap_type);
%     end
%     if ~isfield(chain_mocap,'joi')
%         chain_mocap.joi = get_joi_chain(chain_mocap);
%     end

    if ~isfield(chain_mocap,'joi')
        fprintf(2,'[post_rigging] JOI does not exist for [%s].\n',mocap_name);
        return;
    end

    T_joi_mocap = get_t_joi(chain_mocap,chain_mocap.joi);
    chain_mocap = move_chain_two_feet_on_ground(chain_mocap);
    
    % Update pre-rigging results
    q_rev_pre  = q_revs_pre_smt(tick,:);
    T_root_pre = T_roots_pre_smt{tick};
    chain_rig_pre.joint(root_idx_rig).p = t2p(T_root_pre);
    chain_rig_pre.joint(root_idx_rig).R = t2r(T_root_pre);
    chain_rig_pre = update_chain_q(...
        chain_rig_pre,chain_rig_pre.rev_joint_names,q_rev_pre,'FV',1,'RESET',RESET);
    chain_rig_pre = move_chain_two_feet_on_ground(chain_rig_pre);
    [com_pre,com_ground_pre,zmp_ground_pre] = get_com_zmp_ground(chain_rig_pre,'fig_idx',1);
    
    % Upright with torso yaw range
    chain_rig_upright = chain_rig_pre;
    if ~isempty(root_yaw_range_rad)
        chain_rig_upright = upright_chain_v2(chain_rig_upright,root_yaw_range_rad);
        yaw_rad = get_chain_root_yaw(chain_rig_upright);
        % fprintf('yaw_rad:[%.2f]deg \n',yaw_rad*180/pi);
    end

    % Move the chain's two feet on the ground and compute com and zmp
    chain_rig_upright = move_chain_two_feet_on_ground(chain_rig_upright);
    [com_upright,com_ground_upright,zmp_ground_upright] = ...
        get_com_zmp_ground(chain_rig_upright,'fig_idx',2);

    % a) Post-rigging process
    T_joi_upright   = get_t_joi(chain_rig_upright,chain_rig_upright.joi);
    p_torso_upright = t2p(T_joi_upright.torso);
    p_rp_upright    = t2p(T_joi_upright.rp);
    p_rk_upright    = t2p(T_joi_upright.rk);
    p_ra_upright    = t2p(T_joi_upright.ra);
    p_la_upright    = t2p(T_joi_upright.la);
    % Set right and left ankle positions (fix the width between two)
    p_ca_z = 0.5*(p_ra_upright(3)+p_la_upright(3)); % avg z
    p_ca = cv([p_torso_upright(1),p_torso_upright(2),p_ca_z]); % center of ankles to be torso
    if tick == 1
        rs2ls_dist = norm(t2p(T_joi_upright.rs)-t2p(T_joi_upright.ls));
    end
    p_offset_ca2ra = 0.5*rs2ls_dist*cv([0,-1,0]);
    p_offset_ca2la = 0.5*rs2ls_dist*cv([0,+1,0]);
    p_ra = p_ca + p_offset_ca2ra; 
    p_la = p_ca + p_offset_ca2la;
    [~,closest_pnt] = get_dist_point2line(com_ground_upright(1:2),p_ra(1:2),p_la(1:2));
    p_ca_diff = com_ground_upright(1:2) - closest_pnt(1:2);
    p_ca_post = p_ca; 
    p_ca_post(1:2) = p_ca(1:2) + p_ca_diff;
    p_ra_post = p_ca_post + p_offset_ca2ra;
    p_la_post = p_ca_post + p_offset_ca2la;
    % Set right and left knee positions (based on updated ankle positions)
    p_rk_post = p_ra_post; p_lk_post = p_la_post;
    p_rk_y = p_rk_post(2); p_lk_y = p_lk_post(2);
    p_ck_y = 0.5*(p_rk_y + p_lk_y); % center knee y
    rp2lp_dist = norm(t2p(T_joi_upright.rp)-t2p(T_joi_upright.lp));
    p_rk_post(2) = p_ck_y + sqrt(rp2lp_dist/rs2ls_dist)*(p_rk_y-p_ck_y);
    p_lk_post(2) = p_ck_y + sqrt(rp2lp_dist/rs2ls_dist)*(p_lk_y-p_ck_y);
    a2k_dist = norm(p_ra_upright-p_rk_upright); % ankle to knee distance
    k2p_dist = norm(p_rk_upright-p_rp_upright); % ankle to knee distance
    p_rk_post(3) = p_rk_post(3) + a2k_dist;
    p_lk_post(3) = p_lk_post(3) + a2k_dist;
    % Push knee x-positions
    p_rk_post(1) = p_rk_post(1) + 0.1;
    p_lk_post(1) = p_lk_post(1) + 0.1;
    % Avoid too much knee bending 
    a2p_dist   = a2k_dist + k2p_dist; % ankle to pelvis distance
    p2a_z_dist = p_rp_upright(3)-p_ra_post(3);
    z_diff4a   = a2p_dist - p2a_z_dist; % z-difference for ankle position
    torso_x    = p_torso_upright(1);
    torso_z    = p_torso_upright(3);

    % Modify IK targets of x- and z-positions of ankles to reflect 'knee_unbend_rate'
    p_ra_post(1) = (1-knee_unbend_rate)*p_ra_post(1) + knee_unbend_rate*torso_x;
    p_la_post(1) = (1-knee_unbend_rate)*p_la_post(1) + knee_unbend_rate*torso_x;
    p_ra_post(3) = p_ra_post(3) - knee_unbend_rate*z_diff4a;
    p_la_post(3) = p_la_post(3) - knee_unbend_rate*z_diff4a;
    % Modify IK target of x- and z-position of knees
    z_knee_unbend = 0.5*(torso_z + p_ra_post(3));
    p_rk_post(1) = (1-knee_unbend_rate)*p_rk_post(1) + knee_unbend_rate*torso_x;
    p_lk_post(1) = (1-knee_unbend_rate)*p_lk_post(1) + knee_unbend_rate*torso_x;
    p_rk_post(3) = (1-knee_unbend_rate)*p_rk_post(3) - knee_unbend_rate*z_knee_unbend;
    p_lk_post(3) = (1-knee_unbend_rate)*p_lk_post(3) - knee_unbend_rate*z_knee_unbend;


    % Set IK targets (knee and ankle positions and feet rotations)
    p_trgts.ra = p_ra_post; 
    p_trgts.la = p_la_post;
    p_trgts.rk = p_rk_post; 
    p_trgts.lk = p_lk_post;
    R_trgts.rf = [0,0,1; 1,0,0; 0,1,0]; 
    R_trgts.lf = [0,0,1; 1,0,0; 0,1,0];
    chain_rig_post.joint(root_idx_rig).p = chain_rig_upright.joint(root_idx_rig).p;
    chain_rig_post.joint(root_idx_rig).R = chain_rig_upright.joint(root_idx_rig).R;
    % Update the upper-body joints only
    rev_joint_upper_names = {'root1','root2','root3',...
        'rs1','rs2','rs3','re2','rw1','rw2','rw3',...
        'ls1','ls2','ls3','le2','lw1','lw2','lw3',...
        'spine','head1','head2','head3' ...
        };
    % Initialize with the joint position of uprighted rig
    q_rev_upright = get_q_chain(chain_rig_upright,rev_joint_upper_names);
    chain_rig_post = update_chain_q(...
        chain_rig_post,rev_joint_upper_names,q_rev_upright,'FV',1,'RESET',RESET);
    % IK loop
    ik_err_th = 1.0; dq_th = 10*D2R; step_size = 1.0; 
    ik_info_lower_p = get_ik_info_from_joi_targets(chain_rig_post,p_trgts,R_trgts,...
        'ik_err_th',ik_err_th,'dq_th',dq_th,'step_size',step_size,...
        'joi_p_trgts',{'ra','la','rk','lk'},'joi_R_trgts',{},...
        'joi_p_weights',[1,1,1,1]);
    ik_info_lower_R = get_ik_info_from_joi_targets(chain_rig_post,p_trgts,R_trgts,...
        'ik_err_th',ik_err_th,'dq_th',dq_th,'step_size',step_size,...
        'joi_R_trgts',{},'joi_R_trgts',{'rf','lf'},...
        'joi_R_weights',[1,1]);
    for ik_tick = 1:max_ik_tick % for each IK tick
        unit_dq_rad = 5*D2R*(1-ik_tick/max_ik_tick);
        % IK step for lower body positions
        DISREGARD_UNINFLUENTIAL_JOINT = 1;
        [dq,joint_names_to_ctrl,~,ik_err_lower_p,~] = one_step_ik(...
            chain_rig_post,ik_info_lower_p,'CONSIDER_JOINT_LIMIT',1,...
            'UNIT_DQ_HEURISTIC',1,'unit_dq_rad',unit_dq_rad,...
            'DISREGARD_UNINFLUENTIAL_JOINT',DISREGARD_UNINFLUENTIAL_JOINT);
        chain_rig_post = update_chain_q(chain_rig_post,joint_names_to_ctrl,...
            get_q_chain(chain_rig_post,joint_names_to_ctrl)+dq,'IGNORE_LIMIT',0);
        % IK step for lower body rotations
        [dq,joint_names_to_ctrl,~,ik_err_lower_R,~] = one_step_ik(...
            chain_rig_post,ik_info_lower_R,'CONSIDER_JOINT_LIMIT',1,...
            'UNIT_DQ_HEURISTIC',1,'unit_dq_rad',unit_dq_rad,...
            'DISREGARD_UNINFLUENTIAL_JOINT',DISREGARD_UNINFLUENTIAL_JOINT);
        chain_rig_post = update_chain_q(chain_rig_post,joint_names_to_ctrl,...
            get_q_chain(chain_rig_post,joint_names_to_ctrl)+dq,'IGNORE_LIMIT',0);
        ik_err = [ik_err_lower_p; ik_err_lower_R]; % concat errors
        err_rmse = sqrt(mean(ik_err.^2)); % IK error
        % Debug IK plot
        if PLOT_IK_INSIDE && ((ik_tick==1) || (mod(ik_tick,20)==0))
            fig_idx = 9; view_info = [90,5]; ral = chain_rig_post.sz.xyz_len(3)/20;
            plot_chain(chain_rig_post,'fig_idx',fig_idx,'fig_pos',[0.0,0.2,0.2,0.3],...
                'view_info',view_info,'PLOT_ROTATE_AXIS',1,'ral',ral);
            sr = chain_rig_post.sz.xyz_len(3)/30; sfa = 0.8;
            all = chain_rig_post.sz.xyz_len(3)/5;
            plot_ik_targets('chain_robot',chain_rig_post,...
                'ik_plot_info',get_ik_plot_info_from_ik_info(ik_info_lower_p),...
                'fig_idx',fig_idx,'subfig_idx',1,'sr',sr,'sfa',sfa,'all',all);
            plot_ik_targets('chain_robot',chain_rig_post,...
                'ik_plot_info',get_ik_plot_info_from_ik_info(ik_info_lower_R),...
                'fig_idx',fig_idx,'subfig_idx',2,'sr',sr,'sfa',sfa,'all',all);
            title_str = sprintf('IK:[%d/%d] err_rmse:[%.4f]',...
                ik_tick,max_ik_tick,err_rmse);
            plot_title(title_str,'fig_idx',fig_idx,'interpreter','latex');
            drawnow;
        end
    end % for ik_tick = 1:max_ik_tick % for each IK tick
    
    % b) Two ankle positions (and feet orientations) fine-tuning
    T_joi_temp = get_t_joi(chain_rig_post,chain_rig_post.joi);
    p_ra_temp  = t2p(T_joi_temp.ra);
    p_la_temp  = t2p(T_joi_temp.la);
    p_ca       = 0.5*(p_ra_temp+p_la_temp);
    p_ca_z     = 0.5*(p_ra_temp(3)+p_la_temp(3)); % avg z
    p_ca(3)    = p_ca_z;
    p_offset_ca2ra = 0.5*rs2ls_dist*cv([0,-1,0]);
    p_offset_ca2la = 0.5*rs2ls_dist*cv([0,+1,0]);
    p_ra       = p_ca + p_offset_ca2ra; 
    p_la       = p_ca + p_offset_ca2la;
    p_trgts.ra = p_ra;
    p_trgts.la = p_la;
    ik_info_lower_pR = get_ik_info_from_joi_targets(chain_rig_post,p_trgts,R_trgts,...
        'ik_err_th',ik_err_th,'dq_th',dq_th,'step_size',step_size,...
        'joi_p_trgts',{'ra','la'},'joi_R_trgts',{'rf','lf'},...
        'joi_p_weights',[1,1],'joi_R_weights',[1,1]);
    for ik_tick = 1:max_ik_tick % for each IK tick
        unit_dq_rad = 3*D2R*(1-ik_tick/max_ik_tick);
        % IK step for lower body positions
        UNIT_DQ_HEURISTIC = 0;
        DISREGARD_UNINFLUENTIAL_JOINT = 1;
        [dq,joint_names_to_ctrl,~,ik_err_lower_pR,~] = one_step_ik(...
            chain_rig_post,ik_info_lower_pR,'CONSIDER_JOINT_LIMIT',1,...
            'UNIT_DQ_HEURISTIC',UNIT_DQ_HEURISTIC,'unit_dq_rad',unit_dq_rad,...
            'DISREGARD_UNINFLUENTIAL_JOINT',DISREGARD_UNINFLUENTIAL_JOINT);
        chain_rig_post = update_chain_q(chain_rig_post,joint_names_to_ctrl,...
            get_q_chain(chain_rig_post,joint_names_to_ctrl)+dq,'IGNORE_LIMIT',0);
        ik_err = ik_err_lower_pR; % concat errors
        err_rmse = sqrt(mean(ik_err.^2)); % IK error
        % Debug IK plot
        if PLOT_IK_INSIDE && ((ik_tick==1) || (mod(ik_tick,20)==0))
            fig_idx = 10; view_info = [90,5]; ral = chain_rig_post.sz.xyz_len(3)/20;
            fig = plot_chain(chain_rig_post,'fig_idx',fig_idx,'fig_pos',[0.2,0.2,0.2,0.3],...
                'view_info',view_info,'PLOT_ROTATE_AXIS',1,'ral',ral);
            sr = chain_rig_post.sz.xyz_len(3)/30; sfa = 0.8;
            all = chain_rig_post.sz.xyz_len(3)/5;
            plot_ik_targets('chain_robot',chain_rig_post,...
                'ik_plot_info',get_ik_plot_info_from_ik_info(ik_info_lower_pR),...
                'fig_idx',fig_idx,'subfig_idx',1,'sr',sr,'sfa',sfa,'all',all);
            title_str = sprintf('IK:[%d/%d] err_rmse:[%.4f]',...
                ik_tick,max_ik_tick,err_rmse);
            plot_title(title_str,'fig_idx',fig_idx,'interpreter','latex');
            drawnow; pause_invalid_handle(fig);
        end
    end % for ik_tick = 1:max_ik_tick % for each IK tick
    
    % Move two feet on the center of the ground and compute ground projected com and zmp
    chain_rig_post = move_chain_two_feet_on_ground(chain_rig_post);
    [com_post,com_ground_post,zmp_ground_post] = ...
        get_com_zmp_ground(chain_rig_post,'fig_idx',3);

    % Append the root and joint position information
    T_roots_pre{tick}     = get_T_root_chain(chain_rig_pre);
    q_rev_pre             = get_q_chain(chain_rig_pre,chain_rig_pre.rev_joint_names);
    q_revs_pre            = cat(1,q_revs_pre,rv(q_rev_pre));
    T_roots_upright{tick} = get_T_root_chain(chain_rig_upright);
    q_rev_upright         = get_q_chain(chain_rig_upright,chain_rig_upright.rev_joint_names);
    q_revs_upright        = cat(1,q_revs_upright,rv(q_rev_upright));
    T_roots_post{tick}    = get_T_root_chain(chain_rig_post);
    q_rev_post            = get_q_chain(chain_rig_post,chain_rig_post.rev_joint_names);
    q_revs_post           = cat(1,q_revs_post,rv(q_rev_post));
    % Animate
    if PLOT_EACH_TICK
        % Figure1: Animate the mocap
        axis_info = [-1,+1,-1,+1,-0.1,2]; view_info = [80,11]; fig_h_rig = 0.3;
        fig_idx = 1; fig_pos = [0.0,0.7,0.15,fig_h_rig];
        fig1 = plot_chain(chain_mocap,'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
            'axis_info',axis_info,'view_info',view_info,...
            'PLOT_LINK',1,'PLOT_JOINT_AXIS',0,'jalw',3,'PLOT_JOINT_SPHERE',1,'jsr',0.025);
        title_str = sprintf('[%d/%d] MoCap Skeleton \n (%s)',tick,L,mocap_name); tfs = 15;
        plot_title(title_str,'fig_idx',fig_idx,'tfs',tfs,'interpreter','latex');
        % Figure2: Animate the pre-rigging results
        fig_idx = 2; fig_pos = [0.15,0.7,0.15,fig_h_rig];
        zmp_col = 'r'; com_col = 0.5*[1,1,1];
        fig2 = plot_chain(chain_rig_pre,...
            'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
            'SET_MATERIAL','DULL','axis_info',axis_info,'view_info',view_info,...
            'PLOT_LINK',0,'PLOT_JOINT_AXIS',0,'jalw',3,...
            'PLOT_JOINT_SPHERE',1,'jsr',0.01,'bafa',0.5,'PLOT_CAPSULE',1,'cfc','k','cfa',0.3);
        plot_T(p2t(com_ground_pre),'fig_idx',fig_idx,'subfig_idx',1,...
            'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.04,'sfc',com_col,'sfa',0.9);
        plot_T(p2t(zmp_ground_pre),'fig_idx',fig_idx,'subfig_idx',2,...
            'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.04,'sfc',zmp_col,'sfa',0.9);
        title_str = sprintf('[%d/%d] Pre-Rigging \n (%s)',tick,L,mocap_name);
        plot_title(title_str,'fig_idx',fig_idx,'tfs',tfs,'interpreter','latex');
        fig_idx = 5; fig_y_com = 0.45; fig_h_com = 0.19;
        fig_pos = [0.15,fig_y_com,0.15,fig_h_com];
        axis_info_com = [-0.5,+0.5,-1.0,+1.0,-inf,+inf];
        fig5 = plot_chain_feet_and_com(chain_rig_pre,...
            'fig_idx',fig_idx,'fig_pos',fig_pos,'title_str',title_str,'tfs',tfs,...
            'zmp_ground',zmp_ground_pre,'PLOT_ZMP',1,'axis_info',axis_info_com);
        % Figure3: Animate the uprighted pre-rigging results
        fig_idx = 3; fig_pos = [0.3,0.7,0.15,fig_h_rig];
        fig3 = plot_chain(chain_rig_upright,...
            'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
            'SET_MATERIAL','DULL','axis_info',axis_info,'view_info',view_info,...
            'PLOT_LINK',0,'PLOT_JOINT_AXIS',0,'jalw',3,...
            'PLOT_JOINT_SPHERE',1,'jsr',0.01,'bafa',0.5,'PLOT_CAPSULE',1,'cfc','k','cfa',0.3);
        plot_T(p2t(com_ground_upright),'fig_idx',fig_idx,'subfig_idx',1,...
            'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.04,'sfc',com_col,'sfa',0.9);
        plot_T(p2t(zmp_ground_upright),'fig_idx',fig_idx,'subfig_idx',2,...
            'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.04,'sfc',zmp_col,'sfa',0.9);
        title_str = sprintf('[%d/%d] Uprighting \n (%s)',tick,L,mocap_name);
        plot_title(title_str,'fig_idx',fig_idx,'tfs',tfs,'interpreter','latex');
        fig_idx = 6; fig_pos = [0.3,fig_y_com,0.15,fig_h_com];
        fig6 = plot_chain_feet_and_com(chain_rig_upright,...
            'fig_idx',fig_idx,'fig_pos',fig_pos,'title_str',title_str,'tfs',tfs,...
            'zmp_ground',zmp_ground_upright,'PLOT_ZMP',1,'axis_info',axis_info_com);
        % Figure4: Animate the post-rigging results
        fig_idx = 4; fig_pos = [0.45,0.7,0.15,fig_h_rig];
        fig4 = plot_chain(chain_rig_post,...
            'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
            'SET_MATERIAL','DULL','axis_info',axis_info,'view_info',view_info,...
            'PLOT_LINK',0,'PLOT_JOINT_AXIS',0,'jalw',3,...
            'PLOT_JOINT_SPHERE',1,'jsr',0.01,'bafa',0.5,'PLOT_CAPSULE',1,'cfc','k','cfa',0.3);
        plot_T(p2t(com_ground_post),'fig_idx',fig_idx,'subfig_idx',1,...
            'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.04,'sfc',com_col,'sfa',0.9);
        plot_T(p2t(zmp_ground_post),'fig_idx',fig_idx,'subfig_idx',2,...
            'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.04,'sfc',zmp_col,'sfa',0.9);
        title_str = sprintf('[%d/%d] Post-Rigging \n (%s)',tick,L,mocap_name);
        plot_title(title_str,'fig_idx',fig_idx,'tfs',tfs,'interpreter','latex');
        fig_idx = 7; fig_pos = [0.45,fig_y_com,0.15,fig_h_com];
        fig7 = plot_chain_feet_and_com(chain_rig_post,...
            'fig_idx',fig_idx,'fig_pos',fig_pos,'title_str',title_str,'tfs',tfs,...
            'zmp_ground',zmp_ground_post,'PLOT_ZMP',1,'axis_info',axis_info_com);
        drawnow; pause_invalid_handle([fig1,fig2,fig3,fig4,fig5,fig6,fig7]);
    end
end % for tick = 1:L % for each tick

% Save data
if SAVE_MAT
    [p,~,~] = fileparts(mat_path);
    make_dir_if_not_exist(p);
    save(mat_path,...
        'chains','secs',...
        'chain_rig_pre','T_roots_pre','q_revs_pre',...
        'chain_rig_upright','T_roots_upright','q_revs_upright',...
        'chain_rig_post','T_roots_post','q_revs_post' ...
        );
    fprintf(2,'[%s] saved.\n',mat_path);
end
