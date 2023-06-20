function animate_post_rigging_results(mocap_name,secs,chains,chain_rig,...
    T_roots_pre,q_revs_pre,T_roots_upright,q_revs_upright,T_roots_post,q_revs_post,varargin)
%
% Playback post-rigging results (pre-rigging + uprighting)
%

% Parse options
iP = inputParser;
addParameter(iP,'folder_path','../vid/postrig');
addParameter(iP,'T_parts','');
addParameter(iP,'SMOOTH_TRAJ',1);
addParameter(iP,'PLOT_EACH_TICK',1);
addParameter(iP,'SAVE_VID',1);
addParameter(iP,'SKIP_IF_MP4_EXIST',1);
addParameter(iP,'fig_w',0.2);
addParameter(iP,'fig_h',0.3);
addParameter(iP,'AXIS_OFF',1);
addParameter(iP,'SET_AXISLABEL',0);
addParameter(iP,'axis_info',[-1,+1,-1,+1,-0.1,2]);
addParameter(iP,'axes_info',[0,0,1,1]);
addParameter(iP,'view_info',[80,12]);
parse(iP,varargin{:});
folder_path        = iP.Results.folder_path;
T_parts            = iP.Results.T_parts;
SMOOTH_TRAJ        = iP.Results.SMOOTH_TRAJ;
PLOT_EACH_TICK     = iP.Results.PLOT_EACH_TICK;
SAVE_VID           = iP.Results.SAVE_VID;
SKIP_IF_MP4_EXIST  = iP.Results.SKIP_IF_MP4_EXIST;
fig_w              = iP.Results.fig_w;
fig_h              = iP.Results.fig_h;
AXIS_OFF           = iP.Results.AXIS_OFF;
SET_AXISLABEL      = iP.Results.SET_AXISLABEL;
axis_info          = iP.Results.axis_info;
axes_info          = iP.Results.axes_info;
view_info          = iP.Results.view_info;

% Rigs
chain_rig_pre     = chain_rig;
chain_rig_upright = chain_rig;
chain_rig_post    = chain_rig;

L = length(secs); HZ = round(L/(secs(end)-secs(1)));

% Smooth
if SMOOTH_TRAJ
    [q_revs_pre,T_roots_pre] = smooth_q_T(...
        secs,q_revs_pre,T_roots_pre,secs,'intp_type','linear_GRP',...
        'hyp_mu',[1,0.2],'hyp_var',[1,0.1],'meas_noise_std',1e-2,'PLOT_DEBUG',0);
    [q_revs_upright,T_roots_upright] = smooth_q_T(...
        secs,q_revs_upright,T_roots_upright,secs,'intp_type','linear_GRP',...
        'hyp_mu',[1,0.2],'hyp_var',[1,0.1],'meas_noise_std',1e-2,'PLOT_DEBUG',0);
    [q_revs_post,T_roots_post] = smooth_q_T(...
        secs,q_revs_post,T_roots_post,secs,'intp_type','linear_GRP',...
        'hyp_mu',[1,0.2],'hyp_var',[1,0.1],'meas_noise_std',1e-2,'PLOT_DEBUG',0);
end

% T parts
if ~isempty(T_parts)
    T_rhs   = T_parts{1};
    T_lhs   = T_parts{2};
    T_heads = T_parts{3};
    T_rfs   = T_parts{4};
    T_lfs   = T_parts{5};
end

% Video saver
vidpath1 = sprintf('%s/%s_1_mocap.mp4',folder_path,mocap_name);
vidpath2 = sprintf('%s/%s_2_pre.mp4',folder_path,mocap_name);
vidpath3 = sprintf('%s/%s_3_upright.mp4',folder_path,mocap_name);
vidpath4 = sprintf('%s/%s_4_post.mp4',folder_path,mocap_name);
vidpath5 = sprintf('%s/%s_5_pre_feet.mp4',folder_path,mocap_name);
vidpath6 = sprintf('%s/%s_6_upright_feet.mp4',folder_path,mocap_name);
vidpath7 = sprintf('%s/%s_7_post_feet.mp4',folder_path,mocap_name);
if exist(vidpath1,'file') && SKIP_IF_MP4_EXIST
    fprintf(2,'[animate_post_rigging_results] Skip as [%s] exists.\n',vidpath1);
    return;
end
vobj{1} = init_vid_record(vidpath1,'HZ',HZ,'SAVE_VID',SAVE_VID);
vobj{2} = init_vid_record(vidpath2,'HZ',HZ,'SAVE_VID',SAVE_VID);
vobj{3} = init_vid_record(vidpath3,'HZ',HZ,'SAVE_VID',SAVE_VID);
vobj{4} = init_vid_record(vidpath4,'HZ',HZ,'SAVE_VID',SAVE_VID);
vobj{5} = init_vid_record(vidpath5,'HZ',HZ,'SAVE_VID',SAVE_VID);
vobj{6} = init_vid_record(vidpath6,'HZ',HZ,'SAVE_VID',SAVE_VID);
vobj{7} = init_vid_record(vidpath7,'HZ',HZ,'SAVE_VID',SAVE_VID);

% Animate
for tick = 1:L % for each tick
    if tick == 1, RESET = 1; else, RESET = 0; end
    % Update mocap
    sec = secs(tick); chain_mocap = chains{tick};
    if ~isfield(chain_mocap,'joi')
        fprintf(2,'[animate_post_rigging_results] JOI does not exist. \n');
    end
    T_joi_mocap = get_t_joi(chain_mocap,chain_mocap.joi);
    [chain_mocap,p_diff] = move_chain_two_feet_on_ground(chain_mocap);
    % Body part transformation matrices
    if ~isempty(T_parts)
        T_rh = T_rhs{tick}; T_lh = T_lhs{tick};
        T_head = T_heads{tick};
        T_rf = T_rfs{tick}; T_lf = T_lfs{tick};
    end
    % Update rigs
    q_rev_pre = q_revs_pre(tick,:); T_root_pre = T_roots_pre{tick};
    chain_rig_pre = update_chain_q_root_T(...
        chain_rig_pre,q_rev_pre,T_root_pre,'FV',1,'RESET',RESET);
    chain_rig_pre = move_chain_two_feet_on_ground(chain_rig_pre);
    [com_pre,com_ground_pre,zmp_ground_pre] = get_com_zmp_ground(chain_rig_pre,'fig_idx',1);
    q_rev_upright = q_revs_upright(tick,:); T_root_upright = T_roots_upright{tick};
    chain_rig_upright = update_chain_q_root_T(...
        chain_rig_upright,q_rev_upright,T_root_upright,'FV',1,'RESET',RESET);
    chain_rig_upright = move_chain_two_feet_on_ground(chain_rig_upright);
    [com_upright,com_ground_upright,zmp_ground_upright] = ...
        get_com_zmp_ground(chain_rig_upright,'fig_idx',2);
    q_rev_post = q_revs_post(tick,:); T_root_post = T_roots_post{tick};
    chain_rig_post = update_chain_q_root_T(...
        chain_rig_post,q_rev_post,T_root_post,'FV',1,'RESET',RESET);
    chain_rig_post = move_chain_two_feet_on_ground(chain_rig_post);
    [com_post,com_ground_post,zmp_ground_post] = get_com_zmp_ground(chain_rig_post,'fig_idx',3);

    % Animate
    if PLOT_EACH_TICK % plot each tick

        % Plot configuration
        tfs     = 15;
        zmp_col = 'r'; 
        com_col = 0.5*[1,1,1];

        % Figure 1: mocap skeleton
        fig_idx = 1; fig_pos = [0.0,0.6,fig_w,fig_h];
        fig1 = plot_chain(chain_mocap,'fig_idx',fig_idx,'subfig_idx',1,...
            'fig_pos',fig_pos,'PLOT_LINK',1,'DISREGARD_JOI_GUIDE',1,...
            'PLOT_JOINT_AXIS',0,'jalw',3,'PLOT_JOINT_SPHERE',1,'jsr',0.025,...
            'AXIS_OFF',AXIS_OFF,'SET_AXISLABEL',SET_AXISLABEL,...
            'axis_info',axis_info,'axes_info',axes_info,'view_info',view_info);
        title_str = sprintf('[%d/%d][%.2f]sec MoCap Skeleton \n (%s)',tick,L,sec,mocap_name);
        if ~AXIS_OFF
            plot_title(title_str,'fig_idx',fig_idx,'tfs',tfs,'interpreter','latex');
        end

        % Figure 2: pre-rigging
        fig_idx = 2; fig_pos = [fig_w,0.6,fig_w,fig_h];
        fig2 = plot_chain(chain_rig_pre,...
            'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,'SET_MATERIAL','DULL',...
            'PLOT_LINK',0,'PLOT_JOINT_AXIS',0,'jalw',3,'PLOT_ROTATE_AXIS',0,...
            'PLOT_JOINT_SPHERE',0,'jsr',0.01,'bafa',0.5,'PLOT_CAPSULE',1,'cfc','k','cfa',0.3,...
            'DISREGARD_JOI_GUIDE',1,...
            'AXIS_OFF',AXIS_OFF,'SET_AXISLABEL',SET_AXISLABEL,...
            'axis_info',axis_info,'axes_info',axes_info,'view_info',view_info);
        plot_T(p2t(com_ground_pre),'fig_idx',fig_idx,'subfig_idx',1,...
            'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.04,'sfc',com_col,'sfa',0.9);
        plot_T(p2t(zmp_ground_pre),'fig_idx',fig_idx,'subfig_idx',2,...
            'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.04,'sfc',zmp_col,'sfa',0.9);
        title_str = sprintf('[%d/%d][%.2f]sec Pre-Rigging \n (%s)',tick,L,sec,mocap_name);
        if ~AXIS_OFF
            plot_title(title_str,'fig_idx',fig_idx,'tfs',tfs,'interpreter','latex');
        end

        % Figure 5: pre-rigging feet
        fig_idx = 5; fig_h_com = 0.15;
        fig_pos = [fig_w,0.55-fig_h_com,fig_w,fig_h_com];
        axis_info_com = [-0.5,+0.5,-1.0,+1.0,-inf,+inf];
        fig5 = plot_chain_feet_and_com(chain_rig_pre,...
            'fig_idx',fig_idx,'fig_pos',fig_pos,'title_str','','tfs',tfs,...
            'zmp_ground',zmp_ground_pre,'PLOT_ZMP',1,'axis_info',axis_info_com);

        % Figure 3: uprighting
        fig_idx = 3; fig_pos = [2*fig_w,0.6,fig_w,fig_h];
        fig3 = plot_chain(chain_rig_upright,...
            'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,'SET_MATERIAL','DULL',...
            'PLOT_LINK',0,'PLOT_JOINT_AXIS',0,'jalw',3,'PLOT_ROTATE_AXIS',0,...
            'PLOT_JOINT_SPHERE',0,'jsr',0.01,'bafa',0.5,'PLOT_CAPSULE',1,'cfc','k','cfa',0.3,...
            'DISREGARD_JOI_GUIDE',1,...
            'AXIS_OFF',AXIS_OFF,'SET_AXISLABEL',SET_AXISLABEL,...
            'axis_info',axis_info,'axes_info',axes_info,'view_info',view_info);
        plot_T(p2t(com_ground_upright),'fig_idx',fig_idx,'subfig_idx',1,...
            'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.04,'sfc',com_col,'sfa',0.9);
        plot_T(p2t(zmp_ground_upright),'fig_idx',fig_idx,'subfig_idx',2,...
            'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.04,'sfc',zmp_col,'sfa',0.9);
        title_str = sprintf('[%d/%d][%.2f]sec Uprighting \n (%s)',tick,L,sec,mocap_name);
        if ~AXIS_OFF
            plot_title(title_str,'fig_idx',fig_idx,'tfs',tfs,'interpreter','latex');
        end

        % Figure 6: uprighting feet
        fig_idx = 6; 
        fig_pos = [2*fig_w,0.55-fig_h_com,fig_w,fig_h_com];
        fig6 = plot_chain_feet_and_com(chain_rig_upright,...
            'fig_idx',fig_idx,'fig_pos',fig_pos,'title_str','','tfs',tfs,...
            'zmp_ground',zmp_ground_upright,'PLOT_ZMP',1,'axis_info',axis_info_com);

        % Figure 4: post-rigging
        fig_idx = 4; 
        fig_pos = [3*fig_w,0.6,fig_w,fig_h];
        fig4 = plot_chain(chain_rig_post,...
            'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,'SET_MATERIAL','DULL',...
            'PLOT_LINK',0,'PLOT_JOINT_AXIS',0,'jalw',3,'PLOT_ROTATE_AXIS',0,...
            'PLOT_JOINT_SPHERE',0,'jsr',0.01,'bafa',0.5,'PLOT_CAPSULE',1,'cfc','k','cfa',0.3,...
            'DISREGARD_JOI_GUIDE',1,...
            'AXIS_OFF',AXIS_OFF,'SET_AXISLABEL',SET_AXISLABEL,...
            'axis_info',axis_info,'axes_info',axes_info,'view_info',view_info);
        plot_T(p2t(com_ground_post),'fig_idx',fig_idx,'subfig_idx',1,...
            'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.04,'sfc',com_col,'sfa',0.9);
        plot_T(p2t(zmp_ground_post),'fig_idx',fig_idx,'subfig_idx',2,...
            'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.04,'sfc',zmp_col,'sfa',0.9);
        title_str = sprintf('[%d/%d][%.2f]sec Post-Rigging \n (%s)',tick,L,sec,mocap_name);
        if ~AXIS_OFF
            plot_title(title_str,'fig_idx',fig_idx,'tfs',tfs,'interpreter','latex');
        end

        % Figure 7: post-rigging feet
        fig_idx = 7; 
        fig_pos = [3*fig_w,0.55-fig_h_com,fig_w,fig_h_com];
        fig7 = plot_chain_feet_and_com(chain_rig_post,...
            'fig_idx',fig_idx,'fig_pos',fig_pos,'title_str','','tfs',tfs,...
            'zmp_ground',zmp_ground_post,'PLOT_ZMP',1,'axis_info',axis_info_com);
        drawnow;

        % Record video
        record_vid(vobj{1},'fig',fig1);
        record_vid(vobj{2},'fig',fig2);
        record_vid(vobj{3},'fig',fig3);
        record_vid(vobj{4},'fig',fig4);
        record_vid(vobj{5},'fig',fig5);
        record_vid(vobj{6},'fig',fig6);
        record_vid(vobj{7},'fig',fig7);

    end % if PLOT_EACH_TICK % plot each tick
end % for tick = 1:L % for each tick

% Finalize video recording
for v_idx = 1:length(vobj)
    end_vid_record(vobj{v_idx});
end

