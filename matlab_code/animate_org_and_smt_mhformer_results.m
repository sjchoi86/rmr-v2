function animate_org_and_smt_mhformer_results(secs,chains_org,chains_smt,mocap_name,varargin)
%
% Animate original and smoothed MHFORMER pose estimation results
%

% Parse
iP = inputParser;
addParameter(iP,'folder_path','../vid/mocap');
addParameter(iP,'view_info',[88,16]);
addParameter(iP,'PLOT_JOI_TRAJ',1);
addParameter(iP,'SAVE_VID',1);
addParameter(iP,'SKIP_IF_MP4_EXIST',1);
parse(iP,varargin{:});
folder_path       = iP.Results.folder_path;
view_info         = iP.Results.view_info;
PLOT_JOI_TRAJ     = iP.Results.PLOT_JOI_TRAJ;
SAVE_VID          = iP.Results.SAVE_VID;
SKIP_IF_MP4_EXIST = iP.Results.SKIP_IF_MP4_EXIST;

% Save videos
vid_org_path = sprintf('%s/%s_org.mp4',folder_path,mocap_name);
vid_smt_path = sprintf('%s/%s_smt.mp4',folder_path,mocap_name);
if exist(vid_org_path,'file') && exist(vid_smt_path,'file') && SKIP_IF_MP4_EXIST
    fprintf(2,'Skip this as [%s] and [%s] exist \n',vid_org_path,vid_smt_path)
    return;
end

% JOI trajectories
if PLOT_JOI_TRAJ
    joi_traj_org = get_joi_traj(chains_org);
    joi_traj_smt = get_joi_traj(chains_smt);
end

% Animate
HZ       = round(length(secs)/(secs(end)-secs(1)));
vobj_org = init_vid_record(vid_org_path,'HZ',HZ,'SAVE_VID',SAVE_VID);
vobj_smt = init_vid_record(vid_smt_path,'HZ',HZ,'SAVE_VID',SAVE_VID);

ca; % close all
L = length(secs);
axis_info = get_axis_info_from_chains(chains_org);
for tick = 1:L
    % Plot original chain
    sec     = secs(tick);
    chain   = chains_org{tick};
    T_torso = get_t_torso_mhformer(chain);
    fig_idx = 1;
    fig1 = plot_chain(chain,'fig_idx',fig_idx,'fig_pos',[0.0,0.6,0.15,0.3],'view_info',view_info,...
        'PLOT_JOINT_SPHERE',1,'jsr',0.025,'jsfa',0.1,'PLOT_JOINT_NAME',1,...
        'axis_info',axis_info);
    plot_T(T_torso,'fig_idx',fig_idx,'subfig_idx',1,'all',0.3,'alw',3,'PLOT_AXIS_TIP',1);
    if PLOT_JOI_TRAJ
        plot_traj(joi_traj_org.rw,'fig_idx',fig_idx,'subfig_idx',1,'tlc','r','tlw',1/3);
        plot_traj(joi_traj_org.lw,'fig_idx',fig_idx,'subfig_idx',2,'tlc','b','tlw',1/3);
    end
    tfs = 15; % title font size
    plot_title(sprintf('[%d/%d][%.2f]s \nOriginal [%s]',tick,L,sec,mocap_name),...
        'fig_idx',fig_idx,'tfs',tfs);
    % Plot smoothed chain
    sec     = secs(tick);
    chain   = chains_smt{tick};
    T_torso = get_t_torso_mhformer(chain);
    fig_idx = 2;
    fig2 = plot_chain(chain,'fig_idx',fig_idx,'fig_pos',[0.15,0.6,0.15,0.3],'view_info',view_info,...
        'PLOT_JOINT_SPHERE',1,'jsr',0.025,'jsfa',0.1,'PLOT_JOINT_NAME',1,...
        'axis_info',axis_info);
    plot_T(T_torso,'fig_idx',fig_idx,'subfig_idx',1,'all',0.3,'alw',3,'PLOT_AXIS_TIP',1);
    if PLOT_JOI_TRAJ
        plot_traj(joi_traj_smt.rw,'fig_idx',fig_idx,'subfig_idx',1,'tlc','r','tlw',1/3);
        plot_traj(joi_traj_smt.lw,'fig_idx',fig_idx,'subfig_idx',2,'tlc','b','tlw',1/3);
    end
    plot_title(sprintf('[%d/%d][%.2f]s \nSmoothed [%s]',tick,L,sec,mocap_name),...
        'fig_idx',fig_idx,'tfs',tfs);
    drawnow;
    pause_invalid_handle([fig1,fig2]);
    % Save video
    record_vid(vobj_org,'fig',fig1);
    record_vid(vobj_smt,'fig',fig2);
end

% Finalize video recording (if necessary)
end_vid_record(vobj_org);
end_vid_record(vobj_smt);
