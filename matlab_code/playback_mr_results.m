function playback_mr_results(mocap_name,robot_name,...
    secs,chain_rig,T_roots_rig,q_revs_rig,chain_robot,T_roots_robot,q_revs_robot,varargin)
%
% Playback motion retarget results
%

% Parse options
iP = inputParser;
addParameter(iP,'vid_folder_path','../vid/mr');     % folder path to save videos
addParameter(iP,'mfa',0.4);
addParameter(iP,'SAVE_VID',0);
addParameter(iP,'SKIP_IF_VID_EXISTS',0);
addParameter(iP,'fig_w',0.2);
addParameter(iP,'fig_h',0.3);
addParameter(iP,'axis_info',[-1,+1,-1,+1,-0.1,2]);
addParameter(iP,'AXIS_OFF',1);
addParameter(iP,'SET_AXISLABEL',0);
addParameter(iP,'axes_info',[0,0,1,1]);
addParameter(iP,'view_info',[80,12]);
parse(iP,varargin{:});
vid_folder_path    = iP.Results.vid_folder_path;
mfa                = iP.Results.mfa;
SAVE_VID           = iP.Results.SAVE_VID;
SKIP_IF_VID_EXISTS = iP.Results.SKIP_IF_VID_EXISTS;
fig_w              = iP.Results.fig_w;
fig_h              = iP.Results.fig_h;
axis_info          = iP.Results.axis_info;
AXIS_OFF           = iP.Results.AXIS_OFF;
SET_AXISLABEL      = iP.Results.SET_AXISLABEL;
axes_info          = iP.Results.axes_info;
view_info          = iP.Results.view_info;

% Motion length
L = length(secs); HZ = round(L/(secs(end)-secs(1)));

% If 'chain_rig' is empty
if isempty(chain_rig)
    RIG_EXISTS = 0;
else
    RIG_EXISTS = 1;
end

% Animate loop
if RIG_EXISTS
    vid_path_rig = sprintf('%s/%s_rig.mp4',vid_folder_path,mocap_name);
else
    vid_path_rig = '';
end
vid_path_robot = sprintf('%s/%s_to_%s.mp4',vid_folder_path,mocap_name,robot_name);


if (~RIG_EXISTS)
    if SKIP_IF_VID_EXISTS && ...
            exist(vid_path_robot,'file')
        fprintf(2,'Skip this as [%s] exists.\n',vid_path_robot);
        return;
    end
else
    if SKIP_IF_VID_EXISTS && ...
            exist(vid_path_rig,'file') && ...
            exist(vid_path_robot,'file')
        fprintf(2,'Skip this as [%s] and [%s] exist.\n',vid_path_rig,vid_path_robot);
        return;
    end
end


if RIG_EXISTS
    vobj_rig = init_vid_record(vid_path_rig,'HZ',HZ,'SAVE_VID',SAVE_VID);
end
vobj_robot = init_vid_record(vid_path_robot,'HZ',HZ,'SAVE_VID',SAVE_VID);
ca; % close all
for tick = 1:L
    % Update
    if RIG_EXISTS
        q_rev_rig = q_revs_rig(tick,:); T_root_rig = T_roots_rig{tick};
        chain_rig = update_chain_q_T_root(chain_rig,q_rev_rig,T_root_rig);
    end
    q_rev_robot = q_revs_robot(tick,:); T_root_robot = T_roots_robot{tick};
    chain_robot = update_chain_q_T_root(chain_robot,q_rev_robot,T_root_robot);
    % Plot common-rig
    if RIG_EXISTS
        fig_idx = 1; fig_pos = [0.0,0.6,fig_w,fig_h];
        fig_rig = plot_chain(chain_rig,'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
            'view_info',view_info,'axis_info',axis_info,'AXIS_OFF',AXIS_OFF,...
            'SET_AXISLABEL',SET_AXISLABEL,'axes_info',axes_info,...
            'PLOT_LINK',0,'PLOT_ROTATE_AXIS',0,'PLOT_CAPSULE',1,'cfc',0.3*[1,1,1],'bafa',0.05,...
            'DISREGARD_JOI_GUIDE',1);
        % title_str = sprintf('[%d/%d]\n%s',tick,L,mr_name);
        % plot_title(title_str,'fig_idx',fig_idx,'tfs',13);
    end
    % Plot robot
    fig_idx = 2; fig_pos = [fig_w,0.6,fig_w,fig_h];
    chain_robot_ground = move_chain_two_feet_on_ground(chain_robot,'xy_offset',cv([0,0]));
    ral = chain_robot_ground.sz.xyz_len(3)/15;
    fig_robot = plot_chain(chain_robot_ground,'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
        'view_info',view_info,'axis_info',axis_info,'AXIS_OFF',AXIS_OFF,...
        'SET_AXISLABEL',SET_AXISLABEL,'axes_info',axes_info,...
        'PLOT_LINK',0,'PLOT_ROTATE_AXIS',1,'ral',ral,...
        'PLOT_CAPSULE',0,'mfa',mfa,'bafa',0.15,...
        'DISREGARD_JOI_GUIDE',1);
    drawnow;
    if RIG_EXISTS
        record_vid(vobj_rig,'fig',fig_rig);
    end
    record_vid(vobj_robot,'fig',fig_robot);
    if RIG_EXISTS
        if (~ishandle(fig_rig)), fprintf(2,'figure closed\n'); pause; end
    end
    if (~ishandle(fig_robot)), fprintf(2,'figure closed\n'); pause; end
end
if RIG_EXISTS
    end_vid_record(vobj_rig);
end
end_vid_record(vobj_robot);
