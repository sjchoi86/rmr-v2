function animate_sch_results(chain_name,secs,chain,...
    T_roots_org,q_revs_org,T_roots_cf,q_revs_cf,varargin)
%
% Playback self collision handled results
%

% Parse options
iP = inputParser;
addParameter(iP,'folder_path','../vid/post_rig_cf');
addParameter(iP,'PLOT_EACH_TICK',1);
addParameter(iP,'SAVE_VID',1);
addParameter(iP,'SKIP_IF_VID_EXISTS',1);
addParameter(iP,'mfa',0.4);
addParameter(iP,'cfc',0.4*[1,1,1]);
addParameter(iP,'cfa',0.4);
addParameter(iP,'fig_w',0.2);
addParameter(iP,'fig_h',0.3);
addParameter(iP,'AXIS_OFF',1);
addParameter(iP,'SET_AXISLABEL',0);
addParameter(iP,'axis_info',[-1,+1,-1,+1,-0.1,2]);
addParameter(iP,'axes_info',[0,0,1,1]);
addParameter(iP,'view_info',[80,12]);
parse(iP,varargin{:});
folder_path        = iP.Results.folder_path;
PLOT_EACH_TICK     = iP.Results.PLOT_EACH_TICK;
SAVE_VID           = iP.Results.SAVE_VID;
SKIP_IF_VID_EXISTS = iP.Results.SKIP_IF_VID_EXISTS;
mfa                = iP.Results.mfa;
cfc                = iP.Results.cfc;
cfa                = iP.Results.cfa;
fig_w              = iP.Results.fig_w;
fig_h              = iP.Results.fig_h;
AXIS_OFF           = iP.Results.AXIS_OFF;
SET_AXISLABEL      = iP.Results.SET_AXISLABEL;
axis_info          = iP.Results.axis_info;
axes_info          = iP.Results.axes_info;
view_info          = iP.Results.view_info;

% Configuration
L = length(secs); HZ = round(L/(secs(end)-secs(1)));
% Video path
vid_path_org = sprintf('%s/%s_org.mp4',folder_path,chain_name);
vid_path_cf  = sprintf('%s/%s_cf.mp4',folder_path,chain_name);
if exist(vid_path_org,'file') && exist(vid_path_cf,'file') && SKIP_IF_VID_EXISTS
    fprintf(2,'[animate_sch_results] Skip as [%s] and [%s] exist. \n',vid_path_org,vid_path_cf);
    return;
end
vobj_org  = init_vid_record(vid_path_org,'HZ',HZ,'SAVE_VID',SAVE_VID);
vobj_cf   = init_vid_record(vid_path_cf,'HZ',HZ,'SAVE_VID',SAVE_VID);
chain_org = chain;
chain_cf  = chain;
for tick = 1:L % for each tick
    if tick == 1, RESET = 1; else, RESET = 0; end
    sec      = secs(tick);
    q_rev    = q_revs_org(tick,:); T_root = T_roots_org{tick};
    chain_org    = update_chain_q_root_T(chain_org,q_rev,T_root,'FV',1,'RESET',RESET);
    chain_org    = move_chain_two_feet_on_ground(chain_org);
    q_rev_cf = q_revs_cf(tick,:); T_root_cf = T_roots_cf{tick};
    chain_cf = update_chain_q_root_T(chain_cf,q_rev_cf,T_root_cf,'FV',1,'RESET',RESET);
    chain_cf = move_chain_two_feet_on_ground(chain_cf);
    % Animate
    if PLOT_EACH_TICK
        % Plot original skeleton
        fig_idx = 1; fig_pos = [0.0,0.6,fig_w,fig_h];
        fig1 = plot_chain(chain_org,...
            'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,'AXIS_OFF',AXIS_OFF,...
            'SET_AXISLABEL',SET_AXISLABEL,'axes_info',axes_info,...
            'SET_MATERIAL','DULL','axis_info',axis_info,'view_info',view_info,...
            'PLOT_ROTATE_AXIS',0,'PLOT_LINK',0,'PLOT_JOINT_AXIS',0,'jalw',3,...
            'PLOT_JOINT_SPHERE',0,'jsr',0.01,'bafa',0.5,'mfa',mfa,...
            'PLOT_CAPSULE',1,'cfc',cfc,'cfa',cfa,'DISREGARD_JOI_GUIDE',1);
        [SC,sc_link_pairs] = check_sc(chain_org,'collision_margin',0.0);
        plot_capsule('','fig_idx',fig_idx,'RESET',1); % reset capsule
        for sc_idx = 1:size(sc_link_pairs,1) % plot colliding capsules
            % original collision capsules
            [cap_i,T_i] = get_chain_capsule(chain_org,sc_link_pairs(sc_idx,1));
            [cap_j,T_j] = get_chain_capsule(chain_org,sc_link_pairs(sc_idx,2));
            cap_i = get_capsule_shape('T_offset',cap_i.T_offset,...
                'radius',cap_i.radius*1.001,'height',cap_i.height);
            cap_j = get_capsule_shape('T_offset',cap_j.T_offset,...
                'radius',cap_j.radius*1.001,'height',cap_j.height);
            plot_capsule(cap_i,'fig_idx',fig_idx,'subfig_idx',2*sc_idx-1,...
                'T',T_i,'cfc','r','cfa',0.9,'cec','k','cea',0.9);
            plot_capsule(cap_j,'fig_idx',fig_idx,'subfig_idx',2*sc_idx,...
                'T',T_j,'cfc','r','cfa',0.9,'cec','k','cea',0.9);
        end
        if SC, tfc = 'r'; else, tfc = 'k'; end; tfs = 15;
        title_str = sprintf('[%d/%d][%.1f]sec Original SC:[%d] \n (%s)',...
            tick,L,sec,SC,chain_name);
        if ~AXIS_OFF
            plot_title(title_str,'fig_idx',fig_idx,'tfs',tfs,'interpreter','latex','tfc',tfc);
        end
        drawnow; record_vid(vobj_org,'fig',fig1);

        % Plot collision-free skeleton
        fig_idx = 2; fig_pos = [fig_w,0.6,fig_w,fig_h];
        fig2 = plot_chain(chain_cf,...
            'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,'AXIS_OFF',AXIS_OFF,...
            'SET_AXISLABEL',SET_AXISLABEL,'axes_info',axes_info,...
            'SET_MATERIAL','DULL','axis_info',axis_info,'view_info',view_info,...
            'PLOT_ROTATE_AXIS',0,'PLOT_LINK',0,'PLOT_JOINT_AXIS',0,'jalw',3,...
            'PLOT_JOINT_SPHERE',0,'jsr',0.01,'bafa',0.5,'mfa',mfa,...
            'PLOT_CAPSULE',1,'cfc',cfc,'cfa',cfa,'DISREGARD_JOI_GUIDE',1);
        [SC_cf,sc_link_pairs_cf] = check_sc(chain_cf,'collision_margin',0.0);
        plot_capsule('','fig_idx',fig_idx,'RESET',1); % reset capsule
        for sc_idx = 1:size(sc_link_pairs_cf,1) % plot colliding capsules
            % original collision capsules
            [cap_i,T_i] = get_chain_capsule(chain_cf,sc_link_pairs_cf(sc_idx,1));
            [cap_j,T_j] = get_chain_capsule(chain_cf,sc_link_pairs_cf(sc_idx,2));
            cap_i = get_capsule_shape('T_offset',cap_i.T_offset,...
                'radius',cap_i.radius*1.001,'height',cap_i.height);
            cap_j = get_capsule_shape('T_offset',cap_j.T_offset,...
                'radius',cap_j.radius*1.001,'height',cap_j.height);
            plot_capsule(cap_i,'fig_idx',fig_idx,'subfig_idx',2*sc_idx-1,...
                'T',T_i,'cfc','r','cfa',0.9,'cec','k','cea',0.9);
            plot_capsule(cap_j,'fig_idx',fig_idx,'subfig_idx',2*sc_idx,...
                'T',T_j,'cfc','r','cfa',0.9,'cec','k','cea',0.9);
        end
        if SC_cf, tfc = 'r'; else, tfc = 'k'; end; tfs = 15;
        title_str = sprintf('[%d/%d][%.1f]sec Collision-Free SC:[%d] \n (%s)',...
            tick,L,sec,SC_cf,chain_name);
        if ~AXIS_OFF
            plot_title(title_str,'fig_idx',fig_idx,'tfs',tfs,'interpreter','latex','tfc',tfc);
        end
        drawnow; record_vid(vobj_cf,'fig',fig2);

        % Figure close handling
        pause_invalid_handle([fig1,fig2]);

    end
end % for tick = 1:L % for each tick

% Finalize video saving
end_vid_record(vobj_org);
end_vid_record(vobj_cf);
