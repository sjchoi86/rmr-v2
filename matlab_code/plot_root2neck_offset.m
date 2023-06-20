function plot_root2neck_offset(robot_name,varargin)
%
% Plot root to neck offset of a humanoid robot
%

% Parse options
iP = inputParser;
addParameter(iP,'fig_idx',1);
addParameter(iP,'fig_pos',[0.15,0.5,0.25,0.4]);
addParameter(iP,'view_info',[147,20]);
addParameter(iP,'axis_info',[-inf,inf,-inf,inf,0,inf]);
addParameter(iP,'mfa',0.02);
addParameter(iP,'PLOT_BOX_ADDED',1);
addParameter(iP,'bafc','k');
addParameter(iP,'bafa',0.05);
parse(iP,varargin{:});
fig_idx        = iP.Results.fig_idx;
fig_pos        = iP.Results.fig_pos;
view_info      = iP.Results.view_info;
axis_info      = iP.Results.axis_info;
mfa            = iP.Results.mfa;
PLOT_BOX_ADDED = iP.Results.PLOT_BOX_ADDED;
bafc           = iP.Results.bafc;
bafa           = iP.Results.bafa;

% T-poses of common rig and humanoid robot for computing root2neck quat offset
chain_rig_t_pose   = get_chain('common_rig','T_POSE',1,'RE',0);
chain_robot_t_pose = get_chain(robot_name,'T_POSE',1,'RE',0);
uv_rig_t_pose      = get_uv_chain(chain_rig_t_pose);
uv_robot_t_pose    = get_uv_chain(chain_robot_t_pose);
len_rig            = get_len_chain(chain_rig_t_pose);
len_robot          = get_len_chain(chain_robot_t_pose);
quat_offset        = get_q_uv1_to_uv2(uv_rig_t_pose.root2neck,uv_robot_t_pose.root2neck);
raxis_offset       = uv(quat_offset(2:4)); % rotation axis
T_rig              = get_t_joi(chain_rig_t_pose,chain_rig_t_pose.joi);
T_robot            = get_t_joi(chain_robot_t_pose,chain_robot_t_pose.joi);

% Configuration
PLOT_LINK = 1; PLOT_ROTATE_AXIS = 0; llw = 1/2; AXIS_OFF = 1;
arrow_len = chain_robot_t_pose.sz.xyz_len(3)*0.2; % len_robot.root2neck*0.7;
sw = arrow_len/20; tw = sw*2; text_fs = 20; text_p2_offset = 0.05;

% Plot T-pose of robot
plot_chain(chain_robot_t_pose,'fig_idx',fig_idx,'subfig_idx',2,'fig_pos',fig_pos,...
    'view_info',view_info,'PLOT_LINK',PLOT_LINK,'llw',llw,...
    'mfa',mfa,'PLOT_CAPSULE',0,'cfc',0.5*[1,1,1],'PLOT_ROTATE_AXIS',PLOT_ROTATE_AXIS,...
    'PLOT_BOX_ADDED',PLOT_BOX_ADDED,'bafc',bafc,'bafa',bafa,...
    'DISREGARD_JOI_GUIDE',1,'axis_info',axis_info,'AXIS_OFF',AXIS_OFF);

% Plot rotate plnae
plot_plane('fig_idx',fig_idx,'subfig_idx',1,...
    'xmin',-arrow_len,'xmax',arrow_len,'xres',arrow_len/10,...
    'ymin',-arrow_len,'ymax',arrow_len,'yres',arrow_len/10,...
    'plane_normal',raxis_offset,'plane_center',t2p(T_robot.torso),...
    'pfc','none');

% Plot arrows
plot_arrow_3d(t2p(T_robot.torso),t2p(T_robot.torso)+arrow_len*uv_rig_t_pose.root2neck,...
    'fig_idx',fig_idx,'subfig_idx',3,'alpha',0.5,'color','r','sw',sw,'tw',tw,...
    'text_str','$\mathbf{v}_{1}$','text_fs',text_fs,'text_p2_offset',text_p2_offset,...
    'text_color','r',...
    'interpreter','latex'); % unit vector of a rig

plot_arrow_3d(t2p(T_robot.torso),t2p(T_robot.torso)+arrow_len*uv_robot_t_pose.root2neck,...
    'fig_idx',fig_idx,'subfig_idx',2,'alpha',0.5,'color','b','sw',sw,'tw',tw,...
    'text_str','$\mathbf{v}_{2}$','text_fs',text_fs,'text_p2_offset',text_p2_offset,...
    'text_color','b',...
    'interpreter','latex'); % unit vector of a robot

plot_arrow_3d(t2p(T_robot.torso),t2p(T_robot.torso)+raxis_offset*arrow_len*0.75,...
    'fig_idx',fig_idx,'subfig_idx',4,...
    'alpha',0.5,'color','y','sw',sw,'tw',tw,...
    'text_str','Rotate Axis','text_fs',text_fs,'text_p2_offset',text_p2_offset,...
    'interpreter','latex'); % rotate axis

% Plot title
plot_title(sprintf('%s',upper(robot_name)),'fig_idx',fig_idx,'tfs',20);
