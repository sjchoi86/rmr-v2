addpath_yart
%% Common-Rig
ccc
chain_rig = get_chain('common_rig','T_POSE',1,'RHU_POSE',0,'RE',0);
axis_info = [-inf,inf,-inf,inf,0,inf];
animate_chain_with_joint_control_using_sliders(chain_rig,'axis_info',axis_info,...
    'PLOT_CAPSULE',1,'cec','none','cfc',0.5*[1,1,1],'cfa',0.2,'mfa',0.2,...
    'PLOT_ROTATE_AXIS',1,'PLOT_JOINT_AXIS',0,'jal',0.05,...
    'llw',2,'PLOT_JOINT_SPHERE',0,'mfc',0.9*[1,1,1],'mfa',0.5,...
    'PLOT_JOINT_NAME',0,'PLOT_SC',0,'PRINT_JOINT_POS',0);

%% Ambidex
ccc
chain_robot = get_chain('ambidex','T_POSE',1,'RE',0);
axis_info = [-inf,inf,-inf,inf,0,inf];
animate_chain_with_joint_control_using_sliders(chain_robot,'axis_info',axis_info,...
    'PLOT_CAPSULE',0,'cec','k','cfc','b','cfa',0.2,...
    'PLOT_ROTATE_AXIS',1,'PLOT_JOINT_AXIS',0,'jal',0.05,...
    'llw',2,'PLOT_JOINT_SPHERE',0,'mfc',0.9*[1,1,1],'mfa',0.2,...
    'PLOT_JOINT_NAME',0,'PLOT_SC',0);

%% Atlas
ccc
chain_robot = get_chain('atlas','T_POSE',1,'RE',0);
axis_info = [-inf,inf,-inf,inf,-inf,inf];
animate_chain_with_joint_control_using_sliders(chain_robot,'axis_info',axis_info,...
    'PLOT_CAPSULE',0,'cec','k','cfc','none','cfa',0.2,...
    'PLOT_ROTATE_AXIS',1,'PLOT_JOINT_AXIS',0,'jal',0.1,...
    'llw',2,'PLOT_JOINT_SPHERE',0,'mfc',0.9*[1,1,1],'mfa',0.2,...
    'PLOT_JOINT_NAME',0,'PLOT_SC',0,'PLOT_GRAPH',0);

%% COMAN
ccc
chain_robot = get_chain('coman','T_POSE',1,'RE',0);
axis_info = [-inf,inf,-inf,inf,0,inf];
animate_chain_with_joint_control_using_sliders(chain_robot,'axis_info',axis_info,...
    'PLOT_CAPSULE',0,'cec','k','cfc','none','cfa',0.2,...
    'PLOT_ROTATE_AXIS',1,'PLOT_JOINT_AXIS',1,'jal',0.1,...
    'llw',2,'PLOT_JOINT_SPHERE',0,'mfc',0.9*[1,1,1],'mfa',0.2,...
    'PLOT_JOINT_NAME',0,'PLOT_SC',0,'PLOT_GRAPH',0);

%% THORMANG
ccc
chain_robot = get_chain('thormang','T_POSE',1,'RE',0);
axis_info = [-inf,inf,-inf,inf,0,inf];
animate_chain_with_joint_control_using_sliders(chain_robot,'axis_info',axis_info,...
    'PLOT_CAPSULE',0,'cec','k','cfc','none','cfa',0.2,...
    'PLOT_ROTATE_AXIS',1,'PLOT_JOINT_AXIS',1,'jal',0.1,...
    'llw',2,'PLOT_JOINT_SPHERE',0,'mfc',0.9*[1,1,1],'mfa',0.2,...
    'PLOT_JOINT_NAME',0,'PLOT_SC',0,'PLOT_GRAPH',0);

%% Plot common-rig and multiple humanoid robots
ccc
chain_names = {'common_rig','ambidex','atlas','coman','thormang'};
T_POSE = 1; PLOT_CAPSULE_HUMANOID = 0; axis_info = [-inf,inf,-inf,inf,0,2];
plot_chains_side_by_side(chain_names,'T_POSE',T_POSE,'RE',0,...
    'fig_idx',1,'fig_pos',[0.0,0.6,0.7,0.3],'view_info',[90,0],...
    'PLOT_CAPSULE_HUMANOID',PLOT_CAPSULE_HUMANOID,'axis_info',axis_info,...
    'title_str','Common Rig and Different Humanoid Robots');
plot_chains_side_by_side(chain_names,'T_POSE',T_POSE,'RE',0,...
    'fig_idx',2,'fig_pos',[0.0,0.3,0.7,0.3],'view_info',[90,90],...
    'PLOT_CAPSULE_HUMANOID',PLOT_CAPSULE_HUMANOID,'axis_info',axis_info,...
    'title_str','Common Rig and Different Humanoid Robots');

%% Plot root to neck quaternion offfsets
ccc
axis_info = [-inf,inf,-inf,inf,0,1.8];
PLOT_BOX_ADDED = 1; bafc = 'k'; bafa = 0.05; mfa = 0.05;
plot_root2neck_offset('ambidex','fig_idx',1,'fig_pos',[0.0,0.5,0.25,0.4],'axis_info',axis_info,...
    'PLOT_BOX_ADDED',PLOT_BOX_ADDED,'bafc',bafc,'bafa',bafa,'mfa',mfa);
plot_root2neck_offset('atlas','fig_idx',2,'fig_pos',[0.25,0.5,0.25,0.4],'axis_info',axis_info,...
    'PLOT_BOX_ADDED',PLOT_BOX_ADDED,'bafc',bafc,'bafa',bafa,'mfa',mfa);
plot_root2neck_offset('coman','fig_idx',3,'fig_pos',[0.5,0.5,0.25,0.4],'axis_info',axis_info,...
    'PLOT_BOX_ADDED',PLOT_BOX_ADDED,'bafc',bafc,'bafa',bafa,'mfa',mfa);
plot_root2neck_offset('thormang','fig_idx',4,'fig_pos',[0.75,0.5,0.25,0.4],'axis_info',axis_info,...
    'PLOT_BOX_ADDED',PLOT_BOX_ADDED,'bafc',bafc,'bafa',bafa,'mfa',mfa);

%%






