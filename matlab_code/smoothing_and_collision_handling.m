function [q_revs_cf,T_roots_cf] = smoothing_and_collision_handling(...
    chain,secs,T_roots,q_revs,varargin)
%{
    Smoothing and collision handling of kinematic chains
%}

% Parse options
D2R = pi/180;
iP = inputParser;
addParameter(iP,'data_folder_path','../data/sch');
addParameter(iP,'robot_name','');
addParameter(iP,'mocap_name','');
addParameter(iP,'joint_names_to_ctrl',chain.rev_joint_names);
addParameter(iP,'PLOT_CHAIN_ZERO_POSE',1);
addParameter(iP,'ANIMATE_SC_CHECK',1);
addParameter(iP,'ANIMATE_SC_HANDLING',1);
addParameter(iP,'PLOT_CH_SMT_TRAJ',1);
addParameter(iP,'VERBOSE',1);
addParameter(iP,'SAVE_MAT',1);
addParameter(iP,'SKIP_IF_MAT_EXIST',1);
addParameter(iP,'smoothing_method','grp'); % smoothing method:{'grp','grp_multidim'}

% Simple GRP smoothing
addParameter(iP,'hyp_mu',[1,0.2]);
addParameter(iP,'meas_noise_std',1e-2);

% Selective GRP smoothing
addParameter(iP,'len_params',1.0*ones(1,size(q_revs,2)));

addParameter(iP,'sc_checks_margin_offset',0.0);
% Selective smoothing
addParameter(iP,'SEL_SMOOTH',0);

parse(iP,varargin{:});
data_folder_path        = iP.Results.data_folder_path;
robot_name              = iP.Results.robot_name;
mocap_name              = iP.Results.mocap_name;
joint_names_to_ctrl     = iP.Results.joint_names_to_ctrl;
PLOT_CHAIN_ZERO_POSE    = iP.Results.PLOT_CHAIN_ZERO_POSE;
ANIMATE_SC_CHECK        = iP.Results.ANIMATE_SC_CHECK;
ANIMATE_SC_HANDLING     = iP.Results.ANIMATE_SC_HANDLING;
PLOT_CH_SMT_TRAJ        = iP.Results.PLOT_CH_SMT_TRAJ;
VERBOSE                 = iP.Results.VERBOSE;
SAVE_MAT                = iP.Results.SAVE_MAT;
SKIP_IF_MAT_EXIST       = iP.Results.SKIP_IF_MAT_EXIST;
smoothing_method        = iP.Results.smoothing_method;
hyp_mu                  = iP.Results.hyp_mu;
meas_noise_std          = iP.Results.meas_noise_std;
len_params              = iP.Results.len_params;
sc_checks_margin_offset = iP.Results.sc_checks_margin_offset;
SEL_SMOOTH              = iP.Results.SEL_SMOOTH;

% Constant
R2D = 180/pi;

% Save
if isempty(robot_name) % if robot name does not exist
    sch_name = sprintf('%s',mocap_name);
else
    sch_name = sprintf('%s_%s',robot_name,mocap_name);
end
mat_path = sprintf('%s/%s.mat',data_folder_path,sch_name);
if SKIP_IF_MAT_EXIST && exist(mat_path,'file')
    fprintf(2,'[smoothing_and_collision_handling] Skip as [%s] exists.\n',mat_path);
    return;
end

% Length
L = size(q_revs,1);

% Initalize collsion pairs using the zero-pose
collision_margin = 0.0;
chain.sc_checks = get_sc_checks(chain,...
    'collision_margin',collision_margin+sc_checks_margin_offset,'UPDATE_WITH_ZERO_POSE',1);
if VERBOSE
    fprintf('  Start smoothing and collision handling of [%s]. \n',sch_name);
end
if PLOT_CHAIN_ZERO_POSE
    ca; % close all
    axis_info = [-1,+1,-1,+1,-0.1,2]; view_info = [80,11];
    fig_idx = 1; fig_pos = [0.0,0.6,0.15,0.3];
    root_idx = get_topmost_idx(chain);
    chain = update_chain_q_root_T(...
        chain,zeros(chain.n_rev_joint,1),pr2t(chain.joint(root_idx).p,eye(3,3)));
    plot_chain(chain,...
        'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
        'SET_MATERIAL','DULL','axis_info',axis_info,'view_info',view_info,...
        'PLOT_ROTATE_AXIS',0,'PLOT_LINK',0,'PLOT_JOINT_AXIS',0,'jalw',3,...
        'PLOT_JOINT_SPHERE',0,'jsr',0.01,'bafa',0.5,...
        'PLOT_CAPSULE',1,'cfc','k','cfa',0.3,'DISREGARD_JOI_GUIDE',1);
    plot_title('Zero-pose','fig_idx',fig_idx,'tfs',15,'interpreter','latex','tfc','k');
    if VERBOSE
        fprintf('  Plot the zero-pose of a chain. Paused. \n');
    end
    pause;
end

% First, smooth
% Smooth trajectories
switch smoothing_method
    case 'grp'
        q_revs_smt = smooth_traj(secs,q_revs,secs,...
            'intp_type','GRP','hyp_mu',hyp_mu,'meas_noise_std',meas_noise_std);
    case 'grp_multidim'
        q_revs_smt = q_revs;
        for d_idx = 1:size(q_revs,2)
            hyp_mu(2)  = len_params(d_idx);
            q_revs_smt(:,d_idx) = smooth_traj(secs,q_revs(:,d_idx),secs,...
                'intp_type','GRP','hyp_mu',hyp_mu,'meas_noise_std',meas_noise_std);
        end
    otherwise
        fprintf(2,'[smoothing_and_collision_handling] Unknown smoothing method:[%s]. \n',...
            smoothing_method);
        q_revs_smt = q_revs; % no smoothing
end

% Loop until smoothed trajectories are collision-free
n_try = 0;
collision_margin = 0.0;
q_revs_in = q_revs_smt; q_revs_in_init = q_revs_smt;
q_revs_cf = q_revs_smt; T_roots_cf = T_roots; % if the initial trajectories are collision-free
% q_revs_in = q_revs; q_revs_in_init = q_revs;
while 1 % loop
    %{
    - Step1: Check self-collision of each poses, exit if no collision ocurred
    - If collision ocurred, 
    - Step2: Collision-handle of each poses
    - Step3: Smooth the collision-free trajectories and goto Step1
    %}
    n_try = n_try + 1;
    if VERBOSE
        fprintf('  n_try;[%d] \n',n_try)
    end
    % Step1, check the self-collision with zero margin
    % However, collision pairs should properly reflect 'collision_margin'
    chain.sc_checks = get_sc_checks(chain,...
        'collision_margin',collision_margin+sc_checks_margin_offset,'UPDATE_WITH_ZERO_POSE',1);
    if VERBOSE
        fprintf('  collision_margin:[%.3f]. Number of collision paris:[%d]. \n',...
            collision_margin,size(chain.sc_checks,1));
    end
    if ANIMATE_SC_CHECK
        ca; % close all
    end
    if VERBOSE
        fprintf('  Start chekcing self-collision \n');
    end
    % Dense, interpolated self-collision check (to handle tunneling)

    % Here, we will simply 'check' whether a collision happens or not 
    n_sc_check = 0; sc_ticks = [];
    for tick = 1:L % for each tick
        if tick == 1, RESET = 1; else, RESET = 0; end
        T_root = T_roots{tick};
        q_rev  = q_revs_in(tick,:);
        chain = update_chain_q_root_T(chain,q_rev,T_root,'FK',1,'FV',0,'RESET',RESET);
        chain = move_chain_two_feet_on_ground(chain);
        % Check self-collision with zero margin
        [SC,sc_link_pairs] = check_sc(chain,'collision_margin',0.0);
        if SC % if collision ocurrs
            n_sc_check = n_sc_check + 1;
            sc_ticks = [sc_ticks, tick];
        end
        if ANIMATE_SC_CHECK
            axis_info = [-1,+1,-1,+1,-0.1,2]; view_info = [80,11];
            fig_idx = 2; fig_pos = [0.0,0.6,0.15,0.3];
            fig2 = plot_chain(chain,...
                'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
                'SET_MATERIAL','DULL','axis_info',axis_info,'view_info',view_info,...
                'PLOT_ROTATE_AXIS',0,'PLOT_LINK',0,'PLOT_JOINT_AXIS',0,'jalw',3,...
                'PLOT_JOINT_SPHERE',0,'jsr',0.01,'bafa',0.5,...
                'PLOT_CAPSULE',1,'cfc','k','cfa',0.3,'DISREGARD_JOI_GUIDE',1);
            plot_capsule('','fig_idx',fig_idx,'RESET',1); % reset capsule
            for sc_idx = 1:size(sc_link_pairs,1) % plot colliding capsules
                [cap_i,T_i] = get_chain_capsule(chain,sc_link_pairs(sc_idx,1));
                [cap_j,T_j] = get_chain_capsule(chain,sc_link_pairs(sc_idx,2));
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
            title_str = sprintf('[%d/%d] Original Motion SC:[%d] \n (%s)',...
                tick,L,SC,mocap_name);
            plot_title(title_str,'fig_idx',fig_idx,'tfs',tfs,'interpreter','latex','tfc',tfc);
            drawnow; pause_invalid_handle(fig2);
        end
    end
    if VERBOSE
        fprintf('  The number of self-collision is [%d]. \n',n_sc_check);
    end
    % Termination condition
    if n_sc_check == 0
        if VERBOSE
            fprintf(2,'  No collision ocurred. breaking out of this routine. \n');
        end
        q_revs_cf = q_revs_in;
        break;
    else
        if VERBOSE
            fprintf('  We have [%d] collisions to handle. \n',n_sc_check);
        end
    end

    % Step2. Collision handling routine
    %{
        * Note that we increase the collision margin to give additional safety
    %}
    if ANIMATE_SC_HANDLING
        ca; % close all
    end
    % Increase collision margin and update collision pairs
    chain_collision = chain;
    collision_margin = chain_collision.sz.xyz_len(3)*0.005*n_try; % increase margin slightly
    chain_collision.sc_checks = get_sc_checks(chain_collision,...
        'collision_margin',collision_margin+sc_checks_margin_offset,...
        'UPDATE_WITH_ZERO_POSE',1); % update collision pairs
    if VERBOSE
        fprintf('  collision_margin:[%.3f]. Number of collision paris:[%d]. \n',...
            collision_margin,size(chain_collision.sc_checks,1));
    end
    q_revs_cf  = zeros(L,chain_collision.n_rev_joint);
    T_roots_cf = cell(1,L);
    ch_ticks   = []; % collision handle ticks
    for tick = 1:L
        if tick == 1, RESET = 1; else, RESET = 0; end
        T_root = T_roots{tick}; q_rev = q_revs_in(tick,:);
        chain_collision = update_chain_q_root_T(...
            chain_collision,q_rev,T_root,'FK',1,'FV',0,'RESET',RESET);
        chain_collision = move_chain_two_feet_on_ground(chain_collision);
        [SC,~] = check_sc(chain_collision,'collision_margin',collision_margin);
        if SC
            ch_ticks = [ch_ticks, tick]; % append collision handling ticks
        end
        ch_iter = 0; % collision handle counter
        while SC % until collision-free
            ch_iter = ch_iter + 1;
            [chain_collision,sc_ik_info,sc_link_pairs_temp] = ...
                handle_sc_with_ik(chain_collision,'joint_names_to_ctrl',joint_names_to_ctrl,...
                'len_offset',chain_collision.sz.xyz_len(3)/20,...
                'UNIT_DQ_HEURISTIC',1,'unit_dq_rad',2*D2R,'collision_margin',collision_margin);
            [SC,sc_link_pairs] = check_sc(chain_collision,'collision_margin',collision_margin);
            if ANIMATE_SC_HANDLING
                axis_info = [-1,+1,-1,+1,-0.1,2]; view_info = [80,11];
                fig_idx = 3; fig_pos = [0.0,0.6,0.15,0.3];
                fig3 = plot_chain(chain_collision,...
                    'fig_idx',fig_idx,'subfig_idx',1,'fig_pos',fig_pos,...
                    'SET_MATERIAL','DULL','axis_info',axis_info,'view_info',view_info,...
                    'PLOT_ROTATE_AXIS',0,'PLOT_LINK',0,'PLOT_JOINT_AXIS',0,'jalw',3,...
                    'PLOT_JOINT_SPHERE',0,'jsr',0.01,'bafa',0.5,...
                    'PLOT_CAPSULE',1,'cfc','k','cfa',0.3,'DISREGARD_JOI_GUIDE',1);
                plot_capsule('','fig_idx',fig_idx,'RESET',1); % reset capsule
                for sc_idx = 1:size(sc_link_pairs,1) % plot colliding capsules
                    [cap_i,T_i] = get_chain_capsule(...
                        chain_collision,sc_link_pairs(sc_idx,1));
                    [cap_j,T_j] = get_chain_capsule(...
                        chain_collision,sc_link_pairs(sc_idx,2));
                    cap_i = get_capsule_shape('T_offset',cap_i.T_offset,...
                        'radius',cap_i.radius*1.001,'height',cap_i.height);
                    cap_j = get_capsule_shape('T_offset',cap_j.T_offset,...
                        'radius',cap_j.radius*1.001,'height',cap_j.height);
                    plot_capsule(cap_i,'fig_idx',fig_idx,'subfig_idx',2*sc_idx-1,...
                        'T',T_i,'cfc','r','cfa',0.9,'cec','k','cea',0.9);
                    plot_capsule(cap_j,'fig_idx',fig_idx,'subfig_idx',2*sc_idx,...
                        'T',T_j,'cfc','r','cfa',0.9,'cec','k','cea',0.9);
                end
                title_str = sprintf('[%d/%d][%d] Collision handling routine \n (%s)',...
                    tick,L,ch_iter,sch_name);
                plot_title(title_str,'fig_idx',fig_idx,...
                    'tfs',15,'interpreter','latex','tfc','k');
                drawnow; pause_invalid_handle(fig3);
            end
        end
        % Append collision-free trajectories
        q_revs_cf(tick,:) = get_q_chain(chain_collision,chain_collision.rev_joint_names);
        T_roots_cf{tick}  = get_T_root_chain(chain_collision);
    end
    if VERBOSE
        fprintf('  We have handled [%d] collided ticks. \n',length(ch_ticks));
    end

    % Smooth trajectories
    switch smoothing_method
        case 'grp'
            q_revs_smt = smooth_traj(secs,q_revs_cf,secs,...
                'intp_type','GRP','hyp_mu',hyp_mu,'meas_noise_std',meas_noise_std);
        case 'grp_multidim'
            q_revs_smt = q_revs_cf;
            for d_idx = 1:size(q_revs_cf,2)
                hyp_mu(2)  = len_params(d_idx);
                q_revs_smt(:,d_idx) = smooth_traj(secs,q_revs_cf(:,d_idx),secs,...
                    'intp_type','GRP','hyp_mu',hyp_mu,'meas_noise_std',meas_noise_std);
            end
        otherwise
            fprintf(2,'Unknown smoothing method:[%s]. \n',smoothing_method);
            q_revs_smt = q_revs_cf; % no smoothing
    end

    % (Optional) additional selective smoothing here
    if SEL_SMOOTH
        D = size(q_revs_smt,2);
        for d_idx = 1:D
            t_in = secs; x_in = q_revs_smt(:,d_idx);
            sel_acc_th = 2000*D2R; sel_len_base = 1/5;
            SEL_DEBUG_PLOT = 0;
            [q_rev_smt,~,~] = do_smoothing(t_in,x_in,...
                'smt_type','NSGP','acc_th',sel_acc_th,...
                'gain',1.0,'len_base',sel_len_base,'sig2w',1e-6,'len_rate',0.3,...
                'intv_pad',1.0,'n_smt_iter',5,'DEBUG_PLOT',SEL_DEBUG_PLOT);
            q_revs_smt(:,d_idx) = q_rev_smt;
        end
    end

    % Plot collision-handled and smoothed trajectories
    if PLOT_CH_SMT_TRAJ
        ca; % close all
        % Plot origina, collion-handled, and smoothed joint trajectories
        fig_idx = 4;
        fig4 = set_fig(figure(fig_idx),'pos',[0.0,0.6,0.5,0.3],'AXIS_EQUAL',0,...
            'ax_str','Time [sec]','ay_str','','afs',15,'axes_info',[0.05,0.1,0.9,0.82]);
        hold on;
        for d_idx = 1:chain.n_rev_joint
            h_init   = plot(secs,q_revs_in_init(:,d_idx),'-','Color',0.5*[1,1,1],'LineWidth',1/2);
            h_org    = plot(secs,q_revs_in(:,d_idx),'-','Color','black');
            h_col    = plot(secs(ch_ticks),q_revs_in(ch_ticks,d_idx),'s','Color','black');
            h_col2   = plot(secs(ch_ticks),q_revs_smt(ch_ticks,d_idx),'o','Color','red');
            h_cf_smt = plot(secs,q_revs_smt(:,d_idx),'-','Color','red');
        end
        plot_legend(...
            [h_init,h_org,h_col,h_col2,h_cf_smt],...
            {'Initial','Original','Collision','Collision handled','Smoothed collision-free'},...
            'fig_idx',fig_idx,'ll','SouthEast','lfc','white');
        plot_title('Original, Collision-handled, and Smoothed Joint Trajectories',...
            'fig_idx',fig_idx,'tfs',15);

        % Plot numerical vel and acc of smoothed trajectories (q_revs_smt)
        [vel,acc] = get_vel_acc_numerical(secs,q_revs_smt);
        fig_idx = 5;
        fig5 = set_fig(figure(fig_idx),'pos',[0.0,0.3,0.5,0.3],'AXIS_EQUAL',0,...
            'ax_str','Time [sec]','ay_str','','afs',15,'axes_info',[0.05,0.1,0.9,0.82]);
        for d_idx = 1:chain.n_rev_joint
            plot(secs,vel(:,d_idx)*R2D,'-','Color','k','LineWidth',1);
        end
        plot_title('Velocity of Smoothed Trajectories',...
            'fig_idx',fig_idx,'tfs',15);
        fig_idx = 6;
        fig6 = set_fig(figure(fig_idx),'pos',[0.0,0.0,0.5,0.3],'AXIS_EQUAL',0,...
            'ax_str','Time [sec]','ay_str','','afs',15,'axes_info',[0.05,0.1,0.9,0.82]);
        for d_idx = 1:chain.n_rev_joint
            plot(secs,acc(:,d_idx)*R2D,'-','Color','k','LineWidth',1);
        end
        plot_title('Acceleration of Smoothed Trajectories',...
            'fig_idx',fig_idx,'tfs',15);

        if VERBOSE
            fprintf('  Plot original and collision handled joint trajectories. Paused. \n');
        end
        pause;
        ca; % close all 
    end

    %  GOTO Step1
    q_revs_in = q_revs_smt;

end % while 1 % loop

% Save the results to a mat file
if SAVE_MAT
    [p,~,~] = fileparts(mat_path); make_dir_if_not_exist(p);
    save(mat_path,'robot_name','mocap_name','chain','secs','T_roots','q_revs',...
        'q_revs_cf','T_roots_cf');
    fprintf(2,'[smoothing_and_collision_handling] [%s] saved.\n',mat_path);
end
