function data_smt = filter_mhformer_data_with_caching(mocap_name,data,joint_names,secs,varargin)
%
% Filtering MHFormer data by removing outliers with caching
%

% Parse input arguments
iP = inputParser;
addParameter(iP,'RE',0);
addParameter(iP,'cache_folder','../cache');
addParameter(iP,'joint_names2examine',{'RWrist','LWrist'});
addParameter(iP,'joint_vel_threshold',2);
addParameter(iP,'hyp',[1,0.5]);
addParameter(iP,'meas_noise_std',1e-4);
addParameter(iP,'max_iter',100);
addParameter(iP,'PLOT_EACH_ITER',0);
addParameter(iP,'PLOT_FINAL_RES',0);
addParameter(iP,'VERBOSE',1);
parse(iP,varargin{:});
RE                  = iP.Results.RE;
cache_folder        = iP.Results.cache_folder;
joint_names2examine = iP.Results.joint_names2examine;
joint_vel_threshold = iP.Results.joint_vel_threshold;
hyp                 = iP.Results.hyp;
meas_noise_std      = iP.Results.meas_noise_std;
max_iter            = iP.Results.max_iter;
PLOT_EACH_ITER      = iP.Results.PLOT_EACH_ITER;
PLOT_FINAL_RES      = iP.Results.PLOT_FINAL_RES;
VERBOSE             = iP.Results.VERBOSE;

% Cache path
cache_path = sprintf('%s/mocap/data_%s.mat',cache_folder,mocap_name);
if exist(cache_path,'file') && (RE==0)
    % Load if cache file exists
    l = load(cache_path);
    data_smt = l.data_smt;
    fprintf(2,'[filter_mhformer_data_with_caching] [%s] loaded.\n',cache_path);
else
    % Concatenate data to examine
    data_examine = [];
    for j_idx = 1:length(joint_names2examine)
        joint_name2examine = joint_names2examine{j_idx};
        joint_idx = idx_cell(joint_names,joint_name2examine);
        d_temp = squeeze(data(:,joint_idx,:));
        if isempty(d_temp)
            fprintf(2,'[filter_mhformer_data] [%s] does not exist.\n',joint_name2examine);
            continue;
        end
        data_examine = [data_examine, d_temp];
    end
    L = size(data,1);
    idxs_vld = 1:L; % valid indices, we will remove some from this
    idxs_rmv = [];
    for iter = 1:max_iter % for each iteration
        % Compute numerical velocity and acceleration
        [vel,acc] = get_vel_acc_numerical(secs(idxs_vld),data_examine(idxs_vld,:));
        if iter == 1
            vel_init = vel;
        end

        % Compute velocity exceeding indices
        vel_abs_max = max(abs(vel),[],2);
        idxs_exceed = find(vel_abs_max>joint_vel_threshold);
        n_exceed    = length(idxs_exceed);
        n_rmv       = length(idxs_rmv);
        if VERBOSE
            fprintf(' [%d/%d] max_abs_vel:[%.3f] n_exceed:[%d] n_rmv:[%d/%d] \n',...
                iter,max_iter,max(vel_abs_max),n_exceed,n_rmv,L);
        end
        % GRP interpolation
        mu_hat = smooth_traj(secs(idxs_vld),data_examine(idxs_vld,:),secs,...
            'intp_type','linear_GRP','hyp_mu',hyp,'meas_noise_std',meas_noise_std);

        % Plot
        if PLOT_EACH_ITER
            ca; % close all
            % Plot position
            fig_idx = 11;
            set_fig(figure(fig_idx ),'pos',[0.0,0.6,0.4,0.3],...
                'AXIS_EQUAL',0,'ax_str','Time [s]','ay_str','','afs',15,...
                'axes_info',[0.05,0.1,0.9,0.8]);
            h_grp = plot(secs,mu_hat,'-','color','r','linewidth',1);
            h_org = plot(secs,data_examine,'-','color','k','linewidth',1/2);
            h_vld = plot(secs(idxs_vld),data_examine(idxs_vld,:),'-','color','b','linewidth',1);
            h_exc = plot(secs(idxs_exceed),data_examine(idxs_exceed,:),'o',...
                'color','r','linewidth',1,'markersize',5);
            plot_title(sprintf('Position'),'fig_idx',fig_idx ,'tfs',20);
            plot_legend([h_org(1),h_vld(1),h_grp(1),h_exc(1)],...
                {'Original','Outlier-removed','GRP-smoothed','Outliers'},...
                'fig_idx',fig_idx,'ll','SouthEast');
            xlim([0,max(secs)]);

            % Plot velocity
            fig_idx = 12;
            set_fig(figure(fig_idx ),'pos',[0.0,0.3,0.4,0.3],...
                'AXIS_EQUAL',0,'ax_str','Time [s]','ay_str','','afs',15,...
                'axes_info',[0.05,0.1,0.9,0.8]);
            plot(secs,vel_init,'-','color','k','linewidth',1/2);
            plot(secs(idxs_exceed),vel_init(idxs_exceed,:),'o',...
                'color','r','linewidth',1,'markersize',5);
            plot(secs(idxs_vld),vel,'-','color','b','linewidth',1);
            plot_title(sprintf('Velocity'),'fig_idx',fig_idx ,'tfs',20);
            plot(secs,joint_vel_threshold*ones(L,2)*diag([1,-1]),'--','color','r');
            xlim([0,max(secs)]);
            drawnow;

            pause(); % pause for now
        end

        % Update
        if isempty(idxs_exceed) % if no velocity violation
            if VERBOSE
                fprintf(2,'[filter_mhformer_data] max vel:[%.3f] < vel threshold:[%.3f] \n',...
                    max(vel_abs_max),joint_vel_threshold);
            end
            break;
        end
        % Remove the 'first' exceeding index
        idx_exceed_first = idxs_exceed(1);
        idx2rmv          = idxs_vld(idx_exceed_first);
        idxs_rmv         = [idxs_rmv, idx2rmv];
        idxs_vld(idx_exceed_first) = [];
    end % for each iteration

    % Plot final results
    if PLOT_FINAL_RES

        ca; % close all
        fig_idx = 11;
        set_fig(figure(fig_idx ),'pos',[0.0,0.6,0.4,0.3],...
            'AXIS_EQUAL',0,'ax_str','Time [s]','ay_str','','afs',15,...
            'axes_info',[0.05,0.1,0.9,0.8]);
        h_rmv = plot(secs(idxs_rmv),data_examine(idxs_rmv,:),'x',...
            'color',0.5*ones(1,3),'linewidth',1,'markersize',5);
        h_org = plot(secs,data_examine,'-','color','k','linewidth',1/2);
        h_vld = plot(secs(idxs_vld),data_examine(idxs_vld,:),'-','color','b','linewidth',1);
        h_grp = plot(secs,mu_hat,'-','color','r','linewidth',1);
        plot(secs(idxs_exceed),data_examine(idxs_exceed,:),'o',...
            'color','r','linewidth',1,'markersize',5);
        plot_legend([h_org(1),h_rmv(1),h_vld(1),h_grp(1)],...
            {'Original','Outliers','Outlier-removed','GRP-smoothed'},'fig_idx',fig_idx,...
            'lfs',15,'ll','SouthEast');
        plot_title(sprintf('Position'),'fig_idx',fig_idx ,'tfs',20);
        xlim([0,max(secs)]);

        % Plot velocity
        [vel_smt,acc_smt] = get_vel_acc_numerical(secs,mu_hat);
        fig_idx = 12;
        set_fig(figure(fig_idx ),'pos',[0.0,0.3,0.4,0.3],...
            'AXIS_EQUAL',0,'ax_str','Time [s]','ay_str','','afs',15,...
            'axes_info',[0.05,0.1,0.9,0.8]);
        h_org = plot(secs,vel_init,'-','color','k','linewidth',1/2);
        plot(secs(idxs_exceed),vel_init(idxs_exceed,:),'o',...
            'color','r','linewidth',1,'markersize',5);
        h_vld = plot(secs(idxs_vld),vel,'-','color','b','linewidth',1);
        h_grp = plot(secs,vel_smt,'-','color','r','linewidth',1);
        plot(secs,joint_vel_threshold*ones(L,2)*diag([1,-1]),'--','color','r');
        plot_legend([h_org(1),h_vld(1),h_grp(1)],...
            {'Original','Outlier-removed','GRP-smoothed'},...
            'fig_idx',fig_idx,'lfs',15,'ll','SouthEast');
        plot_title(sprintf('Velocity'),'fig_idx',fig_idx ,'tfs',20);
        xlim([0,max(secs)]);
        drawnow; pause;
    end

    % Get final smoothed trajectories with excluding outliers
    data_smt = zeros(size(data));
    for j_idx = 1:length(joint_names)
        data_j = squeeze(data(:,j_idx,:));
        mu_hat = smooth_traj(secs(idxs_vld),data_j(idxs_vld,:),secs,...
            'intp_type','linear_GRP','hyp_mu',hyp,'meas_noise_std',meas_noise_std);
        data_smt(:,j_idx,:) = mu_hat;
    end

    % Save cache file
    [path_str,~,~] = fileparts(cache_path);
    make_dir_if_not_exist(path_str);
    save(cache_path,'data_smt');
    fprintf(2,'[filter_mhformer_data_with_caching] [%s] saved.\n',cache_path);
end