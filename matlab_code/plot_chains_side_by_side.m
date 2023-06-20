function plot_chains_side_by_side(chain_names,varargin)
%
% Plot multiple chains side by side
%

% Parse options
iP = inputParser;
addParameter(iP,'T_POSE',0);
addParameter(iP,'RE',0);
addParameter(iP,'fig_idx',1);
addParameter(iP,'fig_pos',[0.0,0.5,0.7,0.4]);
addParameter(iP,'view_info',[90,0]); % [90,0] / [90,90]
addParameter(iP,'axis_info',[-inf,inf,-inf,inf,0,inf]);
addParameter(iP,'title_str','plot_chains_side_by_side');
addParameter(iP,'tfs',20);
addParameter(iP,'PLOT_CAPSULE_HUMANOID',0);
parse(iP,varargin{:});
T_POSE    = iP.Results.T_POSE;
RE        = iP.Results.RE;
fig_idx   = iP.Results.fig_idx;
fig_pos   = iP.Results.fig_pos;
view_info = iP.Results.view_info;
axis_info = iP.Results.axis_info;
title_str = iP.Results.title_str;
tfs       = iP.Results.tfs;
PLOT_CAPSULE_HUMANOID = iP.Results.PLOT_CAPSULE_HUMANOID;

n_chain = length(chain_names);
chains = cell(1,n_chain);
chain_heights = zeros(1,n_chain); chain_widths = zeros(1,n_chain);
for c_idx = 1:n_chain
    chain_name = chain_names{c_idx};
    chain = get_chain(chain_name,'T_POSE',T_POSE,'RE',RE); 
    chains{c_idx} = chain; sz = get_chain_sz(chain);
    chain_heights(c_idx) = sz.xyz_len(3); chain_widths(c_idx) = sz.xyz_len(2);
end
[~,sorted_idx] = sort(chain_heights); % sort with heights
y_offset = 0;
for c_idx = 1:n_chain
    sort_idx = sorted_idx(c_idx);
    chain = chains{sort_idx}; chain_name = chain_names{sort_idx};
    idx_top = get_topmost_idx(chain);
    chain = move_chain_two_feet_on_ground(chain);
    if c_idx == 1
    else
        y_offset = y_offset + 0.1 + ...
            0.5*(chain_widths(sorted_idx(c_idx-1)) + chain_widths(sorted_idx(c_idx)));
    end
    chain = move_chain(chain,chain.joint(idx_top).p + cv([0.0,y_offset,0.0]));
    axes_info = [0.02,0.01,0.95,0.9];
    switch chain_name
        case 'common_rig'
            PLOT_CAPSULE = 1;
            cfc = 0.5*[1,1,1];
            cec = 'none';
        otherwise
            PLOT_CAPSULE = PLOT_CAPSULE_HUMANOID;
            cfc = 'none';
            cec = 'k';
    end
    plot_chain(chain,'fig_idx',fig_idx,'subfig_idx',c_idx,'fig_pos',fig_pos,...
        'axis_info',axis_info,'axis_info',axis_info,'AXIS_OFF',1,'axes_info',axes_info,'mfa',0.1,...
        'PLOT_CAPSULE',PLOT_CAPSULE,'cfc',cfc,'cec',cec,'view_info',view_info,...
        'PLOT_LINK',1,'PLOT_BOX_ADDED',1,'PLOT_ROTATE_AXIS',1,'DISREGARD_JOI_GUIDE',0);
end
plot_title(title_str,'fig_idx',fig_idx,'tfs',tfs);
