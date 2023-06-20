
function chains = get_chains_from_mhformer_data(data,joint_names)
%
% Get chains from mhformer data
%

D2R    = pi/180;
L      = size(data,1);
chains = cell(1,L);
for tick = 1:L % for each tick
    % Append joint information
    p_joi = struct;
    for j_idx = 1:length(joint_names)
        joint_name = joint_names{j_idx};
        joint_name = strrep(joint_name,'/','_');
        p_joints   = squeeze(data(tick,j_idx,:));
        p_joi      = setfield(p_joi,joint_name,p_joints);
    end
    % Generate kinematic chain
    joint_fr_list = {'','Hip','Spine','Thorax','Neck_Nose',...
        'Thorax','RShoulder','RElbow','Thorax','LShoulder','LElbow',...
        'Hip','RHip','RKnee','Hip','LHip','LKnee'};
    joint_to_list = {'Hip','Spine','Thorax','Neck_Nose','Head',...
        'RShoulder','RElbow','RWrist','LShoulder','LElbow','LWrist',...
        'RHip','RKnee','RFoot','LHip','LKnee','LFoot'};
    chain = init_chain('name','mhformer_chain');
    for j_idx = 1:length(joint_fr_list)
        parent_name = joint_fr_list{j_idx};
        curr_name   = joint_to_list{j_idx};
        p_curr      = getfield(p_joi,curr_name);
        chain = add_joint_to_chain(chain,'name',curr_name,...
            'p',p_curr,'parent_name',parent_name,'p_offset','','R_offset',eye(3,3));
    end
    joint_name_list = {'torso','spine','neck','nose','head',...
        'rs','re','rw','ls','le','lw','rp','rk','ra','lp','lk','la'};
    for j_idx = 1:length(joint_name_list)
        joint_name_org = joint_to_list{j_idx};
        joint_name_new = joint_name_list{j_idx};
        joint_idx = idx_cell(chain.joint_names,joint_name_org);
        chain.joint(joint_idx).name = joint_name_new;
        chain.joint_names{joint_idx} = joint_name_new;
    end
    % Get JOI
    chain.joi = get_joi_chain(chain,'APPEND_ALL',1);
    % Rotate chain so that the skeleton is heading X-axis
    chain.joint(1).R = rpy2r([0,0,-90]*D2R) * chain.joint(1).R;
    chain = fk_chain(chain,'');
    % Set the initial yaw to be zero
    T_torso = get_t_torso_mhformer(chain);
    if tick == 1
        R_torso_init   = t2r(T_torso);
        rpy_torso_init = r2rpy(R_torso_init);
        rpy_torso_init(1:2) = 0; % only use z-rotation
        R_torso_init   = rpy2r(rpy_torso_init);
    end
    chain.joint(1).R = R_torso_init' * chain.joint(1).R; % pre-multiply: global
    chain = fk_chain(chain,'');
    % Append
    chains{tick} = chain;
end