function T_torso = get_t_torso_mhformer(chain)
%
% Get 'T_torso' of mhformer chain
% - 'left pelvis' - 'torso' - 'spine' 
%
D2R     = pi/180;
T_joi   = get_t_joi(chain,chain.joi);
R_torso = get_R_from_three_points(t2p(T_joi.lp),t2p(T_joi.torso),t2p(T_joi.spine));
R_torso = R_torso * rpy2r([0,0,90]*D2R) * rpy2r([0,90,0]*D2R);
T_torso = pr2t(t2p(T_joi.torso),R_torso);
