function info = get_mhformer_info(varargin)
%
% Get pose estimation results from mhformer
%

% Parse input options
iP = inputParser;
addParameter(iP,'folder_path','../mocap/mhformer/');
addParameter(iP,'VERBOSE',1);
parse(iP,varargin{:});
folder_path = iP.Results.folder_path;
VERBOSE     = iP.Results.VERBOSE;

% Directory
info = dir_compact(folder_path);

% Add more fields
for d_idx = 1:length(info)
    name       = info(d_idx).name;
    folder     = info(d_idx).folder;
    mat_path   = [folder,'/',name,'/mat/',name,'.mat'];
    img_folder = [folder,'/',name,'/img'];
    d_img      = dir_compact(img_folder);
    n_img      = length(d_img);
    % Load 
    l          = load(mat_path);
    HZ         = double(l.fps{1});
    % Append
    info(d_idx).mat_path   = mat_path;
    info(d_idx).img_folder = img_folder;
    info(d_idx).n_img      = n_img;
    info(d_idx).L          = n_img;
    info(d_idx).HZ         = HZ;
    if VERBOSE
        fprintf('[%02d/%02d] name      : [%s] \n',d_idx,length(info),name);
        fprintf(' mat_path         : [%s] \n',mat_path )
        fprintf(' img_folder       : [%s] \n',img_folder )
        fprintf(' number of images : [%d] \n',n_img )
        fprintf(' seconds          : [%.2f] \n',n_img/HZ)
        fprintf(' HZ               : [%d] \n',HZ)
        if d_idx < length(info)
            fprintf ("\n")
        end
    end
end
