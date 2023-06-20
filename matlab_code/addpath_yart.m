function addpath_yart()
%
% Add YART path
%

addpath('../../yet-another-robotics-toolbox/code/'); % this is YART
addpath('../../yet-another-robotics-toolbox/util/npy-matlab/'); % for loading npz files

% Clear command window, memory, and close all figures
clc; 
clear all; 
close all;

% Print
fprintf('YART package added.\n');