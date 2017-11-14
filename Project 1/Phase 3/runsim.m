close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
map = load_map('map0.txt',  0.1, 2.0, 0.3);
% start = {[0 10 0.5]};
% stop  = {[20 13 0.5]};
start = {[0.0  -4.9 0.2]};
stop  = {[8.0  18.0 3.0]};
nquad = length(start);
for qn = 1:nquad
    path{qn} = dijkstra(map, start{qn}, stop{qn}, true);
end
if nquad == 1
    plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
end

%% Additional init script
init_script;

%% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
