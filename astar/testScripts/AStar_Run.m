%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter
% DATE CREATED: 2016-06-01
% PURPOSE: Run fast version of structure AStar algorithm
% REFERENCE: http://ch.mathworks.com/matlabcentral/fileexchange/26248-a---a-star--search-for-path-planning-tutorial
% CONTACT INFO: wachterfreddy@gmail.com

% Please refer to the Wiki for instructions on how to use this script
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB


% Go to the folder that this file is from
filePath = mfilename('fullpath');
cd(filePath(1:(find(filePath=='/',1,'last')-1)));
clear filePath;

% Uncomment the desired map
% load('../maps/20x20/sparse/map1.mat');
% load('../maps/20x20/sparse/map2.mat');
% load('../maps/20x20/orthogonal/map1.mat');
load('../maps/20x20/orthogonal/map2.mat');
% load('../maps/20x20/orthogonal/map3.mat');
% load('../maps/10x10/orthogonal/map1.mat');
% load('../maps/10x10/diagonal/map1.mat');
% load('../maps/10x10/diagonal/map2.mat');

% Initialize the AStar structure if it does not exist
if ~(exist('astar','var'))
    astar = AStar_Structure_Fast;
end

% Run AStar algorithm
solution = astar.runMap(map.data,map.criteria);

% Display optimized solution
astar.displayOptimizedSolution();


