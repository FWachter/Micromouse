%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter
% DATE CREATED: 2016-06-01
% PURPOSE: Run fast version of structure AStar algorithm
% REFERENCE: http://ch.mathworks.com/matlabcentral/fileexchange/26248-a---a-star--search-for-path-planning-tutorial
% CONTACT INFO: wachterfreddy@gmail.com

% Please refer to the Wiki for instructions on how to use this script
% GITHUB WIKI: www.github.com/FWchter/Micromouse/Wiki

cd(pwd)
[fileName,pathName] = uigetfile('*.mat');
load([pathName,'/',fileName]);
map = map.data;

astar = AStar_Structure_Fast;
astar.runMap(map,1);


