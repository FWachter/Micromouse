%EXAMPLESCRIPT   Example script for running A* algorithm
%
%   EXAMPLESCRIPT
%   loads a 20x20 map into the workspace and runs the A* algorithm on the
%   map and displays the solution, cut corner path, and polynomial
%   interpolation path in a figure.
%
%   See also ASTAR_STRUCTURE_FAST, MAPGENERATOR, RUNMAPGENERATOR.
%
%   Please refer to the wiki for more information: 
%   https://github.com/FWachter/Micromouse/wiki/MATLAB

%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter
% DATE CREATED: 2016-06-01
% PURPOSE: Example of running A* algorithm
% CONTACT INFO: wachterfreddy@gmail.com

% Please refer to the Wiki for instructions on how to use this script
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB

astar = AStar_Structure_Fast;
load('maps/20x20/orthogonal/map3.mat');
astar.runMap(map.data, map.criteria);

astar.displayCutCornerPath; % show the convex decimator solution
astar.displayOptimizedSolution; % show the smoothed polynomial solution

% astar.removeSolution; % Remove A* solution
% astar.removeCutCornerPath; % Remove convex decimator solution
% astar.removeOptimizedSolution; % Remove smoothed polynomial solution