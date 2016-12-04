%RUNMAPGENERATOR   Script to run MAPGENERATOR class
%
%   RUNMAPGENERATOR
%   runs the map generator script that allows for maps to be created with
%   the format required for A* class included in this repository
%
%   See also MAPGENERATOR, EXAMPLESCRIPT, ASTAR_STRUCTURE_FAST.
%
%   Please refer to the wiki for more information: 
%   https://github.com/FWachter/Micromouse/wiki/MATLAB

%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter
% DATE CREATED: 2016-06-02
% PURPOSE: Run the micromouse map generator
% CONTACT INFO: wachterfreddy@gmail.com

% Please refer to the Wiki for instructions on how to use this script
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB


generator = mapGenerator;
generator.generateMap;