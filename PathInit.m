%!octave

clear all;

% Add subdirectories
addpath("Plot", "Test");

% Declare and initialize global variables
global PATH_MOVE_NONE 				= 0;  % Undefined
global PATH_MOVE_DECACC 			= 1;  % DecAcc profile with peak (dip)
global PATH_MOVE_DECACCSATURATED 	= 2;  % DecAcc profile saturated at v_min
global PATH_MOVE_ACCDEC 		= 10; % AccDec profile with peak
global PATH_MOVE_ACCDECSATURATED 	= 20; % AccDec profile saturated at v_max
global PATH_MOVE_DECDEC 			= 30; % DecDec profile (PathVel)
global PATH_MOVE_ACCACC 			= 40; % AccAcc profile (PathVel)
global PATH_POINTS_MAX_INDEX 		= 19; % Size of velocity & time point arrays for PathTime inputs
