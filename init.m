%!octave

% Add subdirectories
addpath("lib", "samples", "plot", "test", "diag");

% Declare and initialize global variables
global PATH_MOVE_NONE 				= 0;  % Undefined
global PATH_MOVE_DECACCPEAK 		= 1;  % DecAcc profile with peak (dip)
global PATH_MOVE_DECACCSATURATED 	= 2;  % DecAcc profile saturated at v_min
global PATH_MOVE_ACCDECPEAK 		= 10; % AccDec profile with peak
global PATH_MOVE_ACCDECSATURATED 	= 20; % AccDec profile saturated at v_max
global PATH_MOVE_DECDEC 			= 30; % DecDec profile (PathVel)
global PATH_MOVE_ACCACC 			= 40; % AccAcc profile (PathVel)
global PATH_POINTS_MAX_INDEX 		= 19; % Size of velocity & time point arrays for PathTime inputs

% Depricated global variables
global PATH_MOVE_NONE 			= 0;
global PATH_DEC_ACC_PEAK 		= 1;
global PATH_DEC_ACC_SATURATED 	= 2;
global PATH_ACC_DEC_PEAK 		= 10;
global PATH_ACC_DEC_SATURATED 	= 20;
global PATH_DEC_DEC 			= 30;
global PATH_ACC_ACC 			= 40;
global PATH_MAX_POINTS 			= 20;
