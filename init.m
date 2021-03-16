%!octave

% Add lib, samples, and plot subdirectories
addpath("lib", "samples", "plot", "test", "diag");

global PATH_MOVE_NONE 			= 0;
global PATH_DEC_ACC_PEAK 		= 1;
global PATH_DEC_ACC_SATURATED 	= 2;
global PATH_ACC_DEC_PEAK 		= 10;
global PATH_ACC_DEC_SATURATED 	= 20;
global PATH_DEC_DEC 			= 30;
global PATH_ACC_ACC 			= 40;
global PATH_MAX_POINTS 			= 20;
