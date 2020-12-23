%!octave

% Add lib, samples, and plot subdirectories
addpath("lib", "samples", "plot", "test");

global KIN_MOVE_NONE 	= 0;
global KIN_DEC_ACC_TRI 	= 1;
global KIN_DEC_ACC_TRAP = 2;
global KIN_ACC_DEC_TRI 	= 10;
global KIN_ACC_DEC_TRAP = 20;
