%!octave

function [Solution, Valid] = GetAccInTimeDiff(tdiff, dx, v0, vf, vmin, vmax, PrintResult = false)
	% Determine the minimum acceleration required to achieve both movement extremes
	% within a specific time difference
	% Assumptions:
	% 	- Positive distance and velocity
	% 	- Symmetric acceleration and deceleration
	% 	- Zero jerk
	% Date: 2020-04-01
	% Created by: Tyler Matijevich
	
	% Reference global variables
	global PATH_MOVE_NONE;
	global PATH_DEC_ACC_PEAK;
	global PATH_DEC_ACC_SATURATED;
	global PATH_ACC_DEC_PEAK;
	global PATH_ACC_DEC_SATURATED;
	
	% Reset solution
	Solution.vmin.t 	= [0.0, 0.0, 0.0, 0.0];
	Solution.vmin.dx 	= 0.0;
	Solution.vmin.v 	= [0.0, 0.0, 0.0, 0.0];
	Solution.vmin.a 	= 0.0;
	Solution.vmin.Move 	= PATH_MOVE_NONE;
	Solution.vmax.t 	= [0.0, 0.0, 0.0, 0.0];
	Solution.vmax.dx 	= 0.0;
	Solution.vmax.v 	= [0.0, 0.0, 0.0, 0.0];
	Solution.vmax.a 	= 0.0;
	Solution.vmax.Move 	= PATH_MOVE_NONE;
	
	% Input requirements
	% #1: Plausible velocity limits
	if (vmin < 0.0) || (vmax <= vmin)
		printf("GetAccInTimeDiff call failed: Implausible velocity limits %.3f, %.3f\n", vmin, vmax); 
		Valid = false; 
		return;
	
	% #2: Endpoint velocities within limits
	elseif (v0 < vmin) || (v0 > vmax) || (vf < vmin) || (vf > vmax)
		printf("GetAccInTimeDiff call failed: Endpoint velocities %.3f, %.3f exceed limits %.3f, %.3f\n", v0, vf, vmin, vmax); 
		Valid = false; 
		return;
	
	% #3: Positive time and distance
	elseif (tdiff <= 0.0) || (dx <= 0.0)
		printf("GetAccInTimeDiff call failed: Time difference or distance non-positive %.3f, %.3f\n", tdiff, dx); 
		Valid = false; 
		return;
		
	% #4: Valid distance given velocity limits
	elseif (tdiff >= (dx / vmin - dx / vmax))
		printf("GetAccInTimeDiff call failed: Impossible time difference %.3f given velocity limits %.3f\n", tdiff, dx / vmin - dx / vmax); 
		Valid = false; 
		return;
		
	end % Requirements
	
	% Determine the saturated acceleration limit for the time minimizing profile
	VmaxSatAccLimit = (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * dx); % Accelerations higher than this will saturate the vmax profile
	VminSatAccLimit = (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * dx);
	
	% Determine the time duration at the saturation acceleration limit
	VmaxSatTimeLimit = (2.0 * vmax - v0 - vf) / VmaxSatAccLimit; % Any time duration larger that this will saturate the vmax profile
	VminSatTimeLimit = (v0 + vf - 2.0 * vmin) / VminSatAccLimit;
	
	% Determine the time duration at the alternative profile when saturated
	if VminSatAccLimit < VmaxSatAccLimit
		% The vmax profile is not saturated
		PeakVel = sqrt(dx * VminSatAccLimit + (v0 ^ 2 + vf ^ 2) / 2.0);
		VmaxTimeAtVminSatLimit = (2.0 * PeakVel - v0 - vf) / VminSatAccLimit;
	else
		% The vmax profile is saturated
		SaturatedTime = (dx - (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * VminSatAccLimit)) / vmax;
		VmaxTimeAtVminSatLimit = (2.0 * vmax - v0 - vf) / VminSatAccLimit + SaturatedTime;
	end % Vmin saturated acc limit versus vmax
	
	if VmaxSatAccLimit < VminSatAccLimit
		% The vmin profile is not saturated
		DipVel = sqrt(dx * VmaxSatAccLimit - (v0 ^ 2 + vf ^ 2) / 2.0);
		VminTimeAtVmaxSatLimit = (v0 + vf - 2.0 * DipVel) / VmaxSatAccLimit;
	else
		% The vmin profile is saturated
		SaturatedTime = (dx - (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * VmaxSatAccLimit)) / vmin;
		VminTimeAtVmaxSatLimit = (v0 + vf - 2.0 * vmin) / VmaxSatAccLimit + SaturatedTime;
	end % Vmax saturated acc limit verus vmin
	
	% Determine the time differences at the saturation limits
	TimeDiffAtVmaxSatLimit = VminTimeAtVmaxSatLimit - VmaxSatTimeLimit; % For the same acceleration, the vmin profile time duration is always greater than the vmax profile time
	TimeDiffAtVminSatLimit = VminSatTimeLimit - VmaxTimeAtVminSatLimit;
	
	
	
	% Calculate constants
	cVminTime 		= v0 + vf - 2.0 * vmin;
	cVminDistance 	= (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * vmin);
	cVmaxTime 		= 2.0 * vmax - v0 - vf;
	cVmaxDistance 	= (2.0 * vmax ^ 2.0 - v0 ^ 2 - vf ^ 2) / (2.0 * vmax);
	
	% Test the solution of two saturated profiles
	SaturatedAcc = ((cVminDistance - cVmaxDistance) - (cVminTime - cVmaxTime)) / ((dx / vmin - dx / vmax) - tdiff);
	if SaturatedAcc <= 0.0
		printf("GetAccInTimeDiff call failed: Implausible acceleration %.3f\n", SaturatedAcc);
		Valid = false;
		return;
	end
	
	VminAcc = (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * dx);
	VmaxAcc = (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * dx);
	
	if SaturatedAcc < VminAcc 
		printf("GetAccInTimeDiff call failed: Solution requires higher order solver\n");
		Valid = false;
		return;
		
	elseif SaturatedAcc < VmaxAcc
		% Calculate the ACC_DEC vmax profile
		Solution.vmax.Move = PATH_ACC_DEC_PEAK;
		Solution.vmin.Move = PATH_DEC_ACC_SATURATED;
		
		c3 = cVminTime - cVminDistance + v0 + vf;
		c4 = tdiff - dx / vmin;
		c5 = (v0 ^ 2 + vf ^ 2) / 2.0;
		
		[RootsSolution, RootsValid] = SecondOrderRoots(0.25 * c4 ^ 2, - 0.5 * c3 * c4 - dx, 0.25 * c3 ^ 2 - c5);
		if !RootsValid
			printf("GetAccInTimeDiff call failed: Imaginary roots\n");
			Valid = false;
			return;
		
		else % Roots are valid
			MinRoot = min(RootsSolution.r1, RootsSolution.r2);
			MaxRoot = max(RootsSolution.r1, RootsSolution.r2);
			if (MaxRoot > 0.0) && ((MinRoot < 0.0) || (MinRoot == MaxRoot))
				Solution.vmin.a = MaxRoot;
				Solution.vmax.a = MaxRoot;
			else
				printf("GetAccInTimeDiff call failed: Invalid roots for cases %d, %d\n", Solution.vmin.Move, Solution.vmax.Move);
				Valid = false;
				return;
			end % Root finding
		end % Valid roots
		
	else % Both profiles are saturated
		Solution.vmin.Move 	= PATH_DEC_ACC_SATURATED;
		Solution.vmax.Move 	= PATH_ACC_DEC_SATURATED;
		Solution.vmin.a 	= SaturatedAcc;
		Solution.vmax.a 	= SaturatedAcc;
		
	end % Test saturated acceleration against nominal values
	
	% Vmin profile is always saturated (higher order)
	Solution.vmin.t(1) 	= 0.0;
	Solution.vmin.t(2) 	= (v0 - vmin) / Solution.vmin.a;
	VminDistance 		= (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * Solution.vmin.a);
	Solution.vmin.t(3) 	= (v0 - vmin) / Solution.vmin.a + (dx - VminDistance) / vmin;
	Solution.vmin.t(4) 	= (v0 - vmin) / Solution.vmin.a + (dx - VminDistance) / vmin + (vf - vmin) / Solution.vmin.a;
	Solution.vmin.dx 	= dx;
	Solution.vmin.v 	= [v0, vmin, vmin, vf];
	
	[TimeSolution, TimeValid] = GetTime(dx, v0, vf, vmin, vmax, Solution.vmax.a);
	if TimeValid
		Solution.vmax.t = TimeSolution.t;
		Solution.vmax.v = TimeSolution.v;
	else
		printf("GetAccInTimeDiff call failed: An unexpected error occured, please see octave function\n");
		Valid = false;
		return;
	end
	Solution.vmax.dx = dx;
	
	Valid = true;
	
	if PrintResult
		printf("GetAccInTimeDiff call: Acc %.3f, Cases %d, %d\n", Solution.vmin.a, Solution.vmin.Move, Solution.vmax.Move);
	end
	
end % Function
