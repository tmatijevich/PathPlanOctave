%!octave

function [Solution, Valid] = GetTimeDiff(dx, v0, vf, vmin, vmax, a, PrintResult = false)
	% Determine the differnce between the time minimizing and time maximizing velocity profiles
	% Assumptions:
	% 	- Positive distance and velocity
	% 	- Symmetric acceleration and deceleration
	% 	- Zero jerk
	% Date: 2020-03-25
	% Created by: Tyler Matijevich
	
	% Reference global variables
	global PATH_MOVE_NONE;
	global PATH_DEC_ACC_PEAK;
	global PATH_DEC_ACC_SATURATED;
	global PATH_ACC_DEC_PEAK;
	global PATH_ACC_DEC_SATURATED;
	
	% Reset solution
	Solution.vA = [0.0, 0.0, 0.0, 0.0];
	Solution.tA = [0.0, 0.0, 0.0, 0.0];
	Solution.MoveA = PATH_MOVE_NONE;
	Solution.vB = [0.0, 0.0, 0.0, 0.0];
	Solution.tB = [0.0, 0.0, 0.0, 0.0];
	Solution.MoveB = PATH_MOVE_NONE;
	Solution.tdiff = 0.0;
	
	% Input requirements
	% #1: Plausible velocity limits
	if (vmin <= 0.0) || (vmax <= vmin) % *** Requires non-zero vmin (& vmax)
		printf("GetTimeDiff call failed: Implausible velocity limits %1.3f, %1.3f\n", vmin, vmax); 
		Valid = false; 
		return;
	
	% #2: Endpoint velocities within limits
	elseif (v0 < vmin) || (v0 > vmax) || (vf < vmin) || (vf > vmax) % *** Thus requires non-zero endpoint velocities
		printf("GetTimeDiff call failed: Endpoint velocities %1.3f, %1.3f exceed limits %1.3f, %1.3f\n", v0, vf, vmin, vmax); 
		Valid = false; 
		return;
	
	% #3: Positive distance and acceleration
	elseif (dx <= 0.0) || (a <= 0.0)
		printf("GetTimeDiff call failed: Distance or acceleration non-positive %1.3f, %1.3f\n", dx, a); 
		Valid = false; 
		return;
		
	% #4: Valid distance given acceleration
	elseif dx < (abs(v0 ^ 2 - vf ^ 2) / (2.0 * a))
		printf("GetTimeDiff call failed: Implausible distance %1.3f given minimum %1.3f\n", dx, abs(v0 ^ 2 - vf ^ 2) / (2.0 * a)); 
		Valid = false; 
		return;
		
	end
	
	% Determine the time minimizing profile
	VmaxDistance = (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * a);
	if dx < VmaxDistance % Acc/dec profile with peak
		Solution.MoveA = PATH_ACC_DEC_PEAK;
		
		% Determine the peak velocity
		Solution.vA(2) = sqrt(dx * a + (v0 ^ 2 + vf ^ 2) / 2.0);
		Solution.vA(3) = Solution.vA(2);
		Solution.tA(2) = (Solution.vA(2) - v0) / a;
		Solution.tA(3) = (Solution.vA(2) - v0) / a;
		Solution.tA(4) = (Solution.vA(2) - v0) / a + (Solution.vA(3) - vf) / a;
		
	else % Acc/dec profile saturated at vmax
		Solution.MoveA = PATH_ACC_DEC_SATURATED;
		
		% Determine time at set velocity
		tVmax12 = (dx - VmaxDistance) / vmax;
		Solution.vA(2) = vmax;
		Solution.vA(3) = vmax;
		Solution.tA(2) = (vmax - v0) / a;
		Solution.tA(3) = (vmax - v0) / a + tVmax12;
		Solution.tA(4) = (vmax - v0) / a + tVmax12 + (vmax - vf) / a;
		
	end % Vmax distance threshold?
	
	% Determine the time maximizing profile
	VminDistance = (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * a);
	if dx < VminDistance % Dec/acc profile with dip
		Solution.MoveB = PATH_DEC_ACC_PEAK;
		
		% Determine the dip velocity
		Solution.vB(2) = sqrt((v0 ^ 2 + vf ^ 2) / 2.0 - dx * a);
		Solution.vB(3) = Solution.vB(2);
		Solution.tB(2) = (v0 - Solution.vB(2)) / a;
		Solution.tB(3) = (v0 - Solution.vB(2)) / a;
		Solution.tB(4) = (v0 - Solution.vB(2)) / a + (vf - Solution.vB(3)) / a;
		
	else % Dec/acc profile saturated at vmin
		Solution.MoveB = PATH_DEC_ACC_SATURATED;
		
		% Determine the time at set velocity
		tVmin12 = (dx - VminDistance) / vmin;
		Solution.vB(2) = vmin;
		Solution.vB(3) = vmin;
		Solution.tB(2) = (v0 - vmin) / a;
		Solution.tB(3) = (v0 - vmin) / a + tVmin12;
		Solution.tB(4) = (v0 - vmin) / a + tVmin12 + (vf - vmin) / a;
		
	end % Vmin distance threshold?
	
	% Set solution
	Solution.vA(1) = v0;
	Solution.vA(4) = vf;
	Solution.vB(1) = v0;
	Solution.vB(4) = vf;
	Solution.tdiff = Solution.tB(4) - Solution.tA(4);
	Valid = true;
	
	if PrintResult
		printf("GetTimeDiff call: Time diff %.3f - %.3f = %.3f, Moves %d, %d\n", Solution.tB(4), Solution.tA(4), Solution.tdiff, Solution.MoveA, Solution.MoveB);
	end
	
end % Function
