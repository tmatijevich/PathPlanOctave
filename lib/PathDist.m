%!octave

function [Solution, Valid] = GetDist(dt, v0, vf, vmin, vmax, a, PrintResult = false)
	% Determine the maximum distance to change velocity with acceleration in time
	% Assumptions:
	% 	- Positive distance and velocity
	% 	- Symmetric acceleration and deceleration
	% 	- Zero jerk
	% Date: 2020-12-29
	% Created by: Tyler Matijevich
	
	% Reference global variables
	global PATH_MOVE_NONE;
	global PATH_ACC_DEC_PEAK;
	global PATH_ACC_DEC_SATURATED;
	
	% Reset solution
	Solution.t = [0.0, 0.0, 0.0, 0.0];
	Solution.dx = 0.0;
	Solution.v = [0.0, 0.0, 0.0, 0.0];
	Solution.a = 0.0;
	Solution.Move = PATH_MOVE_NONE;
	
	% Input requirements
	% #1: Plausible velocity limits
	if (vmin < 0.0) || (vmax <= vmin)
		printf("GetDist call failed: Implausible velocity limits %1.3f, %1.3f\n", vmin, vmax); 
		Valid = false; 
		return;
	
	% #2: Endpoint velocities within limits
	elseif (v0 < vmin) || (v0 > vmax) || (vf < vmin) || (vf > vmax)
		printf("GetDist call failed: Endpoint velocities %1.3f, %1.3f exceed limits %1.3f, %1.3f\n", v0, vf, vmin, vmax); 
		Valid = false; 
		return;
	
	% #3: Positive timespan and acceleration
	elseif (dt <= 0.0) || (a <= 0.0)
		printf("GetDist call failed: Timespan or acceleration non-positive %1.3f, %1.3f\n", dt, a); 
		Valid = false; 
		return;
		
	% #4: Valid timespan given acceleration
	elseif dt < (abs(v0 - vf) / a)
		printf("GetDist call failed: Impossible timepsan %1.3f given minimum %1.3f\n", dt, abs(v0 - vf) / a); 
		Valid = false; 
		return;
		
	end % Requirements
	
	% There is no distance advantage to decelerate then accelerate, therefore only ACC_DEC profiles will be considered
	VmaxTime = (2 * vmax - v0 - vf) / a;
	
	% Check if saturated profile
	if dt < VmaxTime % Acc/dec profile with peak
		Solution.Move = PATH_ACC_DEC_PEAK;
		
		% Determine the peak velocity
		vpeak = (dt * a + v0 + vf) / 2.0;
		
		% Set the solution
		Solution.dx = (2.0 * vpeak ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * a);
		Solution.t(2) = (vpeak - v0) / a;
		Solution.t(3) = (vpeak - v0) / a;
		Solution.v(2) = vpeak;
		Solution.v(3) = vpeak;
		
	else % Acc/dec profile saturated at vmax
		Solution.Move = PATH_ACC_DEC_SATURATED;
		
		% Determine the distance at vmax
		dx12 = (dt - (2.0 * vmax - v0 - vf) / a) * vmax;
		
		% Set the solution
		Solution.dx = dx12 + (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * a);
		Solution.t(2) = (vmax - v0) / a;
		Solution.t(3) = (vmax - v0) / a + dx12 / vmax;
		Solution.v(2) = vmax;
		Solution.v(3) = vmax;
		
	end
	
	% Set common solution values and validate
	Solution.t(4) = dt;
	Solution.v(1) = v0;
	Solution.v(4) = vf;
	Solution.a = a;
	Valid = true;
	
	if PrintResult
		printf("GetDist call: Dist %.3f, Vel %.3f, Move %d\n", Solution.dx, Solution.v(2), Solution.Move);
	end
	
end
