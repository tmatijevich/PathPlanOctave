%!octave

function [Solution, Valid] = GetTimeDur(dx, v0, vf, vmin, vmax, a, PrintResult = false)
	% Determine the minimum time to change velocity with acceleration over a distance
	% Assumptions:
	% 	- Positive distance and velocity
	% 	- Symmetric acceleration and deceleration
	% 	- Zero jerk
	% Date: 2020-12-23
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
		printf("GetTimeDur call failed: Implausible velocity limits %1.3f, %1.3f\n", vmin, vmax); 
		Valid = false; 
		return;
	
	% #2: Endpoint velocities within limits
	elseif (v0 < vmin) || (v0 > vmax) || (vf < vmin) || (vf > vmax)
		printf("GetTimeDur call failed: Endpoint velocities %1.3f, %1.3f exceed limits %1.3f, %1.3f\n", v0, vf, vmin, vmax); 
		Valid = false; 
		return;
	
	% #3: Positive distance and acceleration
	elseif (dx <= 0.0) || (a <= 0.0)
		printf("GetTimeDur call failed: Distance or acceleration non-positive %1.3f, %1.3f\n", dx, a); 
		Valid = false; 
		return;
		
	% #4: Valid distance given acceleration
	elseif dx < (abs(v0 ^ 2 - vf ^ 2) / (2.0 * a))
		printf("GetTimeDur call failed: Implausible distance %1.3f given minimum %1.3f\n", dx, abs(v0 ^ 2 - vf ^ 2) / (2.0 * a)); 
		Valid = false; 
		return;
		
	end % Requirements
	
	% There is no time advantage to decelerating below the final velocity, therefore only ACC_DEC profile will be considered
	VmaxDistance = (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * a);
	
	% Check if saturated profile
	if dx < VmaxDistance % Acc/dec profile with peak
		Solution.Move = PATH_ACC_DEC_PEAK;
		
		% Determine the peak velocity
		vpeak = sqrt(dx * a + (v0 ^ 2 + vf ^ 2) / 2.0);
		
		% Set the solution
		Solution.t(2) = (vpeak - v0) / a;
		Solution.t(3) = (vpeak - v0) / a;
		Solution.t(4) = (vpeak - v0) / a + (vpeak - vf) / a;
		Solution.v(2) = vpeak;
		Solution.v(3) = vpeak;
		
	else % Acc/dec profile saturated at vmax
		Solution.Move = PATH_ACC_DEC_SATURATED;
		
		% Determine the time at vmax
		t12 = (dx - VmaxDistance) / vmax;
		
		% Set the solution
		Solution.t(2) = (vmax - v0) / a;
		Solution.t(3) = (vmax - v0) / a + t12;
		Solution.t(4) = (vmax - v0) / a + t12 + (vmax - vf) / a;
		Solution.v(2) = vmax;
		Solution.v(3) = vmax;
		
	end % Profile type
	
	% Set common solution values and validate
	Solution.dx = dx;
	Solution.v(1) = v0;
	Solution.v(4) = vf;
	Solution.a = a;
	Valid = true;
	
	if PrintResult
		printf("GetTimeDur call: Time %1.3f, Vel %1.3f, Move %d\n", Solution.t(4), Solution.v(2), Solution.Move);
	end
	
end % Function