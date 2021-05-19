%!octave

function [Solution, Valid] = GetAcc(dt, dx, v0, vf, vmin, vmax, PrintResult = false)
	% Determine the minimum acceleration to change velocity in time over a distance
	% Assumptions:
	% 	- Positive distance and velocity
	% 	- Symmetric acceleration and deceleration
	% 	- Zero jerk
	% Date: 2020-04-10
	% Created by: Tyler Matijevich
	
	% Reference global variables
	global PATH_MOVE_NONE;
	global PATH_DEC_ACC_PEAK;
	global PATH_DEC_ACC_SATURATED;
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
		printf("GetAcc call failed: Implausible velocity limits %.3f, %.3f\n", vmin, vmax); 
		Valid = false; 
		return;
	
	% #2: Endpoint velocities within limits
	elseif (v0 < vmin) || (v0 > vmax) || (vf < vmin) || (vf > vmax)
		printf("GetAcc call failed: Endpoint velocities %.3f, %.3f exceed limits %.3f, %.3f\n", v0, vf, vmin, vmax); 
		Valid = false; 
		return;
	
	% #3: Positive time duration and distance
	elseif (dt <= 0.0) || (dx <= 0.0)
		printf("GetAcc call failed: Time or distance non-positive %.3f, %.3f\n", dt, dx); 
		Valid = false; 
		return;
		
	% #4: Valid distance given velocity limits
	elseif (dx <= (vmin * dt)) || (dx >= (vmax * dt))
		printf("GetAcc call failed: Impossible distance %.3f given limits %.3f, %.3f\n", dx, vmin * dt, vmax * dt); 
		Valid = false; 
		return;
		
	end % Requirements
	
	% The intermediate velocity point v12 is either >= v0, vf or <= v0, vf for a symmetric acc/dec profile
	% Determine if 1. ACC 2. DEC or 1. DEC 2. ACC
	NominalDistance = 0.5 * dt * (v0 + vf); % Area of a trapezoid
	
	if dx >= NominalDistance % 1. ACC 2. DEC
		% Determine if saturated
		VmaxDistance = (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * ((2.0 * vmax - v0 - vf) / dt));
		% NOTE: There is no dx >= NominalDistance when v0 = vf = vmax that also passes requirement #4. This protects against divide by zero.
		
		if dx < VmaxDistance % Acc/dec profile with peak
			Solution.Move = PATH_ACC_DEC_PEAK;
			
		else % Acc/dec profile saturated at vmax
			Solution.Move = PATH_ACC_DEC_SATURATED;
			Solution.a = ((2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / 2.0 - (2.0 * vmax - v0 - vf) * vmax) / (dx - dt * vmax); % Protected by requirement #4
			if Solution.a > 0.0
				Solution.v(2) = vmax;
				Solution.v(3) = vmax;
				Solution.t(2) = (vmax - v0) / Solution.a;
				Solution.t(3) = dt - (vmax - vf) / Solution.a;
			else
				printf("GetAcc call warning: Unexpected non-positive acceleration\n"); % Should not happen
				Solution.v(2) = v0;
				Solution.v(3) = v0;
				Solution.t(2) = 0.0;
				Solution.t(3) = 0.0;
			end % Positive acceleration
			
		end % VmaxDistance?
		
	else % 1. DEC 2. ACC
		% Determine if saturated profile
		VminDistance = (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * ((v0 + vf - 2.0 * vmin) / dt));
		% NOTE: There is no dx < NominalDistance when v0 = vf = vmin that also passes requirement #4. This protects against divide by zero.
		
		if dx > VminDistance % Dec/acc profile with dip
			Solution.Move = PATH_DEC_ACC_PEAK;
			
		else % Dec/acc profile saturated at vmin
			Solution.Move = PATH_DEC_ACC_SATURATED;
			Solution.a = ((v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / 2.0 - (v0 + vf - 2.0 * vmin) * vmin) / (dx - dt * vmin); % Protected by requirement #4
			if Solution.a > 0.0
				Solution.v(2) = vmin;
				Solution.v(3) = vmin;
				Solution.t(2) = (v0 - vmin) / Solution.a;
				Solution.t(3) = dt - (vf - vmin) / Solution.a;
			else 
				printf("GetAcc call warning: Unexpected non-positive acceleration\n"); % Should not happen
				Solution.v(2) = v0;
				Solution.v(3) = v0;
				Solution.t(2) = 0.0;
				Solution.t(3) = 0.0;
			end % Positive acceleration
		end % VminDistance?
		
	end % NominalDistance?
	
	if (Solution.Move == PATH_ACC_DEC_PEAK) || (Solution.Move == PATH_DEC_ACC_PEAK)
		p2 = 2.0 * dt;
		p1 = -4.0 * dx;
		p0 = 2.0 * dx * (v0 + vf) - dt * (v0 ^ 2 + vf ^ 2);
		[RootsSolution, RootsValid] = SecondOrderRoots(p2, p1, p0);
		
		if !RootsValid
			printf("GetAcc call failed: Invalid roots for peak movement\n");
			Valid = false;
			return;
			
		else % Roots are valid
			if Solution.Move == PATH_ACC_DEC_PEAK % Vmax
				Solution.v(2) = max(RootsSolution.r1, RootsSolution.r2);
				Solution.v(3) = Solution.v(2);
				
			else % Vmin
				Solution.v(2) = min(RootsSolution.r1, RootsSolution.r2);
				Solution.v(3) = Solution.v(2);
				
			end
			
			Solution.a = abs(2.0 * Solution.v(2) - v0 - vf) / dt;
			if Solution.a > 0.0
				Solution.t(2) = abs(Solution.v(2) - v0) / Solution.a;
				Solution.t(3) = Solution.t(2);
			else % A flat line, dx = NominalDistance and v0 = vf
				Solution.t(2) = 0.0;
				Solution.t(3) = 0.0;
			end
			
		end % Roots valid?
	end % Peak movement?
	
	% Set common solution values and validate
	Solution.t(4) = dt;
	Solution.dx = dx;
	Solution.v(1) = v0;
	Solution.v(4) = vf;
	Valid = true;
	
	if PrintResult
		printf("GetAcc call: Acc %.3f, Vel %.3f, Move %d\n", Solution.a, Solution.v(2), Solution.Move);
	end
	
end % Function
