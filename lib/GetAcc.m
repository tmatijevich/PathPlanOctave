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
	global KIN_MOVE_NONE;
	global KIN_DEC_ACC_TRI;
	global KIN_DEC_ACC_TRAP;
	global KIN_ACC_DEC_TRI;
	global KIN_ACC_DEC_TRAP;
	
	% Reset solution
	Solution.t = [0.0, 0.0, 0.0, 0.0];
	Solution.v = [0.0, 0.0, 0.0, 0.0];
	Solution.a = 0.0;
	Solution.Move = KIN_MOVE_NONE;
	
	% Input requirements
	% #1: Plausible velocity limits
	if (vmin < 0.0) || (vmax <= vmin)
		printf("GetAcc call failed: Implausible velocity limits %1.3f, %1.3f\n", vmin, vmax); 
		Valid = false; 
		return;
	
	% #2: Endpoint velocities within limits
	elseif (v0 < vmin) || (v0 > vmax) || (vf < vmin) || (vf > vmax)
		printf("GetAcc call failed: Endpoint velocities %1.3f, %1.3f exceed limits %1.3f, %1.3f\n", v0, vf, vmin, vmax); 
		Valid = false; 
		return;
	
	% #3: Positive time and distance
	elseif (dt <= 0.0) || (dx <= 0.0)
		printf("GetAcc call failed: Time or distance non-positive %1.3f, %1.3f\n", dt, dx); 
		Valid = false; 
		return;
		
	% #4: Valid distance given velocity limits
	elseif (dx <= (vmin * dt)) || (dx >= (vmax * dt))
		printf("GetAcc call failed: Impossible distance %1.3f given limits %1.3f, %1.3f\n", dx, vmin * dt, vmax * dt); 
		Valid = false; 
		return;
		
	end % Requirements
	
	% v12 > v0, vf or v12 < v0, vf for a velocity profile with symmetric accel and decel
	% Determine if 1. ACC 2. DEC or 1. DEC 2. ACC
	NominalDistance = 0.5 * dt * (v0 + vf); % Area of a trapezoid
	
	if dx >= NominalDistance % 1. ACC 2. DEC
		% Determine if triangle or trapezoid profile
		VmaxDistance = (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * ((2.0 * vmax - v0 - vf) / dt));
		
		if dx < VmaxDistance % Triangle profile with a peak
			Solution.Move = KIN_ACC_DEC_TRI;
			
		else % Trapezoid profile at vmax
			Solution.Move = KIN_ACC_DEC_TRAP;
			Solution.a = ((2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / 2.0 - (2.0 * vmax - v0 - vf) * vmax) / (dx - dt * vmax);
			Solution.v(2) = vmax;
			Solution.v(3) = vmax;
			Solution.t(2) = (vmax - v0) / Solution.a;
			Solution.t(3) = dt - (vmax - vf) / Solution.a;
			
		end % VmaxDistance?
		
	else % 1. DEC 2. ACC
		% Determine if triangle or trapezoid profile
		VminDistance = (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * ((v0 + vf - 2.0 * vmin) / dt));
		
		if dx > VminDistance % Triangle profile with a dip
			Solution.Move = KIN_DEC_ACC_TRI;
			
		else % Trapezoid profile at vmin
			Solution.Move = KIN_DEC_ACC_TRAP;
			Solution.a = ((v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / 2.0 - (v0 + vf - 2.0 * vmin) * vmin) / (dx - dt * vmin);
			Solution.v(2) = vmin;
			Solution.v(3) = vmin;
			Solution.t(2) = (v0 - vmin) / Solution.a;
			Solution.t(3) = dt - (vf - vmin) / Solution.a;
			
		end % VminDistance?
		
	end % NominalDistance?
	
	if (Solution.Move == KIN_ACC_DEC_TRI) || (Solution.Move == KIN_DEC_ACC_TRI)
		p2 = 2.0 * dt;
		p1 = -4.0 * dx;
		p0 = 2.0 * dx * (v0 + vf) - dt * (v0 ^ 2 + vf ^ 2);
		[RootsSolution, RootsValid] = SecondOrderRoots(p2, p1, p0);
		
		if !RootsValid
			printf("GetAcc call failed: Invalid roots for triangle movement\n");
			Valid = false;
			return;
			
		else % Roots are valid
			if Solution.Move == KIN_ACC_DEC_TRI % Vmax
				Solution.v(2) = max(RootsSolution.r1, RootsSolution.r2);
				Solution.v(3) = Solution.v(2);
				
			else % Vmin
				Solution.v(2) = min(RootsSolution.r1, RootsSolution.r2);
				Solution.v(3) = Solution.v(2);
				
			end
			
			Solution.a = abs(2.0 * Solution.v(2) - v0 - vf) / dt;
			Solution.t(2) = abs(Solution.v(2) - v0) / Solution.a;
			Solution.t(3) = Solution.t(2);
			
		end % Roots valid?
	end % Triangle movement?
	
	% Set common solution values and validate
	Solution.t(4) = dt;
	Solution.v(1) = v0;
	Solution.v(4) = vf;
	Valid = true;
	
	if PrintResult
		printf("GetAcc call: Acc %1.3f, Vel %1.3f, Move %d\n", Solution.a, Solution.v(2), Solution.Move);
	end
	
end % Function
