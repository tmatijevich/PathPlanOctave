%!octave

function [Solution, Valid] = GetVel(dt, dx, v0, vf, vmin, vmax, a, PrintResult = false)
	% Determine the minimum intermediate velocity to change velocity with acceleration 
	% in time over a distance
	% Assumptions:
	% 	- Positive distance and velocity
	% 	- Symmetric acceleration and deceleration
	% 	- Zero jerk
	% Date: 2020-12-30
	% Created by: Tyler Matijevich
	
	% Reference global variables
	global KIN_MOVE_NONE;
	global KIN_DEC_ACC_TRI;
	global KIN_DEC_ACC_TRAP;
	global KIN_DEC_DEC_TRAP;
	global KIN_ACC_DEC_TRI;
	global KIN_ACC_DEC_TRAP;
	global KIN_ACC_ACC_TRAP;
	
	% Reset solution
	Solution.t = [0.0, 0.0, 0.0, 0.0];
	Solution.dx = 0.0;
	Solution.v = [0.0, 0.0, 0.0, 0.0];
	Solution.a = 0.0;
	Solution.Move = KIN_MOVE_NONE;
	
	% Input requirements
	% #1: Plausible velocity limits
	if (vmin < 0.0) || (vmax <= vmin)
		printf("GetVel call failed: Implausible velocity limits %.3f, %.3f\n", vmin, vmax); 
		Valid = false; 
		return;
	
	% #2: Endpoint velocities within limits
	elseif (v0 < vmin) || (v0 > vmax) || (vf < vmin) || (vf > vmax)
		printf("GetVel call failed: Endpoint velocities %.3f, %.3f exceed limits %.3f, %.3f\n", v0, vf, vmin, vmax); 
		Valid = false; 
		return;
	
	% #3: Positive time, distance, and acceleration
	elseif (dt <= 0.0) || (dx <= 0.0) || (a <= 0.0)
		printf("GetVel call failed: Time, distance, or acceleration non-positive %.3f, %.3f, %.3f\n", dt, dx, a); 
		Valid = false; 
		return;
		
	% #4: Valid time and distance given acceleration
	elseif (dt < abs(v0 - vf) / a) || (dx < abs(v0 ^ 2 - vf ^ 2) / (2.0 * a))
		printf("GetVel call failed: Impossible time %.3f or distance %.3f given limits %.3f, %.3f\n", dt, dx, abs(v0 - vf) / a, abs(v0 ^ 2 - vf ^ 2) / a);
		Valid = false; 
		return;
		
	end % Requirements
	
	% Check if the distance can be fulfilled given the acceleration, max velocity, and time
	vpeak = (dt * a + v0 + vf) / 2.0;
	if vpeak <= vmax
		MaxDistance = (2.0 * vpeak ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * a);
	else
		MaxDistance = (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * a) + vmax * (dt - ((2.0 * vmax - v0 - vf) / a));
	end
	if dx > MaxDistance
		printf("GetVel call failed: Distance %.3f exceeds maximum %.3f\n", dx, MaxDistance);
		Valid = false;
		return;
	end
	
	vdip = (dt * a - v0 - vf) / 2.0;
	if vdip >= vmin
		MinDistance = (v0 ^ 2 + vf ^ 2 - 2.0 * vdip ^ 2) / (2.0 * a);
	else
		MinDistance = (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * a) + vmin * (dt - ((v0 + vf - 2.0 * vmin) / a));
	end
	if dx < MinDistance
		printf("GetVel call failed: Distance %.3f exceeds minimum %.3f\n", dx, MinDistance);
		Valid = false;
		return;
	end
	
	% Determine nominal distance which decide acceleration directions
	NominalTime = abs(v0 - vf) / a;
	NominalDistance = abs(v0 ^ 2 - vf ^ 2) / (2.0 * a);
	a1NominalDistance = v0 * (dt - NominalTime) + NominalDistance;
	a2NominalDistance = NominalDistance + vf * (dt - NominalTime);
	
	% Determine the sign of both acceleration phases
	if dx > a1NominalDistance
		a1Sign = 1.0;
	elseif dx < a1NominalDistance
		a1Sign = -1.0;
	else
		a1Sign = 0.0;
	end
	
	if dx > a2NominalDistance
		a2Sign = -1.0;
	elseif dx < a2NominalDistance
		a2Sign = 1.0;
	else
		a2Sign = 0.0;
	end
	
	% Solve for the three cases
	if a1Sign == 1.0 && a2Sign == -1.0 % ACC_DEC
		% Assume the move is a trapezoid approaching a triangle profile only when dx = MaxDistance and vpeak <= vmax
		Solution.Move = KIN_ACC_DEC_TRAP;
		
		p2 = - 1.0 / a;
		p1 = dt - NominalTime + (2.0 * max(v0, vf)) / a;
		p0 = NominalDistance - dx - (max(v0, vf) ^ 2) / a;
		
	elseif a1Sign == a2Sign % ACC_ACC or DEC_DEC
		if a1Sign == 1.0
			Solution.Move = KIN_ACC_ACC_TRAP;
		else
			Solution.Move = KIN_DEC_DEC_TRAP;
		end
		
		Solution.v(2) = (dx - NominalDistance) / (dt - NominalTime);
		
	else % DEC_ACC
		Solution.Move = KIN_DEC_ACC_TRAP;
		
		p2 = 1.0 / a;
		p1 = dt - NominalTime - (2.0 * min(v0, vf)) / a;
		p0 = NominalDistance - dx + (min(v0, vf) ^ 2) / a;
		
	end
	
	% Use quadratic function
	if (Solution.Move == KIN_ACC_DEC_TRAP) || (Solution.Move == KIN_DEC_ACC_TRAP)
		[RootsSolution, RootsValid] = SecondOrderRoots(p2, p1, p0);
		
		if !RootsValid
			printf("GetVel call failed: Invalid roots for movement\n");
			Valid = false;
			return;
			
		else
			% Choose the appropriate root
			if Solution.Move == KIN_ACC_DEC_TRAP
				Solution.v(2) = min(RootsSolution.r1, RootsSolution.r2);
			else
				Solution.v(2) = max(RootsSolution.r1, RootsSolution.r2);
			end
			
		end
	end
	
	Solution.v(1) = v0;
	Solution.v(3) = Solution.v(2);
	Solution.v(4) = vf;
	Solution.t(2) = abs(v0 - Solution.v(2)) / a;
	Solution.t(3) = dt - abs(Solution.v(3) - vf) / a;
	Solution.t(4) = dt;
	Solution.a = a;
	Solution.dx = dx;
	Valid = true;
	
	if PrintResult
		printf("GetVel call: Vel %.3f, Move %d\n", Solution.v(2), Solution.Move);
	end
	
end
