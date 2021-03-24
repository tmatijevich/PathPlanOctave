%!octave

function [Solution, Valid] = GetPoint(x0, TimePoints, VelocityPoints, NumberOfPoints, t, k = 1.0, PrintResult = false)
	% Determine the point on a piecewise linear velocity profile
	% Date: 2020-04-01
	% Created by: Tyler Matijevich
	
	% Reset the solution
	Solution.x = 0.0;
	Solution.v = 0.0;
	Solution.a = 0.0;
	
	% Input requirements
	% #1: Number of points
	if (NumberOfPoints < 2) || (NumberOfPoints > min(length(TimePoints), length(VelocityPoints)))
		printf("GetPoint call failed: Number of points %d exceeds limits %d, %d\n", NumberOfPoints, 2, min(length(TimePoints), length(VelocityPoints)));
		Valid = false;
		return;
	end
		
	% #2: Sequential times
	for i = 2:NumberOfPoints
		if TimePoints(i) < TimePoints(i - 1)
			printf("GetPoint call failed: Time point %d, %.3f is less than point %d, %.3f\n", i, TimePoints(i), i - 1, TimePoints(i-1));
			Valid = false;
			return;
		end % Higher index, less time?
	end % For loop time array
	
	% #3: Valid request time
	if (t < TimePoints(1)) || (t > TimePoints(NumberOfPoints))
		printf("GetPoint call failed: Requested time value %.3f exceeds time endpoints %.3f, %.3f\n", t, TimePoints(1), TimePoints(NumberOfPoints));
		Valid = false;
		return;
	
	% #4: Valid jerk factor
	elseif (k < 1.0) && (k > 2.0)
		printf("GetPoint call failed: Invalid acceleration gain %.3f\n", k);
		Valid = false;
		return;
	end
	
	% Compute the micro segments given the macro segments
	n = (length(TimePoints) - 1) * 3 + 1; % Number of micro segments (two intermediate points each macro segment)
	t_ 	= zeros(n, 1); % Time
	xt 	= zeros(n, 1); % Position
	vt 	= zeros(n, 1); % Velocity
	at 	= zeros(n, 1); % Acceleration
	jt 	= zeros(n, 1); % Jerk
	
	% Set the initial values of the arrays
	t_(1) 	= TimePoints(1);
	xt(1) 	= x0;
	vt(1) 	= VelocityPoints(1);
	% Jerk and acceleration for a micro segment is set at the start point of the segment
	
	% Loop through each macro segment
	for i = 2:NumberOfPoints
		b = i * 3 - 2; % Endpoint of the current macro segment
		a = b - 3; % Startpoint of the current macro segment
		% 1 2 3 4 5 6 7 8 9 
		% ^     ^
		% a     b
		%       ^     ^ 
		%       a     b 
		% a+1, a+2, a+3 required for Position
		% a+1, a+2      required for Time and Velocity
		% a+0, a+1, a+2 required for Jerk and Acceleration
		
		% Copy time and velocity values at the endpoint
		t_(b) 	= TimePoints(i);
		vt(b) 	= VelocityPoints(i);
		
		% Three cases to consider
		% 1. No time step - stack all values on the startpoint
		% 2. Jerk gain of 1.0 - use the average acceleration (infinite jerk)
		% 3. Jerk gain > 1.0 - determine if the acceleration is saturated
		
		% 1. No time step
		if t_(b) == t_(a)
			jt(a) 	= 0.0; % Zero jerk
			jt(a+1) = 0.0;
			jt(a+2) = 0.0;
			at(a) 	= 0.0; % Zero acceleration
			at(a+1) = 0.0;
			at(a+2) = 0.0;
			vt(a+1) = vt(a); % This does not protect against velocity jump (do not change velocity if there's no time step)
			vt(a+2) = vt(a);
			xt(a+1) = xt(a);
			xt(a+2) = xt(a);
			xt(a+3) = xt(a);
			t_(a+1) 	= t_(a);
			t_(a+2) 	= t_(a);
		else
			% Determine the peak acceleration (assuming a positive velocity change)
			apeak = (2.0 * abs(vt(b) - vt(a))) / (t_(b) - t_(a));
			if vt(b) != vt(a)
				% Determine the sign of the velocity change (sign of acceleration)
				VelocitySign = (vt(b) - vt(a)) / abs(vt(b) - vt(a));
				% Determine the average acceleration
				abar = abs(vt(b) - vt(a)) / (t_(b) - t_(a));
			else
				VelocitySign 	= 1.0;
				abar 			= 0.0;
			endif
			
			if k > 1.0 % Determine if there is infinite jerk (k = 1.0)
				if apeak <= k*abar % Determine if the acceleration profile will be saturated
					% Unstaturated acceleration profile
					t_(a+1) 	= t_(a) + (t_(b) - t_(a)) / 2.0;
					t_(a+2) 	= t_(a+1);
					jt(a) 	= VelocitySign * apeak / ((t_(b) - t_(a)) / 2.0);
					jt(a+1) = 0.0;
					jt(a+2) = (-1.0) * jt(a);
					at(a) 	= 0.0;
					at(a+1) = VelocitySign * apeak;
					at(a+2) = VelocitySign * apeak;
				else
					% Saturated acceleration profile
					NominalJerk = ((k*abar) ^ 2) / (k*abar * (t_(b) - t_(a)) - abs(vt(a) - vt(b)));
					jt(a) 	= VelocitySign * NominalJerk;
					jt(a+1) = 0.0;
					jt(a+2) = (-1.0) * jt(a);
					at(a) 	= 0.0;
					at(a+1) = VelocitySign * k*abar;
					at(a+2) = VelocitySign * k*abar;
					t_(a+1) 	= t_(a) + k*abar / NominalJerk;
					t_(a+2) 	= t_(b) - k*abar / NominalJerk;
				endif % Saturated acceleration?
			else % Infinite jerk
				% Stack the timepoints
				t_(a+1) 	= t_(a);
				t_(a+2) 	= t_(a);
				jt(a) 	= 0.0; % Zero jerk
				jt(a+1) = 0.0;
				jt(a+2) = 0.0;
				at(a) 	= VelocitySign * abar;
				at(a+1) = at(a);
				at(a+2) = at(a);
			endif % Infinite jerk?
			% Set the micro segment Position and Velocity
			vt(a+1) = 0.5 * jt(a+0) * (t_(a+1) - t_(a+0)) ^ 2 + at(a+0) * (t_(a+1) - t_(a+0)) + vt(a+0);
			vt(a+2) = 0.5 * jt(a+1) * (t_(a+2) - t_(a+1)) ^ 2 + at(a+1) * (t_(a+2) - t_(a+1)) + vt(a+1);
			xt(a+1) = (1/6) * jt(a+0) * (t_(a+1) - t_(a+0)) ^ 3 + 0.5 * at(a+0) * (t_(a+1) - t_(a+0)) ^ 2 + vt(a+0) * (t_(a+1) - t_(a+0)) + xt(a+0);
			xt(a+2) = (1/6) * jt(a+1) * (t_(a+2) - t_(a+1)) ^ 3 + 0.5 * at(a+1) * (t_(a+2) - t_(a+1)) ^ 2 + vt(a+1) * (t_(a+2) - t_(a+1)) + xt(a+1);
			xt(a+3) = (1/6) * jt(a+2) * (t_(a+3) - t_(a+2)) ^ 3 + 0.5 * at(a+2) * (t_(a+3) - t_(a+2)) ^ 2 + vt(a+2) * (t_(a+3) - t_(a+2)) + xt(a+2);
		endif % Non-zero time step?
	end % Loop macro segments
	
	% Find the requested segment
	if t == TimePoints(NumberOfPoints)
		seg = n - 1;
	elseif t == TimePoints(1)
		seg = 1;
	else
		for i = 2:n
			if t < t_(i)
				seg = i - 1;
				break;
			end % Within time?
		end % Loop array
	end % Final segment?
	
	% Set solution
	Solution.j = jt(seg);
	Solution.a = jt(seg) * (t - t_(seg)) + at(seg);
	Solution.v = 0.5 * jt(seg) * (t - t_(seg)) ^ 2 + at(seg) * (t - t_(seg)) + vt(seg);
	Solution.x = (1/6) * jt(seg) * (t - t_(seg)) ^ 3 + 0.5 * at(seg) * (t - t_(seg)) ^ 2 + vt(seg) * (t - t_(seg)) + xt(seg);
	Valid = true; 
	
	if PrintResult
		printf("GetPoint call: Pos %.3f, Vel %.3f, Acc %.3f\n", Solution.x, Solution.v, Solution.a);
	end
	
end % Function
