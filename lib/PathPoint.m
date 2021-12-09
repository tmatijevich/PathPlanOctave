%!octave

function [solution, valid] = PathPoint(x_0, t_, v_, n, t, k = 1.0, printResult = false)
	% function [solution, valid] = PathPoint(x_0, t_, v_, n, t, printResult = false)
	% Determine the point on a piecewise linear velocity profile
	% Date: 2020-04-01
	% Created by: Tyler Matijevich
	
	% Reset the solution
	solution.x 	= 0.0;
	solution.v 	= 0.0;
	solution.a 	= 0.0;
	solution.j 	= 0.0;
	valid 		= false;
	
	% Input requirements
	% #1 Number of points
	if (n < 2) || (n > min(length(t_), length(v_)))
		printf("PathPoint call failed: Number of points %d exceeds limits %d, %d\n", n, 2, min(length(t_), length(v_)));
		return;
	end
		
	% #2 Non-decreasing time points
	for i = 2:n
		if t_(i) < t_(i - 1) % Time points can be numerically equal, this is a common result of unsaturated velocity profiles in this library
			printf("PathPoint call failed: Time point %d, %.3f is less than point %d, %.3f\n", i, t_(i), i - 1, t_(i-1));
			return;
		end % Non-decreasing?
	end % Loop time points
	
	% #3 Valid request time
	if (t < t_(1)) || (t > t_(n))
		printf("PathPoint call failed: Requested time value %.3f exceeds time endpoints %.3f, %.3f\n", t, t_(1), t_(n));
		return;
	end
	
	% #4 Valid jerk factor
	if (k < 1.0) || (k > 2.0)
		printf("PathPoint call failed: Invalid jerk factor %.3f\n", k);
		return;
	end
	
	% There are n - 1 "macro" segments
	% There are m - 1 "micro" segments
	m = (n - 1) * 3 + 1;
	
	% Each micro segment implements a jerk phase for the acceleration profile
	% Declare the micro arrays
	ut_ = zeros(m, 1); % Time
	ux_ = zeros(m, 1); % Position Units
	uv_ = zeros(m, 1); % Velocity Units/s
	ua_ = zeros(m, 1); % Acceleration Units/s^2
	uj_ = zeros(m, 1); % Jerk Units/s^3
	
	% Set the initial time, position, and velocity
	ut_(1) = t_(1);
	ux_(1) = x_0;
	uv_(1) = v_(1);
	
	% Loop through each macro segment
	for i = 2:n
		% Determine the endpoints
		b = i * 3 - 2; 	% End of macro segment
		a = b - 3; 		% Beginning of macrosegment
		
		% 1 2 3 4 5 6 7 8 9 
		% ^     ^
		% a     b
		%       ^     ^ 
		%       a     b 
		
		% Assigned the time and velocity endpoint
		ut_(b) = t_(i);
		uv_(b) = v_(i);
		
		% Definitions
		% a+1, a+2, a+3 required for Position
		% a+1, a+2      required for Time and Velocity
		% a+0, a+1, a+2 required for Jerk and Acceleration
		
		% Check if the time points are equal 
		if (ut_(a) == ut_(b)) || (uv_(a) == uv_(b))
			% Stack the beginning of macro segment
			ut_(a + 1) = ut_(a);
			ut_(a + 2) = ut_(a);
			ux_(a + 1) = ux_(a);
			ux_(a + 2) = ux_(a);
			ux_(a + 3) = ux_(a) + uv_(a) * (ut_(a + 3) - ut_(a));
			uv_(a + 1) = uv_(a);
			uv_(a + 2) = uv_(a);
			ua_(a)     = 0.0;
			ua_(a + 1) = 0.0;
			ua_(a + 2) = 0.0;
			uj_(a)     = 0.0;
			uj_(a + 1) = 0.0;
			uj_(a + 2) = 0.0;
			
		else
			% Determine the direction/sign of acceleration in the macro segment
			if uv_(b) > uv_(a)
				a_dir = 1.0;
			elseif uv_(b) < uv_(a)
				a_dir = -1.0;
			end
			
			% Determine the nominal acceleration magnitude
			dt = ut_(b) - ut_(a);
			dv = uv_(b) - uv_(a);
			a_bar = abs(dv) / dt; 
			
			if k == 1.0 % For numeric purposed, infinite jerk (protect divide by zero)
				% Stack the beginning of macro segment
				ut_(a + 1) = ut_(a);
				ut_(a + 2) = ut_(a);
				ua_(a)     = dv / dt;
				ua_(a + 1) = ua_(a);
				ua_(a + 2) = ua_(a);
				uj_(a)     = 0.0; % Technically this is infinite jerk
				uj_(a + 1) = 0.0;
				uj_(a + 2) = 0.0;
			else
				% a_bar != 0.0 because abs(dv) != 0.0
				uj = (k * a_bar) ^ 2 / (k * a_bar * dt - abs(dv)); % k * a_bar > abs(dv) / dt because k > 1.0
				% Numerator and denominator are non-zero
				
				ut_(a + 1) = ut_(a) + (k * a_bar) / uj;
				ut_(a + 2) = ut_(b) - (k * a_bar) / uj;
				ua_(a)     = 0.0;
				ua_(a + 1) = a_dir * k * a_bar;
				ua_(a + 2) = ua_(a + 1);
				uj_(a)     = a_dir * uj;
				uj_(a + 1) = 0.0;
				uj_(a + 2) = (-1.0) * uj_(a);
			end % k == 1?
			
			% Compute position and velocity of the micro segments
			uv_(a+1) = uv_(a+0) + ua_(a+0) * (ut_(a+1) - ut_(a+0)) + 0.5 * uj_(a+0) * (ut_(a+1) - ut_(a+0)) ^ 2;
			uv_(a+2) = uv_(a+1) + ua_(a+1) * (ut_(a+2) - ut_(a+1)) + 0.5 * uj_(a+1) * (ut_(a+2) - ut_(a+1)) ^ 2;
			ux_(a+1) = ux_(a+0) + uv_(a+0) * (ut_(a+1) - ut_(a+0)) + 0.5 * ua_(a+0) * (ut_(a+1) - ut_(a+0)) ^ 2 + (1/6) * uj_(a+0) * (ut_(a+1) - ut_(a+0)) ^ 3;
			ux_(a+2) = ux_(a+1) + uv_(a+1) * (ut_(a+2) - ut_(a+1)) + 0.5 * ua_(a+1) * (ut_(a+2) - ut_(a+1)) ^ 2 + (1/6) * uj_(a+1) * (ut_(a+2) - ut_(a+1)) ^ 3;
			ux_(a+3) = ux_(a+2) + uv_(a+2) * (ut_(a+3) - ut_(a+2)) + 0.5 * ua_(a+2) * (ut_(a+3) - ut_(a+2)) ^ 2 + (1/6) * uj_(a+2) * (ut_(a+3) - ut_(a+2)) ^ 3;
		end % dt > 0 & dv > 0?
	end % Loop macro segments
	
	% Find the requested micro segment
	if t == t_(n)
		ui = m - 1;
	else
		for k = 2:m
			if t < ut_(k)
				ui = k - 1;
				break;
			end % Within time?
		end % Loop array
	end % Final segment?
	
	if exist("ui", "var") == 0
		printf("Micro segment not found for t %.16f s in range [%.16f, %.16f] s\n", t, t_(1), t_(n));
		return;
	end
	
	% Set solution
	solution.j = uj_(ui);
	solution.a = ua_(ui) + uj_(ui) * (t - ut_(ui));
	solution.v = uv_(ui) + ua_(ui) * (t - ut_(ui)) + 0.5 * uj_(ui) * (t - ut_(ui)) ^ 2;
	solution.x = ux_(ui) + uv_(ui) * (t - ut_(ui)) + 0.5 * ua_(ui) * (t - ut_(ui)) ^ 2 + (1/6) * uj_(ui) * (t - ut_(ui)) ^ 3;
	valid = true; 
	
	if printResult
		printf("PathPoint call: Pos %.3f, Vel %.3f, Acc %.3f, Jerk %.3f\n", solution.x, solution.v, solution.a, solution.j);
	end
	
end % Function
