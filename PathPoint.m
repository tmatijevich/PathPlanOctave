%!octave

% FUNCTION NAME:
%   PathPoint
%
% DESCRIPTION:
%   Position, velocity, and acceleration at a point in time along linear segments with parabolic blends
%
% INPUT:
%   x_0 - Initial position [Units]
%   t_  - Time vector
%   v_  - Velocity vector [Units/s]
%   n   - Number of points
%   t   - Request time [s]
%   k   - Jerk factor 1.0..2.0
%   printResult - Print successful completion message
%
% OUTPUT:
%   solution (struct) - Point solution
%     x - Position at time [Units]
%     v - Velocity [Units/s]
%     a - Acceleration [Units/s^2]
%     j - Jerk [Units/s^3]
%   valid - Successful completion
%
% ASSUMPTIONS AND LIMITATIONS:
%   - Linear segments with parabolic blends
%   - Optional jerk factor (> 1.0) increases peak acceleration
% 
% DATE CREATED: 
%   2020-04-01
%
% AUTHOR: 
%   Tyler Matijevich
%

function [solution, valid] = PathPoint(x_0, t_, v_, n, t, k = 1.0, printResult = false)
	
	% Reset the solution
	solution = struct("x", 0.0, "v", 0.0, "a", 0.0, "j", 0.0);
	valid = false;
	
	% Input requirements
	% #1 Number of points
	if n < 2 || n > min(length(t_), length(v_))
		printf("PathPoint call failed: Number of points %d exceeds limits %d, %d\n", n, 2, min(length(t_), length(v_)));
		return;
	end
		
	% #2 Non-decreasing time points
	for i = 2:n
		if t_(i) < t_(i - 1) % Time points may be numerically equal, e.g. an unsaturated profile solution
			printf("PathPoint call failed: Time point %d:%.3f s is non-decreasing of point %d:%.3f s\n", i, t_(i), i - 1, t_(i-1));
			return;
		end 
	end 
	
	% #3 Request time limit
	if t < t_(1) || t > t_(n)
		printf("PathPoint call failed: Requested time %.3f s exceeds endpoints %.3f s, %.3f s\n", t, t_(1), t_(n));
		return;
	end
	
	% #4 Jerk factor limit
	if k < 1.0 || k > 2.0
		printf("PathPoint call failed: Jerk factor %.3f exceeds limits [1.0, 2.0]\n", k);
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
		
		% Check if time or velocity points are equal
		if ut_(a) == ut_(b) || uv_(a) == uv_(b)
			% Stack to beginning of macro segment
			ut_(a + 1) = ut_(a);
			ut_(a + 2) = ut_(a);
			ux_(a + 1) = ux_(a);
			ux_(a + 2) = ux_(a);
			ux_(a + 3) = ux_(a) + uv_(a) * (ut_(a + 3) - ut_(a)); % ux_(b)
			uv_(a + 1) = uv_(a);
			uv_(a + 2) = uv_(a);
			ua_(a)     = 0.0;
			ua_(a + 1) = 0.0;
			ua_(a + 2) = 0.0;
			uj_(a)     = 0.0;
			uj_(a + 1) = 0.0;
			uj_(a + 2) = 0.0;
			
		else
			% Determine the direction of acceleration in the macro segment
			if uv_(b) > uv_(a)
				a_dir = 1.0;
			else
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
		for i = 2:m
			if t < ut_(i)
				ui = i - 1;
				break;
			end % Within time?
		end % Loop array
	end % Final segment?
	
	if exist("ui", "var") == 0
		printf("PathPoint call failed: Request time t %.16f s not found in range [%.16f, %.16f] s\n", t, t_(1), t_(n));
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
	
end % Function definition
