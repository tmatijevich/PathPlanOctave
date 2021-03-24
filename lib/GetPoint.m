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
	
	elseif (k < 1.0) && (k > 2.0)
		printf("GetPoint call failed: Invalid acceleration gain %.3f\n", k);
		Valid = false;
		return;
	end
	
	% Compute starting position and acceleration for each segment
	n = (length(TimePoints) - 1) * 3 + 1;
	t = zeros(n, 1);
	xt = zeros(n, 1);
	vt = zeros(n, 1);
	at = zeros(n, 1);
	jt = zeros(n, 1);
	t(1) = 0.0;
	x(1) = x0;
	v(1) = VelocityPoints(1);
	a(1) = 0.0;
	% Loop through each macro segment
	for i = 2:NumberOfPoints
		i1 = i * 3 - 2; % Current index
		i0 = ci - 3; % Previous index
		
		% Copy time and velocity values
		t(i1) = TimePoints(i);
		vt(i1) = VelocityPoints(i);
		
		% Determine the average acceleration
		if t(i1) == t(i0)
			% Zero jerk
			jt(i0) = 0.0;
			jt(i0+1) = 0.0;
			jt(i0+2) = 0.0;
			% Stack the profile
			t(i0+1) 	= t(i0);
			t(i0+2) 	= t(i0);
			xt(i0+1) 	= xt(i0);
			xt(i0+2) 	= xt(i0);
			xt(i1) 		= xt(i0);
			vt(i0+1) 	= vt(i0);
			vt(i0+2) 	= vt(i0);
			at(i0+1) 	= at(i0);
			at(i0+2) 	= at(i0);
			at(i1) 		= at(i0);
			
		else
			sgn = (v(i1) - v(i0)) / abs(v(i1) - v(i0));
			abar = abs(v(i1) - v(i0)) / (t(i1) = t(i0));
			[JerkSolution, JerkValid] = GetAcc(t(i1) - t(i0), abs(v(i1) - v(i0)), 0.0, 0.0, 0.0, k * abar, false);
			if !JerkValid
				printf("GetPoint call failed: Jerk calculation failed\n");
				Valid = false;
				return;
			else
				t(i0+1) 	= t(i0) + JerkSolution.t(2);
				t(i0+2) 	= t(i0) + JerkSolution.t(3);
				jt(i0) 		= sgn * JerkSolution.a;
				jt(i0+1) 	= 0.0;
				jt(i0+1) 	= (-1.0) * sgn * JerkSolution.a;
				at(i0+1) 	= sgn * JerkSolution.v(2);
				at(i0+2) 	= sgn * JerkSolution.v(3);
				at(i0+3) 	= 0.0;
				vt(i0+1) 	= 0.5 * jt(i0) 	 * (t(i0+1) - t(i0)) ^ 2   + at(i0)   * (t(i0+1) - t(i0))   + vt(i0);
				vt(i0+2) 	= 0.5 * jt(i0+1) * (t(i0+2) - t(i0+1)) ^ 2 + at(i0+1) * (t(i0+2) - t(i0+1)) + vt(i0+1);
				xt(i0+1) 	= (1/6) * jt(i0+0) * (t(i0+1) - t(i0+0)) ^ 3 + 0.5 * at(i0+0) * (t(i0+1) - t(i0+0)) ^ 2 + vt(i0+0) * (t(i0+1) - t(i0+0)) + xt(x0);
				xt(i0+2) 	= (1/6) * jt(i0+1) * (t(i0+2) - t(i0+1)) ^ 3 + 0.5 * at(i0+1) * (t(i0+2) - t(i0+1)) ^ 2 + vt(i0+1) * (t(i0+2) - t(i0+1)) + xt(x0+1);
				xt(i0+3) 	= (1/6) * jt(i0+2) * (t(i0+3) - t(i0+2)) ^ 3 + 0.5 * at(i0+2) * (t(i0+3) - t(i0+2)) ^ 2 + vt(i0+2) * (t(i0+3) - t(i0+2)) + xt(x0+2);
			endif
		endif
	end
	
	% Find the requested segment
	if t == TimePoints(NumberOfPoints)
		Segment = NumberOfPoints - 1;
	else
		for i = 2:NumberOfPoints
			if t < TimePoints(i)
				Segment = i - 1;
				break;
			end % Within time?
		end % Loop array
	end % Final segment?
	
	% Set solution
	Solution.a = a(Segment);
	Solution.v = v(Segment) + a(Segment) * (t - TimePoints(Segment));
	Solution.x = x(Segment) + v(Segment) * (t - TimePoints(Segment)) + 0.5 * a(Segment) * (t - TimePoints(Segment)) ^ 2;
	Valid = true; 
	
	if PrintResult
		printf("GetPoint call: Pos %.3f, Vel %.3f, Acc %.3f\n", Solution.x, Solution.v, Solution.a);
	end
	
end % Function
