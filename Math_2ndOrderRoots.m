%!octave

function [soln, valid] = Math_2ndOrderRoots(p2, p1, p0)
	% Return the real roots of a second order polynomial
	% Date: 2020-05-13
	% Created by: Tyler Matijevich
	
	RootsO2 = roots([p2, p1, p0]);
	if isreal(RootsO2(1))
		soln.r1 = RootsO2(1);
		soln.r2 = 0.0;
		soln.n = length(RootsO2);
		if soln.n > 1
			soln.r2 = RootsO2(2);
		end
		valid = true;
	else
		soln.Message = "Solution has imaginary roots"; valid = false;
	end
end
