%!octave

% FUNCTION NAME:
%   PathRoots
%
% DESCRIPTION:
%   Reals roots of a second order polynomial
%
% INPUT:
%   p_2 - 2nd order coefficient
%   p_1 - 1st order coefficient
%   p_0 - 0th order coefficient
%   printResult - Print successful completion message
%
% OUTPUT:
%   solution (struct) - Roots solution
%     r_1 - First real root
%     r_2 - Second real root (if exists)
%   valid - Successful completion
%
% ASSUMPTIONS AND LIMITATIONS:
%   - Only real roots, returns error if imaginary
%
% DATE CREATED:
%   2020-05-13 
%
% AUTHOR:
%   Tyler Matijevich
%

function [solution, valid] = PathRoots(p_2, p_1, p_0, printResult = false)
	
	% Reset solution
	solution = struct("r_1", 0.0, "r_2", 0.0);
	
	% Compute the discriminant
	discriminant = p_1 ^ 2 - 4.0 * p_2 * p_0; % b^2 - 4ac
	
	% Two roots
	if p_2 != 0.0 % Protect against divide-by-zero
		if discriminant > 0.0 % Two real roots
			solution.r_1 = ((-p_1) + sqrt(discriminant)) / (2.0 * p_2); 
			solution.r_2 = ((-p_1) - sqrt(discriminant)) / (2.0 * p_2);
			
		elseif discriminant < 0.0 % Imaginary roots
			printf("PathRoots call error: solution has imaginary roots\n");
			valid = false;
			return;
			
		else % Multiple root
			solution.r_1 = (-p_1) / (2.0 * p_2);
			solution.r_2 = solution.r_1;
			
		end % Discriminant
	
	else % First order
		if p_1 != 0 % Single root
			solution.r_1 = (-p_0) / p_1;
			solution.r_2 = solution.r_1; % This function does not distinquish between a multiple root and a single root
			
		else % No roots
			printf("PathRoots call warning: First order, no solution\n");
			valid = false;
			return;
			
		end % Single root?
	end % Two roots?
	
	if printResult
		printf("PathRoots call: roots %.3f, %.3f\n", solution.r_1, solution.r_2)
	end
	valid = true;

end % Function
