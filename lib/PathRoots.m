%!octave

function [solution, valid] = PathRoots(p_2, p_1, p_0, printResults = false)
	% Return the real roots of a second order polynomial
	% Date: 2020-05-13
	% Created by: Tyler Matijevich
	
	% Reset solution
	solution.r_1 = 0.0; solution.r_2 = 0.0;
	
	% Compute the discriminant
	Discriminant = p_1 ^ 2 - 4.0 * p_2 * p_0; % b^2 - 4ac
	
	% Two roots
	if p_2 != 0.0 % Protect against divide-by-zero
		if Discriminant > 0.0 % Two real roots
			solution.r_1 = ((-p_1) + sqrt(Discriminant)) / (2.0 * p_2); 
			solution.r_2 = ((-p_1) - sqrt(Discriminant)) / (2.0 * p_2);
			
		elseif Discriminant < 0.0 % Imaginary roots
			printf("PathRoots call warning: solution has imaginary roots\n");
			valid = false;
			return;
			
		else % Multiple root
			solution.r_1 = (-p_1) / (2.0 * p_2);
			solution.r_2 = solution.r_1;
			
		end % discriminant
	
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
	
	if printResults
		printf("PathRoots call: roots %1.3f, %1.3f\n", solution.r_1, solution.r_2)
	end
	valid = true;

end % Function
