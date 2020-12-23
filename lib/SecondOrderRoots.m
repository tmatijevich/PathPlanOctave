%!octave

function [Solution, Valid] = SecondOrderRoots(p2, p1, p0, PrintResult = false)
	% Return the real roots of a second order polynomial
	% Date: 2020-05-13
	% Created by: Tyler Matijevich
	
	% Reset solution
	Solution.r1 = 0.0; Solution.r2 = 0.0;
	
	% Compute the discriminant
	Discriminant = p1 ^ 2 - 4.0 * p2 * p0; % b^2 - 4ac
	
	% Two roots
	if p2 != 0.0 % Protect against divide-by-zero
		if Discriminant > 0.0 % Two real roots
			Solution.r1 = ((-p1) + sqrt(Discriminant)) / (2.0 * p2); 
			Solution.r2 = ((-p1) - sqrt(Discriminant)) / (2.0 * p2);
			
		elseif Discriminant < 0.0 % Imaginary roots
			printf("SecondOrderRoots call warning: Solution has imaginary roots\n");
			Valid = false;
			return;
			
		else % Multiple roots
			Solution.r1 = (-p1) / (2.0 * p2);
			Solution.r2 = Solution.r1;
			
		end % discriminant
	
	else % First order
		if p1 != 0 % Single root
			Solution.r1 = (-p0) / p1;
			Solution.r2 = Solution.r1;
			
		else % No roots
			printf("SecondOrderRoots call warning: First order, no solution\n");
			Valid = false;
			return;
			
		end % Single root?
	end % Two roots?
	
	if PrintResult
		printf("SecondOrderRoots call: roots %1.3f, %1.3f\n", Solution.r1, Solution.r2)
	end
	Valid = true;

end % Function
