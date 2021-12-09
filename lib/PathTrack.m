%!octave

%
%
%

function [solution, valid] = PathTrack(x_0, v_0, t_s, x_1, v_max, a_max, printResult = false)
    % Reference global variables
    run GlobalVars;

    % Reset solution
    solution    = struct('x', 0.0, 'v', 0.0, 'a', 0.0);
    valid       = false;

    % Input requirements
    % #1 Positive inputs
    if t_s <= 0.0 || v_max <= 0.0 || a_max <= 0.0
        printf("PathTrack call failed: Time step %.3f, max velocity %.3f, or max acceleration %.3f non-position\n", t_s, v_max, a_max);        
        return;
    end

    % Check direction and initial velocity
    if x_1 > x_0
        dx_sign = 1.0; 
    elseif x_1 < x_0
        dx_sign = -1.0;
    else
        dx_sign = 0.0;
    end
    if (dx_sign > 0.0 && v_0 < 0.0) || (dx_sign < 0.0 && v_0 > 0.0) || (dx_sign == 0.0 && v_0 != 0.0)
        if abs(v_0) / t_s < a_max 
            % Stop
            solution.a = -1.0 * (v_0 / t_s);
        else
            % Decelerate
            v_sign = v_0 / abs(v_0);
            solution.a = (-1.0 * v_sign) * a_max; 
        end
        solution.v = v_0 + solution.a * t_s;
        solution.x = x_0 + v_0 * t_s + 0.5 * solution.a * t_s ^ 2;
        valid = true;
        if printResult
            printf("PathTrack call: xt %.3f, x %.3f, v %.3f, a %.3f decelerating\n", x_1, solution.x, solution.v, solution.a);
            return;
        end
    end

    [timeSolution, timeValid] = PathTime(abs(x_1 - x_0), abs(v_0), 0.0, 0.0, v_max, a_max);
    if timeValid == false
        if abs(v_0) / t_s < a_max 
            % Stop
            solution.a = -1.0 * (v_0 / t_s);
        else
            % Decelerate
            v_sign = v_0 / abs(v_0);
            solution.a = (-1.0 * v_sign) * a_max; 
        end
        solution.v = v_0 + solution.a * t_s;
        solution.x = x_0 + v_0 * t_s + 0.5 * solution.a * t_s ^ 2;
        valid = true;
        if printResult
            printf("PathTrack call: xt %.3f, x %.3f, v %.3f, a %.3f decelerating\n", x_1, solution.x, solution.v, solution.a);
            return;
        end
    elseif timeSolution.t(4) < t_s
        % Stop
        solution.a  = 0.0;
        solution.v  = 0.0;
        solution.x  = x_0 + v_0 * t_s;
        valid       = true;
        if printResult
            printf("PathTrack call: xt %.3f, x %.3f, v %.3f, a %.3f within time-step %.3f\n", x_1, solution.x, solution.v, solution.a, t_s);
            return;
        end
    end
    [pointSolution, pointValid] = PathPoint(0.0, timeSolution.t, timeSolution.v, 4, t_s, 1.0);
    if pointValid == false 
        return;
    end

    solution.a  = dx_sign * pointSolution.a;
    solution.v  = dx_sign * pointSolution.v;
    solution.x  = x_0 + dx_sign * pointSolution.x;

    valid = true;
    if printResult
        printf("PathTrack call: xt %.3f, x %.3f, v %.3f, a %.3f\n", x_1, solution.x, solution.v, solution.a);
    end

end % Function
