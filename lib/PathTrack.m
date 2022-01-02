%!octave

% 
% 
% 

function [solution, valid] = PathTrack(x_0, v_0, t_s, x_t, v_t, v_max, a_max, printResult = false)
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
    if x_t > x_0
        dx_sign = 1.0; 
    elseif x_t < x_0
        dx_sign = -1.0;
    else
        dx_sign = 0.0;
    end
    if (dx_sign > 0.0 && (v_0 < 0.0 || v_t < 0.0)) || (dx_sign < 0.0 && (v_0 > 0.0 || v_t > 0.0))
        % Decelerate
        [solution, valid] = PathTrackDec(x_0, v_0, t_s, a_max);
        if printResult && valid
            printf("PathTrack call: xt %.3f, x %.3f, v %.3f, a %.3f decelerate\n", x_t, solution.x, solution.v, solution.a);
        end
        return;
    elseif dx_sign == 0.0
        % Match speed
        dv_sign = (v_t - v_0) / abs(v_t - v_0);
        solution.a = dv_sign * min(abs(v_t - v_0) / t_s, a_max);
        solution.v = v_0 + solution.a * t_s;
        solution.x = x_0 + v_0 * t_s + 0.5 * solution.a * t_s ^ 2;
        if printResult
            printf("PathTrack call: xt %.3f, x %.3f, v %.3f, a %.3f match speed\n", x_t, solution.x, solution.v, solution.a);
        end
        valid = true;
        return;
    end

    [timeSolution, timeValid] = PathTime(abs(x_t - x_0), abs(v_0), abs(v_t), 0.0, v_max, a_max);
    if timeValid == false
        % Decelerate
        [solution, valid] = PathTrackDec(x_0, v_0, t_s, a_max);
        if printResult && valid
            printf("PathTrack call: xt %.3f, x %.3f, v %.3f, a %.3f decelerate\n", x_t, solution.x, solution.v, solution.a);
        end
        return;
    elseif timeSolution.t(4) < t_s
        % Stop
        solution.a  = 0.0;
        solution.v  = 0.0;
        solution.x  = x_0 + v_0 * t_s;
        valid       = true;
        if printResult
            printf("PathTrack call: xt %.3f, x %.3f, v %.3f, a %.3f within time-step %.3f\n", x_t, solution.x, solution.v, solution.a, t_s);
        end
        return;
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
        printf("PathTrack call: xt %.3f, x %.3f, v %.3f, a %.3f\n", x_t, solution.x, solution.v, solution.a);
    end

end % Function

function [solution, valid] = PathTrackDec(x_0, v_0, t_s, a_max)

    % Reset solution
    solution    = struct('x', 0.0, 'v', 0.0, 'a', 0.0);
    valid       = false;

    % Input requirements
    % #1 Positive inputs
    if t_s <= 0.0 || a_max <= 0.0
        printf("PathTrackDev call failed: Time step %.3f or max acceleration %.3f non-position\n", t_s, a_max);        
        return;
    end

    if abs(v_0) / t_s < a_max
        % Stop
        solution.a = -1.0 * (v_0 / t_s);
        solution.v = 0.0; % Force strictly zero
        solution.x = x_0 + v_0 * t_s + 0.5 * solution.a * t_s ^ 2;
    else
        % Decelerate
        v_sign = v_0 / abs(v_0);
        solution.a = (-1.0 * v_sign) * a_max; 
        solution.v = v_0 + solution.a * t_s;
        solution.x = x_0 + v_0 * t_s + 0.5 * solution.a * t_s ^ 2;
    end

    valid = true;

end % Function