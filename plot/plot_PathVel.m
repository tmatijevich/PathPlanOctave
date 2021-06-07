%!octave

function [solution] = plot_PathVel(dt, dx, v_0, v_f, v_min, v_max, a)
	
	[solution, valid] = PathVel(dt, dx, v_0, v_f, v_min, v_max, a, true);
	if valid
		figureTitle = sprintf("PlotVel(dt = %.1f, dx = %.1f, v_0 = %.1f, v_f = %.1f, v_min = %.1f, v_max = %.1f)", dt, dx, v_0, v_f, v_min, v_max);
		hFigure = figure(1, "name", figureTitle);
		set(hFigure, "menubar", "none");
		cla;
		plot(solution.t, solution.v);
		currentYAxisLimits = ylim;
		ylim([0.0, currentYAxisLimits(2)]);
	end
	
end % Function
