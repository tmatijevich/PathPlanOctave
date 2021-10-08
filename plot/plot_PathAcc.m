%!octave

function [] = plot_PathAcc(dt, dx, v_0, v_f, v_min, v_max)
	
	% Call PathAcc
	[solution, valid] = PathAcc(dt, dx, v_0, v_f, v_min, v_max, true);
	
	if !valid
		return;
		
	else
		hFigure = figure(1, "name", "PathAcc");
		set(hFigure, "menubar", "none");
		figurePosition = get(gcf, "position");
		set(gcf, "position", [figurePosition(1:2), 575, 680]);
		cla;
		textSize = 12;
		sTitle = sprintf("PathAcc({\\delta}t = %.3f s, {\\delta}x = %.1f u,\nv_0 = %.1f u/s, v_f = %.1f u/s, v_{min} = %.1f u/s, v_{max} = %.1f u/s\n", dt, dx, v_0, v_f, v_min, v_max);
		
		tPlot = 0.0:0.001:dt;
		xPlot = zeros(size(tPlot));
		aPlot = zeros(size(tPlot));
		for i = 1:length(tPlot)
			[pointSolution, pointValid] = PathPoint(0.0, solution.t_, solution.v_, 4, tPlot(i), 1.0, false);
			if pointValid
				xPlot(i) = pointSolution.x;
				aPlot(i) = pointSolution.a;
			end
		end
		
		subplot(3, 1, 1);
		plot(tPlot, xPlot, 'b');
		title(sTitle, "fontsize", textSize + 2, "interpreter", "tex");
		set(gca, "fontsize", textSize);
		ylabel("Position [Units]", "fontsize", textSize);
		
		subplot(3, 1, 2);
		plot(solution.t_, solution.v_, 'g');
		set(gca, "fontsize", textSize);
		ylabel("Velocity [Units/s]", "fontsize", textSize);
		
		subplot(3, 1, 3);
		plot(tPlot, aPlot, 'r');
		set(gca, "fontsize", textSize);
		ylabel("Acceleration [Units/s^2]", "fontsize", textSize);
		xlabel("Time [s]", "fontsize", textSize);
		aValue = sprintf("|a| = %.1f", solution.a);
		legend(aValue, "location", "northeast");
	end
	
end % End function
