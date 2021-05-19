%!octave

% Set inputs
dt 		= 0.3;
dx 		= 0.8;
v0 		= 1.9;
vf 		= 1.2;
vmin 	= 0.75;
vmax 	= 3.0;

[Solution, Valid] = PathAcc(dt, dx, v0, vf, vmin, vmax);

textSize = 13;

if Valid
	hFig = figure(1, "name", "Velocity Profile"); set(hFig, "menubar", "none");
	cla;
	hold on;
	area(Solution.t, Solution.v, "facecolor", [0.8 0.8 0.8], "edgecolor", "none");
	plot([-0.05 0.35], [vmax vmax], "r--");
	plot(Solution.t, Solution.v, "b.-", "markersize", 18.0, "linewidth", 1.5);
	plot([0.0 0.0], [0.0 v0], "k--");
	plot([dt dt], [0.0 vf], "k--");
	ylim([0.0, 3.5]);
	xlim([-0.05 0.35]);
	text(Solution.t(1) - 0.03, Solution.v(1) + 0.15, "v_0, t_0", "fontsize", textSize);
	text(Solution.t(2) - 0.025, Solution.v(2) + 0.15, "v_1, t_1", "fontsize", textSize);
	text(Solution.t(3), Solution.v(3) + 0.15, "v_2, t_2", "fontsize", textSize);
	text(Solution.t(4), Solution.v(4) + 0.15, "v_f, t_f", "fontsize", textSize);
	text(0.135, 1.5, "{\\delta}x", "interpreter", "tex", "fontsize", textSize);
	text(0.32, vmax + 0.15, "v_{max}", "interpreter", "tex", "fontsize", textSize);
	text(0.016, 2.55, "a", "fontsize", textSize);
	text(0.26, 2.25, "a", "fontsize", textSize);
	axis("nolabel");
	box off;
	ylabel("Velocity", "fontsize", textSize);
	xlabel("Time", "fontsize", textSize);
end
