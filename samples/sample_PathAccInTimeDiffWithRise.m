%!octave

dt_tilde 	= 0.150;
dx 			= 0.400;
v_1 		= 1.250;
v_f 		= 1.000;
v_min 		= 0.750;
v_max 		= 2.250;

[solution, valid] = GetAccInTimeDiffWithRise(dt_tilde, dx, v_1, v_f, v_min, v_max, true);

riseDistance = (v_1 ^ 2) / (2.0 * solution.accDec.a);
riseTime = v_1 / solution.accDec.a;
targetTime = solution.accDec.t(5) - riseTime + 0.333 * (solution.decAcc.t(5) - solution.accDec.t(5));

[velSolution, velValid] = GetVel(targetTime, dx - riseDistance, v_1, v_f, v_min, v_max, solution.accDec.a, true);

hFigure = figure(1, "name", "GetAccInTimeDiffWithRise() Sample");
set(hFigure, "menubar", "none");
currentFigurePosition = get(gcf, "position");
set(gcf, "position", [currentFigurePosition(1:2), 600, 500]);

cla;
plot(solution.accDec.t, solution.accDec.v, "r");
hold on;
plot(solution.decAcc.t, solution.decAcc.v, "b");	
