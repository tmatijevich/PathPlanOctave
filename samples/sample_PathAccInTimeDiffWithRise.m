%!octave

v_1 		= 1.900; % Units/s
v_f 		= 1.200; % Units/s
v_min 		= 0.675; % Units/s
v_max 		= 3.250; % Units/s
dt_tilde_ 	= 0.010:0.010:0.500; % s
n = length(dt_tilde_);

dx  		= 0.300; % Units

for i = 1:n
	[solution{i}, valid(i)] = PathAccInTimeDiffWithRise(dt_tilde_(i), dx, v_1, v_f, v_min, v_max, true);
end
for i = 1:n
	case_(i) = solution{i}.case;
	a_(i) = solution{i}.accDec.a;
end
i_4 = find(case_ == 4, 1, 'last');
i_23 = find((case_ == 2) | (case_ == 3), 1, 'last');
i_1 = find(case_ == 1, 1, 'last');

hFig = figure(1, "name", "PathAccInTimeDiffWithRise() Analysis");
set(hFig, "menubar", "none");
currentPosition = get(gcf, "position");
set(gcf, "position", [currentPosition(1:2) 800 600]);
fontSize = 13;
markerSize = 4;
cla;

hPlt(1) = plot(dt_tilde_(1:i_4), a_(1:i_4), "bs", "markersize", markerSize);
hold on;
ylim([0 100]);
hPlt(2) = plot(dt_tilde_(i_4+1:i_23), a_(i_4+1:i_23), "bo", "markersize", markerSize);
hPlt(3) = plot(dt_tilde_(i_23+1:i_1), a_(i_23+1:i_1), "b*", "markersize", markerSize);
hPlt(4) = plot(dt_tilde_(i_1+1:end), a_(i_1+1:end), "bx", "markersize", markerSize);

%-------------------------------------------------------------------------------
dx = 0.400; % Units
for i = 1:n
	[solution{i}, valid(i)] = PathAccInTimeDiffWithRise(dt_tilde_(i), dx, v_1, v_f, v_min, v_max, true);
end
for i = 1:n
	case_(i) = solution{i}.case;
	a_(i) = solution{i}.accDec.a;
end
i_4 = find(case_ == 4, 1, 'last');
i_23 = find((case_ == 2) | (case_ == 3), 1, 'last');
i_1 = find(case_ == 1, 1, 'last');
hPlt(5) = plot(dt_tilde_(1:i_4), a_(1:i_4), "rs", "markersize", markerSize);
hPlt(6) = plot(dt_tilde_(i_4+1:i_23), a_(i_4+1:i_23), "ro", "markersize", markerSize);
hPlt(7) = plot(dt_tilde_(i_23+1:i_1), a_(i_23+1:i_1), "r*", "markersize", markerSize);
hPlt(8) = plot(dt_tilde_(i_1+1:end), a_(i_1+1:end), "rx", "markersize", markerSize);

%-------------------------------------------------------------------------------
dx = 0.500; % Units
for i = 1:n
	[solution{i}, valid(i)] = PathAccInTimeDiffWithRise(dt_tilde_(i), dx, v_1, v_f, v_min, v_max, true);
end
for i = 1:n
	case_(i) = solution{i}.case;
	a_(i) = solution{i}.accDec.a;
end
i_4 = find(case_ == 4, 1, 'last');
i_23 = find((case_ == 2) | (case_ == 3), 1, 'last');
i_1 = find(case_ == 1, 1, 'last');
hPlt(9) = plot(dt_tilde_(1:i_4), a_(1:i_4), "gs", "markersize", markerSize);
hPlt(10) = plot(dt_tilde_(i_4+1:i_23), a_(i_4+1:i_23), "go", "markersize", markerSize);
hPlt(11) = plot(dt_tilde_(i_23+1:i_1), a_(i_23+1:i_1), "g*", "markersize", markerSize);
%hPlt(12) = plot(dt_tilde_(i_1+1:end), a_(i_1+1:end), "gx", "markersize", markerSize);

%-------------------------------------------------------------------------------
set(gca, "fontsize", fontSize);

hLeg = legend([
	hPlt(3),
	hPlt(2),
	hPlt(1),
	hPlt(4),
	hPlt(7),
	hPlt(6),
	hPlt(5),
	hPlt(8),
	hPlt(11),
	hPlt(10),
	hPlt(9)], {
	'\delta x = 0.300 saturated',
	'\delta x = 0.300',
	'\delta x = 0.300 higher order solver',
	'\delta x = 0.300 \infty acceleration',
	'\delta x = 0.400 saturated',
	'\delta x = 0.400',
	'\delta x = 0.400 higher order solver',
	'\delta x = 0.400 \infty acceleration',
	'\delta x = 0.500 saturated',
	'\delta x = 0.500',
	'\delta x = 0.500 higher order solver'}
);
set(hLeg, "fontsize", fontSize);
set(hLeg, "location", "northwest");

xlabel("Time Difference [s]", "fontsize", fontSize);
ylabel('Acceleration [Units/s^2]', "fontsize", fontSize);

title({'Minimum acceleration given difference in path time duration', 'v_1 = 1.9, v_f = 1.2, v_{min} = 0.675, v_{max} = 3.250'}, "fontSize", fontSize + 2, "fontweight", "bold");

%-------------------------------------------------------------------------------
hFig = figure(2, "name", "PathAccInTimeDiffWithRise() Sample");
set(hFig, "menubar", "none");
currentPosition = get(gcf, "position");
set(gcf, "position", [currentPosition(1:2) 800 600]);
cla;

soln = PathAccInTimeDiffWithRise(dt_tilde_(20), 0.400, v_1, v_f, v_min, v_max, true);

lineWidth = 1.5;
hPlt1 = plot(soln.accDec.t(1:2), soln.accDec.v(1:2), "k", "linewidth", lineWidth);
hPlt2 = plot(soln.accDec.t(2:end), soln.accDec.v(2:end), "r--", "linewidth", lineWidth);
hold on;
hPlt3 = plot(soln.decAcc.t(2:end), soln.decAcc.v(2:end), "b--", "linewidth", lineWidth);
hPlt4 = plot([
	soln.accDec.t(end), soln.decAcc.t(end)],
	[soln.accDec.v(end), soln.decAcc.v(end)],
	"g--", "linewidth", lineWidth
);

set(gca, "fontsize", fontSize);

hLeg = legend([
	hPlt1,
	hPlt2,
	hPlt3,
	hPlt4], {
	'Rise',
	'ACC DEC',
	'DEC ACC',
	'Capture window'}
);
% set(hLeg, "fontsize", fontSize);
% 
xlabel("Time [s]", "fontsize", fontSize);
ylabel('Velocity [Units/s]', "fontsize", fontSize);

title({'Minimum acceleration given difference in path time-duration', '{\delta}t_{tilde} = 0.2 s, {\delta}x = 0.400 v_1 = 1.9, v_f = 1.2, v_{min} = 0.675, v_{max} = 3.250'}, "fontSize", fontSize + 2, "fontweight", "bold", "interpreter", "tex");
