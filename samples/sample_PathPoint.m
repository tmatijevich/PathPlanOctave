%!octave

v = [0.0, 1.9, 0.75, 0.75, 1.2];
t = [0.0, 0.25, 0.40, 0.50, 0.60];

N = 5;
tplot = t(1):0.01:t(N); n = length(tplot);
xplot = zeros(n,1);
vplot = zeros(n,1);
aplot = zeros(n,1);
xplot2 = zeros(n,1);
vplot2 = zeros(n,1);
aplot2 = zeros(n,1);

for i = 1:n
	[soln, valid] = GetPoint(0.0, t(1:N), v(1:N), N, tplot(i), k = 1.0);
	if valid
		xplot(i) = soln.x;
		vplot(i) = soln.v;
		aplot(i) = soln.a;
	end
	[soln, valid] = GetPoint(0.0, t(1:N), v(1:N), N, tplot(i), k = 1.25);
	if valid
		xplot2(i) = soln.x;
		vplot2(i) = soln.v;
		aplot2(i) = soln.a;
	end
end

hFig = figure(1, "name", "GetPoint() Sample"); set(hFig, "menubar", "none");
CurrentPosition = get(gcf, "position");
set(gcf, "position", [CurrentPosition(1:2) 600 700]);
pointSize = 9.0;
lineWidth = 0.5;
textSize = 13;
tickSize = 11;

subplot(3,1,1); cla;
plot(tplot, xplot, "b.", "markersize", pointSize);
hold on;
plot(tplot, xplot2, "b", "linewidth", lineWidth);
set(gca, "fontsize", tickSize);
ylabel("Position", "fontsize", textSize);

subplot(3,1,2); cla;
hPlot1 = plot(tplot, vplot, "g.", "markersize", pointSize);
hold on;
hPlot2 = plot(t, v, "k.", "markersize", pointSize);
hPlot3 = plot(tplot, vplot2, "g", "linewidth", lineWidth);
ylim([0.0, 2.5]);
hLeg = legend([hPlot2, hPlot1], {"Input Points", "Interpolated Points"}, "location", "northeast");
set(hLeg, "fontsize", textSize);
set(gca, "fontsize", tickSize);
ylabel("Velocity", "fontsize", textSize);

subplot(3,1,3); cla;
plot(tplot, aplot, "r.", "markersize", pointSize);
hold on;
plot(tplot, aplot2, "r", "linewidth", lineWidth);
set(gca, "fontsize", tickSize);
xlabel("Time [s]", "fontsize", textSize);
ylabel("Acceleration", "fontsize", textSize);
