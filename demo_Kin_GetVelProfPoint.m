%!octave

v = [0.0, 1.9, 0.75, 0.75, 1.2];
t = [0.0, 0.25, 0.40, 0.50, 0.60];

N = 5;
tplot = t(1):0.01:t(N); n = length(tplot);
xplot = zeros(n,1);
vplot = zeros(n,1);
aplot = zeros(n,1);

for i = 1:n
	[soln, valid] = Kin_GetVelProfPoint(0.0, t(1:N), v(1:N), N, tplot(i));
	if valid
		xplot(i) = soln.x;
		vplot(i) = soln.v;
		aplot(i) = soln.a;
	end
end

hFig = figure(1, "name", "Kin_GetVelProfPoint()"); set(hFig, "menubar", "none");
subplot(3,1,1); cla;
plot(tplot, xplot, "b*");
subplot(3,1,2); cla;
plot(tplot, vplot, "g*");
hold on;
plot(t, v, "k*");
subplot(3,1,3); cla;
plot(tplot, aplot, "r*");
