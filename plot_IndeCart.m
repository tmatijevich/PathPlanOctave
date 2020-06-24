%!octave

% To be run within demo_IndeCart.m


% Index profile
iv = zeros(1, 2 + (gPar.ProductsPerPallet - 1) * 4);
it = zeros(size(iv));
iv(1) = 0.0;
it(1) = 0.0;
mk = 2;
for i = 1:(gPar.ProductsPerPallet - 1)
	iv(mk) = gCalc.IndexVelocity;
	iv(mk + 1) = gCalc.IndexVelocity;
	iv(mk + 2) = 0.0;
	iv(mk + 3) = 0.0;
	it(mk) 		= it(mk - 1) + gCalc.IndexTime1;
	it(mk + 1) 	= it(mk - 1) + gCalc.IndexTime2;
	it(mk + 2) 	= it(mk - 1) + gCalc.MinInfeedPeriod - gPar.MinDwellTime;
	it(mk + 3) 	= it(mk - 1) + gCalc.MinInfeedPeriod;
	mk = mk + 4;
end
iv(mk) = gCalc.RiseVelocity;
it(mk) = it(mk - 1) + gCalc.RiseTime;

% Fast/slow profile
dx 		= gPar.DropPosition + gPar.DropOffset - gCalc.IndexPositions(gPar.ProductsPerPallet) - gCalc.RiseDistance;
v0 		= gCalc.RiseVelocity;
vf 		= gCalc.SetOutfeedVelocity;
vmin 	= gCalc.MinCollisionVelocity;
vmax 	= gCalc.ProfMaxVelocity;
a 		= gCalc.ProfAcceleration;
[soln, valid] = Kin_GetTimespan(dx, v0, vf, vmin, vmax, a);
if !valid
	printf("Profile acceleration timespan calc failed\n"); return;
end

f_v = [gCalc.RiseVelocity, soln.v1max, soln.v1max, gCalc.SetOutfeedVelocity];
f_t = [0.0, soln.tVmax1, soln.tVmax2, soln.tVmax] + it(end);
s_v = [gCalc.RiseVelocity, soln.v1min, soln.v1min, gCalc.SetOutfeedVelocity];
s_t = [0.0, soln.tVmin1, soln.tVmin2, soln.tVmin] + it(end);

% Entry profile
et = [0.0, gCalc.EntryTime, gCalc.EntryTime + gCalc.GainTime, gCalc.EntryTime + gCalc.GainTime + gCalc.RiseTime, gCalc.EntryTime + gCalc.GainTime + gCalc.RiseTime + gPar.MinDwellTime];
et = et + gCalc.MinInfeedPeriod * (gPar.ProductsPerPallet) - (gCalc.EntryTime + gCalc.GainTime + gCalc.RiseTime + gPar.MinDwellTime);
ev = [RouteVelocity, gCalc.RiseVelocity, gCalc.RiseVelocity, 0.0, 0.0];
% Prepend a point
et = [0.0, et];
ev = [RouteVelocity, ev];
% Append indexing points
et = [et, et(end) + it];
ev = [ev, iv];


tstep = 0.005;
% Position & acceleration
itplot = it(1):tstep:it(end); n = length(itplot);
ixplot = zeros(1,n);
iaplot = zeros(1,n);
x0 = gPar.LoadPosition;
for i = 1:n
	[soln, valid] = Kin_GetVelProfPoint(x0, it, iv, length(it), itplot(i));
	if valid
		ixplot(i) = soln.x;
		iaplot(i) = soln.a;
	end
end
ftplot = (ceil(f_t(1) / tstep) * tstep):tstep:f_t(end); n = length(ftplot);
fxplot = zeros(1,n);
faplot = zeros(1,n);
x0 = gPar.LoadPosition + gCalc.TotalIndexDistance + gCalc.RiseDistance;
for i = 1:n
	[soln, valid] = Kin_GetVelProfPoint(x0, f_t, f_v, length(f_t), ftplot(i));
	if valid
		fxplot(i) = soln.x;
		faplot(i) = soln.a;
	end
end
stplot = (ceil(s_t(1) / tstep) * tstep):tstep:s_t(end); n = length(stplot);
sxplot = zeros(1,n);
saplot = zeros(1,n);
x0 = gPar.LoadPosition + gCalc.TotalIndexDistance + gCalc.RiseDistance;
for i = 1:n
	[soln, valid] = Kin_GetVelProfPoint(x0, s_t, s_v, length(s_t), stplot(i));
	if valid
		sxplot(i) = soln.x;
		saplot(i) = soln.a;
	end
end
etplot = et(1):tstep:et(end); n = length(etplot);
explot = zeros(1,n);
eaplot = zeros(1,n);
x0 = gPar.LoadPosition - (RouteVelocity ^ 2) / (2.0 * gCalc.ProfAcceleration) - (gCalc.PalletPeriod - RouteVelocity / gCalc.ProfAcceleration) * RouteVelocity;
for i = 1:n
	[soln, valid] = Kin_GetVelProfPoint(x0, et, ev, length(et), etplot(i));
	if valid
		explot(i) = soln.x;
		eaplot(i) = soln.a;
	end
end

dxplot1 = ixplot - explot(1:length(ixplot));
dxplot2 = sxplot(1:(length(explot) - length(ixplot))) - explot((length(ixplot) + 1):end);

% Set default line width
set(groot, "defaultLineLineWidth", 1.75);

hFig = figure(1, "name", "IndeCart"); set(hFig, "menubar", "none");
cur = get(hFig, 'position');
set(hFig, 'position', [cur(1), cur(2), 800, 1000]);
subplot(4,1,1); cla; hold on;
hPlte = plot(etplot, explot, "k", 'linewidth', 1.0);
hPlti = plot(itplot, ixplot, "b");
hPltf = plot(ftplot, fxplot, "r--");
hPlts = plot(stplot, sxplot, "g--");
hLeg = legend([hPlti, hPltf, hPlts, hPlte], {"Current Pallet", "Fastest", "Slowest", "Incoming Pallet"}, 'location', 'northeast');
xlim([0.0, 1.6]);
ylim([round(10 * (gPar.LoadPosition - 0.100)) / 10.0, round(10 * (gPar.DropPosition + gPar.DropOffset + 0.100)) / 10.0]);
set(hLeg, 'fontsize', 12);
set(gca, 'fontsize', 12);
ylabel("Position [m]");

subplot(4,1,2); cla; hold on;
hPlt1 = plot(itplot, dxplot1, "k", 'linewidth', 1.0);
hPlt2 = plot(etplot((length(ixplot) + 1):end), dxplot2, "g--");
hPltMin = plot([itplot(1) etplot(end)], [gCalc.MinPalletSpacing, gCalc.MinPalletSpacing], "m--");
hLeg = legend([hPlt1, hPlt2, hPltMin], {"Current Spacing", "Slow Path Spacing", "Min Spacing"}, 'location', 'northeast');
xlim([0.0, 1.6]);
ylim([0.0 0.5]);
set(hLeg, 'fontsize', 12);
set(gca, 'fontsize', 12);
ylabel("Pallet Spacing [m]");

subplot(4,1,3); cla; hold on;
hPlte = plot(et, ev, "k", 'linewidth', 1.0);
hPlti = plot(it, iv, "b");
hPltf = plot(f_t, f_v, "r--");
hPlts = plot(s_t, s_v, "g--");
hLeg = legend([hPlti, hPltf, hPlts, hPlte], {"Current Pallet", "Fastest", "Slowest", "Incoming Pallet"}, 'location', 'northeast');
xlim([0.0, 1.6]);
set(hLeg, 'fontsize', 12);
set(gca, 'fontsize', 12);
ylabel("Velocity [m/s]");

subplot(4,1,4); cla; hold on;
hPlti = plot(itplot, iaplot, "b");
hPltf = plot(ftplot, faplot, "r--");
hPlts = plot(stplot, saplot, "g--");
hLeg = legend([hPlti, hPltf, hPlts], {"Current Pallet", "Fastest", "Slowest"}, 'location', 'northeast');
xlim([0.0, 1.6]);
set(hLeg, 'fontsize', 12);
set(gca, 'fontsize', 12);
xlabel("Time [s]");
ylabel("Acceleration [m/s^2]");