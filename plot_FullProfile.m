%!octave

iv(1) = RouteVelocity;
it(1) = 0.0;

iv(2:4) = [gCalc.RiseVelocity, gCalc.RiseVelocity, 0.0];
it(2:4) = [gCalc.EntryTime, gCalc.EntryTime + gCalc.GainTime, gCalc.EntryTime + gCalc.GainTime + gCalc.RiseTime];

idx = 5;
for j = 1:(gPar.ProductsPerPallet - 1)
	iv(idx:(idx + 3)) = [gCalc.IndexVelocity, gCalc.IndexVelocity, 0.0, 0.0];
	initialTime = it(idx - 1);
	it(idx:(idx + 3)) = initialTime + [gCalc.IndexTime1, gCalc.IndexTime2, gCalc.MinInfeedPeriod - gPar.MinDwellTime, gCalc.MinInfeedPeriod];
	idx = idx + 4;
endfor

iv(idx) = gCalc.RiseVelocity;
it(idx) = it(idx - 1) + gCalc.RiseTime;
idx = idx + 1;

dt = 0.800;
dx = gPar.DropPosition + gPar.DropOffset - (gCalc.IndexPositions(end) + gCalc.RiseDistance);
v0 = gCalc.RiseVelocity;
vf = gCalc.SetOutfeedVelocity;
vmin = gCalc.MinCollisionVelocity;
vmax = gCalc.ProfMaxVelocity;

[soln, valid] = Kin_GetAcc(dt, dx, v0, vf, vmin, vmax);

if valid
	iv(idx:(idx + 2)) = [soln.v12, soln.v12, vf];
	it(idx:(idx + 2)) = it(idx - 1) + [soln.t1, soln.t2, dt];
else
	printf("Tracking profile failed\n");
endif

idx = idx + 3;
iv(idx) = vf;
it(idx) = it(idx - 1) + abs(gPar.DropOffset) / vf;

tstep = 0.005;
itplot = it(1):tstep:it(end); n = length(itplot);
ixplot = zeros(1, n);
ivplot = zeros(1, n);
iaplot = zeros(1, n);

for i = 1:n
	[soln, valid] = Kin_GetVelProfPoint(0.0, it, iv, length(it), itplot(i));
	if valid
		ixplot(i) = soln.x;
		ivplot(i) = soln.v;
		iaplot(i) = soln.a;
	end
end

% Set default line width
set(groot, "defaultLineLineWidth", 1.5);
hFig = figure(1, "name", "IndeCart"); set(hFig, "menubar", "none");
cur = get(hFig, 'position');
set(hFig, 'position', [cur(1), cur(2), 600, 600]);
subplot(3,1,1); cla; 
plot(itplot, ixplot, 'b'); ylabel('Position [m]');
title('Full Pallet Processing & Syncronization Profile', 'fontsize', 16);
subplot(3,1,2); cla; 
% plot(it, iv, 'b*'); hold on; 
plot(it, iv, 'g'); ylabel('Velocity [m/s]');
subplot(3,1,3); cla; 
plot(itplot, iaplot, 'r'); ylabel('Acceleration [m/s^2]'); xlabel('Time [s]');
