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

hFig = figure(1, "name", "IndeCart"); set(hFig, "menubar", "none");
cla;
plot(et, ev, "k")
plot(it, iv, "b");
hold on;
plot(f_t, f_v, "r--");
plot(s_t, s_v, "g--");
