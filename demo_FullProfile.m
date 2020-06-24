%!octave

clear;

gPar = struct(
	'LoadPosition', 		0.250,
	'DropPosition', 		0.750,
	'DropOffset', 			-0.050,
	'BucketSpacing', 		0.250,
	'ToolingWidth', 		0.200,
	'ToolPocketSpacing',	0.020,
	'ProductsPerPallet', 	10,
	'Payload', 				1.0,
	'MinDwellTime', 		0.010,
	'MaxInfeedRate', 		800,
	'TargetOutfeedRate', 	900,
	'MachinesInSeries', 	1,
	'PhotoeyeDelayTime', 	2.0
);
gCalc = struct(
	'MinPalletSpacing', 		0.0,
	'MinInfeedPeriod', 			0.0,
	'PalletPeriod', 			0.0,
	'MinCollisionVelocity', 	0.0,
	'SetOutfeedPeriod', 		0.0,
	'SetOutfeedVelocity', 		0.0,
	'ProfMaxVelocity', 			0.0,
	'ProfMaxAcceleration', 		0.0,
	'IndexPositions', 			0.0,
	'TotalIndexDistance', 		0.0,
	'IndexTime1', 				0.0,
	'IndexTime2', 				0.0,
	'IndexVelocity', 			0.0,
	'IndexAcceleration', 		0.0,
	'MaxRateRiseVelocity',		0.0,
	'RiseVelocity', 			0.0,
	'MinAccRiseVelocity', 		0.0,
	'MaxAccRiseVelocity', 		0.0,
	'MinCollisionAcceleration', 0.0,
	'MinCaptureAcceleration', 	0.0,
	'ProfAcceleration', 		0.0,
	'RiseTime', 				0.0,
	'RiseDistance', 			0.0,
	'GainTime', 				0.0,
	'GainDistance', 			0.0,
	'EntryTime', 				0.0,
	'EntryDistance', 			0.0
);

RouteVelocity = 2.0;
RouteAcceleration = 22.5;

% Baseline calculations
gCalc.MinPalletSpacing		= max(0.152 + 0.012, gPar.ToolingWidth) + 0.004;
gCalc.MinInfeedPeriod 		= 60.0 / gPar.MaxInfeedRate;
gCalc.PalletPeriod			= 60.0 / (gPar.MaxInfeedRate / gPar.ProductsPerPallet);
gCalc.MinCollisionVelocity 	= gCalc.MinPalletSpacing / gCalc.PalletPeriod;
gCalc.SetOutfeedPeriod 		= 60.0 / (gPar.TargetOutfeedRate / gPar.ProductsPerPallet);
gCalc.SetOutfeedVelocity	= (gPar.BucketSpacing * gPar.MachinesInSeries) / gCalc.SetOutfeedPeriod;

% Profiles maximums
gCalc.ProfMaxVelocity 		= 3.5;
gCalc.ProfMaxAcceleration 	= 32.0;

% Index calculations
if gPar.ProductsPerPallet > 1
	
	for i = 1:gPar.ProductsPerPallet
		gCalc.IndexPositions(i) = gPar.LoadPosition - (((gPar.ProductsPerPallet - 1) * gPar.ToolPocketSpacing) / 2.0) ...
		  + (i - 1) * gPar.ToolPocketSpacing;
	end
	gCalc.TotalIndexDistance = (gPar.ProductsPerPallet - 1) * gPar.ToolPocketSpacing;
	
	dt 		= gCalc.MinInfeedPeriod - gPar.MinDwellTime;
	dx 		= gPar.ToolPocketSpacing;
	vmax 	= gCalc.ProfMaxVelocity;
	[soln, valid] = Kin_GetAcc(dt, dx, 0.0, 0.0, 0.0, vmax);
	
	if valid
		gCalc.IndexTime1 		= soln.t1;
		gCalc.IndexTime2 		= soln.t2;
		gCalc.IndexVelocity 	= soln.v12;
		gCalc.IndexAcceleration = soln.a;
	else
		printf("Index calc failed\n"); return;
	end
	
else
	gCalc.IndexPositions(1) 	= gPar.LoadPosition;
	gCalc.TotalIndexDistance 	= 0.0;
end

% Rise velocity calculations
% The minimum rise velocity in order to maintain pallet throughput
dt 		= gCalc.PalletPeriod;
dx 		= gCalc.MinPalletSpacing;
vmin 	= gCalc.MinCollisionVelocity;
[soln, valid] = Math_2ndOrderRoots(2.0 * dt, -4.0 * dx, - dt * vmin ^ 2 + 2.0 * dx * vmin);
if valid
	gCalc.MaxRateRiseVelocity = max(soln.r1, soln.r2);
else
	printf("Maximum throughput velocity calc failed\n"); return;
end

% The minimum rise velocity for the minimum acceleration (Collision avoidance)
gCalc.MinAccRiseVelocity = (gCalc.MinPalletSpacing - gCalc.TotalIndexDistance) / ((gCalc.MinInfeedPeriod - gPar.MinDwellTime) / 2.0);

% The minimum rise velocity for the maximum acceleration (Collision avoidance)
p2 = 1.0 / gCalc.ProfMaxAcceleration;
p1 = -2.0 * (gCalc.MinCollisionVelocity / gCalc.ProfMaxAcceleration);
p0 = (gCalc.MinInfeedPeriod - gPar.MinDwellTime) * gCalc.MinCollisionVelocity - (gCalc.MinPalletSpacing - gCalc.TotalIndexDistance);
[soln, valid] = Math_2ndOrderRoots(p2, p1, p0);
if valid
	gCalc.MaxAccRiseVelocity = max(soln.r1, soln.r2);
else
	printf("Maximum acceleration rise velocity calc failed\n"); return;
end

% Choose the rise velocity based on the overall maximum
gCalc.RiseVelocity = max([gCalc.IndexVelocity, gCalc.MaxRateRiseVelocity, gCalc.MinAccRiseVelocity, gCalc.MaxAccRiseVelocity]);

% Minimum collision acceleration
dt 		= gCalc.MinInfeedPeriod - gPar.MinDwellTime;
dx 		= gCalc.MinPalletSpacing - gCalc.TotalIndexDistance;
vmax 	= gCalc.RiseVelocity;

[soln, valid] = Kin_GetAcc(dt, dx, 0.0, 0.0, 0.0, vmax);

if valid
	gCalc.MinCollisionAcceleration = soln.a;
else
	printf("Minimum collision acceleration calculation failed\n");
endif

% Determine the exact minimum capture acceleration
tdiff 		= gCalc.SetOutfeedPeriod;
dx 			= gPar.DropPosition + gPar.DropOffset - gCalc.IndexPositions(gPar.ProductsPerPallet);
v1 			= gCalc.RiseVelocity;
vf 			= gCalc.SetOutfeedVelocity;
vmin 		= gCalc.MinCollisionVelocity;
vmax 		= gCalc.ProfMaxVelocity;
[soln, valid] = Kin_GetAccInTimespanPlus(tdiff, dx, v1, vf, vmin, vmax);
if valid
	gCalc.MinCaptureAcceleration = soln.a;
else
	printf("Minimum capture window acceleration calc failed\n"); return;
end

% Choose the profile acceleration
gCalc.ProfAcceleration = max([gCalc.IndexAcceleration, gCalc.MinCollisionAcceleration, gCalc.MinCaptureAcceleration]) * 1.05;

% Supplementary calculations
gCalc.RiseTime 			= gCalc.RiseVelocity / gCalc.ProfAcceleration;
gCalc.RiseDistance		= 0.5 * gCalc.RiseTime * gCalc.RiseVelocity;
gCalc.GainTime			= gCalc.MinInfeedPeriod - 2.0 * gCalc.RiseTime - gPar.MinDwellTime;
if gCalc.GainTime < 0.0
	gCalc.GainTime = 0.0;
endif
gCalc.GainDistance 		= gCalc.GainTime * gCalc.RiseVelocity;
gCalc.EntryTime 		= abs(RouteVelocity - gCalc.RiseVelocity) / gCalc.ProfAcceleration;
gCalc.EntryDistance 	= abs(RouteVelocity ^ 2 - gCalc.RiseVelocity ^ 2) / (2.0 * gCalc.ProfAcceleration);

% Display the results
gCalc

% Plot
plot_FullProfile
