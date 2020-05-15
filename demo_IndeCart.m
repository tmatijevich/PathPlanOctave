%!octave

clear;

gPar = struct(
	'LoadPosition', 		0.250,
	'DropPosition', 		0.750,
	'DropOffset', 			-0.050,
	'BucketSpacing', 		0.155,
	'ToolingWidth', 		0.150,
	'ToolPocketSpacing',	0.050,
	'ProductsPerPallet', 	3,
	'Payload', 				1.0,
	'MinDwellTime', 		0.010,
	'MaxInfeedRate', 		425,
	'TargetOutfeedRate', 	500,
	'MachinesInSeries', 	2,
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
	'IndexPositions', 			[0.0; 0.0; 0.0; 0.0],
	'TotalIndexDistance', 		0.0,
	'IndexTime1', 				0.0,
	'IndexTime2', 				0.0,
	'IndexVelocity', 			0.0,
	'IndexAcceleration', 		0.0,
	'MinRiseVelocity', 			0.0,
	'MinAccelRiseVelocity', 	0.0,
	'RiseVelocity', 			0.0,
	'MinCollisionAcceleration', 0.0,
	'MinCaptureAcceleration', 	0.0,
	'ProfAcceleration', 		0.0,
	'RiseTime', 				0.0,
	'RiseDistance', 			0.0,
	'GainTime', 				0.0,
	'EntryTime', 				0.0
);


% Assume all parameters are valid

% Baseline calculations
gCalc.MinPalletSpacing 		= max(0.152 + 0.020, gPar.ToolingWidth) + 0.004;
gCalc.MinInfeedPeriod		= 60.0 / gPar.MaxInfeedRate;
gCalc.PalletPeriod 			= 60.0 / (gPar.MaxInfeedRate / gPar.ProductsPerPallet);
gCalc.MinCollisionVelocity 	= gCalc.MinPalletSpacing / gCalc.PalletPeriod;
gCalc.SetOutfeedPeriod 		= 60.0 / (gPar.TargetOutfeedRate / gPar.ProductsPerPallet);
gCalc.SetOutfeedVelocity 	= (gPar.BucketSpacing * gPar.MachinesInSeries) / gCalc.SetOutfeedPeriod;

% Profile maximums
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
