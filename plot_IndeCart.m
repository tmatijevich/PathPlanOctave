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
