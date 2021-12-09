%!octave

t_s = 0.005;
tPlot = 0.0:t_s:1.0;
n = length(tPlot);
xtPlot = zeros(size(tPlot));
xPlot = zeros(size(tPlot));
vPlot = zeros(size(tPlot));
aPlot = zeros(size(tPlot));

xtPlot(1) = 250.0; % 250 mm away

for idx = 2:n
    if tPlot(idx) <= 0.250 
        xtPlot(idx) = xtPlot(idx - 1) + 1500.0 * t_s; % Moving at 1500 mm/s
    elseif tPlot(idx) > 0.75
        xtPlot(idx) = xtPlot(idx - 1) - 1500.0 * t_s; % Moving at 1500 mm/s
    else
        xtPlot(idx) = xtPlot(idx - 1);
    end

    printf("%3d %1.3f s ", idx, tPlot(idx));

    [trackSolution, trackValid] = PathTrack(xPlot(idx - 1), vPlot(idx - 1), t_s, xtPlot(idx), 2500.0, 15000.0, true);
    if trackValid
        xPlot(idx) = trackSolution.x;
        vPlot(idx) = trackSolution.v;
        aPlot(idx) = trackSolution.a;
    else
        return;
    end

end