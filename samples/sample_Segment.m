%!octave

t = [0.100, 0.400];
v = [1.200, 1.900];
textSize = 13;

hFig = figure(1, "name", "Velocity Segment"); set(hFig, "menubar", "none");
cla;
hold on;
area(t, v, "facecolor", [0.8 0.8 0.8], "edgecolor", "none");
plot(t, v, "b.-", "markersize", 18.0, "linewidth", 1.5);
xlim([0.0 0.5]);
text(t(1) - 0.03, v(1) + 0.1, "v_0, t_0", "fontsize", textSize);
text(t(2)       , v(2) + 0.1, "v_f, t_f", "fontsize", textSize);
text(0.250, 1.0, "{\\delta}x", "interpreter", "tex", "fontsize", textSize);
text(0.225, 1.6, "a", "fontsize", textSize);
axis("nolabel");
box off;
ylabel("Velocity", "fontsize", textSize);
xlabel("Time", "fontsize", textSize);
