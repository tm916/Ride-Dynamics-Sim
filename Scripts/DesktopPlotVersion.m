data = readtable('amplitude_study_final.csv');
k = unique(data.k_s);
b = unique(data.b_s);
A = reshape(data.amp_body, numel(b), numel(k)) / 0.005;   % amplitude ratio
[K, B] = meshgrid(k, b);

surf(K, B, A)
xlabel('Spring wheel rate k_s (N/m)')
ylabel('Damper wheel rate b_s (Ns/m)')
zlabel('Body amplitude/road amplitude')
title('Body amplitude/road amplitude, 5 Hz road input')
colorbar
view(-37.5, 30)
exportgraphics(gcf, 'amplitude_surface.png', 'Resolution', 300)

figure
contourf(K, B, A, 0.4:0.2:1.6, 'ShowText', 'on')
xlabel('Spring wheel rate k_s (N/m)')
ylabel('Damper wheel rate b_s (Ns/m)')
title('Body amplitude/road amplitude, 5 Hz road input')
colorbar
exportgraphics(gcf, 'amplitude_contour.png', 'Resolution', 300)

py  = readmatrix('python_zs.csv', 'NumHeaderLines', 1);
t_py = py(:,1);  z_py = py(:,2);
out  = sim('QuarterCarSim');
z_sl = interp1(out.zs_sl.Time, squeeze(out.zs_sl.Data), t_py, 'spline');
T = 2*pi/omega;
ss = t_py >= t_py(end) - 5*T;
amp_ss = 0.5*(max(z_py(ss)) - min(z_py(ss)));
err_pct = max(abs(z_sl - z_py)) / amp_ss * 100;
fprintf('Max difference: %.4f%% of steady-state amplitude\n', err_pct)

figure
plot(t_py, z_py, t_py, z_sl, '--')
legend('Python (Verlet)', 'Simulink (ode45)'); xlabel('Time (s)'); ylabel('z_s (m)')