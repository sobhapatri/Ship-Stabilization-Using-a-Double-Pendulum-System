% Define System Matrices
A = [0 1 0 0; -9.81 0 3.924 0; 0 0 0 1; 30.656 0 -12.262 0];
B = [0; 0; 0; 3.125];
C = [0 0 1 0];
D = [0];
Q = [1 0 0 0; 0 1 0 0; 0 0 1000 0; 0 0 0 1];
R = 1;
%LQR Controller
K_lqr = lqr(A, B, Q, R);
Acl_lqr = A - B * K_lqr;
%Pole placement - state feedback
desired_poles = [-2, -3, -4, -5];
K_pp = place(A, B, desired_poles);
Acl_pp = A - B * K_pp;
% Simulation Parameters
t = 0:0.01:20;
r = zeros(size(t));
r(1) = 10 / 0.01;
%Simulation
% Simulate the System for LQR
sys_cl_lqr = ss(Acl_lqr, B, C, D);
[y_lqr, t, x_lqr] = lsim(sys_cl_lqr, r, t);
u_lqr = -K_lqr * x_lqr'; % Control effort for LQR
% Simulate the System for Pole Placement
sys_cl_pp = ss(Acl_pp, B, C, D);
[y_pp, t, x_pp] = lsim(sys_cl_pp, r, t);
u_pp = -K_pp * x_pp'; % Control effort for Pole Placement
%Plots
%Plot Displacement Comparison
figure;
plot(t, y_lqr, 'b', 'LineWidth', 1.5); hold on;
plot(t, y_pp, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Displacement (theta 2)');
title('Displacement vs Time: LQR vs Pole Placement');
legend('LQR Controller', 'Pole Placement Controller',location ='best');
grid on;
% Plot Control Effort Comparison
figure;
plot(t, u_lqr, 'b', 'LineWidth', 1.5); hold on;
plot(t, u_pp, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Effort (u)');
title('Control Effort vs Time: LQR vs Pole Placement');
legend('LQR Controller', 'Pole Placement Controller',location ='best');
grid on;

%Stability Analysis
%Compute poles for both controllers
poles_pp = eig(A - B * K_pp) % Poles for Pole Placement
poles_lqr = eig(A - B * K_lqr) % Poles for LQR
% Plot pole locations
figure;
plot(real(poles_pp), imag(poles_pp), 'rx', 'MarkerSize', 10); % Plot poles for Pole Placement
hold on;
plot(real(poles_lqr), imag(poles_lqr), 'bx', 'MarkerSize', 10); % Plot poles for LQR
xlabel('Real Part');
ylabel('Imaginary Part');
title('Pole Locations (Stability Check)');
legend('Pole Placement', 'LQR',location ='northwest');
grid on;
% Adjust axis limits to make sure poles are visible
axis([-20 0 -10 10]); % Adjust the x and y axis limits for better visibility
sys_cl_pp = ss(Acl_pp, B, C, D); % Closed-loop system for Pole Placement
sys_cl_lqr = ss(Acl_lqr, B, C, D); % Closed-loop system for LQR
% Plot Step Response for both controllers
figure;
step(sys_cl_pp);
hold on;
step(sys_cl_lqr);
xlabel('Time(s)');
ylabel('Displacement (theta 2)');
title('Step Response: Stability Check');
legend('Pole Placement', 'LQR',location ='best');
grid on;
rise_time_lqr = t(find(y_lqr >= 0.9 * max(y_lqr), 1)); % Approximate rise time
settling_time_lqr = t(find(abs(y_lqr - y_lqr(end)) <= 0.02 * y_lqr(end), 1, 'last')); % Settling time
overshoot_lqr = max(y_lqr) - y_lqr(end); % Overshoot
% Pole Placement Metrics
rise_time_pp = t(find(y_pp >= 0.9 * max(y_pp), 1)); % Approximate rise time
settling_time_pp = t(find(abs(y_pp - y_pp(end)) <= 0.02 * y_pp(end), 1,'last')); % Settling time
overshoot_pp = max(y_pp) - y_pp(end); % Overshoot
% Display Metrics
fprintf('Transient Metrics:\n');
fprintf('LQR Controller: Rise Time = %.2f s, Settling Time = %.2f s,Overshoot = %.2f\n', rise_time_lqr, settling_time_lqr, overshoot_lqr);
fprintf('Pole Placement: Rise Time = %.2f s, Settling Time = %.2f s,Overshoot = %.2f\n', rise_time_pp, settling_time_pp, overshoot_pp);
L1 = 1; % Length of the first pendulum (m)
L2 = 0.8; % Length of the second pendulum (m)
m1 = 1; % Mass of the first pendulum (kg)
m2 = 0.5; % Mass of the second pendulum (kg)
g = 9.81; % Gravitational acceleration (m/s^2)
% Scale Angles for Better Visibility
scale_factor = 2; % Adjust this to exaggerate motion
theta1_scaled = scale_factor * x_lqr(:, 1); % Scaled Angle of the first pendulum (rad)
theta2_scaled = scale_factor * x_lqr(:, 2); % Scaled Angle of the second pendulum (rad)

% Animation Parameters
fps = 60; % Frames per second for smoother animation
dt = 1 / fps; % Time step
time_scale = 2; % Slows down the animation by this factor
t_anim = 0:dt:t(end) * time_scale; % Animation time vector
% Interpolate states to match animation time
theta1_interp = interp1(t, theta1_scaled, t_anim);
theta2_interp = interp1(t, theta2_scaled, t_anim);
% Initialize Video Writer
video_filename = 'double_pendulum_animation.mp4'; % Output video file name
video_writer = VideoWriter(video_filename, 'MPEG-4');
video_writer.FrameRate = fps; % Set frame rate
open(video_writer); % Open video writer
% Initialize Figure
fig = figure('Name', 'Double Pendulum Animation', 'NumberTitle', 'off', 'Color', 'w', ...
'MenuBar', 'none', 'ToolBar', 'none');
hold on;
axis equal;
xlim([-2 * (L1 + L2), 2 * (L1 + L2)]);
ylim([-2 * (L1 + L2), 2 * (L1 + L2)]);
title('Double Pendulum Animation');
xlabel('X Position');
ylabel('Y Position');
% Pendulum Plot Elements
line1 = line([0, 0], [0, 0], 'LineWidth', 2, 'Color', 'b'); % First pendulum
line2 = line([0, 0], [0, 0], 'LineWidth', 2, 'Color', 'r'); % Second pendulum
bob1 = plot(0, 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); % First mass
bob2 = plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Second mass
% Animation Loop
for i = 1:length(t_anim)
% Compute positions of pendulum bobs
x1 = L1 * sin(theta1_interp(i));
y1 = -L1 * cos(theta1_interp(i));
x2 = x1 + L2 * sin(theta2_interp(i));
y2 = y1 - L2 * cos(theta2_interp(i));
% Update plot elements
set(line1, 'XData', [0, x1], 'YData', [0, y1]);
set(line2, 'XData', [x1, x2], 'YData', [y1, y2]);
set(bob1, 'XData', x1, 'YData', y1);
set(bob2, 'XData', x2, 'YData', y2);
% Capture Frame for Video
frame = getframe(fig); % Use the explicit figure handle
writeVideo(video_writer, frame); % Write frame to video
% Pause for animation effect
pause(dt / time_scale);
end
% Close Video Writer
close(video_writer);
disp(['Animation saved as ', video_filename]);