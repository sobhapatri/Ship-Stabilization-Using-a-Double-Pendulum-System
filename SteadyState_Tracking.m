A = [0 1 0 0; -9.81 0 3.924 0; 0 0 0 1; 30.656 0 -12.262 0];
B = [0; 0; 0; 3.125];
C = [0 0 1 0];
D = [0];
Q = [1 0 0 0; 0 1 0 0; 0 0 1000 0; 0 0 0 1];
R = 1;
%LQR Controller
K_lqr = lqr(A, B, Q, R);
%Pole placement - state feedback
desired_poles = [-2, -3, -4, -5];
K_pp = place(A, B, desired_poles);
% Simulation Parameters
t = 0:0.01:10;
r = ones(size(t)); % Step input for steady-state tracking
% Closed-Loop Systems
Acl_lqr = A - B * K_lqr;
Acl_pp = A - B * K_pp;
% Simulate LQR System
sys_cl_lqr = ss(Acl_lqr, B, C, D);
[y_lqr, t, x_lqr] = lsim(sys_cl_lqr, r, t);
u_lqr = -K_lqr * x_lqr';
% Simulate Pole Placement System
sys_cl_pp = ss(Acl_pp, B, C, D);
[y_pp, t, x_pp] = lsim(sys_cl_pp, r, t);
u_pp = -K_pp * x_pp';
% Plot Steady-State Tracking
figure;
plot(t, r, 'k--', 'LineWidth', 1.5);
hold on;
plot(t, y_pp, 'r-', 'LineWidth', 1.5);
plot(t, y_lqr, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Output / Reference');
title('Steady-State Tracking: Step Input');
legend('Reference', 'Pole Placement', 'LQR', 'Location', 'best');
grid on;
% Steady-State Error Calculation
steady_state_error_pp = abs(y_pp(end) - r(end));
steady_state_error_lqr = abs(y_lqr(end) - r(end));
% Display Errors
fprintf('Steady-State Error (Pole Placement): %.4f\n',steady_state_error_pp);
fprintf('Steady-State Error (LQR): %.4f\n', steady_state_error_lqr);