%Parameters
m1 = 1.0; % mass 1 (kg)
m2 = 0.5; % mass 2 (kg)
L1 = 1.0; % length 1 (m)
L2 = 0.8; % length 2 (m)
g = 9.81; % gravity (m/s^2)
I1 = m1 * (L1^2); % Moment of inertia for mass 1
I2 = m2 * (L2^2); % Moment of inertia for mass 2
%SSF Matrices
A = [0, 1, 0, 0;
-(m1*g*L1)/I1, 0, (m2*g*L2)/I1, 0;
0, 0, 0, 1;
(m1*g*L1)/I2, 0, -(m2*g*L2)/I2, 0];
disp(A)
fprintf('\n')
B = [0;
0;
0;
1/I2];
disp(B)
fprintf('\n')
C = [0, 0, 1, 0];
disp(C)
fprintf('\n')
% Controllability check
Controllability_Matrix = ctrb(A, B)
Controllability_Rank = rank(Controllability_Matrix)
if Controllability_Rank == size(A, 1)
disp('The system is controllable.');
else
disp('The system is not controllable.');
end
fprintf('\n')
% Observability check
Observability_Matrix = obsv(A, C)
Observability_Rank = rank(Observability_Matrix)
if Observability_Rank == size(A, 1)
disp('The system is observable.');
else
disp('The system is not observable.');
end
fprintf('\n')
% Stability check% Stability check
Eigenvalues = eig(A);
disp('Eigenvalues of A:');
% Round the real part to the nearest integer
%real_part_rounded = round(real(Eigenvalues));
% Keep the imaginary part unchanged
result = real(Eigenvalues) + imag(Eigenvalues)*1i;
% Display the result
disp(result);
if all(real(result) < 0)
disp('The system is stable.');
elseif any(real(result) > 0)
disp('The system is unstable.');
else
disp('Since the real part of eigen values are very close to zero,')
disp('The system is marginally stable.');
end
den_coeffs=[2*250000 0 2*5518000 0 -2*981];
den_roots = roots(den_coeffs);
disp('Poles of the transfer function:');
disp(den_roots);
if all(real(den_roots) < 0)
disp('The system is BIBO stable');
else
disp('The system is BIBO unstable');
end