close all
clearvars
clc

mass = 0.405;
J = [2098e-6, 63.577538e-6, -2.002648e-6; 
     63.577538e-6, 2102e-6, 0.286186e-6; 
     -2.002648e-6, 0.286186e-6, 4068e-6];

A = [zeros(3,3), eye(3); zeros(3,6)];        % System dynamics
B_trans = [zeros(3,3); eye(3)/mass];               % Input matrix
B_rot = [zeros(3,3); inv(J)]; 
C = [eye(3), zeros(3,3)];                    % Output matrix

I3 = eye(3);                                 % Identity matrix

A_ext_trans = [A, B_trans, zeros(6,3);                  % First block row
         zeros(3,6), zeros(3,3), I3;       % Second block row
         zeros(3,6), -0* I3, zeros(3,3)];     % Third block row

A_ext_rot = [A, B_rot, zeros(6,3);                  % First block row
         zeros(3,6), zeros(3,3), I3;       % Second block row
         zeros(3,6), - 0* I3, zeros(3,3)];     % Third block row

C_ext = [C, zeros(3,6)];                    % Output matrix

desired_eigenvalues = [-1, -1, -1, -2, -2, -2, -3, -3, -3, -5, -5, -5];

L_ext_trans = place(A_ext_trans', C_ext', desired_eigenvalues)';  % Transpose for correct dimensions

L_ext_rot = place(A_ext_rot', C_ext', desired_eigenvalues)';  % Transpose for correct dimensions

disp('Translational Extended Observer Gain Matrix (L_ext):');
disp(L_ext_trans);

disp('Rotational Extended Observer Gain Matrix (L_ext):');
disp(L_ext_rot);

