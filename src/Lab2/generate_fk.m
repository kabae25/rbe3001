%% Step 1: Define your symbolic DH table
% See sym_example.m for guidance on how to define symbolic variables
% Remember: you CAN freely mix symbolic variables and plain-ol numbers.
robot = Robot();
% Symbols 
syms theta1 theta2 theta3 theta4

% Constants
L1 = 96.326;
L2 = sqrt(128^2 + 24^2);
L3 = 124;
L4 = 133.4;

Beta = atan2d(24, 128);

%% Step 2: Pass your symbolic DH table into dh2fk to get your symbolic 
% FK matrix
% It's really that simple. MATLAB will leave everything unsolved
% Show this to an SA for SIGN-OFF #4

DH_table = [
            theta1,                 L1,     0,      -90;
            (-90 + Beta) + theta2,   0,    L2,        0;
            ( 90 - Beta) + theta3,   0,    L3,        0;
            theta4,                  0,    L4,        0];

%% Step 3: Feed your symbolic FK matrix into 'matlabFunction' to turn it
% into a floating point precision function that runs fast.
F = robot.dh2fk(DH_table)

matlabFunction(F, 'file', 'generate_fast_fk.m');

% Write the fk_3001 function in Robot.m to complete sign-off #5

% Curiosity bonus (0 points): replicate the timeit experiment I did in
% sym_example.m to compare the matlabFunction FK function to just using
% subs to substitute the variables.

%% Lab 4

% Finding jacobian of translation component
J = jacobian(F(1:3,4), [theta1, theta2, theta3, theta4]);

% Finding jacobian of rotation component

s = size(DH_table, 1);
fkmat = eye(4);
for joint = 1:s % iterate through all joints in the dh table
    J(4:6, joint) = fkmat(1:3, 3);
    fkmat = fkmat * robot.dh2mat(DH_table(joint, :));
end

% Test cases
robot.dk3001([0; 0; 0; 0], [0.1; 0; 0; 0])
robot.dk3001([0; 0; 0; 0], [0; 0.1; 0; 0])

%matlabFunction(J, 'file', 'Jacobian.m');

robot.jacob3001([10, 10, 10, 10])
robot.jacob3001([80, 80, 80, 80])

