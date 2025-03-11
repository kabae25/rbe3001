% Create a robot object
travelTime = .5; % Defines the travel time
robot = Robot();
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode

% Handle FK Tasks
syms theta1 theta2 theta3 theta4 % generate symbols

% Define constants
L1 = 96.326;
L2 = sqrt(128^2 + 24^2);
L3 = 124;
L4 = 133.4;
Beta = atan2d(24, 128);

DH_table = [
            theta1,                 L1,     0,      -90;
            (-90 + Beta) + theta2,   0,    L2,        0;
            ( 90 - Beta) + theta3,   0,    L3,        0;
            theta4,                  0,    L4,        0];

fk = matlabFunction(robot.dh2fk(DH_table));

% Define three joint positions to form the vertices of your triangle
% Make sure theta_1 = 0 for all of these configurations!
positions = [
    0   -45   45   90; % first position - close
    0   15   -15   45; % second position - far 
    0   -30   0   90; % third position - aprex
];

% Move your robot to the first vertex
robot.writeJoints(positions(3, :)); % Write joints to zero position
pause(travelTime); % pause and wait for trajectory completion

recordings = (zeros (100, 5));
% Create a for loop that sends your robot to each vertex, recording the
% joint-space position the entire time


j = 1; % initialize variable for storing row of matrix
for i = 1:3
    i
    pause(travelTime);
    robot.interpolate_jp(positions(i, :), travelTime);
    
    tic;  % Start timer
    while toc < travelTime
        % Read joint positions and timesetamps into vector 
        output = robot.measure_js(true, false);
        recordings(j, 1:4) = output(1, :);
        recordings(j, 5) = toc;
        j = j + 1;
    end
    % See lab_demo.m from lab 1 for inspiration on how to move your robot
    % and collect data
end

% Loop through all collected data and convert it from joint-space to task
% space using your fk3001 function
taskspace = (zeros (100, 4));
for i = 1:size(recordings)
    V = robot.fk_3001(recordings(i,1), recordings(i,2), recordings(i,3), recordings(i,4));
    taskspace(i, 1) = V(1,4);
    taskspace(i, 2) = V(2,4);
    taskspace(i, 3) = V(3,4);
    taskspace(i, 4) = recordings(i, 5);
end

% Plot the trajectory of your robot in x-y-z space using scatter3
% https://www.mathworks.com/help/matlab/ref/scatter3.html
figure
scatter3(taskspace(:,1), taskspace(:,2), taskspace(:,3));
title('End Effector in  Task Space (mm)');
xlabel('Global X Position');
ylabel('Global Y Position');
yscale(100);
zlabel('Global Z Position');
view

% Plot the trajectory of your robot in theta2-theta3-theta-4 space using
% scatter3
figure
scatter3(recordings(:,2), recordings(:,3), recordings(:,4));
title('Trajectory of Robot in theta2-theta3-theta4 Space (deg)');
xlabel('theta2');
ylabel('theta3');
zlabel('theta4');
view

