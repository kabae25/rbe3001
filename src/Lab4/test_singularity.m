robot = Robot();
trajGenerator = TrajGenerator();

% Define parameters
travelTime = 0.55;
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode

% End Effector initial and final positions
Pos = [
  0, 75, 375, -90; %0, 75, 375, -90
  0, -75, 375, -90; % 0, -75, 375, -90
];

% Have the robot go to the starting position and idle for one second
robot.interpolate_jp(robot.ik_3001(Pos(1, :)), 2)
pause(2);

% Trajectory parameters
time = 10; % Trajectory time

j = 1; % Storage Position Counter
recordings = zeros(6000, 10); % Storage for data points

trajectory_coefficients = []; % Empty matrix for trajectory coefficients

for i = 1:4 % loop through each x,y,z,a trajectories
    trajectory_coefficients(i, :) = trajGenerator.quinitic_traj(Pos(1, i), Pos(2, i), 0, 0, 0, 0, 0, time);
end

tic;
while toc < time

    % Evaluate the place in the trajectory and solve for joint angles
    curr_pose = trajGenerator.eval_traj(trajectory_coefficients, toc);
    joints = robot.ik_3001(transpose(curr_pose));

    % Measure the Arm State
    measurement = robot.measure_js(true, false);
    recordings(j, 1:4) = measurement(1, :);

    prediction = robot.fk_3001(joints(1,1), joints(1,2), joints(1,3), joints(1,4));

    recordings(j, 5) = prediction(1, 4);
    recordings(j, 6) = prediction(2, 4);
    recordings(j, 7) = prediction(3, 4);

    jacob = robot.jacob3001(joints)
    recordings(j, 8) = det(jacob(1:3, 1:3));

    recordings(j, 9) = curr_pose(1);
    recordings(j, 10) = curr_pose(2);
    recordings(j, 11) = curr_pose(3);

    recordings(j, 12) = toc;
    j = j + 1;

    % Test for singularities
    % true for estop false for continue
    robot.atSingularity(0.1,joints, true)
 

    % Drive the joints to the angles
    robot.interpolate_jp(joints, 0.08);

end

taskspace = (zeros (100, 4));
for i = 1:size(recordings)
    V = robot.fk_3001(recordings(i,1), recordings(i,2), recordings(i,3), recordings(i,4));
    taskspace(i, 1) = V(1,4);
    taskspace(i, 2) = V(2,4);
    taskspace(i, 3) = V(3,4);
    taskspace(i, 4) = recordings(i, 5);
end


% Plot the trajectory of your robot in x-y-z space using scatter3
figure
subplot(1, 2, 1);
scatter3(taskspace(:,1), taskspace(:,2), taskspace(:,3));
title('Measured Position in Task Space (mm)');
ylabel('Y Position');
xlabel('X Position')
zlabel('Z Position');

% % plot the predicted trajectories
% subplot(2, 2, 2);
% scatter3(recordings(:, 5), recordings(:, 6), recordings(:,7)); 
% title('Predicted Position in Task Space (mm)')
% ylabel('Global Y Position');
% xlabel('Global X position');
% zlabel('Global Z position');

% Plot the determinant of the jacobian
subplot(1, 2, 2);
scatter(recordings(:, 12), recordings(:,8));
xlabel('Time (s)');
ylabel('Determinant');
title('Determinant of the Jacobian');

% % Plot the trajectory generation in task space
% subplot(2, 2, 4);
% scatter3(recordings(:,9), recordings(:,10), recordings(:,11));
% title('Trajectory in Task Space')
% ylabel('Global Y Position');
% xlabel('Global X position');
% zlabel('Global Z position');
% view;