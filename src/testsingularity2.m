%create new robot object
robot = Robot();
trajGenerator = TrajGenerator();

% Define parameters
travelTime = 0.55;
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode

% End Effector initial and final positions
Pos0 = [50, 50, 175, 30;
        50, 150,  175, 30]
%         130.8718, 0,  165.6495, 60;
%         50.4609, -60, 70.4062, 90];

% Pos = [
%   0, 50, 100, 20;
%   0,-10, 100, 20; 
% %     0, 50, 100, 20;
% ];


% Have the robot go to the starting position and idle for one second
robot.interpolate_jp(robot.ik_3001(Pos0(1, :)), 1);
pause(3);

% Trajectory parameters
time = 6; % Trajectory time

j = 1; % Storage Position Counter
recordings = zeros(6000, 1); % Storage for data points

trajectory_coefficients = []; % Empty matrix for trajectory coefficients
for k = 1:1
    tic;
    
    for i = 1:4 % loop through each x,y,z,a trajectories
        trajectory_coefficients(i, :) = trajGenerator.quinitic_traj(Pos(k, i), Pos(k + 1, i), 0, 0, 0, 0, 0, time);
    end

        while toc < time

        % Evaluate the place in the trajectory and solve for joint angles
        thetas = trajGenerator.eval_traj(trajectory_coefficients, toc);
        joints = robot.ik_3001(transpose(thetas));
        if(robot.atSingularity(5, joints))
            disp('AT SINGULARITY')
            break;
        end

        % Drive the joints to the angles
        robot.interpolate_jp(joints, 0.08);
    
        % Measure the Arm State
        output = robot.measure_js(true, false);
        recordings(j, 1:4) = output(1, :);
        recordings(j, 5) = toc;
        j = j + 1;

        end

end

% Process joint recordings into task space
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
scatter3(taskspace(:,1), taskspace(:,2), taskspace(:,3));
title('End Effector in  Task Space (mm)');
xlabel('Global X Position');
ylabel('Global Y Position');
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