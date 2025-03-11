robot = Robot();
trajGenerator = TrajGenerator();

% Define parameters
travelTime = 0.55;
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode

% End Effector initial and final positions
Pos = [50.4609, 0, 70.4062, 90;
        264.6391, 0,  119.4248, 45;
        130.8718, 0,  165.6495, 60;
        50.4609, 0, 70.4062, 90];

% Have the robot go to the starting position and idle for one second
robot.interpolate_jp(robot.ik_3001(Pos(1, :)), 2)
pause(2);

% Trajectory parameters
time = 2; % Trajectory time

j = 1; % Storage Position Counter
recordings = []; % Storage for data points

for k = 1:3
    trajectory_coefficients = []; % Empty matrix for trajectory coefficients

    for i = 1:4 % loop through each x,y,z,a trajectories
        trajectory_coefficients(i, :) = trajGenerator.quinitic_traj(Pos(k, i), Pos(k + 1, i), 0, 0, 0, 0, 0, time);
    end

    traj_vel_coeffs = [];
    for i = 1:4
        traj_vel_coeffs(i, :) = flip(polyder(flip(trajectory_coefficients(i,:))));
    end

    tic;
    while toc < time

    % Evaluate the place in the trajectory and solve for joint angles
    curr_pose = trajGenerator.eval_traj(trajectory_coefficients, toc);
    joints = robot.ik_3001(transpose(curr_pose));
    
    % Measure the Arm State
    recordings(j, 1) = toc + k*time - time; % measure time
   
    target_velocities = trajGenerator.eval_traj(flip(traj_vel_coeffs), toc);
    recordings(j, 2:5) = transpose(flip(target_velocities)); % 2-x, 3-y, 4-z, 5-a
    
    % Joint Values
    jacobian = robot.jacob3001(joints);
    joint_velocities = robot.measure_js(false, true);
    velocities = jacobian*transpose(joint_velocities(2, :));
    recordings(j, 6:11) = transpose(velocities); % 6-x, 7-y, 8-z, 9-roll, 10-pitch, 11-yaw
    
    j = j + 1;
    
    % Test for singularities
    robot.atSingularity(0.1, joints, true)
    
    
    % Drive the joints to the angles
    robot.interpolate_jp(joints, 0.08);
    
    end
end

% Trajectory Velocity Plots
figure
t = tiledlayout('vertical');

nexttile, scatter(recordings(:, 1), recordings(:, 2));
title('Trajectory X velocity');
hold on
scatter(recordings(:,1), recordings(:, 6));
legend('Trajectory Velocity', 'Measured Velocity');
hold off

nexttile, scatter(recordings(:, 1), recordings(:, 3));
title('Trajectory Y velocity');
hold on
scatter(recordings(:,1), recordings(:,7));
hold off

nexttile, scatter(recordings(:, 1), recordings(:, 4));
title('Trajectory Z velocity');
hold on
scatter(recordings(:,1), recordings(:,8));
hold off

ylabel(t, 'Velocity (mm/s)');
xlabel(t, 'Time (s)');
title(t, 'End Effector Velocity');
view;
