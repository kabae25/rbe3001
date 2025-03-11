robot = Robot();  % Create a robot object to use
interpolate_time = 5; % Trajectory time (in seconds)
robot.writeMotorState(true); % Write position mode

%% Send the robot to its 0 position

%robot.writeJoints(zeros (1, 4)); % Write joints to zero position
%pause(2); % Wait for trajectory completion

%% Send the robot to an arbitrary position
pos1 = [0, -10, -30, -45];

% Initialize an array full of 0s to store the positions

positions = (zeros (100, 5));

% Run trajectory
robot.interpolate_jp(pos1, interpolate_time);

% Collect data as the robot moves

tic;  % Start timer
i = 1; % initialize variable for storing row of matrix
while toc < interpolate_time

    % Read joint positions and timesetamps into vector 
    output = robot.measure_js(true, false);
    positions(i, 1:4) = output(1, :);
    positions(i, 5) = toc;
    i = i + 1;
end

%% Plot Joint Positions

figure;
for i = 1:4
    subplot(4, 1, i);
    plot(positions(:, 5), positions(:, i), 'LineWidth', 2);
    title(['Joint ', num2str(i), ' Position vs Time']);
    ylabel('Position (degrees)');
    xlabel('Time (sec)');
    grid on;
end

%% Calculate and dispaly time step statistics
for i=1:length(positions(:, 5)) - 1
    positions(i, 5) = positions(i+1, 5) - positions(i, 5);
end

datastats(positions(1:end-1, 5))
