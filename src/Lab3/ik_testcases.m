% Create new robot object
robot = Robot();
travelTime = 3;
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode

% --- Define three sets of joint angles (in degrees) away from singularities ---
% (Make sure none of the positions lie along a singular configuration like [0,0,1]^T.)
jointSet1 = [0, 0, 0, 0];    % [theta1, theta2, theta3, alpha]
jointSet2 = [40, 55, 65, 80];
jointSet3 = [25, 50, 70, 100];

% --- Compute the corresponding task space positions using fk_3001 ---
% fk_3001 is a method of the robot object, so we need to call it using robot.
% It takes four joint angles as inputs and returns a 4x1 position [x; y; z; alpha].
pos1 = robot.fk_3001(jointSet1(1), jointSet1(2), jointSet1(3), jointSet1(4));
pos2 = robot.fk_3001(jointSet2(1), jointSet2(2), jointSet2(3), jointSet2(4));
pos3 = robot.fk_3001(jointSet3(1), jointSet3(2), jointSet3(3), jointSet3(4));

% Convert positions to proper input format for ik_3001
ik_input1 = [pos1(1, 4); pos1(2, 4); pos1(3, 4); jointSet1(4)]; % Keeping alpha
ik_input2 = [pos2(1, 4); pos2(2, 4); pos2(3, 4); jointSet2(4)];
ik_input3 = [pos3(1, 4); pos3(2, 4); pos3(3, 4); jointSet3(4)];

% --- Now test your inverse kinematics (ik_3001) using the computed positions ---
% ik_3001 is a method of the robot object, so we need to call it using robot.
ikJointSet1 = robot.ik_3001(ik_input1);
ikJointSet2 = robot.ik_3001(ik_input2);
ikJointSet3 = robot.ik_3001(ik_input3);

fk = robot.fk_3001(ikJointSet1(1), ikJointSet1(2), ikJointSet1(3), ikJointSet1(4));
fk1 = robot.fk_3001(ikJointSet2(1), ikJointSet2(2), ikJointSet2(3), ikJointSet2(4));
fk2 = robot.fk_3001(ikJointSet3(1), ikJointSet3(2), ikJointSet3(3), ikJointSet3(4));


% --- Display the results ---
fprintf('Test Case 1:\n');
disp('Original Joint Angles:');
disp(jointSet1);
disp('Task Space Position from fk_3001:');
disp(pos1);
disp('Recovered Joint Angles from ik_3001:');
disp(ikJointSet1);
disp(fk);

fprintf('\nTest Case 2:\n');
disp('Original Joint Angles:');
disp(jointSet2);
disp('Task Space Position from fk_3001:');
disp(pos2);
disp('Recovered Joint Angles from ik_3001:');
disp(ikJointSet2);
disp(fk1);

fprintf('\nTest Case 3:\n');
disp('Original Joint Angles:');
disp(jointSet3);
disp('Task Space Position from fk_3001:');
disp(pos3);
disp('Recovered Joint Angles from ik_3001:');
disp(ikJointSet3);
disp(fk2);