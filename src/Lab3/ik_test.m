%create new robot object
robot = Robot();
travelTime = 3;
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode

testPosition = [0  , -30, 0, 90];

test_fk = robot.fk_3001(testPosition(1), testPosition(2), testPosition(3), testPosition(4))

ik_input = [test_fk(1, 4); test_fk(2, 4); test_fk(3, 4); 60];

test_ik = robot.ik_3001(ik_input);

fk = robot.fk_3001(test_ik(1), test_ik(2), test_ik(3), test_ik(4));


