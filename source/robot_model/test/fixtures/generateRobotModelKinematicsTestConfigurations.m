function generateRobotModelKinematicsTestConfigurations(urdf, nConfigurations)
% Generate configurations and expected results for a robot model
%
%   generateRobotModelKinematicsTestConfigurations expects a file 
%   `panda_arm.urdf` in the same folder and generates 3 random 
%   configurations of joint position and velocity. It calculates the 
%   forward position and velocity kinematics for the end effector and 
%   link 4 and prints out a C++ formatted result to be used in test 
%   fixtures.
%
%   generateRobotModelKinematicsTestConfigurations(urdf) takes a filepath 
%   to a urdf robot model to load.
%
%   generateRobotModelKinematicsTestConfigurations(urdf, nConfigurations) 
%   generates a specified number of random configurations (default 3).
%
%   Requires Robotics System Toolbox

arguments
    urdf string = 'panda_arm.urdf';
    nConfigurations (1,1) double = 3;
end

% load the robot model
robot = importrobot(urdf);
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

% helper function to format numeric array [a, b, c] into '{a, b, c}'
num2stdvec = @(A) (['{', strjoin(compose('%f', A), ', ') '}']);

% code generation (declarations)
fprintf('std::vector<state_representation::JointState> test_configs;\n');
fprintf('std::vector<std::vector<double>> test_fk_ee_expects;\n');
fprintf('std::vector<std::vector<double>> test_fk_link_expects;\n');
fprintf('std::vector<std::vector<double>> test_velocity_fk_expects;\n');

% generate the configurations
for conf = 1:nConfigurations
    % configurations
    q = robot.randomConfiguration;
    v = rand(size(q))*2 - 1;
    
    % jacobians
    ee_jac = robot.geometricJacobian(q, 'panda_link8');
    link4_jac = robot.geometricJacobian(q, 'panda_link4');
    
    % expected results
    ee_pose = ee_jac * q';
    ee_pose = [ee_pose(1:3); eul2quat(ee_pose(4:end)')'];
    link4_pose = link4_jac * q';
    link4_pose = [link4_pose(1:3); eul2quat(link4_pose(4:end)')'];
    ee_twist = ee_jac * v';
    
    % code generation (definitions)
    fprintf('\n// Random test configuration %i:\n', conf);
    fprintf('state_representation::JointState config%i("robot", %i);\n', ...
        conf, numel(q));
    fprintf('config%i.set_positions(%s);\n', conf, num2stdvec(q));
    fprintf('config%i.set_velocities(%s);\n', conf, num2stdvec(v));
    fprintf('test_configs.push_back(config%i);\n', conf);
    
    fprintf('\n// Expected results for configuration %i:\n', conf);
    fprintf('test_fk_ee_expects.push_back(%s);\n', num2stdvec(ee_pose));
    fprintf('test_fk_link4_expects.push_back(%s);\n', num2stdvec(link4_pose));
    fprintf('test_velocity_fk_expects.push_back(%s);\n', num2stdvec(ee_twist));
end