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
fprintf('std::vector<state_representation::CartesianPose> test_fk_ee_expects;\n');
fprintf('std::vector<state_representation::CartesianPose> test_fk_link4_expects;\n');
fprintf('std::vector<state_representation::CartesianTwist> test_velocity_fk_expects;\n');
fprintf('Eigen::Matrix<double, 6, 1> twist;\n');

% generate the configurations
for conf = 1:nConfigurations
    % configurations
    q = robot.randomConfiguration;
    v = rand(size(q))*2 - 1;
    
    % transforms
    ee_T = robot.getTransform(q, 'panda_link8');
    link4_T = robot.getTransform(q, 'panda_link4');
    
    % jacobian
    ee_jac = robot.geometricJacobian(q, 'panda_link8');
    
    % expected results
    ee_pose = [tform2trvec(ee_T), tform2quat(ee_T)]; % (1x7)
    link4_pose = [tform2trvec(link4_T), tform2quat(link4_T)]; % (1x7)
    ee_twist = ee_jac * v'; % (1x6)
    
    % code generation (definitions)
    fprintf('\n// Random test configuration %i:\n', conf);
    fprintf('state_representation::JointState config%i("robot", %i);\n', ...
        conf, numel(q));
    fprintf('config%i.set_positions(%s);\n', conf, num2stdvec(q));
    fprintf('config%i.set_velocities(%s);\n', conf, num2stdvec(v));
    fprintf('test_configs.push_back(config%i);\n', conf);
    
    fprintf('\n// Expected results for configuration %i:\n', conf);
    fprintf(['test_fk_ee_expects.emplace_back(state_representation::CartesianPose' ...
        '("ee", Eigen::Vector3d(%f, %f, %f), Eigen::Quaterniond(%f, %f, %f, %f)));\n'], ...
        ee_pose(1:3), ee_pose(4:7));
    fprintf(['test_fk_link4_expects.emplace_back(state_representation::CartesianPose' ...
        '("link4", Eigen::Vector3d(%f, %f, %f), Eigen::Quaterniond(%f, %f, %f, %f)));\n'], ...
        link4_pose(1:3), link4_pose(4:7));
    fprintf('twist << %f, %f, %f, %f, %f, %f;\n', ee_twist);
    fprintf('test_velocity_fk_expects.emplace_back(state_representation::CartesianTwist("ee", twist));\n');
end