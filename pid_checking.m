% Code for PincherX100 to observe the actual position for different PID values
global Px100;
Px100.DEVICENAME = 'COM4'; % COM port
% Initializing the robot 
init_robot();
Px100.MOVEMENT_THRESH = 1.0;
% Adjust PID gains as needed
Kp = 640;
Ki = 0;
Kd = 4000;
id = 1; % ID of the joint (waist)

% Defining the start and end positions for joint 
start_pos = 0.;  % Start position for joint in radians
end_pos = 1.;    % End position for joint in radians

% Get current joint positions
current_joint_positions = get_joint_pos();
disp(current_joint_positions);
pause(2);

% Set Starting joint position 
current_joint_positions(id) = start_pos;


% Move the robot to the home configuration
tic;  % Start timer
set_joint_pos(current_joint_positions);
time_to_move_start = toc;  % Measure time taken to reach start position
disp(['Time to move to start position: ', num2str(time_to_move_start), ' seconds.']);

% pause(2);
set_dynamixel_pid(id,Kp,Ki,Kd);  % Setting the PID gains as needed

% --- Now move from start position to end position with plotting ---
% Initialize logging data for plotting
timestamps = [];  
joint_positions_log = [];   % Actual joint positions from the servo
target_positions_log = [];  % Target joint positions (intermediate positions)

% Move the robot from start position to end position with plotting
disp('Moving from start to end position with plotting...');
N = 400;  % Number of steps for better resolution in plotting

% Number of steps before target position changes
step_interval = 400;  % Target stays the same for 5 steps
tic;
for i = 1:N
    % Calculate target position for current 5-step interval
    current_target_position = end_pos;
    
    
    % Move the robot to the current target position
    current_joint_positions(id) = current_target_position;
    set_joint_pos(current_joint_positions);
    
    % Log actual joint positions from the servo (frequent sampling)
    actual_joint_position = get_joint_pos();  % Read actual position from servo
    joint_positions_log = [joint_positions_log; actual_joint_position(id)];  % Log actual position 
    
    % Log the intermediate target position
    target_positions_log = [target_positions_log; current_target_position];  % Log target position for each step
    
    % Log timestamps (time in seconds)
    timestamps = [timestamps, toc];  
    pause(0.01);  % Optional: small pause for better resolution 
end

% Plot the actual joint position vs. time and target joint position (intermediate positions)
figure;
plot(timestamps, joint_positions_log, 'b-', 'LineWidth', 2);  % Actual joint positions (from servo)
hold on;
plot(timestamps, target_positions_log, 'r--', 'LineWidth', 2);  % Target joint positions (intermediate)
xlabel('Time (seconds)');
ylabel('Joint 1 Position (radians)');
title(['Joint 1 Actual Position vs Time | Kp = ' num2str(Kp) ', Ki = ' num2str(Ki) ', Kd = ' num2str(Kd)]);
legend('Actual Position', 'Target Position');
grid on;
xlim([0 4]);  % Limit x-axis from 0 to 4 seconds



% Get and display the final joint position after the move
joint_positions = get_joint_pos();
disp(['Final Joint position: ', num2str(joint_positions(1))]);
disp(['Waist joint (joint 1) moved from ', num2str(start_pos), ' radians to ', num2str(end_pos), ' radians.']);

% Resetting the PID gains back to default values
set_dynamixel_pid(id,640,0,4000);