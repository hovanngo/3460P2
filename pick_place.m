% Code for PincherX100 to perform pick-and-place task DEMO
global Px100;
global joint_positions pick_trvec drop_trvec approaching_pick_trvec approaching_drop_trvec obstacle_trvec;
% Setting port number 
Px100.DEVICENAME = 'COM4';  % COM Port


% Initializing the robot 
init_robot();
Px100.homeconfig = homeConfiguration(Px100.robot_model);

% Defining thresholds and limits for the robot's movement
min_z_thresh = Px100.MIN_Z_THRESH;  % Minimum Z threshold is set to 0.02 m
movement_thresh = Px100.MOVEMENT_THRESH;  % Maximum movement distance is set to 0.05 m


% Define pick, drop and obstacle poses (You change it accordingly)
pick_trvec = [0.15, 0.10, 0.05];  % Desired pick position
drop_trvec = [0.15, -0.10, 0.05];  % Desired drop position
obstacle_trvec = [0.15, 0.0, 0.05]; % Obstacle position (optional)

% Defining lifting and approaching poses by adding vertical offset to the pick and drop poses
approaching_pick_trvec = [0.15, 0.1, 0.1]; % Defining a waypoint above the pick pose to avoid hitting obstacle
approaching_drop_trvec = [0.15, -0.1, 0.2]; % Defining a waypoint above the drop pose to avoid hitting obstacle
    

%% Reaching the Payload
disp('Reaching the payload...');
while true
    % Getting current joint positions from the robot,
    current_joint_positions = get_joint_pos();
    config = Px100.currentConfig;  % Start from home config
    for i = 1:4
        config(i).JointPosition = current_joint_positions(i);
    end
    current_pose = getTransform(Px100.robot_model, config, 'px100/ee_gripper_link');
    current_trvec = tform2trvec(current_pose);
    % Use this home configuration while returning back to home configuration after finishing the task
    home_trvec = current_trvec;
	
    % Calculate the no of waypoints required considering the given constraints. Use liner interpolation to plan the trajectory from current_trvec[x,y,z] to the approaching_pick_trvec[x,y,z].
    % Use the function compute_aik.m from Step1 to compute the
    % Inversekinematics to get the joint config of each and every
    % waypoint inbewteen and store the joint configs of all those
    % waypoints as "joint_space_waypoints" n x 4 matrix.

    % The input should the current_trvec[x,y,z] and the approaching_pick_trvec[x,y,z]. The output should be "joint_space_waypoints" n x 4 matrix.


    % ----------------------------------------------/////      TODO       /////-------------------------------------------------------



    joint_space_waypoints = generateWaypoints(current_trvec, approaching_pick_trvec); 
    % Execute the trajectory from current_trvec[x,y,z] to the approaching_pick_trvec[x,y,z]
    executing_trajectory(joint_space_waypoints);
    
    % Calculate the no of waypoints required considering the given constraints. Use liner interpolation to plan the trajectory from approaching_pick_trvec[x,y,z] to the pick_trvec[x,y,z].
    % Use the function compute_aik.m from Step1 to compute the
    % Inversekinematics to get the joint config of each and every
    % waypoint inbewteen and store the joint configs of all those
    % waypoints as "joint_space_waypoints" n x 4 matrix.

    % The input should the approaching_pick_trvec[x,y,z] and the pick_trvec[x,y,z]. The output should be "joint_space_waypoints" n x 4 matrix.


    % ----------------------------------------------/////      TODO       /////-------------------------------------------------------
    


    joint_space_waypoints = generateWaypoints(approaching_pick_trvec, pick_trvec);

    % Execute the trajectory from approaching_pick_trvec[x,y,z] to the pick_trvec[x,y,z]
    executing_trajectory(joint_space_waypoints);
    % Close the gripper to pick the object
    closeGripper(true); 

    pause(0.5); % Waiting for the gripper to fully close

    % Lift the payload to avoid collision
    % Calculate the no of waypoints required considering the given constraints. Use liner interpolation to plan the trajectory from pick_trvec[x,y,z] to the approaching_pick_trvec[x,y,z].
    % Use the function compute_aik.m from Step1 to compute the
    % Inversekinematics to get the joint config of each and every
    % waypoint inbewteen and store the joint configs of all those
    % waypoints as "joint_space_waypoints" n x 4 matrix.

    % The input should the pick_trvec[x,y,z] and the approaching_pick_trvec[x,y,z]. The output should be "joint_space_waypoints" n x 4 matrix.


    % ----------------------------------------------/////      TODO       /////-------------------------------------------------------


    joint_space_waypoints = generateWaypoints(pick_trvec, approaching_pick_trvec);
    % Execute the trajectory from pick_trvec[x,y,z] to the approaching_pick_trvec[x,y,z]
    executing_trajectory(joint_space_waypoints);

    break; % Exiting the loop after lifting the object
end

%% Transporting Payload
disp('Transporting the payload...');
while true
    % Calculate the no of waypoints required considering the given
    % constraints. Use liner interpolation to plan the trajectory from approaching_pick_trvec[x,y,z] to the approaching_drop_trvec[x,y,z].
    % Use the function compute_aik.m from Step1 to compute the Inversekinematics to get the joint config of each and every
    % waypoint inbewteen and store the joint configs of all those waypoints as "joint_space_waypoints" n x 4 matrix.
    

    % The input should the approaching_pick_trvec[x,y,z] and the approaching_drop_trvec[x,y,z]. The output should be "joint_space_waypoints" n x 4 matrix.


    % ----------------------------------------------/////      TODO       /////-------------------------------------------------------



    joint_space_waypoints = generateWaypoints(approaching_pick_trvec, approaching_drop_trvec);

    % Execute the trajectory from approaching_pick_trvec[x,y,z] to the approaching_drop_trvec[x,y,z]
    executing_trajectory(joint_space_waypoints);

    % Calculate the no of waypoints required considering the given
    % constraints. Use liner interpolation to plan the trajectory from approaching_drop_trvec[x,y,z] to the drop_trvec[x,y,z].
    % Use the function compute_aik.m from Step1 to compute the Inversekinematics to get the joint config of each and every
    % waypoint inbewteen and store the joint configs of all those waypoints as "joint_space_waypoints" n x 4 matrix.
    

    % The input should the approaching_drop_trvec[x,y,z] and the drop_trvec[x,y,z]. The output should be "joint_space_waypoints" n x 4 matrix.


    % ----------------------------------------------/////      TODO       /////-------------------------------------------------------


    joint_space_waypoints = generateWaypoints(approaching_drop_trvec, drop_trvec);


    % Execute the trajectory from approaching_drop_trvec[x,y,z] to the drop_trvec[x,y,z]
    executing_trajectory(joint_space_waypoints);

    % Opening the gripper to release the object
    closeGripper('false');  
    pause(0.02); % Waiting for the gripper to fully open

    break; % Exiting the loop after dropping the object
end

%% Returning to Home Configuration
disp('Returning to home configuration...');
while true
    % Get the home position of the robot
        
    % Calculate the no of waypoints required considering the given
    % constraints. Use liner interpolation to plan the trajectory from drop_trvec[x,y,z] to the approaching_drop_trvec[x,y,z].
    % Use the function compute_aik.m from Step1 to compute the Inversekinematics to get the joint config of each and every
    % waypoint inbewteen and store the joint configs of all those waypoints as "joint_space_waypoints" n x 4 matrix.
    

    % The input should the drop_trvec[x,y,z] and the approaching_drop_trvec[x,y,z]. The output should be "joint_space_waypoints" n x 4 matrix.


    % ----------------------------------------------/////      TODO       /////-------------------------------------------------------

    joint_space_waypoints = generateWaypoints(drop_trvec, approaching_drop_trvec);

    % Execute the trajectory from drop_trvec[x,y,z] to the approaching_drop_trvec[x,y,z]
    executing_trajectory(joint_space_waypoints);

    % Calculate the no of waypoints required considering the given
    % constraints. Use liner interpolation to plan the trajectory from home_trvec[x,y,z] to the approaching_drop_trvec[x,y,z].
    % Use the function compute_aik.m from Step1 to compute the Inversekinematics to get the joint config of each and every
    % waypoint inbewteen and store the joint configs of all those waypoints as "joint_space_waypoints" n x 4 matrix.
    

    % The input should the approaching_drop_trvec[x,y,z] and the home_trvec[x,y,z]. The output should be "joint_space_waypoints" n x 4 matrix.


    % ----------------------------------------------/////      TODO       /////-------------------------------------------------------
    home_trvec = [0.2398, 0, 0.2398];


        joint_space_waypoints = generateWaypoints(approaching_drop_trvec, home_trvec);
        % Execute the trajectory from approaching_drop_trvec[x,y,z] to the home_trvec[x,y,z]
        executing_trajectory(joint_space_waypoints);
        break; % Exiting the loop after reaching home configuration
    end
    
    disp("Task Performed") 

%% Functions ( Don't change the given functions)

% Executing the Trajectory
function executing_trajectory(joint_space_waypoints)
    global Px100;

    N = size(joint_space_waypoints, 1);  % Getting the number of waypoints
    
    for i = 2:N
        % Setting the joint positions for each waypoint
        val = set_joint_pos(joint_space_waypoints(i,:));

        while (true)
            % Checking if the position has been successfully set
            if (val)
                break;  % Exiting the loop if the joint positions are set correctly
            else
                continue;  % Retrying if the joint positions were not set
            end
        end
        pause(0.05);  % Small Pause before moving to the next waypoint
    end   
end

% Function to get the exact 4X4 TF using only position [x, y, z] in Cartesian Space
function T = get_exact_pose(trvec)
    % Extract x and y from the translation vector
    x = trvec(1);
    y = trvec(2);
    z = trvec(3);

    % Compute the angle around the Z-axis
    theta = atan2(y, x); 

    % Define the rotation matrix around Z-axis
    Rz = [cos(theta), -sin(theta), 0;
          sin(theta),  cos(theta), 0;
                0,          0,      1];

    % Create the 4x4 transformation matrix
    T = eye(4);
    T(1:3, 1:3) = Rz; 
    T(1:3, 4) = [x; y; z]; 
end

function jsw = generateWaypoints(start_vec, end_vec)
    % enforce movement of less 0.02 (prevent for all coordinates)
    movement_thresh = 0.01;
    % simple interpolation to calculate minimal number of waypoints for
    % given threshold
    no_of_waypoints = ceil(norm(end_vec - start_vec) / movement_thresh);
    waypoints = linspace(0, 1, no_of_waypoints)';
    % use interpolation to get proportional distance between points and
    % store in list
    cart_waypoints = (1 - waypoints) * start_vec + waypoints * end_vec;
    
    jsw = zeros(no_of_waypoints, 4);
    T = eye(4);

    % with list of waypoints get 4x4 matrix to input into compute_aik.m and
    % pass returned values in joint_space_waypoints matrix
    for i = 1 : no_of_waypoints
        T = get_exact_pose(cart_waypoints(i, :));
    
        joint_angles = compute_aik(T);
    
        jsw(i, :) = joint_angles;
    end
end

