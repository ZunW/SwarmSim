classdef DiffDriveBehaviorBasedBlend < control
    %DIFFDRIVEBEHAVIORBASED 
    % behavior based controller for robot to navigate 
    % differential drive dynamics
    % including the following behaviors:
    % Go to Goal
    % Aboid Obstacles
    % (use blend approach instead of finite state machine)
    
    properties
        %fsm
        goal
        behaviors
        numBehaviors
        controllers
        sensors
    end
    
    methods
        function obj = DiffDriveBehaviorBasedBlend(robotInfo,goal)
            %DIFFDRIVEBEHAVIORBASED 
            % contruct a behavior-based controller
            valid_dynamics = ["DiffDrive"];
            if (~ismember(robotInfo.type,valid_dynamics))
                msg = "Behavior-based controller: wrong dynamics type";
                error(msg);
            end
            obj.goal = goal;
            obj.behaviors = {'go-to-goal';'avoid-wall';'random'};
            obj.numBehaviors = size(obj.behaviors,1);
            obj.controllers = cell(1,obj.numBehaviors);
            obj.sensors = RangeFinders(robotInfo);
            pursuitInfo = PursuitInfo(0.35,0.75,1.5);
            for i = 1:obj.numBehaviors
                if (obj.behaviors(i,:) == "go-to-goal")
                    obj.controllers{i} = DiffDriveGoToGoal(robotInfo,pursuitInfo,goal);
                end
                if (obj.behaviors(i,:) == "avoid-wall")
                    obj.controllers{i} = DiffDriveAvoidWall(robotInfo,pursuitInfo);
                end
                if (obj.behaviors(i,:) == "random")
                    obj.controllers{i} = RandomWalkDiffDrive([0 1],[-1 -1]);
                end
            end
            % initialize finite state machine
            %obj.fsm = BehaviorBasedFSM("go-to-goal");
        end
        
        function control = compute_control(obj,poses,raw_reads, robot_id, numRobots)

            %get some preliminary data
            positions = poses(1:2,:);
            own_pose = poses(:,robot_id); 
            
            %set some preliminary parameters
            vip_id = [1];
            w_limit = pi;
            v_limit = 1;
            ideal_distance = 1;
            not_too_far = 1.5;
            turn_angle_limit = pi/2;
            
            vip_w_limit = w_limit*3;
            vip_v_limit = v_limit*3;  %needs to go faster if necessary
            vip_to_centroid_threshold = 0.5;   %for guards' checking vip's status
            vip_to_centroid_coeff = 1;
            vip_to_goal_coeff = 0.5;

            
            to_goal_base_magnitude = 1;
            avoid_xy_coeff = 1;
            
            
            %below are all in [x y] format in world frame
            %get the direction to the goal
            to_goal_xy_unit = obj.to_goal(own_pose);
            to_goal_xy = to_goal_xy_unit * to_goal_base_magnitude;
            
            %get the direction for avoiding obstacles
            avoid_xy_raw = obj.obstacle_avoidance(raw_reads, ideal_distance, own_pose(3));
            avoid_xy = avoid_xy_raw * avoid_xy_coeff;
            
            
            control.wRef = 0;
            control.vRef = 0;
               
            %calculate the "centroid"
            total_position = [0; 0];
            for i = 1:numRobots
                if ismember(i, vip_id)
                    %do nothing
                else
                    position = positions(:,i);
                    total_position = total_position + position;
                end
            end
            centroid = total_position/(numRobots-1); %in world coord

            
            if ismember(robot_id, vip_id)    %important
%                 if centroid(2)>=own_pose(2)
%                     disp("lower or equal to centroid")
%                 else
%                     disp("higher than centroid")
%                 end
%                 disp(own_pose);
                
                x_to_centroid =  (centroid(1) - own_pose(1));
                y_to_centroid =  (centroid(2) - own_pose(2));
                vip_to_centroid_distance = norm([x_to_centroid, y_to_centroid]);
                
                %when the vip is not at centroid, to_centroid will dominate
                %if vip has to go against the group's direction in order to
                %go inside, by to_centroid, others will make way
                x_diff = x_to_centroid * vip_to_centroid_coeff + avoid_xy(1)*0.1;
                y_diff = y_to_centroid * vip_to_centroid_coeff + avoid_xy(2)*0.1;
                final_xy = [x_diff y_diff];
                
                if vip_to_centroid_distance < vip_to_centroid_threshold
                    %we will add a bit to go to goal
                    final_xy = final_xy + to_goal_xy*vip_to_goal_coeff;
                end
                
                target_direction_world = atan2(final_xy(2), final_xy(1));
                turn_theta = obj.get_turn_angle(target_direction_world, own_pose(3));
%                 fprintf("world_direction is %d\n", target_direction_world);
%                 fprintf("turn_theta is %d\n", turn_theta);
                
                speed_coeff = norm(final_xy)*2;

                target_w = turn_theta * speed_coeff; %further from center, faster
                
                control.wRef = target_w;
                %check for speed limit
                if control.wRef >= 0
                    control.wRef = min(control.wRef, vip_w_limit);
                else
                    control.wRef = max(control.wRef, -vip_w_limit);
                end
%                 fprintf("control.wRef is %d\n\n\n", control.wRef);
                

                control.vRef = min(vip_v_limit, speed_coeff);
                %bigger turn_theta means we are turning back, we need to
                %slow down or even stop. tunr_theta in [-pi,pi]
                turn_angle_limit =turn_angle_limit * 2;
                if turn_theta > turn_angle_limit
                    control.vRef = 0;
                else
                    control.vRef = control.vRef * ((turn_angle_limit - abs(turn_theta)) / turn_angle_limit);
                end

            else    %others
                
                to_vip_xy = [0 0];
                for i = 1:length(vip_id)
                    curr = obj.to_vip(own_pose(1:2), poses(1:2, vip_id(i)), not_too_far);
                    to_vip_xy = to_vip_xy + curr;
                end
                to_vip_xy = to_vip_xy / length(vip_id);
                
                final_xy = avoid_xy + to_goal_xy + to_vip_xy;
                
                target_direction_world = atan2(final_xy(2), final_xy(1));
                turn_theta = obj.get_turn_angle(target_direction_world, own_pose(3));
                speed_coeff = norm(final_xy);
                
                control.wRef = turn_theta * speed_coeff;
                
                %check for speed limit
                if control.wRef >= 0
                    control.wRef = min(control.wRef, w_limit);
                else
                    control.wRef = max(control.wRef, -w_limit);
                end

                %bigger target_direct means we are turning back, we need to
                %slow down. turn_theta in [-pi,pi]
                control.vRef = min(v_limit, speed_coeff); 
                if turn_theta > turn_angle_limit
                    control.vRef = 0;
                else
                    control.vRef = control.vRef * ((turn_angle_limit - abs(turn_theta)) / turn_angle_limit);
                end

            end

        end

        
        
        %potential field method
        %output the direciton to avoid the obstacles in cartesian coord. in
        %world frame
        %easier for later computation
        %the magnitude of this direction will tell both v and w
        function avoid_xy = obstacle_avoidance(obj, sensor_readings, ideal_distance, theta)
            
            num_sensors = obj.sensors.numSensors;
            max_range = obj.sensors.max_range;
            
            %sensors read from -pi to pi
            sensor_angles = linspace(-pi,pi,num_sensors);
            
            total_force_x = 0;
            total_force_y = 0;
            count = 0;
            
            for i = 1:num_sensors
                if isnan(sensor_readings(i))
                    %this sensor doesn't see obstacles, do nothing
                
                %sees obstacles, and the distance is very close
                elseif sensor_readings(i) <= ideal_distance   
                    distance = sensor_readings(i);
                    
                    if distance > max_range
                        disp("warning! sensor failure")
                    end
                                        
                    normalize_distance = distance / ideal_distance;
                    force = 1 / normalize_distance^2; %square might be too big
%                     force = 1 / normalize_distance;
                    
                    
                    angle = sensor_angles(i);
                    opposite = 0;
                    if angle >= 0
                        opposite = angle - pi;
                    else 
                        opposite = angle + pi;
                    end
                    force_x = force * cos(opposite);
                    force_y = force * sin(opposite);
                    
                    total_force_x = total_force_x + force_x;
                    total_force_y = total_force_y + force_y;
                    
                    count = count + 1;
                    
                end
            
            end
            
            avoid_xy_body = [total_force_x total_force_y] ./ count;
            
            %if there's no obstacles found, all readings will be NaN
            if isnan(avoid_xy_body(1)) || isnan(avoid_xy_body(2))
                avoid_xy_body = [0 0];
            end
            
            avoid_xy = avoid_xy_body * [cos(theta) sin(theta); -sin(theta) cos(theta)];

        end
        
        
        %will give the unit direction to go to goal
        function to_goal_xy_unit = to_goal(obj, pose)
            goal = obj.goal;
            
            x_diff = goal(1) - pose(1);
            y_diff = goal(2) - pose(2);
            
            magnitude = norm([x_diff y_diff]);
            
            %will use a constant w to turn to goal
            to_goal_xy_unit = [x_diff y_diff] ./magnitude;
        
        end
        
        
        %to the important agents, in world frame
        function to_vip_xy = to_vip(obj, own_position, vip_pose_position, ideal_distance)
            x_diff = vip_pose_position(1) - own_position(1);
            y_diff = vip_pose_position(2) - own_position(2);
            
            distance = norm([x_diff y_diff]);
            unit_direction = [x_diff y_diff] ./ distance;
            
            if distance < ideal_distance
                %already close enough, this component will be zero
                to_vip_xy = [0 0];
            else    %we are a bit too far
                strength = distance - ideal_distance;
                to_vip_xy = unit_direction * strength;
            end
        
        end
        
        
        %to get the turning angle
        %to make sure no angles is beyong [-pi, pi]
        function turn_theta = get_turn_angle(obj, target_direction, current_direction)
            diff = target_direction - current_direction;
            if diff < -pi
                turn_theta = diff + 2*pi;
            elseif diff > pi
                turn_theta = diff - 2*pi;
            else
                turn_theta = diff;
            end
        end
        
        
    end
end



