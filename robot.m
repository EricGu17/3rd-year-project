classdef robot < handle
    properties
        vrep             % Connection to coppeliasim remote api
        clientID        % ID of current simulation
     
        motors          % Handler for wheels
        joints          % Handler for arm joints
        cameras         % Handler for mounted cameras
        proximity       % Handler for proximity sensor
        
        doors           % Handler for doors in simulation world
        
        frame_left      % Container for left camera image
        frame_front     % Container for front camera image
        frame_right     % Container for right camera image
        
        update_pass     % how many times images updated (for saving output)
        resolution      % camera resolution (expected to be same for all)
        
        joint_angle     % the current absolute joint angles of robot arm
        joint_position  % linear translation between revolution joints (constant)
        
    end
    
    methods
        
        % Class Contructor
        function self = robot()
            self.vrep = remApi('remoteApi');
            self.vrep.simxFinish(-1);
            self.clientID = self.vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
            disp("Client ID: " + string(self.clientID));
        end
        
        
        % Sim destructor
        function [] = destructor(self)
            self.vrep.delete();
        end
        
        
        % Initialization function for all motor and camera handles
        function [mot_ret_code, jnt_ret_code, cam_ret_code, prx_ret_code] = initialize_robot(self)

            % Initialize door controls
            door_names = ["_doorJoint#1", "_doorJoint#2", "_doorJoint#3", "_doorJoint#4", "_doorJoint"];
            self.doors = zeros(size(door_names));
            for i=1:length(self.doors)
                [~, self.doors(i)] = self.vrep.simxGetObjectHandle(self.clientID,uint8(char(door_names(i))),self.vrep.simx_opmode_blocking);
            end
            disp("Door handles: " + num2str(self.doors));
            
            
            % Initialize wheels
            wheel_names = ["rollingJoint_fl","rollingJoint_rl", "rollingJoint_rr", "rollingJoint_fr"];
            mot_ret_code = zeros(size(wheel_names));
            self.motors = zeros(size(wheel_names));
            for i=1:length(self.motors)
                [mot_ret_code(i), self.motors(i)] = self.vrep.simxGetObjectHandle(self.clientID,uint8(char(wheel_names(i))),self.vrep.simx_opmode_blocking);
            end
            disp("Wheel motor handles: " + num2str(self.motors));
            disp("Wheel motor handle return codes: " + num2str(mot_ret_code));
            
            
            % Initialize cameras
            camera_names = ["Vision_sensor0", "Vision_sensor", "Vision_sensor1"];
            cam_ret_code = zeros(size(camera_names));
            self.cameras = zeros(size(camera_names));
           
            [cam_ret_code(1), self.cameras(1)] = self.vrep.simxGetObjectHandle(self.clientID,uint8(char(camera_names(1))),self.vrep.simx_opmode_blocking);
            [cam_ret_code(2), self.cameras(2)] = self.vrep.simxGetObjectHandle(self.clientID,uint8(char(camera_names(2))),self.vrep.simx_opmode_blocking);
            [cam_ret_code(3), self.cameras(3)] = self.vrep.simxGetObjectHandle(self.clientID,uint8(char(camera_names(3))),self.vrep.simx_opmode_blocking);
            [~, self.resolution, image1] = self.vrep.simxGetVisionSensorImage2(self.clientID, self.cameras(1), 0, self.vrep.simx_opmode_streaming);
            [~, self.resolution, image2] = self.vrep.simxGetVisionSensorImage2(self.clientID, self.cameras(2), 0, self.vrep.simx_opmode_streaming);
            [~, self.resolution, image3] = self.vrep.simxGetVisionSensorImage2(self.clientID, self.cameras(3), 0, self.vrep.simx_opmode_streaming);
             
            disp("Camera handles: " + num2str(self.cameras));
            disp("Camera handle return codes: " + num2str(cam_ret_code));
            %imshow(image1);

            
            % Initialize proximity
            self.proximity = zeros(1, 2);
            prx_ret_code = zeros(size(self.proximity));
            
            
            % Initialize joints
            joint_names = ["youBotArmJoint0", "youBotArmJoint1", "youBotArmJoint2", "youBotArmJoint3", "youBotArmJoint4", "youBotGripperJoint1", "youBotGripperJoint2"];
            jnt_ret_code = zeros(size(joint_names));
            self.joints = zeros(size(joint_names));
            self.joint_angle = zeros(size(joint_names));

            for i=1:length(self.joints)
                [jnt_ret_code(i), self.joints(i)] = self.vrep.simxGetObjectHandle(self.clientID,uint8(char(joint_names(i))),self.vrep.simx_opmode_blocking);
                [~, self.joint_angle(i)] = self.vrep.simxGetJointPosition(self.clientID, self.joints(i), self.vrep.simx_opmode_blocking);
            end
            
            disp("Arm joint handles: " + num2str(self.joints));
            disp("Arm joint angles: " + num2str(self.joint_angle));
            
        end
                
        % Disconnection from robot
        function [] = terminate_robot(self)
            self.vrep.simxFinish(self.clientID);
        end
        
        
        % Update images from the cameras
        function [ret_code] = update_cameras(self)
            ret_code = zeros(size(self.cameras));
            self.update_pass = self.update_pass + 1;
            
            [ret_code(1), ~ , self.frame_left] = self.vrep.simxGetVisionSensorImage2(self.clientID, self.cameras(1), 0, self.vrep.simx_opmode_buffer);
            [ret_code(2), ~ , self.frame_front] = self.vrep.simxGetVisionSensorImage2(self.clientID, self.cameras(2), 0, self.vrep.simx_opmode_buffer);
            [ret_code(3), ~ , self.frame_right] = self.vrep.simxGetVisionSensorImage2(self.clientID, self.cameras(3), 0, self.vrep.simx_opmode_buffer);

            disp("Left camera: " + num2str(self.frame_left));
            disp("Front camera: " + num2str(self.frame_front));
            disp("Right camera: " + num2str(self.frame_right));
        end
        
        
        % Function to save images to file
        function save_images(self, directory)
            imwrite(self.frame_left,[directory, '/fl_', num2str(self.update_pass), '.jpg']);
            imwrite(self.frame_front,[directory, '/ff_', num2str(self.update_pass), '.jpg']);
            imwrite(self.frame_right,[directory, '/fr_', num2str(self.update_pass), '.jpg']);
        end
        
        
        % Update proximity data (NOT YET IMPLEMENTED)
        function [ret_code] = update_proximity(self)
            ret_code = zeros(size(self.proximity));
        end
        
        
        %For opening/closing doors
        function [ret_code] = control_door(self, door_no, direction)
        if door_no <= 4
            [ret_code] = self.vrep.simxSetJointPosition(self.clientID , self.doors(door_no), -direction*pi/2, self.vrep.simx_opmode_blocking);
            else
            [ret_code] = self.vrep.simxSetJointPosition(self.clientID , self.doors(door_no), direction*2, self.vrep.simx_opmode_blocking);
        end
        end
        
        
        
        % Send velocity commands to wheel motor handlers
        function [ret_code] = set_wheel_velocity(self, wheel_velocity)

            ret_code = zeros(size(self.motors));

            for i=1:length(self.motors)
                [ret_code(i)] = self.vrep.simxSetJointTargetVelocity(self.clientID , self.motors(i), wheel_velocity(i), self.vrep.simx_opmode_blocking);
            end
            disp(ret_code(i));

        end
        
        
        % Send position commands to joint position handlers
        function [ret_code] = set_joint_position(self, joint_target)

            ret_code = zeros(size(self.joints));

            for i=1:5
                [ret_code(i)] = self.vrep.simxSetJointTargetPosition(self.clientID , self.joints(i), joint_target(i), self.vrep.simx_opmode_oneshot);
            end
        end
        
        
        % Update position feedback from joint position handlers
        function [ret_code] = update_joint_angle(self)

            ret_code = zeros(size(self.joints));

            for i=1:length(self.joints)
                [ret_code(i), self.joint_angle(i)] = self.vrep.simxGetJointPosition(self.clientID , self.joints(i), self.vrep.simx_opmode_blocking);
            end

        end
        
        function[] = core_routine(self) % NOT YET IMPLEMENTED, DEMO STRUCTURE

            % demo code for going back and forth 5 times
            for i = 1:5 % change with while loop later
                
                dest_point = input('Which room? ', 's'); %Enter 'Red' to go to red room
                
                keyDest = {'Green','Blue','Red','Purple'};
                valueDest = [1 2 3 4];
                valueReturn = [2 1 4 3];
                
                Destination = containers.Map(keyDest,valueDest);
                Return = containers.Map(keyDest, valueReturn);
                
                %arm routine here to pick up medication
                %self.arm_routine("pick");
            
                %lfr routine here to go to room
                self.lfr_routine(Destination(dest_point));
                
                %arm routine here to put down medication
                %self.arm_routine("place");
                
                %180 turn at the end of forward pass
                flip(self);
        
                %lfr routine here to return to reference point
                self.lfr_routine(Return(dest_point));
                
                %180 turn turn at the end of backward pass
                flip(self);
            end
        end

        function [] = lfr_routine(self, destination)

            %lastSensor = 0;
            baseSpeed = 6;
            Kp = 0.6;
            
            %update the cameras 10 times to avoid frame_right error
            for i=1:10
                self.update_cameras();
            end
            
            j=1;
            
            while true
                
                [sValues, frame_front_binarized] = readSensor(self);
                
                %sliding door control logic
                sensor_door=~frame_front_binarized(1,:);
                sensor_door=all(sensor_door);
                
                if sensor_door  %main door control first time
                    j=0;
                    %self.set_joint_position([0.5, 0.5, 0.5, 0.5, 0.5, 0, 0]);
                    pause (1);
                    %self.set_joint_position([0, 0, 0, 0, 0, 0, 0]);
                    self.control_door(5,1);
                else
                    %error calculation
                    [error, lastSensor] = calculate_error(self, sValues);
                    
                    %control based on error
                    if (error==10)
                        if (lastSensor==1)
                            self.set_wheel_velocity([2,2,-2,-2]); %first two values for right wheels
                        elseif(lastSensor==2)
                            self.set_wheel_velocity([-2,-2,2,2]);
                        elseif(lastSensor==0)
                            self.set_wheel_velocity(zeros(1,4));
                            self.control_door(5,0);
                            break;
                        end
                    else   
                        if(lastSensor==3) %junction check
                            self.set_wheel_velocity([0,0,0,0]);
                            
                            right_wall = check_wall(self, self.frame_right, destination);
                            left_wall = check_wall(self, self.frame_left, destination);
                            front_wall = check_wall(self, self.frame_front, destination);
                            
                            if right_wall
                                %self.set_joint_position([0.5, 0.5, 0.5, 0.5, 0.5, 0, 0]);
                                pause (1);
                                %self.set_joint_position([0, 0, 0, 0, 0, 0, 0]);
                                self.control_door(destination,1);
                                self.control_door(5,0);
                                turn_right(self);
                            elseif left_wall
                                %self.set_joint_position([0.5, 0.5, 0.5, 0.5, 0.5, 0, 0]);
                                pause (1);
                                %self.set_joint_position([0, 0, 0, 0, 0, 0, 0]);
                                self.control_door(destination,1);
                                self.control_door(5,0);
                                turn_left(self);
                            elseif front_wall
                                self.control_door(5,1);
                                if mod(destination,2)==0 
                                    self.control_door(destination-1,0);
                                    turn_left(self);
                                elseif mod(destination,2)==1
                                    self.control_door(destination+1,0);
                                    turn_right(self);
                                end
                            else
                                self.set_wheel_velocity([5,5,5,5]);
                                pause(2);
                            end
                        end
                        
                        delSpeed = Kp*error;
                        self.set_wheel_velocity([baseSpeed-delSpeed, baseSpeed-delSpeed, baseSpeed+delSpeed, baseSpeed+delSpeed]);
                    end
                end
            end
            
        end
        
    end
end