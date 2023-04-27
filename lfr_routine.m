function [] = lfr_routine(self, destination)

%lastSensor = 0;
baseSpeed = 6;
Kp = 0.6;

%update the cameras 10 times to avoid frame_right error
for i=1:10
    self.update_cameras();
end

%j=1;

while true
    [sValues, frame_front_binarized] = readSensor(self);
    %sValues
    
    %sliding door control logic
    sensor_door=~frame_front_binarized(1,:);
    sensor_door=all(sensor_door);
    
    if sensor_door  %main door control first time
        %j=0;
        %self.set_joint_position([0.1, 0.1, 0.1, 0.1, 0.1, 0, 0]);
        %pause (1);
        %self.set_joint_position([0, 0, 0, 0, 0, 0, 0]);
        self.control_door(5,1);
    else
        %error calculation
        [error, lastSensor] = calculate_error(self, sValues);
        
        %control based on error
        if (error==10)
            if (lastSensor==1)
                self.set_wheel_velocity([1,1,-1,-1]); %first two values for right wheels
            elseif(lastSensor==2)
                self.set_wheel_velocity([-1,-1,1,1]);
            elseif(lastSensor==0)
                self.set_wheel_velocity(zeros(1,4));
                self.control_door(5,0);
                break;
            end
        else   
            if(lastSensor==1)
                self.set_wheel_velocity([0,0,0,0]);
                turn_left(self);
                self.set_wheel_velocity([5,5,5,5]);
            end

            if(lastSensor==3) %junction check
                self.set_wheel_velocity([0,0,0,0]);

                right_wall = check_wall(self, self.frame_right, destination);
                left_wall = check_wall(self, self.frame_left, destination);
                front_wall = check_wall(self, self.frame_front, destination);
                
                if right_wall
                    %self.set_joint_position([0.1, 0.1, 0.1, 0.1, 0.1, 0, 0]);
                    pause (1);
                    %self.set_joint_position([0, 0, 0, 0, 0, 0, 0]);
                    self.control_door(destination,1);
                    self.control_door(5,0);
                    turn_right(self);
                elseif left_wall
                    %self.set_joint_position([0.1, 0.1, 0.1, 0.1, 0.1, 0, 0]);
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