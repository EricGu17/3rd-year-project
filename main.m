clear
clc

simulation = robot();

if (simulation.clientID > -1)
    disp('Connection to robot successful');
    [~, ~, ~, ~] = simulation.initialize_robot();

    simulation.core_routine();

    simulation.terminate_robot();
    
else
    disp('Failed connecting to remote API server');
end

disp("Simulation ended");

simulation.destructor();

disp('Connection terminated');
