% Load the simulation settings (for further explanation see simSettings.m)
simSettings;

% Add the different modules of the simulator to MATLAB path
setupSimulator;


startTraci(sumo);

vehicles = [];

 for i = 1:sumo.endTime
        vehicleIDs = traci.vehicle.getIDList();
        
        for j = 1:length(vehicleIDs)
            traci.vehicle.setLaneChangeMode(vehicleIDs{j}, 0b001000000000);
            traci.vehicle.setSpeed(vehicleIDs{j}, 29);
        end
        
        pedestrianIDs = traci.person.getIDList();
        timeStep = traci.simulation.getTime;
        fprintf('Preprocessing for t= %f\n',i)
        
        %vehicleTimestep is all the vehicles positions at the current
        %timestep (ID, X, Y, timestep, id)
        [ vehicleTimestep, pedestrianTimestep ] = getVehiclesAndPedestrians(sumo,vehicleIDs,pedestrianIDs,timeStep);
        vehicles = [ vehicles ; vehicleTimestep ];
        
        [tmp,~] = size(vehicleTimestep);

        if tmp>1
            vehiclesIDsAtTime{timeStep} = vehicleTimestep(:,1);
            
        end
  
        % Progress to the timestep
        traci.simulationStep;
end



fprintf("");
traci.close();