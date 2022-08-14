function [vehicles,pedestrians] = ...
            runSUMO(sumo,map,BS,outputMap,distanceTiles,sortedIndexes)
%RUNMATCHING This is the main function for the matching scenario. 
%
%  Input  :
%     sumo          : Structure containing all the SUMO settings (maximum
%                     number of vehicles, start time, end time, etc.)
%     map           : Structure containing all map settings (filename, path,
%                     simplification tolerance to be used, etc).
%     outputMap     : The map structure containing all the map information.
%     distanceTiles : The distanceTiles of each tile from another given tile.
%     sortedIndexes : The sorted indexes for the nearby tiles, given
%                     from the closest to the furthest one.
%
%  Output :
%     vehicles      : Array containing all the information about the
%                     vehicles for the entire simulation time.
%
% Copyright (c) 2022, Matthew Szeto
% email: maszeto@asu.edu


    tic 
    
    % Progress to the first simulation step
    traci.simulationStep;
    timeStep = traci.simulation.getTime;

    % Create the vehicle and pedestrian arrays
    vehicles = [];
    pedestrians = [];
    vehiclesStruct = {};
    pedestrianStruct = {};

    
    % Start iterating for all the timesteps
    for i = 1:sumo.endTime
        vehicleIDs = traci.vehicle.getIDList();
        pedestrianIDs = traci.person.getIDList();
        timeStep = traci.simulation.getTime;
        fprintf('The timestep is: %f\n',timeStep)
        
        [ vehicleTimestep, pedestrianTimestep ] = getVehiclesAndPedestrians(sumo,vehicleIDs,pedestrianIDs,timeStep);
        vehicles = [ vehicles ; vehicleTimestep ];
        
        % Progress to the timestep
        traci.simulationStep;
        
    end
    
    %Parse mobility files with the vehicle information
    [ vehiclesStruct, pedestriansStruct ] = parseMobility(sumo, vehicles, pedestrians);
    

end

