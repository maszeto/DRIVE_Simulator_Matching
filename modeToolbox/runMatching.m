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
    global MATCHING

    tic 
    
    % Progress to the first simulation step
    traci.simulationStep;
    timeStep = traci.simulation.getTime;

    % Create the vehicle and pedestrian arrays
    vehicles = [];
    pedestrians = [];
    vehiclesStruct = {};
    pedestrianStruct = {};

    
    % Start iterating for all the timesteps, not the most effecient but
    % good enough for now
    for i = 1:sumo.endTime
        vehicleIDs = traci.vehicle.getIDList();
        pedestrianIDs = traci.person.getIDList();
        timeStep = traci.simulation.getTime;
        fprintf('The timestep is: %f\n',timeStep)
        
        %vehicleTimestep is all the vehicles positions at the current
        %timestep (ID, X, Y, timestep, id)
        [ vehicleTimestep, pedestrianTimestep ] = getVehiclesAndPedestrians(sumo,vehicleIDs,pedestrianIDs,timeStep);
        vehicles = [ vehicles ; vehicleTimestep ];
        
        [tmp,~] = size(vehicleTimestep);

        if tmp>1
            %distanceVehicle is a matrix representing distance of each vehicle
            %to all other vehicles in the map (vehicle id is row and column
            %index, we want the vehicle id to correlate to row and column,
            %so we need to sort vehicleTimestep first
            vehicleTimestep = sortrows(vehicleTimestep);
            distanceVehicle = pdist2(vehicleTimestep(:,2:3), vehicleTimestep(:,2:3), 'euclidean');
            
            %Check which vehicles are within LiDAR range
            vehiclesInRange = distanceVehicle<=MATCHING.lidarRad;%Checking if it is in range, if it is then a link is created
            
            %Then check LoS
            
        
        end
        % Get tile each vehicle is closest to (this is only for the current
        % timestep) 
        %DistanceVehicleTile - Distance of vehicle from tile (vehicle id is index) 
        %idxVehicleTile - Tile vehicle is closest to (vehicle id is index)
        [distanceVehicleTile,idxVehicleTile,distancePedestrianTile,idxPedestrianTile] = ...
            nearbyTile(outputMap,vehicleTimestep,pedestrianTimestep);
        

        
        
        % Progress to the timestep
        traci.simulationStep;
        
    end
    
    %Parse mobility files with the vehicle information, now we have vehicle
    %coordinate and type at each timestep. 
    [ vehiclesStruct, pedestriansStruct ] = parseMobility(sumo, vehicles, pedestrians);
    

end

