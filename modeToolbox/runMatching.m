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
    
    %Holds vehicles in view at every timestep
    viewedVehicles = {};
    
    % Start iterating for all the timesteps, not the most effecient but
    % good enough for now
    fprintf("Preprocessing mobility traces ...\n");
    for i = 1:sumo.endTime
        vehicleIDs = traci.vehicle.getIDList();
        pedestrianIDs = traci.person.getIDList();
        timeStep = traci.simulation.getTime;
        
        %vehicleTimestep is all the vehicles positions at the current
        %timestep (ID, X, Y, timestep, id)
        [ vehicleTimestep, pedestrianTimestep ] = getVehiclesAndPedestrians(sumo,vehicleIDs,pedestrianIDs,timeStep);
        vehicles = [ vehicles ; vehicleTimestep ];
        
        [tmp,~] = size(vehicleTimestep);

        if tmp>1
            %Then check which vehicles have LoS with each other
            viewedVehicles{timeStep} = getViewedVehicles(sumo,map, outputMap, vehicleTimestep);

 
        end
  
        % Progress to the timestep
        traci.simulationStep;
        
    end
    
    %Parse mobility files with the vehicle information, now we have vehicle
    %coordinate and type at each timestep. 
    [ vehiclesStruct, ~ ] = parseMobility(sumo, vehicles, pedestrians);
    
    %Parse vehicle viewedVehicles to add information to vehiclesStruct as
    %to which vehicles are in view
    [vehiclesStruct] = addViewedVehicles(viewedVehicles, vehiclesStruct);
    
    %Run Matching
    fprintf("Running Stable Fixtures Matching ...\n");
    matches = {};
    plCap = 2 * ones(length(vehiclesStruct.vehNode),1);
    for i = 1:sumo.endTime
        fprintf('The timestep is: %f\n',i)
        if ~isempty(viewedVehicles{i})
            utilityFunc = @u_nearest;
            [a_iElements, S] = stableFixtures(vehiclesStruct, viewedVehicles, utilityFunc, plCap, i);
            matches{i} = a_iElements;
        end
    end
    
    
    
    
    %iterate through timesteps
    %   runMatching()
    %   if (beamselsim)
    %       runBeamSel(vehicles, buildings)
    
    
end

