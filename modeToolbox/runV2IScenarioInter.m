function [vehicles,pedestrians, outputMap, xyLinks, matchingSim] = ...
            runV2I(sumo,map,BS,outputMap,potentialPos,chosenRSUpos, tilesCovered,distanceTiles,...
            sortedIndexes,losNlosStatus,rssAll,distanceBuildings, sortedIndexesBuildings, rssAllBuildings)
%RUNSUMO This is the main function of the SUMO mode. It initialises the
% highestRSS, the servingBSs and the tiles covered variables. Later, it
% iterates for all the timesteps and calculates the users per area on each
% given timestep. It also updates the policies of certain given BSs at
% specific times and recalculates the necessary variables.
%
%  Input  :
%     sumo             : Structure containing all the SUMO settings (maximum
%                        number of vehicles, start time, end time, etc.)
%     map              : Structure containing all map settings (filename, path,
%                        simplification tolerance to be used, etc).
%     outputMap        : The map structure containing all the map information.
%     potentialPos     : All the potential positions for all the different RATs.
%     distanceTiles    : The distanceTiles of each tile from a given BS.
%     sortedIndexes    : The sorted indexes for the tile close to BS, given
%                        from the closest to the furthest one.
%     losNlosStatus    : The classification of each tile (LOS/NLOS) for a
%                        given BS - 0 is NLOS, 1 is LOS
%     rssAll           : The received signal strength for the all given tiles.
%     distanceBuilding : The distance of each building from a given BS.
%     sortedIndexesBuildings : The sorted indexes for the building close to BS, given
%                              from the closest to the furthest one.
%     rssAllBuildings  : The received signal strength for the all given buildings.
%
%  Output :
%     vehicles         : Array containing all the information about the
%                        vehicles for the entire simulation time.
%     pedestrians      : Array containing all the information about the
%                        pedestrians for the entire simulation time.
%
% Copyright (c) 2019-2020, Ioannis Mavromatis
% email: ioan.mavromatis@bristol.ac.uk
% email: ioannis.mavromatis@toshiba-bril.com

    global VERBOSELEVEL
    tic 
    
    % For debug purposes only
    if VERBOSELEVEL >= 1
        ratName = BS.rats{2};
        [ servingBSId.(ratName),highestRSS.(ratName),highestRSSPlot.(ratName),losNlos.(ratName),tilesCoveredIDs.(ratName), tilesNum.(ratName) ] = ...
                      highestRSSValues(chosenRSUpos.(ratName),outputMap,sortedIndexes{2}, rssAll{2},losNlosStatus{2});
        figure
        heatmapPrint(outputMap,map,highestRSSPlot.(ratName),chosenRSUpos.(ratName),potentialPos.(ratName).pos,tilesCoveredIDs.(ratName)) 
        
    end
    
    % Progress to the first simulation step
    traci.simulationStep;
    timeStep = traci.simulation.getTime;
    
    % Initialise the user density in buildings
    [randomValues,coordinates,initialX,initialY,interpX,interpY] = initialiseUsersPerBuilding(outputMap,'sumo',map,sumo);
    outputMap = usersPerBuilding(outputMap,timeStep,randomValues,coordinates,initialX,initialY,interpX,interpY,map);
    
    % Create the vehicle and pedestrian arrays
    vehicles = [];
    pedestrians = [];
    close all;
    
    matchingSim = MatchingSim("V2I Test", "V2I");
    
     fprintf("Preprocessing mobility traces ...\n");
    for i = 1:sumo.endTime
        vehicleIDs = traci.vehicle.getIDList();
        pedestrianIDs = traci.person.getIDList();
        timeStep = traci.simulation.getTime;
        
        %Force blockages by not allowing lane changes
%         for j = 1:length(vehicleIDs)
%             traci.vehicle.setLaneChangeMode(vehicleIDs{j}, 0b001000000000);
%             traci.vehicle.setSpeed(vehicleIDs{j}, 29);
%         end
        
        fprintf('Preprocessing for t= %f\n',timeStep)
        
        %vehicleTimestep is all the vehicles positions at the current
        %timestep (ID, X, Y, timestep, id)
        [ vehicleTimestep, ~ ] = getVehiclesAndPedestrians(sumo,vehicleIDs,pedestrianIDs,timeStep);
        vehicles = [ vehicles ; vehicleTimestep];
        
        [tmp,~] = size(vehicleTimestep);

        if tmp>1
            viewedVehicles{timeStep} = [];
            vehiclesIDsAtTime{timeStep} = vehicleTimestep(:,1);
 
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
    
    matchingSim = matchingSim.addVehiclesByStruct(vehiclesStruct);
    matchingSim.vehicleIDsByTime = vehiclesIDsAtTime;
    matchingSim = matchingSim.addRSUs(potentialPos, chosenRSUpos);
    matchingSim.potentialPos = potentialPos;
    matchingSim.sortedIndexes = sortedIndexes;
    matchingSim.rssAll = rssAll;
    matchingSim = matchingSim.fixVehiclesByTime();
    
    matchingSim.outputMap = outputMap;
    matchingSim = matchingSim.setBuildingLines();
    matchingSim = matchingSim.addBuildingLinesToRSUs();
    save('matchingSim');
    %matchingSim = matchingSim.calculateTileLOS(BS,  BS.rats{2});
    
%   Get blockages
    matchingSim = matchingSim.resetStats();
    %matchingSim = matchingSim.createXYLinksV2I();
    %matchingSim.viewSimulation();
    matchingSim = matchingSim.createRSUConnectionScheduleGreedy();
    %Running the baseline scenario
    for i = 1:sumo.endTime
        matchingSim = matchingSim.runGreedyScenario(i, .32);
    end
    
    greedyHandovers = matchingSim.getHandoversByVehicleIndex();
    greedyData = matchingSim.getDataTransmittedByVehicle(.32);
    [greedyPlanFail, greedyLinkBreak] = matchingSim.getStats();
    matchingSim = matchingSim.createXYLinksV2I();
    %matchingSim.viewSimulation();
    matchingSim = matchingSim.clearRSUConnections(sumo.endTime);
    matchingSim = matchingSim.clearBlockageTimes();
    matchingSim = matchingSim.resetStats();
    %Running SMART Scenario
    for i = 1:sumo.endTime
        matchingSim = matchingSim.runSMARTScenario(i, -30, .32);
    end
    matchingSim = matchingSim.createXYLinksV2I();
%     matchingSim.viewSimulation();
    smartHandovers = matchingSim.getHandoversByVehicleIndex();
    smartData = matchingSim.getDataTransmittedByVehicle(.32);
    [smartPlanFail, smartLinkBreak] = matchingSim.getStats();
    
    %Running for our method, reset connectedVehicles
    matchingSim = matchingSim.clearRSUConnections(sumo.endTime);
    matchingSim = matchingSim.clearBlockageTimes();
    matchingSim = matchingSim.createRSUConnectionScheduleNearest(matchingSim.buildingLines);
    matchingSim = matchingSim.resetStats();
    for i = 1:sumo.endTime
        
        matchingSim = matchingSim.connectVehiclesToRSU(i,.1984,.32);
        matchingSim = matchingSim.updateVehiclesSchedules(i, 10, 200);
        
    end
    
    goodHandovers = matchingSim.getHandoversByVehicleIndex();
    goodData = matchingSim.getDataTransmittedByVehicle();
    [goodPlanFail, goodLinkBreak] = matchingSim.getStats();
    outputMap.RSUs = matchingSim.rsuList;
    matchingSim = matchingSim.createXYLinksV2I();
    matchingSim.viewSimulation();
    xyLinks = matchingSim.xyLinks;

    
    verbose('The entire SUMO mode took: %f seconds.', toc); 
end

