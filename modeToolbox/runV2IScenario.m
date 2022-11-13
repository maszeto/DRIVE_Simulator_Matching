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
        for j = 1:length(vehicleIDs)
            traci.vehicle.setLaneChangeMode(vehicleIDs{j}, 0b001000000000);
            traci.vehicle.setSpeed(vehicleIDs{j}, 29);
        end
        
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
    matchingSim = matchingSim.addRSUs(potentialPos);
    
    matchingSim.outputMap = outputMap;
    save('matchingSim');
    %matchingSim = matchingSim.calculateTileLOS(BS,  BS.rats{2});
    
%     matchingSim = matchingSim.setBuildingLines();
%     matchingSim = matchingSim.createRSUConnectionScheduleNearest(matchingSim.buildingLines);
    
%   Highway scenario, so no initial blockages
    matchingSim = matchingSim.createRSUConnectionScheduleNearest([]);
    
    %Running the baseline scenario
    for i = 1:sumo.endTime
        fprintf('RSU Selection for t= %f using greedy method\n',i)
        for j = 1:length(matchingSim.vehicleIDsByTime{i})
            curVeh = matchingSim.getVehicleByID(matchingSim.vehicleIDsByTime{i}(j));
            timeIndex = curVeh.getTimeIndex(i);
            antennaPos = curVeh.getAntennaPosAtTime(i);
            xPos = antennaPos(1);
            yPos = antennaPos(2);
            blockages = matchingSim.getPotentialBlockages(i, curVeh.vehicleId, 75);
            %nearestRSUIndex = curVeh.findNearestRSUInLOS(xPos, yPos, matchingSim.rsuList, blockages);
            selectedRSUIndex = curVeh.selectRSUAtTime(i, matchingSim.rsuList, blockages); 

            curVeh.RSUs(timeIndex) = selectedRSUIndex;
            matchingSim.vehiclesByIndex(curVeh.vehicleIndex) = curVeh;
            if selectedRSUIndex ~= -1
                matchingSim = matchingSim.updateRSUConnectionsAtTime(i,selectedRSUIndex, curVeh.vehicleIndex);
            end
        end
    end
    
    greedyHandovers = matchingSim.getHandoversByVehicleIndex()
    
    %Running for our method, reset connectedVehicles
    matchingSim = matchingSim.clearRSUConnections(sumo.endTime);
    matchingSim = matchingSim.createRSUConnectionScheduleGreedy();
    
    for i = 1:sumo.endTime
        fprintf('RSU Selection for t= %f using predictive method\n',i)
        
        %For each vehicle in this timestep
        %connect to RSU from schedule 
        %if non los with RSU, then connect to nearest in LOS
        for j = 1:length(matchingSim.vehicleIDsByTime{i})
            curVeh = matchingSim.getVehicleByID(matchingSim.vehicleIDsByTime{i}(j));
            timeIndex = curVeh.getTimeIndex(i);
            selectedRSUIndex = curVeh.RSUPlan(timeIndex);
            antennaPos = curVeh.getAntennaPosAtTime(i);
            xPos = antennaPos(1);
            yPos = antennaPos(2);
            blockages = matchingSim.getPotentialBlockages(i, curVeh.vehicleId, 75);

            if (matchingSim.connectionFailed(i, curVeh.vehicleId, selectedRSUIndex))
                selectedRSUIndex = curVeh.selectRSUAtTime(i, matchingSim.rsuList, blockages);
            end
            matchingSim = matchingSim.updateRSUConnectionsAtTime(i,selectedRSUIndex, curVeh.vehicleIndex);
            curVeh.RSUs(timeIndex) = selectedRSUIndex;
            matchingSim.vehiclesByIndex(curVeh.vehicleIndex) = curVeh;

        end
        
        %for each RSU, get list of currently connected vehicles
        %check for blockages with other connected vehicles
        %update that vehicles schedule, assume RSUs are connected, so they
        %make decision together
        
        for j = 1:length(matchingSim.rsuList)            
            if(mod(j,2)==0)
                continue;
            end
            
            % For all odd RSUs (We assume RSUs work in pairs of 2)
            curRSU1 = matchingSim.rsuList(j);
            curRSU2 = matchingSim.rsuList(j+1);
            connectedVehicles = [curRSU1.connectedVehicles{i}, ...
                                    curRSU2.connectedVehicles{i}];
            if isempty(connectedVehicles)
                continue;
            end
            
            otherVehicles = SimVehicle(1,length(connectedVehicles));
            
            for k = 1:length(connectedVehicles)
                otherVehicles(k) = matchingSim.vehiclesByIndex(connectedVehicles(k));  
            end
            
            for k = 1:length(connectedVehicles)
                curVeh = matchingSim.vehiclesByIndex(connectedVehicles(k));
                matchingSim.vehiclesByIndex(connectedVehicles(k)) = ...
                    curRSU1.updateSimVehSchedule(i, curVeh, otherVehicles, matchingSim.rsuList, 10);
            end
            
        end
        
    end
    
    goodHandovers = matchingSim.getHandoversByVehicleIndex()
    
    outputMap.RSUs = matchingSim.rsuList;
    matchingSim = matchingSim.createXYLinksV2I();
    matchingSim.viewSimulation();
    xyLinks = matchingSim.xyLinks;
    
%     % Start iterating for all the timesteps
%     for i = 1:sumo.endTime
%         vehicleIDs = traci.vehicle.getIDList();
%         pedestrianIDs = traci.person.getIDList();
%         timeStep = traci.simulation.getTime;
%         fprintf('The timestep is: %f\n',timeStep)
%         
%         [ vehicleTimestep, pedestrianTimestep ] = getVehiclesAndPedestrians(sumo,vehicleIDs,pedestrianIDs,timeStep);
%         vehicles = [ vehicles ; vehicleTimestep ];
%         pedestrians = [ pedestrians ; pedestrianTimestep ];
%         
%         [distanceVehicleArea,idxVehicleArea,distancePedestrianArea,idxPedestrianArea,usersPerArea] = ...
%             usersPerAreaCalculation(outputMap,vehicleTimestep,pedestrianTimestep);
% 
%         [distanceVehicleTile,idxVehicleTile,distancePedestrianTile,idxPedestrianTile] = ...
%             nearbyTile(outputMap,vehicleTimestep,pedestrianTimestep);
%         
%        
%         rssHighest = highestRSS.(BS.rats{2})(idxVehicleTile);
%         bsServing = servingBSId.(BS.rats{2})(idxVehicleTile);
%         losNlosLink = losNlos.(BS.rats{2})(idxVehicleTile);
% 
%         dataRateTmp = [];
%         for l = 1:length(rssHighest)
%             if bsServing(l)>=1
%                 if losNlosLink(l) == 1
%                     idx = potentialPos.(BS.rats{2}).linkBudget(bsServing(l)).signalReceivedLos == rssHighest(l);
%                     dataRateTmp(l) = potentialPos.(BS.rats{2}).linkBudget(bsServing(l)).dataRateLos(idx);
%                 else
%                     idx = potentialPos.(BS.rats{2}).linkBudget(bsServing(l)).signalReceivedNLos == rssHighest(l);
%                     dataRateTmp(l) = potentialPos.(BS.rats{2}).linkBudget(bsServing(l)).dataRateNLos(idx);
%                 end
%             else
%                 dataRateTmp(l) = 0;
%             end
%         end
%         dataRate{2}(timeStep) = mean(dataRateTmp);        
% 
%         
%         % Progress to the timestep
%         traci.simulationStep;
%         
%     end
    

%     fprintf('The average datarate for the mmWave communication plane is: %f Mbits/s\n',2,mean(dataRate{2})/10^6);
    
    
    verbose('The entire SUMO mode took: %f seconds.', toc); 
end

