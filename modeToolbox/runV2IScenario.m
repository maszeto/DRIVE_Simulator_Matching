function [vehicles,pedestrians, outputMap, xyLinks] = ...
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
    
    [losIDs, nLosIDs, losNlosStatus, distanceTiles, sortedIndexes] = losNLosV2V(outputMap,BS,BS.rats{2});
    
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
        fprintf('Preprocessing for t= %f\n',i)
        
        %vehicleTimestep is all the vehicles positions at the current
        %timestep (ID, X, Y, timestep, id)
        [ vehicleTimestep, pedestrianTimestep ] = getVehiclesAndPedestrians(sumo,vehicleIDs,pedestrianIDs,timeStep);
        vehicles = [ vehicles ; vehicleTimestep ];
        
        [tmp,~] = size(vehicleTimestep);

        if tmp>1
            %Then check which vehicles have LoS with each other
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
    %matchingSim = matchingSim.setVehicleIDsByTime(viewedVehicles);
    matchingSim.vehicleIDsByTime = vehiclesIDsAtTime;
    matchingSim = matchingSim.addRSUs(potentialPos);
    
    matchingSim.outputMap = outputMap;
    
    matchingSim = matchingSim.calculateTileLOS(BS,  BS.rats{2});
    
    matchingSim = matchingSim.createRSUConnectionScheduleNearest();
    outputMap.RSUs = matchingSim.rsuList;
    matchingSim = matchingSim.createXYLinksV2I();
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

