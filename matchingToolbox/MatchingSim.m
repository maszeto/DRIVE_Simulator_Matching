classdef MatchingSim
    %MATCHINGSIM Manages objects used in matching simulation
    %   Detailed explanation goes here
    
    properties
        name                % Name of the simulation
        numVehicles         % Number of vehicles in the simulation
        vehicleIDList       % List of vehicle IDs used in simulation
        vehicleIDsByTime    % List of vehicle IDs at each timestep
        matchingType        % Type of matching algorithm
        utilityFunction     % Utility function used in matching algo
        vehiclesByIndex     % Get vehicles by their index (sorted by ID)
        xyLinks             % List of coordinate pairs forming links
        timesteps           % Timesteps of simulation
        mode                % Matching with 'V2V' or 'V2I'
        rsuList             % List of RSU objects in simulation
        outputMap           % Map struct containing map data like building information
        buildingLines       % line segments creating buildings
        losIDs              % Which tiles have LOS, index is tile ID
        nLosIDs             % Which tiles don't have LOS, index is tile ID
        curTimestep         % For graphing
        sortedIndexes
        potentialPos
        rssAll
        
    end
    
    methods
        
        function obj = MatchingSim(name, mode)
            %MATCHINGSIM Construct an instance of this class
            %   Detailed explanation goes here
            obj.name = name;
            obj.mode = mode;
        end
        
        function obj = initSimVehiclesList(obj, numVehicles)
            %INITSIMVEHICLES Creates list of vehicle Objects in simulation
            %   Detailed explanation goes here
            
            obj.numVehicles = numVehicles;
            simVehiclesList(obj.numVehicles) = SimVehicle(obj.numVehicles);
    
            for i = 1:obj.numVehicles
                simVehiclesList(i) = SimVehicle();
            end
            
            obj.vehiclesByIndex = simVehiclesList;
        end
        
        function obj = addVehiclesByStruct(obj, vehiclesStruct)
            obj = obj.initSimVehiclesList(length(vehiclesStruct.vehNode));
            obj.timesteps = 1:vehiclesStruct.simTime;
            obj.vehicleIDList = [vehiclesStruct.vehNode.id];
            for i = 1:obj.numVehicles
                obj.vehiclesByIndex(i) = obj.vehiclesByIndex(i).initFromStruct(i, vehiclesStruct);
            end
            
        end
        
        function obj = addRSUs(obj, potentialPos, chosenRSUpos)
            global SIMULATOR
            
            if(SIMULATOR.bsPlacement == "userdef")
                rsuList(length(potentialPos.mmWaves.pos)) = RSU(length(potentialPos.mmWaves.pos));

                for i = 1:length(potentialPos.mmWaves.pos)
                    rsuList(i) = rsuList(i).initRSU(i, i, potentialPos.mmWaves.pos(i,2), ...
                        potentialPos.mmWaves.pos(i,1), potentialPos.mmWaves.pos(i,3), length(obj.timesteps));
                end
                obj.rsuList = rsuList;
            else
                rsuList(length(chosenRSUpos.mmWaves)) = RSU(length(chosenRSUpos.mmWaves));

                for i = 1:length(chosenRSUpos.mmWaves)
                    chosenIndex = chosenRSUpos.mmWaves(i);
                    rsuList(i) = rsuList(i).initRSU(i, i, potentialPos.mmWaves.pos(chosenIndex,2), ...
                        potentialPos.mmWaves.pos(chosenIndex,1), potentialPos.mmWaves.pos(chosenIndex,3), length(obj.timesteps));
                end
                obj.rsuList = rsuList;
            end

        end
        
        function obj = calculateTileLOS(obj, BS, ratName)
            if isempty(obj.outputMap)
                fprintf("ERROR: Set output map before calculating tile LOS\n")
            end
            [losIDs, nLosIDs, ~, ~, ~] = losNLosV2V(obj.outputMap,BS,ratName);
            obj.losIDs = losIDs;
            obj.nLosIDs = nLosIDs;   
        end
            
        function tileID = getVehicleTile(obj, vehicleID, timeStep)
            % Get the tileID of the current tile the vehicle is closest to
            vehiclePos = [obj.getVehicleByID(vehicleID).getXPosAtTime(timeStep) ...
                            obj.getVehicleByID(vehicleID).getYPosAtTime(timeStep)];
            vehiclePos = obj.getVehicleByID(vehicleID).getAntennaPosAtTime(timeStep);
            tileCenters = obj.outputMap.inCentresTile(:,1:2);
            nearestTileIndex = dsearchn(tileCenters, vehiclePos);
            tileID = nearestTileIndex;
        end
        
        function datarate = calcDataRateAtTime(obj, vehicleID, rsuID, timeStep)
            
            if rsuID == -1
                datarate = 0;
                return
            end
            
            tile = obj.getVehicleTile(vehicleID, timeStep);
            tileIndex = find(obj.sortedIndexes{2}{rsuID} == tile);
            rss = obj.rssAll{2}{rsuID}(tileIndex);
            if isempty(rss)
                datarate=0;
                return
            end
            datarateIndex = find(obj.potentialPos.('mmWaves').linkBudget(rsuID).signalReceivedLos == rss);
            
            if isempty(datarateIndex)
                fprintf('Vehicle %d does not have LOS with RSU %d at time %d\n',vehicleID, rsuID, timeStep);
                datarate = 0;
                return
            end
            datarate = obj.potentialPos.('mmWaves').linkBudget(rsuID).dataRateLos(datarateIndex);
            
        end
        
        function datarate = calcDataRateAtTimeNLOS(obj, vehicleID, rsuID, timeStep)
            datarate = 0;
            return;
            
%             if rsuID == -1
%                 datarate = 0;
%                 return
%             end
%             
%             tile = obj.getVehicleTile(vehicleID, timeStep);
%             tileIndex = find(obj.sortedIndexes{2}{rsuID} == tile);
%             rss = obj.rssAll{2}{rsuID}(tileIndex);
%             if isempty(rss)
%                 datarate=0;
%                 return
%             end
%             datarateIndex = find(obj.potentialPos.('mmWaves').linkBudget(rsuID).signalReceivedLos == rss);
%             
%             if isempty(datarateIndex)
%                 fprintf('Vehicle %d does not have LOS with RSU %d at time %d\n',vehicleID, rsuID, timeStep);
%                 datarate = 0;
%                 return
%             end
%             datarate = obj.potentialPos.('mmWaves').linkBudget(rsuID).dataRateNLos(datarateIndex);
            
        end
        
        function rss = getRSSAtTime(obj, vehicleID, rsuID, timeStep)
            if rsuID == -1
                rss = -999999;
                return
            end
            
            tile = obj.getVehicleTile(vehicleID, timeStep);
            tileIndex = find(obj.sortedIndexes{2}{rsuID} == tile);
            rss = obj.rssAll{2}{rsuID}(tileIndex);
            if isempty(rss)
                rss=-999999;
                return
            end
        end
        
        function obj = adjustDataRate(obj, handoverPenalty, blockedPenalty)
            
            for i = 1:length(obj.vehiclesByIndex)
                curVeh = obj.vehiclesByIndex(i);
                
                for j = 2:length(curVeh.times)
                    if(curVeh.RSUs(j) ~= curVeh.RSUs(j-1))
                        %handover has occured 
                        curVeh.adjustedDataRate(j) = curVeh.datarate(j)*(1-handoverPenalty);
                    end
                    
                end
                obj.vehiclesByIndex(i) = curVeh;
            end
            
        end
        
        function dataTransmittedByVehicle = getDataTransmittedByVehicle(obj, handoverPenalty)
            obj = obj.adjustDataRate(handoverPenalty, 0);
            dataTransmittedByVehicle = zeros(1, length(obj.vehiclesByIndex));
            
            for i = 1:length(obj.vehiclesByIndex)
                curVeh = obj.vehiclesByIndex(i);
                
                dataTransmittedByVehicle(i) = sum(curVeh.adjustedDataRate);
                
            end

        end
        
        
        function nearestRSUID = getNearestRSU(obj, vehicleID, timeStep)
            curVeh = obj.getVehicleByID(vehicleID);
            xPos = curVeh.getXPosAtTime(timeStep);
            yPos = curVeh.getYPosAtTime(timeStep);

            nearestRSUIndex = curVeh.findNearestRSUInLOS(xPos, yPos, matchingSim.rsuList, []);
        end
        
        function hasLOS = hasLOS(obj, x1, y1, x2, y2, timeStep, fastCalculation)
            %Check if there is LOS between two points at a given timestep
            %Should be two methods, one for speed one for raw calculation 
            %TODO: Add vehicles 
            if(fastCalculation == 0)
                intersectCnt = segments_intersect_test([x1,y1,x2,y2], obj.buildingLines);%number of intersections
                hasLOS = (intersectCnt == 0);
            else
                if(isempty(obj.losIDs) || isempty(obj.nLosIDs))
                    fprintf("ERROR: Call calculateTileLOS before hasLOS\n")
                end
                pos1 = [x1 y1];
                pos2 = [x2 y2];
                tileCenters = obj.outputMap.inCentresTile(:,1:2);
                nearestTileIndex1 = dsearchn(tileCenters, pos1);
                nearestTileIndex2 = dsearchn(tileCenters, pos2);
                
                hasLOS = ~isempty(find(obj.losIDs{nearestTileIndex1} == nearestTileIndex2));
                
            end

        end
        
        function obj = createRSUConnectionScheduleNearest(obj, blockages)
            %MATCHINGSIM Create the matching schedule for each vehicle
            %object
            %   Detailed explanation goes here
            if isempty(obj.outputMap)
                fprintf("ERROR: Set output map before creating RSU connection schedule\n")
            end
            
            for i = 1:obj.numVehicles
                obj.vehiclesByIndex(i) = obj.vehiclesByIndex(i).createRSUConnectionScheduleNearest(obj.rsuList, blockages);
            end
        end
        
        function obj = createRSUConnectionScheduleGreedy(obj)
            %MATCHINGSIM Create the matching schedule for each vehicle
            %object
            %   Detailed explanation goes here
            
            for i = 1:obj.numVehicles
                obj.vehiclesByIndex(i) = obj.vehiclesByIndex(i).createRSUConnectionScheduleGreedy(obj.rsuList);
            end
        end
        
        function obj = setBuildingLines(obj)
            
            if isempty(obj.outputMap)
                fprintf("ERROR: Set output map before setting building lines\n")
            end
            buildingsToTest = [];
            buildingIds = obj.outputMap.buildingIncentre(:,1);
            
            for i = 1:length(buildingIds)
                building = obj.outputMap.buildings(find(ismember(obj.outputMap.buildings(:,1), buildingIds(i), 'rows')), [2,3]); 
                %building is of the form YX, so we convert to XY for use in segment
                %intersection
                buildingsToTest = [ buildingsToTest ;
                    building(1:end-1,2) ...
                    building(1:end-1,1) ...
                    building(2:end,2)   ...
                    building(2:end,1) ];
            end

            obj.buildingLines = buildingsToTest;
        end

        function obj = createXYLinksV2I(obj)
            %MATCHINGSIM get xyLinks
            %object
            %   Detailed explanation goes here
            for timeStep=1:length(obj.timesteps)
                xyLinksCur = [];
                if ~isempty(obj.vehicleIDsByTime{timeStep})
                    for i=1:length(obj.vehicleIDsByTime{timeStep})
                        simVeh = obj.getVehicleByID(obj.vehicleIDsByTime{timeStep}(i));
                        if ~isempty(simVeh)
                            matchedRSUID = simVeh.getRSUIDAtTime(timeStep);
                            if(matchedRSUID ~= -1)
                                rsu = obj.rsuList(matchedRSUID);
                                %xyLinksCur = [xyLinksCur; rsu.x, rsu.y, simVeh.getXPosAtTime(timeStep), simVeh.getYPosAtTime(timeStep)] ;
                                xyLinksCur = [xyLinksCur; rsu.x, rsu.y, simVeh.getAntennaPosAtTime(timeStep)]; 
                            end
                        end
                    end
                end
                obj.xyLinks{timeStep} = xyLinksCur;

            end
        end
        
        function simVeh = getVehicleByID(obj, vehicleID)
            vehicleIndex = find(obj.vehicleIDList == vehicleID);
            if ~isempty(vehicleIndex)
                simVeh = obj.getVehicleByIndex(vehicleIndex);
            else
                fprintf("Invalid vehicle ID:\t " + num2str(vehicleID) + "\n");
                simVeh = []; 
            end
        end
        
        function simVehs = getVehiclesByIDs(obj, vehicleIDList)
            simVehs(length(vehicleIDList)) = SimVehicle(length(vehicleIDList));
            
            for i = 1:length(vehicleIDList)
                if ~isempty(obj.getVehicleByID(vehicleIDList(i)))
                    simVehs(i) = obj.getVehicleByID(vehicleIDList(i));
                else
                    simVehs = [];
                    return
                end
            end
        end
        
        function simVeh = getVehicleByIndex(obj, vehicleIndex)
            if vehicleIndex >= 1 && vehicleIndex <= obj.numVehicles
                simVeh = obj.vehiclesByIndex(vehicleIndex);
            else
                fprintf("Invalid vehicle index:\t " + num2str(vehicleIndex) + "\n");
                simVeh = [];
            end
        end
        
        function obj = setVehicleIDsByTime(obj, viewedVehicles)
            for i = 1:length(obj.timesteps)
                if ~isempty(viewedVehicles{i})
                    vehicleIDsByTime{i} = viewedVehicles{i}(:,1)';
                else
                    vehicleIDsByTime{i} = [];
                end
            end
            obj.vehicleIDsByTime = vehicleIDsByTime;
        end
        
        function vehicleIDsAtT = getVehicleIDsAtTime(obj, time)
            vehicleIDsAtT = obj.vehicleIDsByTime{time};
        end
        
        function obj = updateRSUConnectionsAtTime(obj, time, rsuID, vehicleID)
            if(rsuID == -1)
                return;
            end
            currentRSU = obj.rsuList(rsuID);
            currentRSU = currentRSU.addConnectedVehicleAtTime(time, vehicleID);
            obj.rsuList(rsuID)=currentRSU;
        end
        
        function obj = clearRSUConnections(obj, simTime)
            for i = 1:length(obj.rsuList)
                obj.rsuList(i).connectedVehicles = cell(1,simTime);
            end
            
        end
        
        function obj = clearBlockageTimes(obj)
            for i = 1:length(obj.vehiclesByIndex)
                obj.vehiclesByIndex(i).blockageTimes = [];
            end
            
        end
        
        function obj = runGreedyScenario(obj, timeStep)
            %fprintf('RSU Selection for t= %f using greedy method\n',timeStep)
            
            for j = 1:length(obj.vehicleIDsByTime{timeStep})
                noBlockage = 1;
                curVeh = obj.getVehicleByID(obj.vehicleIDsByTime{timeStep}(j));
                timeIndex = curVeh.getTimeIndex(timeStep);
                antennaPos = curVeh.getAntennaPosAtTime(timeStep);
                xPos = antennaPos(1);
                yPos = antennaPos(2);
                blockages = obj.getPotentialBlockages(timeStep, curVeh.vehicleId, 200);
                [selectedRSUIndex, curVeh] = curVeh.selectRSUAtTime(timeStep, obj.rsuList, blockages); 
                if(~isempty(find(curVeh.blockageTimes == timeIndex)))
                    noBlockage = 0;
                end
                curVeh.RSUs(timeIndex) = selectedRSUIndex;
                curVeh.datarate(timeIndex) = obj.calcDataRateAtTime(curVeh.vehicleId, selectedRSUIndex, timeStep) * noBlockage;
                curVeh.rss(timeIndex) = obj.getRSSAtTime(curVeh.vehicleId, selectedRSUIndex, timeStep);
                obj.vehiclesByIndex(curVeh.vehicleIndex) = curVeh;
                if selectedRSUIndex ~= -1
                    obj = obj.updateRSUConnectionsAtTime(timeStep,selectedRSUIndex, curVeh.vehicleIndex);
                end
            end
        end
        
        function obj = runSMARTScenario(obj, timeStep, threshold)
            % Runs the SMART handoff policy 
            for j = 1:length(obj.vehicleIDsByTime{timeStep})
                noBlockage = 1;
                curVeh = obj.getVehicleByID(obj.vehicleIDsByTime{timeStep}(j));
                timeIndex = curVeh.getTimeIndex(timeStep);
                antennaPos = curVeh.getAntennaPosAtTime(timeStep);
                xPos = antennaPos(1);
                yPos = antennaPos(2);
                blockages = obj.getPotentialBlockages(timeStep, curVeh.vehicleId, 75);
                
                if(timeIndex >= 2)
                    selectedRSUIndex  = curVeh.RSUs(timeIndex - 1);
                else
                    selectedRSUIndex = -1;
                end
                
                servingRSS = obj.getRSSAtTime(curVeh.vehicleId, selectedRSUIndex, timeStep);
                hasLOS = obj.hasLOSRSU(timeStep, curVeh.vehicleId, selectedRSUIndex);
                
                if(~hasLOS && selectedRSUIndex ~= -1)
                    noBlockage = 0;
                    fprintf("t=%d v=%d SMART Connection blocked\n", timeStep, curVeh.vehicleId);
                end
                
                if(selectedRSUIndex == -1 || servingRSS < threshold || ~hasLOS)
                    selectedRSUIndex = obj.selectRSUAtTimeSMART(curVeh.vehicleId, timeStep, blockages, threshold); 
                end
                
                if(selectedRSUIndex == -1)
                    %Then just pair with nearest one 
                    [selectedRSUIndex, curVeh] = curVeh.selectRSUAtTime(timeStep, obj.rsuList, blockages);
                end
                
                curVeh.RSUs(timeIndex) = selectedRSUIndex;
                curVeh.datarate(timeIndex) = obj.calcDataRateAtTime(curVeh.vehicleId, selectedRSUIndex, timeStep) * noBlockage;
                curVeh.rss(timeIndex) = obj.getRSSAtTime(curVeh.vehicleId, selectedRSUIndex, timeStep);
                obj.vehiclesByIndex(curVeh.vehicleIndex) = curVeh;
                if selectedRSUIndex ~= -1
                    obj = obj.updateRSUConnectionsAtTime(timeStep,selectedRSUIndex, curVeh.vehicleIndex);
                end
            end
            
        end
        
        function selectedRSU = selectRSUAtTimeSMART(obj, vehicleID, timeStep, blockages, threshold)
            %Find nearest RSUs 
            %For each, find datarate until RSS falls below threshold 
            curVeh = obj.getVehicleByID(vehicleID);
            timeIndex = curVeh.getTimeIndex(timeStep);
            antennaPos = curVeh.getAntennaPosAtTime(timeStep);
            xPos = antennaPos(1);
            yPos = antennaPos(2);
            blockages = obj.getPotentialBlockages(timeStep, curVeh.vehicleId, 200);
            potentialRSUs = [];

            for i = 1:length(obj.rsuList)
                hasLOS = obj.hasLOSRSU(timeStep, curVeh.vehicleId, i);
                distance = pdist([obj.rsuList(i).x, obj.rsuList(i).y;xPos, yPos], 'euclidean');
                
                if hasLOS && distance <= 200
                    % Potential RSU for pairing
                    potentialRSUs = [potentialRSUs, i];
                end
                
            end
            
            maxDataTransmitted = 0;
            bestIndex = -1;
            
            for i = 1:length(potentialRSUs)
                rsuIndex = potentialRSUs(i);
                dataTransmitted = obj.getDataTransmitted(curVeh, rsuIndex, timeStep, threshold);
                if(dataTransmitted > maxDataTransmitted)
                    maxDataTransmitted = dataTransmitted;
                    bestIndex = rsuIndex;
                end 
                
            end
            selectedRSU = bestIndex;
        end
        
        function dataTransmitted = getDataTransmitted(obj, curVeh, rsuIndex, timeStep, threshold)
            % Sum the datarate at each timestep until RSS falls below
            % threshold 
            rss = obj.getRSSAtTime(curVeh.vehicleId, obj.rsuList(rsuIndex).id, timeStep);
            totalData = 0;
            while(rss > threshold && timeStep <= 200)
                rss = obj.getRSSAtTime(curVeh.vehicleId, obj.rsuList(rsuIndex).id, timeStep);
                totalData = totalData + obj.calcDataRateAtTime(curVeh.vehicleId, obj.rsuList(rsuIndex).id, timeStep);
                timeStep=timeStep+1;
            end
            
            dataTransmitted = totalData; 
        end
        
        
        function obj = connectVehiclesToRSU(obj, timeStep)
            %fprintf('RSU Selection for t= %f using predictive method\n',timeStep)
            %For each vehicle in this timestep
            %connect to RSU from schedule 
            %if non los with RSU, then connect to nearest in LOS
            for j = 1:length(obj.vehicleIDsByTime{timeStep})
                curVeh = obj.getVehicleByID(obj.vehicleIDsByTime{timeStep}(j));
                timeIndex = curVeh.getTimeIndex(timeStep);
                
                antennaPos = curVeh.getAntennaPosAtTime(timeStep);
                xPos = antennaPos(1);
                yPos = antennaPos(2);
                blockages = obj.getPotentialBlockages(timeStep, curVeh.vehicleId, 200);
                
%                 if(curVeh.speed(timeIndex) == 0)
%                     [selectedRSUIndex, curVeh] = curVeh.selectRSUAtTime(timeStep, obj.rsuList, blockages);
%                 else
%                     selectedRSUIndex = curVeh.RSUPlan(timeIndex);
%                 end
                selectedRSUIndex = curVeh.RSUPlan(timeIndex);

                if (obj.connectionFailed(timeStep, curVeh.vehicleId, selectedRSUIndex) && timeIndex > 1)
                    
                    if(selectedRSUIndex == curVeh.RSUs(timeIndex-1))
                        fprintf("Link to %d interrupted for vehicle id %d at t=%d\n", ...
                        selectedRSUIndex, curVeh.vehicleId, timeStep);
                    end
                    badIndex = selectedRSUIndex;
                    [selectedRSUIndex, curVeh] = curVeh.selectRSUAtTime(timeStep, obj.rsuList, blockages);
                    fprintf("Schedule for %d failed to predict blockage at t=%d (planned %d selected %d)\n", ...
                        curVeh.vehicleId, timeStep, badIndex, selectedRSUIndex);
                end

                obj = obj.updateRSUConnectionsAtTime(timeStep,selectedRSUIndex, curVeh.vehicleIndex);
                curVeh.RSUs(timeIndex) = selectedRSUIndex;
                curVeh.datarate(timeIndex) = obj.calcDataRateAtTime(curVeh.vehicleId, selectedRSUIndex, timeStep);
                curVeh.rss(timeIndex) = obj.getRSSAtTime(curVeh.vehicleId, selectedRSUIndex, timeStep);
                obj.vehiclesByIndex(curVeh.vehicleIndex) = curVeh;
            end
        end
        
        function obj = updateVehiclesSchedules(obj, timeStep, depth)
            %for each RSU, get list of currently connected vehicles
            %check for blockages with other connected vehicles
            %update that vehicles schedule
        
            for j = 1:length(obj.rsuList)
                curRSU1 = obj.rsuList(j);

                connectedVehicles = [curRSU1.connectedVehicles{timeStep}];
                if isempty(connectedVehicles)
                    continue;
                end

                otherVehicles = SimVehicle(1,length(connectedVehicles));

                for k = 1:length(connectedVehicles)
                    otherVehicles(k) = obj.vehiclesByIndex(connectedVehicles(k));  
                end

                for k = 1:length(connectedVehicles)
                    curVeh = obj.vehiclesByIndex(connectedVehicles(k));
                   obj.vehiclesByIndex(connectedVehicles(k)) = ...
                       curRSU1.getBestScheduleDAG(timeStep, curVeh, otherVehicles, obj.rsuList, depth, obj);
                       %curRSU1.updateScheduleGreedy(timeStep, curVeh, otherVehicles, obj.rsuList, depth, obj); %breaks OOP I'm pretty sure, oops
                        
                end

            end
        end

        function obj = updateVehiclesSchedulesRSUPairs(obj, timeStep, depth)
            %for each RSU, get list of currently connected vehicles
            %check for blockages with other connected vehicles
            %update that vehicles schedule, assume RSUs are connected, so they
            %make decision together
        
            for j = 1:length(obj.rsuList)
                if(mod(j,2)==0)
                    continue;
                end

                % For all odd RSUs (We assume RSUs work in pairs of 2)
                curRSU1 = obj.rsuList(j);
                curRSU2 = obj.rsuList(j+1);
                connectedVehicles = [curRSU1.connectedVehicles{timeStep}, ...
                                        curRSU2.connectedVehicles{timeStep}];
                if isempty(connectedVehicles)
                    continue;
                end

                otherVehicles = SimVehicle(1,length(connectedVehicles));

                for k = 1:length(connectedVehicles)
                    otherVehicles(k) = obj.vehiclesByIndex(connectedVehicles(k));  
                end

                for k = 1:length(connectedVehicles)
                    curVeh = obj.vehiclesByIndex(connectedVehicles(k));
                   obj.vehiclesByIndex(connectedVehicles(k)) = ...
                       curRSU1.updateSchedule(timeStep, curVeh, otherVehicles, obj.rsuList, depth);
                end

            end
        end

        function obj = runMatching(obj, algorithm, utilityFunction)
            loadingBar = waitbar(0, "Running matching");
            runTime = length(obj.timesteps);
            for i = 1:runTime
                
                fprintf('Matching for t= %f\n',i);
                if ~isempty(obj.getVehicleIDsAtTime(i))
                    obj = obj.runMatchingAtTime(i, algorithm, utilityFunction);
                end
                waitbar(i/runTime, loadingBar, sprintf('Progress: %d %%', floor(i/runTime*100)));
            end
        end
        
        function obj = runMatchingAtTime(obj, time, algorithm, utilityFunction)
            curVehicles = obj.getVehicleIDsAtTime(time);
            
            for i = 1:length(curVehicles)
                
                curVehicle = obj.getVehicleByID(curVehicles(i));
                
                if ~isempty(curVehicle)
                    matchingCandidateIDs = curVehicle.getVehiclesInViewAtTime(time);

                    if ~isempty(matchingCandidateIDs)
                        matchingCandidates = obj.getVehiclesByIDs(matchingCandidateIDs);
                        curVehicle = curVehicle.addMatchingCandidatesAtTime(time, matchingCandidates);
                        curVehicle = curVehicle.runMatchingAtTime(time, "SF", @u_nearest);
                    end
                    obj.vehiclesByIndex(curVehicle.vehicleIndex) = curVehicle;
                end
                
            end

        end
        
        function nearbyVehicles = getVehiclesNearby(obj, time, vehicleID, range)
            vehicleIDsAtT = obj.vehicleIDsByTime{time};
            vehicleX = obj.getVehicleByID(vehicleID).getXPosAtTime(time);
            vehicleY = obj.getVehicleByID(vehicleID).getYPosAtTime(time);
            nearbyVehicles = [];
            
            for i = 1:length(vehicleIDsAtT)
                if(isempty(obj.getVehicleByID(vehicleIDsAtT(i))))
                    continue;
                end
       
                curX = obj.getVehicleByID(vehicleIDsAtT(i)).getXPosAtTime(time);
                curY = obj.getVehicleByID(vehicleIDsAtT(i)).getYPosAtTime(time);
                
                distanceVehicle = pdist([vehicleX, vehicleY;curX, curY], 'euclidean');
            
                if distanceVehicle<=range && distanceVehicle ~= 0
                    nearbyVehicles = [nearbyVehicles, obj.getVehicleByID(vehicleIDsAtT(i))];
                end
            end
            
        end
        
        function blockages = getPotentialBlockages(obj, time, vehicleID, range)
            curVeh = obj.getVehicleByID(vehicleID);
            nearbyVehicles = obj.getVehiclesNearby(time, vehicleID, range);
            potentialBlockages = [];
    
            if(curVeh.vehicleType ~= 1)
                for i = 1:length(nearbyVehicles)
                    if (nearbyVehicles(i).vehicleType == 1)
                        %Then it is a truck 
                        potentialBlockages = [potentialBlockages; ...
                                                nearbyVehicles(i).getSegments(time)];
                    end
                end
            end

            buildingBlockages = obj.buildingLines;
            
            blockages = [potentialBlockages; buildingBlockages];
            
        end
        
        function los = hasLOSRSU(obj, timestep, vehicleID, rsuIndex)
            
            if rsuIndex == -1
                los = 0;
                return;
            end
            
            curVeh = obj.getVehicleByID(vehicleID);
            
            if(curVeh.vehicleType == 1)
                los = 1;
                return;
            end
            
            antennaPos = curVeh.getAntennaPosAtTime(timestep);
            xPos = antennaPos(1);
            yPos = antennaPos(2);
            rsuX = obj.rsuList(rsuIndex).x;
            rsuY = obj.rsuList(rsuIndex).y;
            blockages = obj.getPotentialBlockages(timestep, curVeh.vehicleId, 75);
            
            los = hasLOS(xPos, yPos, rsuX, rsuY, blockages);
            
        end
        
        function failed = connectionFailed(obj, timestep, vehicleID, rsuIndex)
            
            if rsuIndex == -1
                failed = 0;
                return;
            end
            
            curVeh = obj.getVehicleByID(vehicleID);
            antennaPos = curVeh.getAntennaPosAtTime(timestep);
            xPos = antennaPos(1);
            yPos = antennaPos(2);
            rsuX = obj.rsuList(rsuIndex).x;
            rsuY = obj.rsuList(rsuIndex).y;
            blockages = obj.getPotentialBlockages(timestep, curVeh.vehicleId, 200);
            
            distance =  pdist([xPos,yPos;rsuX,rsuY], 'euclidean');

            failed = (~hasLOS(xPos, yPos, rsuX, rsuY, blockages))...
                || distance > 200;
        end
        
        function obj = fixVehiclesByTime(obj)
            for i = 1:length(obj.vehicleIDsByTime)
                needToRemove = 1;
                while(needToRemove == 1)
                    if(isempty(obj.vehicleIDsByTime{i}))
                        needToRemove = 0;
                        continue;
                    end
                    for j = 1:length(obj.vehicleIDsByTime{i})
                        vehicle = obj.getVehicleByID(obj.vehicleIDsByTime{i}(j));
                        if isempty(vehicle)
                            fprintf("Vehicle id %d removed\n", obj.vehicleIDsByTime{i}(j)); 
                            obj.vehicleIDsByTime{i}(j) = [];
                            break;
                        elseif j == length(obj.vehicleIDsByTime{i})
                            needToRemove = 0;
                        end
                    end
                end
            end 
            
        end
        
        function obj = addBuildingLinesToRSUs(obj)
            for i = 1:length(obj.rsuList)
                obj.rsuList(i).buildingLines = obj.buildingLines;
            end
        end
        
        function handovers = getHandoversByVehicleIndex(obj)
            handovers = ones(1, length(obj.vehiclesByIndex));
            for i =1:length(obj.vehiclesByIndex)
                handovers(i) = obj.vehiclesByIndex(i).getHandovers();
            end
        end
        
        function vehicleIndex = getVehicleIndex(obj, vehicleID)
            %Returns vehicle index in matchingSim's vehicles array
            vehicleIndex = find(obj.vehicleIDList == vehicleID);
        end
        
        function [] = viewSimulation(obj)
            userInput = 1;
            timestep = 1;
            while(1)
                viewTimestep(obj, timestep)
                fprintf("Current timestep is: %d\n", timestep);
                userInput = input("Enter timestep, -1 exit, 0 back, enter to continue\n");
                fprintf("Inputted: %d\n", userInput);
                
                
                if(isempty(userInput))
                    timestep = timestep + 1;
                else
                    if(userInput == 0)
                        timestep = timestep - 1;
                    elseif(userInput == -1)
                        break;
                    else
                        timestep = userInput;
                    end
                end
            end
        end
        
        function [] = viewTimestep(obj, timestep)
            % Creates plot of objects in the timestep 
            %plot RSU Positions 
            clf;
            hold on;
            axis equal;
            mapPrint(obj.outputMap)
            for i = 1:length(obj.rsuList)
                xPos = obj.rsuList(i).x;
                yPos = obj.rsuList(i).y;
                viscircles([xPos, yPos], 1);
                viscircles([xPos, yPos], 200, 'LineStyle', '--');
                text(xPos, yPos, num2str(i));
            end
            
            curVehicleIDs = obj.getVehicleIDsAtTime(timestep); 
            
            for i = 1:length(curVehicleIDs)
                
                curVehicle = obj.getVehicleByID(curVehicleIDs(i));
                xPos = curVehicle.getXPosAtTime(timestep);
                yPos = curVehicle.getYPosAtTime(timestep);
                id = curVehicle.vehicleId;
                vehicleLines = curVehicle.getSegments(timestep);
                text(xPos, yPos, num2str(id));
                if(curVehicle.vehicleType == 1)
                    plot([vehicleLines(:,1)';vehicleLines(:,3)'],[vehicleLines(:,2)';vehicleLines(:,4)'], ...
                            'Color', 'Red');
                else
                    plot([vehicleLines(:,1)';vehicleLines(:,3)'],[vehicleLines(:,2)';vehicleLines(:,4)'], ...
                            'Color', 'Green');
                end

            end
            
            curLinks = obj.xyLinks{timestep};
            if ~isempty(curLinks)
                plot([curLinks(:,1)';curLinks(:,3)'],[curLinks(:,2)';curLinks(:,4)']);
            end
            
            hold off;
            
            
        end
        
    end
end

