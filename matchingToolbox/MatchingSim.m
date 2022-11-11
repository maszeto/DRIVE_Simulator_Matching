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
        
        function obj = addRSUs(obj, potentialPos)
            rsuList(length(potentialPos.mmWaves.pos)) = RSU(length(potentialPos.mmWaves.pos));
            
            for i = 1:length(potentialPos.mmWaves.pos)
                rsuList(i) = rsuList(i).initRSU(i, i, potentialPos.mmWaves.pos(i,2), ...
                    potentialPos.mmWaves.pos(i,1), potentialPos.mmWaves.pos(i,3), length(obj.timesteps));
            end
            obj.rsuList = rsuList;
            
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
            tileCenters = obj.outputMap.inCentresTile(:,1:2);
            nearestTileIndex = dsearchn(tileCenters, vehiclePos);
            tileID = nearestTileIndex;
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
                                xyLinksCur = [xyLinksCur; rsu.x, rsu.y, simVeh.getXPosAtTime(timeStep), simVeh.getYPosAtTime(timeStep)] ;
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
        
        function simVehList = getVehiclesAtTime(obj, time)
        end
        
        function obj = updateRSUConnectionsAtTime(obj, time, rsuID, vehicleID)
            currentRSU = obj.rsuList(rsuID);
            currentRSU = currentRSU.addConnectedVehicleAtTime(time, vehicleID);
            obj.rsuList(rsuID)=currentRSU;
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
                
       
                curX = obj.getVehicleByID(vehicleIDsAtT(i)).getXPosAtTime(time);
                curY = obj.getVehicleByID(vehicleIDsAtT(i)).getYPosAtTime(time);
                
                distanceVehicle = pdist([vehicleX, vehicleY;curX, curY], 'euclidean');
            
                if distanceVehicle<=range
                    nearbyVehicles = [nearbyVehicles, obj.getVehicleByID(vehicleIDsAtT(i))];
                end
            end
            
        end
        
        function blockages = getPotentialBlockages(obj, time, vehicleID, range)
            curVeh = obj.getVehicleByID(vehicleID);
            nearbyVehicles = obj.getVehiclesNearby(time, vehicleID, range);
            potentialBlockages = [];
            for i = 1:length(nearbyVehicles)
                if (nearbyVehicles(i).vehicleType == 1)
                    %Then it is a truck 
                    potentialBlockages = [potentialBlockages; ...
                                            nearbyVehicles(i).getSegments(time)];
                end
            end
            
            blockages = potentialBlockages;
            
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
        
        
    end
end

