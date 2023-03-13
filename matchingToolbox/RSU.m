classdef RSU
    %RSU Road side unit class
    %   Detailed explanation goes here
    
    properties
        id          % Basestation ID
        index       % Index in "potential pos" array
        x           % X position
        y           % Y position
        height      % Height I think
        connectedVehicles %List of connected vehicles at T
        buildingLines %Building Lines from matching sim (static blockages)
    end
    
    methods
        function obj = RSU(id)
        end
        function obj = initRSU(obj, id,index,x,y,height, simTime)
            %RSU Construct an instance of this class
            %   Detailed explanation goes here
            obj.id = id;
            obj.index = index;
            obj.x = x;
            obj.y = y;
            obj.height = height;
            obj.connectedVehicles = cell(1,simTime);
        end
        
        function obj = addConnectedVehicleAtTime(obj,time, vehicleID)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            if isempty(obj.connectedVehicles{time})
                obj.connectedVehicles{time} = [vehicleID];
            else
                obj.connectedVehicles{time}(end+1) = vehicleID;
            end
        end
        
        function simVeh = getBestScheduleDAG(obj, timestep, egoVeh, otherVehicles, rsuList, depth, matchingSim)
            timeIndex = egoVeh.getTimeIndex(timestep);     
            maxTimestep = length(egoVeh.times);
            if timeIndex + depth > maxTimestep
                %return as much as possible 
                depth = maxTimestep - timeIndex;
            end
            
            
            potentialRSUs = obj.getPotentialRSUsForDepth(timestep, egoVeh, otherVehicles, rsuList, depth, matchingSim);
            dagPlan = obj.calcBestScheduleDAG(potentialRSUs, timestep, egoVeh, otherVehicles, rsuList, depth, matchingSim);
            
            egoVeh.RSUPlan(timeIndex+1:timeIndex+depth) = dagPlan;
            simVeh = egoVeh;
        end
        
        function potRSUs = getPotentialRSUsForDepth(obj, timestep, egoVeh, otherVehicles, rsuList, depth, matchingSim)
            % Calculates the potential rsus at each timestep. If there are no rsus in LOS, an RSU with id -1 is added.

            potRSUs = {};

            for curDepth = 1:depth
                %Get vehicle position at that time
                if rand <= .25 && curDepth >= 7
                    antennaPos = egoVeh.getGuessedAntennaPosAtTime(timestep + curDepth);
                else
                    antennaPos = egoVeh.getAntennaPosAtTime(timestep + curDepth);
                end
                
                xPos = antennaPos(1);
                yPos = antennaPos(2);
                
                %Get environment info at that time
                blockages = [obj.getBlockagesFromVehicles(egoVeh, otherVehicles, timestep + curDepth); matchingSim.buildingLines];
                
                potentialRSUs = egoVeh.getRSUsInRangeLOS(xPos, yPos, rsuList, blockages);
                
                if(isempty(potentialRSUs))
                    potentialRSUs = [-1];
                end
                potRSUs{curDepth,1} = potentialRSUs;
            end
        end
        
        function schedule = calcBestScheduleDAG(obj, potentialRSUs, startTime, egoVeh, otherVehicles, rsuList, depth, matchingSim)
            % potential RSUs is a cell of arrays
            potRSUs = [{[obj.id]}; potentialRSUs; {[0]}]; % add a start and stop node, 0 is the stop node

            nNum = 0; %vertex num
            nNumTime = [];
            nNames = []; %vertex names
            nIDs = []; %ID vertex corresponds to
            
            %Get node names and IDs
            for i = 1:size(potRSUs)
                nNum = nNum + length(potRSUs{i});
                nNumTime = [nNumTime, length(potRSUs{i})];
                for j = 1:length(potRSUs{i})
                    nNames = [nNames, num2str(potRSUs{i}(j)) + "_{" + num2str(i-1) + "}"];
                    nIDs = [nIDs, potRSUs{i}(j)];
                end
            end
            
            % Get ajacencyasdkhfa matrix to construct graph
            aDAG = obj.getAMatrix(nNum, nNumTime, nNames, nIDs, potRSUs, startTime, egoVeh, otherVehicles, rsuList, depth, matchingSim);
            
            %create graph and find shortest path
            sDAG = digraph(aDAG, nNames);
            %p = plot(sDAG,'EdgeLabel',sDAG.Edges.Weight);
            sPath = shortestpath(sDAG, num2str(obj.id) + "_{0}","0_{11}",'Method','acyclic');
            %highlight(p,sPath,'EdgeColor','g')
            
            % Convert path to schedulew 
            schedule = [];
            for i = 1:length(sPath)
                if( i ~= 1 && i ~= length(sPath))
                    nIndex = find(sPath(i) == nNames);
                    id = nIDs(nIndex);
                    schedule = [schedule, id];
                end
            end
            
        end
        
        function aMatrix = getAMatrix(obj, nNum, nNumTime, nNames, nIDs, potRSUs, startTime, egoVeh, otherVehicles, rsuList, depth, matchingSim)
            aDAG = zeros(nNum, nNum); %ajaceny matrix
            nNumTimeSum = cumsum(nNumTime);
            % populate matrix
            aDAGYIndex = 1;
            for i = 1:length(potRSUs)
                for j = 1:length(potRSUs{i})
                    if(i < length(potRSUs))
                        aDAGXIndex = nNumTimeSum(i);
                        % current vertex is connected to all others in next time
                        for k = 1:length(potRSUs{i+1})
                            aDAGXIndex = aDAGXIndex + 1;

                            if(i ~= length(potRSUs) - 1) %If we are not on the very last node
                                aDAG(aDAGYIndex, aDAGXIndex) = -1 * obj.getExpectedData(potRSUs{i}(j),potRSUs{i+1}(k), egoVeh, otherVehicles, startTime + i, matchingSim);%call to get datarate function
                            else
                                aDAG(aDAGYIndex, aDAGXIndex) = .00000000000000001; %infinetely small
                            end         
                        end
                    end

                    aDAGYIndex = aDAGYIndex + 1;
                end
            end
            aMatrix = aDAG;
        end
        
        function expectedData = getExpectedData(obj, sRSU, tRSU, egoVeh, otherVehicles, timestep, matchingSim)
            expectedData = rand; %TODO - replace with actual calc
        end
        
        
        function simVeh = updateSimVehSchedule(obj, timestep, egoVeh, otherVehicles, rsuList, depth)
            if(egoVeh.vehicleType == 1)
                simVeh = egoVeh;
                return;
            end
            
            %vehicle and other vehicles are objects
            %depth is how many timesteps into the future
            timeIndex = egoVeh.getTimeIndex(timestep);     
            maxTimestep = length(egoVeh.times);
            if timeIndex + depth > maxTimestep
                %return as much as possible 
                depth = maxTimestep - timeIndex;
            end
            
            proposedPlan = egoVeh.RSUPlan(timeIndex+1:timeIndex+depth);
            
            %find when blockages occurs
            blockageOccurences = obj.findBlockageOccurences(egoVeh, otherVehicles, timestep, depth, proposedPlan, rsuList);
            badConnectionIndices = find(blockageOccurences~=0);
            
            if(isempty(badConnectionIndices))
                simVeh = egoVeh;
                return
            else
                %update schedule 
                for i = 1:length(badConnectionIndices)
                    
                    badRSU = blockageOccurences(badConnectionIndices(i));
                    badSelections = find(proposedPlan == badRSU);
                    
                    if badRSU == -1
                        goodRSU = badRSU;
                    elseif(mod(badRSU, 2) == 0)
                        goodRSU = badRSU - 1;
                    else
                        goodRSU = badRSU + 1;
                    end
                    
                    if(goodRSU == 0)
                        fprintf("oops");
                    end
                    
                    for j = 1:length(badSelections)
                        proposedPlan(badSelections(j)) = goodRSU;
                    end
                        
                end
                
            end
            
            egoVeh.RSUPlan(timeIndex+1:timeIndex+depth) = proposedPlan;
            simVeh = egoVeh;
        end
        
        function simVeh = updateSchedule(obj, timestep, egoVeh, otherVehicles, rsuList, depth)
            if(egoVeh.vehicleType == 1)
                simVeh = egoVeh;
                return;
            end
            
            %vehicle and other vehicles are objects
            %depth is how many timesteps into the future
            timeIndex = egoVeh.getTimeIndex(timestep);     
            maxTimestep = length(egoVeh.times);
            if timeIndex + depth > maxTimestep
                %return as much as possible 
                depth = maxTimestep - timeIndex;
            end
            
            proposedPlan = egoVeh.RSUPlan(timeIndex+1:timeIndex+depth);
            
            updatedPlan = obj.removeScheduledBlockages(timestep, egoVeh, proposedPlan, otherVehicles, rsuList, depth, []);
            
            egoVeh.RSUPlan(timeIndex+1:timeIndex+depth) = updatedPlan;
            simVeh = egoVeh;
        end
        
        function simVeh = updateScheduleGreedy(obj, timestep, egoVeh, otherVehicles, rsuList, depth, matchingSim)            
            %vehicle and other vehicles are objects
            %depth is how many timesteps into the future
            timeIndex = egoVeh.getTimeIndex(timestep);     
            maxTimestep = length(egoVeh.times);
            if timeIndex + depth > maxTimestep
                %return as much as possible 
                depth = maxTimestep - timeIndex;
            end
            
            proposedPlan = egoVeh.RSUPlan(timeIndex+1:timeIndex+depth);
            
            greedyPlan = obj.getGreedyPlan(timestep, egoVeh, proposedPlan, otherVehicles, rsuList, depth, matchingSim);
            
            egoVeh.RSUPlan(timeIndex+1:timeIndex+depth) = greedyPlan;
            simVeh = egoVeh;
        end
        
        function greedyPlan = getGreedyPlan(obj, timestep, egoVeh, proposedPlan, otherVehicles, rsuList, depth, matchingSim)
            
            % At each timestep, taking into account the previous selection,
            % find the next best selection 
            prevRSUID = obj.id;
            
            greedyPlan = proposedPlan;
            
            for curDepth = 1:depth
                
                %Get vehicle position at that time
                if rand <= .25 && curDepth >= 7
                    antennaPos = egoVeh.getGuessedAntennaPosAtTime(timestep + curDepth);
                else
                    antennaPos = egoVeh.getAntennaPosAtTime(timestep + curDepth);
                end
                
                xPos = antennaPos(1);
                yPos = antennaPos(2);
                
                %Get environment info at that time
                blockages = [obj.getBlockagesFromVehicles(egoVeh, otherVehicles, timestep + curDepth); obj.buildingLines];
                
                potentialRSUs = egoVeh.getRSUsInRangeLOS(xPos, yPos, rsuList, blockages);
                
                greedyPlan(curDepth) = obj.selectRSUGreedy(timestep+curDepth, egoVeh, otherVehicles, prevRSUID, potentialRSUs, rsuList, matchingSim);
                
                prevRSUID = greedyPlan(curDepth);
            end
            
        end
        
        function chosenRSU = selectRSUGreedy(obj, curTime, egoVeh, otherVehicles, prevRSUID, potentialRSUs, rsuList, matchingSim)
            if(isempty(potentialRSUs))
                chosenRSU = -1;
                return;
            end
            
            maxData = 0;
            chosenRSU = potentialRSUs(1);
            for i = 1:length(potentialRSUs)
               curRSUID = potentialRSUs(i);
               
               data = obj.estimateMaxDataTime(prevRSUID, curRSUID, curTime, egoVeh, otherVehicles, rsuList, matchingSim);
               
               if(data > maxData)
                   maxData = data;
                   chosenRSU = curRSUID;
               end
            end
            
        end
        
        function bestPlan = calcBestPlan(obj, potPlans, timestep, egoVeh, otherVehicles, rsuList, depth, matchingSim)
            
            maxData = 0;
            bestPlan = potPlans(1,1:depth);
            for i = 1:length(potPlans)
                curPlan = potPlans(i,1:depth);
                curData = obj.estimateMaxDataPlan(curPlan, timestep, egoVeh, otherVehicles,rsuList, depth, matchingSim);
                
                if(curData > maxData)
                    maxData = curData;
                    bestPlan = curPlan;
                end
            end
            
        end
        
        function maxData = estimateMaxDataPlan(obj, curPlan, timestep, egoVeh, otherVehicles, rsuList, depth, matchingSim)
            
            totalData = 0;
            prevRSUID = obj.index;
            
            for curDepth = 1:depth
                totalData = totalData + obj.estimateMaxDataTime(prevRSUID,curPlan(curDepth), timestep+depth, egoVeh, otherVehicles, rsuList, matchingSim);
            end
            
            maxData = totalData;
        end
        
        function maxData = estimateMaxDataTime(obj, prevRSUID, curRSUID, curTime, egoVeh, otherVehicles, rsuList, matchingSim)
            % Estimates the datarate for a given link with the following
            % overheads
            plannedOverhead = .1984; % Planned handover overhead in seconds
            
            antennaPos = egoVeh.getAntennaPosAtTime(curTime);
            xPos = antennaPos(1);
            yPos = antennaPos(2);
            
            xPos2 = rsuList(curRSUID).x;
            yPos2 = rsuList(curRSUID).y;
            
            if(prevRSUID ~= curRSUID)
                handover = 1;
            else
                handover = 0;
            end
            
            blockages = [obj.getBlockagesFromVehicles(egoVeh, otherVehicles, curTime); obj.buildingLines];
            
            if(hasLOS(xPos,yPos,xPos2,yPos2,blockages))
                datarate = matchingSim.calcDataRateAtTime(egoVeh.vehicleId, curRSUID, curTime);
            else
                datarate = matchingSim.calcDataRateAtTimeNLOS(egoVeh.vehicleId, curRSUID, curTime);
            end
            
            maxData = datarate*(1- (handover * plannedOverhead));
            
        end
        
        function simVeh = updateScheduleMaxData(obj, timestep, egoVeh, otherVehicles, rsuList, depth, matchingSim)
            timeIndex = egoVeh.getTimeIndex(timestep);     
            maxTimestep = length(egoVeh.times);
            if timeIndex + depth > maxTimestep
                %return as much as possible 
                depth = maxTimestep - timeIndex;
            end
            
            proposedPlan = egoVeh.RSUPlan(timeIndex+1:timeIndex+depth);
            
            bestPlan = obj.findBestPlan(timestep, egoVeh, otherVehicles, rsuList, depth, matchingSim);
            
            egoVeh.RSUPlan(timeIndex+1:timeIndex+depth) = bestPlan;
            simVeh = egoVeh;
        end
        
        function bestPlan = findBestPlan(obj, timestep, egoVeh, otherVehicles, rsuList, depth, matchingSim)
            
            % Find potential RSUs at each timestep
            potentialRSUs = {};
            
            for curDepth = 1:depth
                
                %Get vehicle position at that time
                if rand <= .25 && curDepth >= 7
                    antennaPos = egoVeh.getGuessedAntennaPosAtTime(timestep + curDepth);
                else
                    antennaPos = egoVeh.getAntennaPosAtTime(timestep + curDepth);
                end
                
                xPos = antennaPos(1);
                yPos = antennaPos(2);
                
                potentialRSUs{curDepth} = egoVeh.getRSUsInRangeLOS(xPos, yPos, rsuList, []);
            end
            
            % Calculate all potential schedules
            potentialRSUs2 = potentialRSUs;
            [potentialRSUs2{:}] = ndgrid(potentialRSUs{:});
            potPlans = cell2mat(cellfun(@(m)m(:),potentialRSUs2,'uni',0));

            % calculate best plan, check if LOS using blockages at each
            % time to determine datarate
            
            bestPlan = obj.calcBestPlan(potPlans, timestep, egoVeh, otherVehicles, rsuList, depth, matchingSim);
            
            %blockages = [obj.getBlockagesFromVehicles(egoVeh, otherVehicles, timestep + curDepth); obj.buildingLines];
            
        end
        
        function proposedPlan = removeScheduledBlockages(obj, timestep, egoVeh, proposedPlan, otherVehicles, rsuList, depth, badRSUs)
        
            %find when blockages occurs
            blockageOccurences = obj.findBlockageOccurences(egoVeh, otherVehicles, timestep, depth, proposedPlan, rsuList);
            badConnectionIndices = find(blockageOccurences>0);
            
            if(isempty(badConnectionIndices))
                
                return
            end
            %update schedule 
            for i = 1:length(badConnectionIndices)
                badIndex = badConnectionIndices(i);
                badRSU = blockageOccurences(badIndex);
                badRSUs = [badRSUs, badRSU];
                blockages = [obj.getBlockagesFromVehicles(egoVeh, otherVehicles, timestep + badIndex); obj.buildingLines];
                antennaPos = egoVeh.getAntennaPosAtTime(timestep + badIndex);
                xPos = antennaPos(1);
                yPos = antennaPos(2);
                
                goodRSU = egoVeh.findNearestRSUInLOS(xPos, yPos, rsuList, blockages);
                
                if(goodRSU == -1)
                    %Unavoidable outage, but at least we predicted it
                    %fprintf("Predicted Outage for vehicle %d at t=%d\n", egoVeh.vehicleId, timestep);
                    
                end
                
                %try and replace all 
                badSelections = find(proposedPlan(1:badIndex) == badRSU);
                improvedPlan = proposedPlan;
                for j = 1:length(badSelections)
                    improvedPlan(badSelections(j)) = goodRSU;
                end
                newBlockageOccurences = obj.findBlockageOccurences(egoVeh, otherVehicles, timestep, badIndex, improvedPlan, rsuList);
                if(~isempty(find(newBlockageOccurences>0)) || goodRSU == -1)
                    proposedPlan(badIndex) = goodRSU;
                else
                    proposedPlan = improvedPlan;
                end
            end
            
           %refine schedule 
            proposedPlan = obj.removeScheduledBlockages(timestep, egoVeh, proposedPlan, otherVehicles, rsuList, depth, badRSUs);
 
        end
        
        function blockageOccurences = findBlockageOccurences(obj, egoVeh, otherVehicles, timestep, depth, proposedPlan, rsuList)
            blockageOccurences = zeros(1, depth);
            
            %find Ideal schedule, plan, track if a blockage occurs  
            curDepth = 1;
            while curDepth <= depth
                blockages = [obj.getBlockagesFromVehicles(egoVeh, otherVehicles, timestep + curDepth); obj.buildingLines];
                
                if rand <= .25 && curDepth > 8
                    antennaPos = egoVeh.getGuessedAntennaPosAtTime(timestep + curDepth);
                else
                    antennaPos = egoVeh.getAntennaPosAtTime(timestep + curDepth);
                end
                    
                xPos = antennaPos(1);
                yPos = antennaPos(2);
                proposedRSU = proposedPlan(curDepth);
                greedyFailed = egoVeh.checkIfGreedyFailed(xPos, yPos, proposedRSU, rsuList, blockages);
                if(greedyFailed)
                    blockageOccurences(curDepth) = proposedRSU;
                end
                curDepth = curDepth + 1;
            end 
        end
        
        function blockages = getBlockagesFromVehicles(obj,curVeh, vehicles, timestep)
            blockages = [];
            for i = 1:length(vehicles)
                if (vehicles(i).vehicleType == 1 && vehicles(i).vehicleId ~= curVeh.vehicleId)
                    %Then it is a truck 
                    blockages = [blockages; ...
                                            vehicles(i).getSegments(timestep)];
                end
            end
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
            blockages = obj.getPotentialBlockages(timestep, curVeh.vehicleId, 75);
            
            distance =  pdist([xPos,yPos;rsuX,rsuY], 'euclidean');

            failed = (~hasLOS(xPos, yPos, rsuX, rsuY, blockages) && curVeh.vehicleType ~= 1)...
                || distance > 200;
        end
        
    end
end

