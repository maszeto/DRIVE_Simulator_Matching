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
                blockages = [obj.getBlockagesFromVehicles(otherVehicles, timestep + badIndex); obj.buildingLines];
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
                blockages = [obj.getBlockagesFromVehicles(otherVehicles, timestep + curDepth); obj.buildingLines];
                
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
        
        function blockages = getBlockagesFromVehicles(obj,vehicles, timestep)
            blockages = [];
            for i = 1:length(vehicles)
                if (vehicles(i).vehicleType == 1)
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

