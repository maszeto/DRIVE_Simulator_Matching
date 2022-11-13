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
            
            blockageOccurences = zeros(1, depth);
            proposedPlan = egoVeh.RSUPlan(timeIndex+1:timeIndex+depth);
            %find Ideal schedule, plan, track if a blockage occurs  
            curDepth = 1;
            while curDepth <= depth
                blockages = obj.getBlockagesFromVehicles(otherVehicles, timestep + curDepth);
                antennaPos = egoVeh.getAntennaPosAtTime(timestep + curDepth);
                xPos = antennaPos(1);
                yPos = antennaPos(2);
                proposedRSU = proposedPlan(curDepth);
                greedyFailed = egoVeh.checkIfGreedyFailed(xPos, yPos, proposedRSU, rsuList, blockages);
                if(greedyFailed)
                    blockageOccurences(curDepth) = proposedRSU;
                end
                curDepth = curDepth + 1;
            end 
            
            %find where blockages occurs
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
        
        
        
    end
end

