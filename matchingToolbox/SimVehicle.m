classdef SimVehicle
    %SIMVEHICLE Vehicle used in the matching simulation
    %   Detailed explanation goes here
       
    
    properties 
        vehiclesInRadius    % List of vehicles in radius indexed by timeIndex
        vehiclesInView      % List of vehicles in radius and with LOS by timeIndex
        x                   % X position indexed by timeIndex
        y                   % Y position indexed by timeIndex
        xy                  % XY position pair indexed by timeIndex
        PL                  % Preference list at a timeIndex
        times               % List of times vehicle is in sim, index in this list is timeIndex
        matches             % Matches (list of simvehicles) indexed by timeIndex
        RSUs                % RSU Chosen (list of RSU objects) indexed by timeIndex
        vehicleId           % Vehicle ID given by Traci, must be 1 or greater
        vehicleIndex        % Vehicle Index in the list of vehicles from Traci
        vehicleType         % Type of vehicle
        vehicleRadius       % Radius in which vehicle can see others
        matchingCandidates  % List of SimVehicle Objects indexed by timeIndex
        
    end
    
    methods

        function obj = SimVehicle(vehicleIndex, vehiclesStruct)
            %SIMVEHICLE Construct an instance of this class
            %   Detailed explanation goes here
            
        end
        
        function obj = initFromStruct(obj, vehicleIndex, vehiclesStruct)
            obj.vehicleIndex = vehicleIndex;
            obj.vehicleId = vehiclesStruct.vehNode(vehicleIndex).id;
            obj.vehicleType = vehiclesStruct.vehNode(vehicleIndex).vehicleType;
            obj.times = vehiclesStruct.vehNode(vehicleIndex).time;
            obj.x = vehiclesStruct.vehNode(vehicleIndex).x;
            obj.y = vehiclesStruct.vehNode(vehicleIndex).y;
            obj.vehiclesInView = vehiclesStruct.vehNode(vehicleIndex).inView;
            
        end
        
        function obj = createRSUConnectionScheduleNearest(obj,rsuList, buildingLines)
            RSUs = length(obj.times);
            for i = 1:length(obj.times)
                %For each timestep 
                RSUs(i) = obj.findNearestRSUInLOS(i, rsuList, buildingLines);
            end
            obj.RSUs = RSUs;
        end
        
        function nearestRSUIndex = findNearestRSUInLOS(obj, timeIndex, rsuList, buildingLines)
            nearestRSUIndex = -1;
            distanceToClosestRSU = 9999;
            for i = 1:length(rsuList)               
                x1 = obj.x(timeIndex);
                y1 = obj.y(timeIndex);
                x2 = rsuList(i).x;
                y2 = rsuList(i).y;
                distance = abs(pdist([x1,y1;x2,y2], 'euclidean'));
                if distance < distanceToClosestRSU && hasLOS(x1, y1, x2, y2, buildingLines)
                    nearestRSUIndex = i;
                    distanceToClosestRSU = distance;
                end
            end
            
            
        end
        
        function doesExist = doesVehicleExist(obj, timeStep)
            %Check if a vehicle exists at the given sim timestep
        end
        
        function rsuID = getRSUIDAtTime(obj, timeStep)
            timeIndex = find(obj.times == timeStep);
            rsuID = obj.RSUs(timeIndex);
        end
        
        function viewedVehicleIDs = getVehiclesInViewAtTime(obj, timeStep)
            timeIndex = find(obj.times == timeStep);
            viewedVehicleIDs = obj.vehiclesInView{timeIndex};
        end
        
        
        function obj = runMatchingAtTime(obj, timeStep, algorithm, utilityFunction)
        end
        
        function xPos = getXPosAtTime(obj, timeStep)
            timeIndex = find(obj.times == timeStep);
            xPos = obj.x(timeIndex);
        end
        
        function yPos = getYPosAtTime(obj, timeStep)
            timeIndex = find(obj.times == timeStep);
            yPos = obj.y(timeIndex);
        end
        
        
        function obj = addMatchingCandidatesAtTime(obj, timeStep, simVehList)
            timeIndex = find(obj.times == timeStep);
            obj.matchingCandidates{timeIndex} = simVehList;
        end
        
    end
end

