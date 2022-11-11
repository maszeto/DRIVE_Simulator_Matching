classdef SimVehicle
    %SIMVEHICLE Vehicle used in the matching simulation
    %   Detailed explanation goes here
       
    
    properties 
        vehiclesInRadius    % List of vehicles in radius indexed by timeIndex
        vehiclesInView      % List of vehicles in radius and with LOS by timeIndex
        x                   % X position indexed by timeIndex
        y                   % Y position indexed by timeIndex
        xy                  % XY position pair indexed by timeIndex
        speed               % Speed indexed by timeIndex
        accel               % Acceleration indexed by timeIndex
        angle               % Angle indexed by time index 
        lane                % Lane indexed by time index
        PL                  % Preference list at a timeIndex
        times               % List of times vehicle is in sim, index in this list is timeIndex
        matches             % Matches (list of simvehicles) indexed by timeIndex
        RSUs                % RSU Chosen (list of RSU ids) indexed by timeIndex
        RSUPlan             % RSUs planned indexed by timeIndex
        targetRSUs          % List of RSU the vehicle is targeting at the next timeStep
        vehicleId           % Vehicle ID given by Traci, must be 1 or greater
        vehicleIndex        % Vehicle Index in the list of vehicles from Traci
        vehicleType         % Type of vehicle, index in list of vehicle types
        vehicleRadius       % Radius in which vehicle can see others
        matchingCandidates  % List of SimVehicle Objects indexed by timeIndex
        route               % Line segments which make up the route
        routePlan           % Projected route based on plan 
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
            obj.speed = vehiclesStruct.vehNode(vehicleIndex).speed;
            obj.accel = vehiclesStruct.vehNode(vehicleIndex).accel;
            obj.angle = vehiclesStruct.vehNode(vehicleIndex).angle;
            obj.lane = vehiclesStruct.vehNode(vehicleIndex).lane;
            obj.vehiclesInView = vehiclesStruct.vehNode(vehicleIndex).inView;
            
        end
        
        function obj = createRoute(obj)
            %Using the data points provided from the actual route create
            %route (x & y line segments), with distance
            [x1, y1] = reducem(obj.x, obj.y,1);
            distance = hypot(diff(x1), diff(y1));
            distanceTraveled = [0; cumsum(distance)];
            obj.route = [x1,y1,distanceTraveled];
        end
        
        function obj = createRoutePlan(obj)
            obj = obj.createRoute();
            
        end
        
        function obj = createRSUConnectionScheduleNearest(obj,rsuList, buildingLines)
            obj = obj.createRoute();
            RSUs = length(obj.times);
            
            for i = 1:length(obj.times)
                %For each timestep 
                xPos = obj.x(i);
                yPos = obj.y(i);
                speed = 29; %m/s 
                [xPos, yPos] = obj.getExpectedPositionAtTimeIndex(i, speed);
                
                RSUs(i) = obj.findNearestRSUInLOS(xPos, yPos, rsuList, buildingLines);
            end
            obj.RSUs = RSUs;
            obj.RSUPlan = RSUs;
        end
        
        function [xPosExpected, yPosExpected] = getExpectedPositionAtTimeIndex(obj, timeIndex, speed)
            % returns expected position based on route, speed is in m/s 
            estimatedDistanceTraveled = timeIndex * speed;
            
            i = 1;
            while (obj.route(i,3) < estimatedDistanceTraveled) && (i < length(obj.route(:,3)))
                i = i + 1;
            end
            
            % Now we have the upper bound 
            p0 = [obj.route(i-1,1) obj.route(i-1,2)];
            p1 = [obj.route(i,1) obj.route(i,2)];
            v = p1 - p0;
            vunit = v/norm(v);
            pt = p0 + (estimatedDistanceTraveled - obj.route(i-1,3))*vunit;
            xPosExpected = pt(1);
            yPosExpected = pt(2);
        end
        
        function nearestRSUIndex = findNearestRSUInLOS(obj, xPos, yPos, rsuList, blockages)
            nearestRSUIndex = -1;
            distanceToClosestRSU = 9999;
            for i = 1:length(rsuList)               
                x2 = rsuList(i).x;
                y2 = rsuList(i).y;
                distance = abs(pdist([xPos,yPos;x2,y2], 'euclidean'));
                if distance < distanceToClosestRSU && hasLOS(xPos, yPos, x2, y2, blockages)
                    nearestRSUIndex = i;
                    distanceToClosestRSU = distance;
                end
            end
            
            %Think you could also do something like this > [[matchingSim.rsuList.x]' [matchingSim.rsuList.y]'] 
        end
        
        
        function handoverNum = getHandovers(obj)
            % A handover is defined as when an vehicle switches RSUs, we
            % can calculate this based on RSUs list 
            rsuIDs = obj.RSUs';
            handoverList = [NaN;cumsum(diff(rsuIDs)~=0)];
            handoverNum = handoverList(end);
        end
        
        function doesExist = doesVehicleExist(obj, timeStep)
            %Check if a vehicle exists at the given sim timestep
        end
        
        function segments = getSegments(obj, timeStep)
            % get the 4 line segments that make up the vehicle,assuming
            % horizontal
            if(obj.vehicleType==1) %truck
                length = 21.9456;
                width = 2.5908;
            else 
                %Passenger 
                length = 5;
                width = 1.6;
            end
            
            timeIndex = obj.getTimeIndex(timeStep);
            %position is of the front bumper
            x = obj.x(timeIndex);
            y = obj.y(timeIndex);
            
            %Assume horizontal movement to the left
            x1 = x - length;
            y1 = y + width/2;
            y2 = y - width/2;
            
            segments = [ ...
            x, y1, x, y2;
            x1, y1, x1, y2;
            x, y1, x1, y1;
            x, y2, x1, y2;
            ];
   
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
            fprintf("Matching at t=%d for vehicle %d\n", timeStep, obj.vehicleId);
        end
        
        function xPos = getXPosAtTime(obj, timeStep)
            timeIndex = find(obj.times == timeStep);
            xPos = obj.x(timeIndex);
        end
        
        function yPos = getYPosAtTime(obj, timeStep)
            timeIndex = find(obj.times == timeStep);
            yPos = obj.y(timeIndex);
        end
        
        function [] = plotXYPositions(obj)
            %Plots a given Vehicles XY positions in the simulation over
            %time
            vehiclePosPoints = [obj.x obj.y];
            plot(vehiclePosPoints(:,1), vehiclePosPoints(:,2), 'linestyle','--', 'marker','.', 'markersize',50);
        end
        
        function [] = plotRoute(obj)
            plot(obj.route(:,1), obj.route(:,2), 'linestyle','--', 'marker','.', 'markersize',50);
        end
        
        function obj = addMatchingCandidatesAtTime(obj, timeStep, simVehList)
            timeIndex = find(obj.times == timeStep);
            obj.matchingCandidates{timeIndex} = simVehList;
        end
        
        function timeIndex = getTimeIndex(obj, timeStep)
            timeIndex = find(obj.times == timeStep);
        end
        
        
    end
end

