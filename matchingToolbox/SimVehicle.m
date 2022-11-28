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
        blockageTimes       % Unexpected blockage time indexes, to track what time index there was an unexpected blockage
        datarate            % achieved datarate at each timestep
        rss                 % The RSS at each timestep
        adjustedDataRate    % Calculated after taking to account handover overheads etc. 
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
            obj.datarate = zeros(1, length(obj.times));
            obj.rss = zeros(1, length(obj.times));
            
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
        
        function obj = createRSUConnectionScheduleGreedy(obj,rsuList)
            RSUs = length(obj.times);
            
            for i = 1:length(obj.times)
                %For each timestep 
                xPos = obj.x(i);
                yPos = obj.y(i);
                RSUs(i) = obj.getNearestRSUWithGreedy(xPos, yPos, rsuList);
            end
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
        
        function [selectedRSUIndex, obj] = selectRSUAtTime(obj, timeStep, rsuList, blockages)
            timeIndex = obj.getTimeIndex(timeStep);
            antennaPos = obj.getAntennaPosAtTime(timeStep);
            xPos = antennaPos(1);
            yPos = antennaPos(2);
            
            
            if timeIndex == 1 || obj.RSUs(timeIndex-1)==-1
                selectedRSUIndex = obj.findNearestRSUInLOS(xPos, yPos, rsuList, blockages);
                return;
            end
            
            if(obj.vehicleType == 1)
                %Truck, so no chance for blockage
                selectedRSUIndex = obj.getNearestRSUWithGreedy(xPos, yPos, rsuList);
                return;
            end
            
            %prioritize current connection 
            prevRSU = obj.RSUs(timeIndex-1);
            rsuX = rsuList(prevRSU).x;
            rsuY = rsuList(prevRSU).y;
            
            greedyRSUIndex = obj.getNearestRSUWithGreedy(xPos, yPos, rsuList);
            if ~obj.checkIfGreedyFailed(xPos, yPos, greedyRSUIndex, rsuList, blockages)
                selectedRSUIndex = greedyRSUIndex;
            else
                selectedRSUIndex = obj.findNearestRSUInLOS(xPos, yPos, rsuList, blockages);
            end
            
            if ~hasLOS(xPos, yPos, rsuX, rsuY, blockages) && prevRSU == greedyRSUIndex && obj.vehicleType==2
                obj.blockageTimes = [obj.blockageTimes, timeIndex];
                fprintf("t=%f v=%d greedy connection blocked\n", timeStep, obj.vehicleId);
            end
        

        end
        
        function nearestRSUIndex = getNearestRSUWithGreedy(obj, xPos, yPos, rsuList)
            nearestRSUIndex = -1;
            distanceToClosestRSU = 9999;
            maxTxDistance = 200;
            for i = 1:length(rsuList)               
                x2 = rsuList(i).x;
                y2 = rsuList(i).y;
                distance = abs(pdist([xPos,yPos;x2,y2], 'euclidean'));
                if distance < distanceToClosestRSU && distance < maxTxDistance
                        nearestRSUIndex = rsuList(i).index;
                        distanceToClosestRSU = distance;
                end
            end
        end

        function nearestRSUIndex = getNearestRSUWithGreedyRSUPairs(obj, xPos, yPos, rsuList)
            nearestRSUIndex = -1;
            distanceToClosestRSU = 9999;
            maxTxDistance = 200;
            for i = 1:length(rsuList)               
                x2 = rsuList(i).x;
                y2 = rsuList(i).y;
                distance = abs(pdist([xPos,yPos;x2,y2], 'euclidean'));
                isEven = mod(i,2) == 0;
                if distance < distanceToClosestRSU && ~isEven && distance < maxTxDistance
                        nearestRSUIndex = i;
                        distanceToClosestRSU = distance;
                end
            end
        end
        
        function nearestRSUIndex = getNearestRSUWithGreedyIgnore(obj, xPos, yPos, rsuList, badRSUs)
            nearestRSUIndex = -1;
            distanceToClosestRSU = 9999;
            maxTxDistance = 200;
            for i = 1:length(rsuList)               
                x2 = rsuList(i).x;
                y2 = rsuList(i).y;
                distance = abs(pdist([xPos,yPos;x2,y2], 'euclidean'));
                if distance < distanceToClosestRSU && distance < maxTxDistance && isempty(find(badRSUs == i))
                        nearestRSUIndex = rsuList(i).index;
                        distanceToClosestRSU = distance;
                end
            end
        end
        
        function greedyFailed = checkIfGreedyFailed(obj, xPos, yPos, greedyRSUIndex, rsuList, blockages)
            if greedyRSUIndex == -1
                greedyFailed = 1;
            elseif isempty(rsuList(greedyRSUIndex))
                greedyFailed = 1;
            else
                rsuX = rsuList(greedyRSUIndex).x;
                rsuY = rsuList(greedyRSUIndex).y;
                distance =  pdist([xPos,yPos;rsuX,rsuY], 'euclidean');
                greedyFailed = ~hasLOS(xPos, yPos, rsuX, rsuY, blockages) || distance > 200;
            end
        end
         
        function nearestRSUIndex = findNearestRSUInLOS(obj, xPos, yPos, rsuList, blockages)
            nearestRSUIndex = -1;
            distanceToClosestRSU = 9999;
            maxTxDistance = 200;
            for i = 1:length(rsuList)               
                x2 = rsuList(i).x;
                y2 = rsuList(i).y;
                distance = abs(pdist([xPos,yPos;x2,y2], 'euclidean'));
                if distance < distanceToClosestRSU && distance < maxTxDistance
                    if (hasLOS(xPos, yPos, x2, y2, blockages) || obj.vehicleType==1)
                        nearestRSUIndex = i;
                        distanceToClosestRSU = distance;
                    end
                end
            end
            
            %Think you could also do something like this > [[matchingSim.rsuList.x]' [matchingSim.rsuList.y]'] 
        end
        
        function antennaPos = getAntennaPosAtTime(obj, timeStep)
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
            frontX = obj.x(timeIndex);
            frontY = obj.y(timeIndex);
            
            if (timeIndex > 1)
                prevX = obj.x(timeIndex-1);
                prevY = obj.y(timeIndex-1);
                cur = [frontX, frontY];
                prev = [prevX, prevY];
                v = prev - cur;
                vunit = v/norm(v);
                antennaPos = cur + (length/2)*vunit;
            else
                antennaPos = [frontX, frontY];
            end
            
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
            % horizontal moving to the left (negative x dir) 
           
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
            frontX = obj.x(timeIndex);
            frontY = obj.y(timeIndex);
            
            if (timeIndex > 1)
                prevX = obj.x(timeIndex-1);
                prevY = obj.y(timeIndex-1);
                cur = [frontX, frontY];
                prev = [prevX, prevY];
                v = prev - cur;
                vunit = v/norm(v);
                rear = cur + length*vunit; %since we are moving in neg X dir
                
                vunit2 = [vunit(2), -vunit(1)];%perpendicular unit vector
                p0 = cur + (width/2)*vunit2;
                p1 = cur - (width/2)*vunit2;
                p2 = rear + (width/2)*vunit2;
                p3 = rear - (width/2)*vunit2;
                segments = [p0,p1;p0,p2;p3,p1;p3,p2];
            else
                %Assume horizontal movement to the left
                x1 = frontX + length;
                y1 = frontY + width/2;
                y2 = frontY - width/2;

                segments = [ ...
                frontX, y1, frontX, y2;
                x1, y1, x1, y2;
                frontX, y1, x1, y1;
                frontX, y2, x1, y2;
                ];
            end

   
        end
        
        function rsuID = getRSUIDAtTime(obj, timeStep)
            timeIndex = find(obj.times == timeStep);
            rsuID = obj.RSUs(timeIndex);
        end
        
        function exactPos = getPosNext(obj, timeStep, posCount)
            %Returns the vehicles exact position for the next steps timesteps
            exactPos = [];
            timeIndex = obj.getTimeIndex(timeStep);
            maxTimestep = length(obj.x);
            if timeIndex + posCount > maxTimestep
                %return as much as possible 
                posCount = maxTimestep - timeIndex;
            end
            
            curPos = 1;
            
            while curPos <= posCount
                exactPos = [exactPos; obj.x(timeIndex + curPos), obj.y(timeIndex + curPos)];
                curPos = curPos + 1;
            end 
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

