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
    end
end

