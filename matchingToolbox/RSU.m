classdef RSU
    %RSU Road side unit class
    %   Detailed explanation goes here
    
    properties
        id          % Basestation ID
        index       % Index in "potential pos" array
        x           % X position
        y           % Y position
        height      % Height I think
    end
    
    methods
        function obj = RSU(id)
        end
        function obj = initRSU(obj, id,index,x,y,height)
            %RSU Construct an instance of this class
            %   Detailed explanation goes here
            obj.id = id;
            obj.index = index;
            obj.x = x;
            obj.y = y;
            obj.height = height;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

