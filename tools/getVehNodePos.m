function [pos] = getVehNodePos(vid, vehiclesStruct, timeStep)
%GETVEHNODEPOS Gets a given vehicle Nodes position in the form [x y] for a given
%timestep
%   Detailed explanation goes here
    pos = [];
    idx = find(vehiclesStruct.vehNode(vid).time==timeStep);
    x = vehiclesStruct.vehNode(vid).x(idx);
    y = vehiclesStruct.vehNode(vid).y(idx);
    
    pos = [pos; x y];
end

