function [pos] = getVehNodePos(vid, vehiclesStruct, timeStep)
%GETVEHNODEPOS Gets a given vehicle Nodes position in the form [x y] for a given
%timestep
%   Detailed explanation goes here
    pos = [];
    vehicleIdList = [vehiclesStruct.vehNode.id];
    vehicleIndex = find(vehicleIdList == vid) ;
    
    timeIndex = find(vehiclesStruct.vehNode(vehicleIndex).time==timeStep);
    
    x = vehiclesStruct.vehNode(vehicleIndex).x(timeIndex);
    y = vehiclesStruct.vehNode(vehicleIndex).y(timeIndex);
    
    pos = [pos; x y];
end

