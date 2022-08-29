function [distance] = getVehNodeDistance(vid1, vid2, vehiclesStruct,timeStep)
%GETVEHNODEDISTANCE Gets distance between 2 vehicle Nodes
%   Detailed explanation goes here
    x1 = vehiclesStruct.vehNode(vid1).x(timeStep);
    y1 = vehiclesStruct.vehNode(vid1).y(timeStep);
    x2 = vehiclesStruct.vehNode(vid2).x(timeStep);
    y2 = vehiclesStruct.vehNode(vid2).y(timeStep);
    distance = abs(pdist([x1,y1;x2,y2], 'euclidean'));
end

