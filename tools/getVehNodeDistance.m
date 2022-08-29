function [distance] = getVehNodeDistance(vid1, vid2, vehiclesStruct,timeStep)
%GETVEHNODEDISTANCE Gets distance between 2 vehicle Nodes
%   Detailed explanation goes here
    %we need to find the index that correlates to the timestep for the
    %given time
    idx1 = find(vehiclesStruct.vehNode(vid1).time==timeStep);
    idx2 = find(vehiclesStruct.vehNode(vid2).time==timeStep);
        
    x1 = vehiclesStruct.vehNode(vid1).x(idx1);
    y1 = vehiclesStruct.vehNode(vid1).y(idx1);
    x2 = vehiclesStruct.vehNode(vid2).x(idx2);
    y2 = vehiclesStruct.vehNode(vid2).y(idx2);
    distance = abs(pdist([x1,y1;x2,y2], 'euclidean'));
end

