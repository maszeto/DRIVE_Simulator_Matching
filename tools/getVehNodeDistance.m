function [distance] = getVehNodeDistance(vid1, vid2, vehiclesStruct,timeStep)
%GETVEHNODEDISTANCE Gets distance between 2 vehicle Nodes
%   Detailed explanation goes here
    %we need to find the index that correlates to the timestep for the
    %given time
    vehicleIdList = [vehiclesStruct.vehNode.id];
    vehicleIdx1 = find(vehicleIdList == vid1);
    vehicleIdx2 = find(vehicleIdList == vid2);
    
    %Below is no longer true, delete later
    %Vehicle ID may be in view, but the id will not exist in vehiclesStruct
    %because of how it is created in parseMobility
    if (isempty(vehicleIdx1) || isempty(vehicleIdx2))
        distance = [];
    else
        idx1 = find(vehiclesStruct.vehNode(vehicleIdx1).time==timeStep);
        idx2 = find(vehiclesStruct.vehNode(vehicleIdx2).time==timeStep);

        x1 = vehiclesStruct.vehNode(vehicleIdx1).x(idx1);
        y1 = vehiclesStruct.vehNode(vehicleIdx1).y(idx1);
        x2 = vehiclesStruct.vehNode(vehicleIdx2).x(idx2);
        y2 = vehiclesStruct.vehNode(vehicleIdx2).y(idx2);
        distance = abs(pdist([x1,y1;x2,y2], 'euclidean'));
    end

end

