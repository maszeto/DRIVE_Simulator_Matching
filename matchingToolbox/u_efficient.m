function [PL] = u_efficient(vehiclesStruct, viewedVehicles, timeStep)
%U_EFFECIENT The utility function used in "Effecient V2V Communication
%Scheme"
%   Detailed explanation goes here

totalVehiclesNum = length(vehiclesStruct.vehNode);
maxRank = totalVehiclesNum;
PL = zeros(totalVehiclesNum, totalVehiclesNum);

curViewedVehicles = viewedVehicles{timeStep};

for i=1:length(curViewedVehicles(:,1))
    
    curVid = curViewedVehicles(i,1);%get vehicle id
    vehicleIdList = [vehiclesStruct.vehNode.id];
    vehicleIndex = find(vehicleIdList == curVid) ; 
    curVehicleIndex = 2;
    
    
    j = 2; %second index to iterate through vehicles which are in view
    
    while curViewedVehicles(i, j) ~= 0
        
        %get distance between vehicle we are evaluating and vehicle in view
        %[vid, distance between vid and curVid]
        %if for some reason, vehicle can see that vehicle, but it is not in
        %the simulation yet
        if ~isempty(getVehNodeDistance(curVid, curViewedVehicles(i,j), vehiclesStruct, timeStep))
            curDist = [curDist; [curViewedVehicles(i,j), getVehNodeDistance(curVid, curViewedVehicles(i,j), vehiclesStruct, timeStep)]];
        end
        
        j = j + 1;
    end
    
    %sort based on distance
    if ~isempty(curDist)
        curDist = sortrows(curDist, 2, 'ascend');
    end

    if ~isempty(curDist)
        for k = 1:length(curDist(:,1))
            vehicleIndex2 = find(vehicleIdList == curDist(k,1));
            PL(vehicleIndex, vehicleIndex2) = maxRank - k + 1;        
        end
    end
    
    
end


end

