function [PL] = u_nearest(vehiclesStruct, viewedVehicles, timeStep)
%U_NEAREST Matching utility function which ranks vehicles based on distance
%alone
%   Detailed explanation goes here

maxRank = length(vehiclesStruct.vehNode);
PL = zeros(length(vehiclesStruct.vehNode), length(vehiclesStruct.vehNode)); %index corresponds to vehicle ID

%We can use viewedVehicles to get candidates for the matching game, then
%rank them based on distance from vehiclesStruct

curViewedVehicles = viewedVehicles{timeStep};


%for each vehicle in this timestep
for i=1:length(curViewedVehicles(:,1))
    
    curVid = curViewedVehicles(i,1);%get vehicle id
    curDist = [];
    j = 2; %second index to iterate through vehicles which are in view
    
    while curViewedVehicles(i, j) ~= 0
        
        %get distance between vehicle we are evaluating and vehicle in view
        %[vid, distance between vid and curVid]
        curDist = [curDist; [curViewedVehicles(i,j), getVehNodeDistance(curVid, curViewedVehicles(i,j), vehiclesStruct, timeStep)]];
        j = j + 1;
    end
    
    %sort based on distance
    if ~isempty(curDist)
        curDist = sortrows(curDist, 2, 'ascend');
    end
    %Fill in PL accordingly
    if ~isempty(curDist)
        for k = 1:length(curDist(:,1))
            PL(curVid, curDist(k,1)) = maxRank - k + 1;        
        end
    end
    
    
end

end

