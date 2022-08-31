function [vehiclesStruct] = addViewedVehicles(viewedVehicles,vehiclesStruct)
%ADDVIEWEDVEHICLES Adds viewed vehicles to vehicle struct
%   Modifies the vehicle struct created from parseMobility. At each
%   timestep, every vehicle now has an array of vehicles it has within it's
%   view

for idx = 1:length(vehiclesStruct.vehNode)
    
    vehiclesStruct.vehNode(idx).inView = cell([length(vehiclesStruct.vehNode(idx).time) 1]);%init cell arr of size time
    
    for j = 1:length(vehiclesStruct.vehNode(idx).time)
        
        timeStep = vehiclesStruct.vehNode(idx).time(j);
        
        %Get the array of which vehicles are available at this timestep
        if ~isempty(viewedVehicles{timeStep})
            row = find(ismember(viewedVehicles{timeStep}(:,1), idx));
            vehicles = viewedVehicles{timeStep}(row,:);
            if ~isempty(vehicles)
                vehicles(1) = []; %delete first col (vehicleID)
                vehicles = nonzeros(vehicles);
            end

        else
            vehicles = [];
        end
        
        vehiclesStruct.vehNode(idx).inView{j} = vehicles;
               
    end
    
    
    
end



end

