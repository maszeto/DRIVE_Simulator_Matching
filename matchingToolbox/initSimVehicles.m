function [simVehiclesList] = initSimVehicles(vehiclesStruct)
%INITSIMVEHICLES Creates list of vehicle Objects in simulation
%   Detailed explanation goes here
    numVehicles = length(vehiclesStruct.vehNode)
    simVehiclesList(numVehicles) = SimVehicle(numVehicles);
    
    for i = 1:vehiclesStruct.vehNode
        
    end
    



end

