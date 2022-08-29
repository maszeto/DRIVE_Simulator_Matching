function [vehicleStruct] = getVehicleStruct(vehiclesStruct,timeStep)
%GETVEHICLESTRUCT This gets the vehicle struct for a certain timestep,
%using vehiclesStruct
%This gets the vehicle struct for a certain timestep,
%using vehiclesStruct, which contains all data about every vehicle for the
%entire sim. The matching game takes vehicleStruct as inputs at each
%timestep.

vehicleStruct = {};

vehicleStruct.type = vehiclesStruct.type;
vehicleStruct.simTime = timeStep;
vehicleStruct.missingIDs = vehiclesStruct.missingIDs;
vehicleStruct.simStep = vehiclesStruct.simStep;

for i=1:length(vehiclesStruct.vehNode)
    
    %Check which vehicles are in this sim at this timestep and take a
    %snapshot of their data
    if ismember(timeStep, vehiclesStruct.vehNode(i).time, "rows")
        vehicleStruct.vehNode(i).ID = i;
        
        dataIndex = find(vehiclesStruct.vehNode(i).time == timeStep);
        
        vehicleStruct.vehNode(i).time = vehiclesStruct.vehNode(i).time(dataIndex);
        vehicleStruct.vehNode(i).x = vehiclesStruct.vehNode(i).x(dataIndex);
        vehicleStruct.vehNode(i).y = vehiclesStruct.vehNode(i).y(dataIndex);
        vehicleStruct.vehNode(i).vehicleType = vehiclesStruct.vehNode(i).vehicleType;
        vehicleStruct.vehNode(i).inView = vehiclesStruct.vehNode(i).inView(dataIndex);
    end
    
    
end

vehicleStruct.numNodes = length(vehicleStruct.vehNode);%Needed for sf stuff


end

