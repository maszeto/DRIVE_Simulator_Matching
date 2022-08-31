function [ vehicleStruct ] = regionalData(vehicleStruct,closeVehicles)
%REGIONALDATA This function calculates the generated regional sensor data
%for each vehicle. The amount of data is calculated with respect to the
%circular radius given at the beginning of the simulation and is the sum
%of all the data generated from all vehicles within this radius.
%
% Input:
%   vehicleStruct     : The struct containing all the vehicles
%   closeVehicles     : The cell array with the close vehicles (within
%                       radius)
%   vehicleStruct     : The vehicle struct is updated with the regional
%                       sensor data generated for each vehicle
%
% Copyright (c) 2016-2017, Ioannis Mavromatis

    time = round(0:vehicleStruct.simStep:vehicleStruct.simTime,3);

    % Shape generated data to a better format.
    for nodeIndex = 1:vehicleStruct.numNodes
        arrayPos = find(ismember(time,vehicleStruct.vehNode(nodeIndex).V_TIME)==1)';
        genDataArray(arrayPos,nodeIndex) = vehicleStruct.vehNode(nodeIndex).GENDATA;
    end

    % Calculate the regional sensor data for each vehicle. Two values are
    % calculated. The sum all all the data, and a normalised value.
    for i = 1:length(closeVehicles)-1
        regDataArray = [];
        counter = 1;
        for nodeIndex = 1:length(closeVehicles{i})
            indexData = closeVehicles{i}{nodeIndex};
            regData = sum(genDataArray(i,indexData));
            regDataArray(counter) = regData;
            counter = counter + 1;
            vehicleStruct.vehNode(nodeIndex).REGDATA(i) = regData;
        end
        regDataMaximum = max(regDataArray);
        for nodeIndex = 1:length(closeVehicles{i})
            vehicleStruct.vehNode(nodeIndex).REGDATANORM(i) = ...
                       vehicleStruct.vehNode(nodeIndex).REGDATA(i)/regDataMaximum;
        end
    end
    
    for nodeIndex = 1:vehicleStruct.numNodes
        vehicleStruct.vehNode(nodeIndex).REGDATA = vehicleStruct.vehNode(nodeIndex).REGDATA';
        vehicleStruct.vehNode(nodeIndex).REGDATANORM = vehicleStruct.vehNode(nodeIndex).REGDATANORM';
    end
end

