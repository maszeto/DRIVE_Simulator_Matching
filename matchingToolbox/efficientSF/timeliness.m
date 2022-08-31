function [ vehicleStruct ] = timeliness(vehicleStruct)
% TIMELINESS 
%
% Input:
%       vehicleStruct with the following fields:
%           
%
% Output:
%   vehicleStruct :   This structure is updated with the timeliness information
%                     based on their moving direction
%
% Copyright (c) 2016-2017, Ioannis Mavromatis

    normalizeDeg=@(x)(-mod(-x+180,360)+180);
% Put vehicle motion dynamics to proper form to calculate the timeliness
    for nodeIndex = 1:vehicleStruct.numNodes
        valuesTmp = vehicleStruct.vehNode(nodeIndex).VEHDYNAMICS;
        indexForValues = valuesTmp>180;
        valuesTmp = (valuesTmp-360).*indexForValues + valuesTmp.*~indexForValues;
%         valuesTmp = deg2rad(valuesTmp);
        vehicleStruct.vehNode(nodeIndex).VEHDYNAMICS = valuesTmp;
    end
    
    maxLength = length(vehicleStruct.vehNode(1).V_TIME);
    for nodeIndex = 2:vehicleStruct.numNodes
        if length(vehicleStruct.vehNode(nodeIndex).V_TIME) > maxLength 
            maxLength = length(vehicleStruct.vehNode(nodeIndex).V_TIME);
        end
    end
    
    for nodeIndex = 1:vehicleStruct.numNodes
        for nodeIndex2 = nodeIndex:vehicleStruct.numNodes
            if nodeIndex ~= nodeIndex2
                index = ismember(vehicleStruct.vehNode(nodeIndex).V_TIME,vehicleStruct.vehNode(nodeIndex2).V_TIME);
                a = vehicleStruct.vehNode(nodeIndex).VEHDYNAMICS(index);
                b = vehicleStruct.vehNode(nodeIndex2).VEHDYNAMICS;
                absDiffDeg = abs(normalizeDeg(normalizeDeg(a)-normalizeDeg(b)));
                timelinessTmp = (180 - absDiffDeg)/180;
                
%                 timelinessTmp = abs((abs(vehicleStruct.vehNode(nodeIndex).VEHDYNAMICS(index)) -...
%                      abs(vehicleStruct.vehNode(nodeIndex2).VEHDYNAMICS)))/180;
                timelinessTmp = padarray(timelinessTmp,[maxLength-length(timelinessTmp)],0,'pre');
                
                vehicleStruct.vehNode(nodeIndex).TIMELINESS(nodeIndex2).VEHICLE = timelinessTmp;
                vehicleStruct.vehNode(nodeIndex2).TIMELINESS(nodeIndex).VEHICLE = timelinessTmp;
            else
                timelinessTmp = zeros(length(vehicleStruct.vehNode(nodeIndex).VEHDYNAMICS),1);
                timelinessTmp =  padarray(timelinessTmp,[maxLength-length(timelinessTmp)],0,'pre');
                vehicleStruct.vehNode(nodeIndex).TIMELINESS(nodeIndex2).VEHICLE = timelinessTmp;
                vehicleStruct.vehNode(nodeIndex2).TIMELINESS(nodeIndex).VEHICLE = timelinessTmp;
        
            end
        end  
    end
end

