function [ linkUtil,dataExchanged, regDataAccess, emerVehLUtil, emerVehDataExchanged, emerRegDataAccess ] = linkUtilisation( vehicleStruct, calcDataRate, matchingSetProcessed )
%LINKUTILISATION This function calculates the link utilisation for all
%vehicles and emergency vehicles independently, the amount of sensor data
%exchanged per timeslot and the amount of regional sensor data that a
%vehicle has access to per timeslot
%
% I think we will have to modify this for our vehicleStructs, not sure what fields we need yet
%
% Input:
%   vehicleStruct        : This structure contains all the information about the vehicles
%                          such as the real position, the GPS position, the beacons, etc.
%   calcDataRate         : This is the calculated data rate for each
%                          vehicle. Based on the MCS chosen per timeslot
%   matchingSetProcessed : A matrix containing only the stable matches give
%                          from the matching algorithm
%
% Output:
%   linkUtil             : The link utilisation for all vehicles (given as
%                          the data exchanged over the maximum data rate on
%                          a timeslot
%   dataExchanged        : The amount of data exchanged per timeslot
%   regDataAccess        : The amount of regional sensor data tha each a
%                          vehicle has access per timeslot (after the
%                          matching game)
%   emerVehLUtil         : The link utilisation for the emergency vehicles
%                          (given as the data exchanged over the maximum
%                          data rate on a timeslot.
%   emerVehDataExchanged : The amount of data exchanged for the e-CAVs per timeslot
%   emerRegDataAccess    : The amount of regional sensor data tha each a
%                          emergency vehicle has access per timeslot
%                          (after the matching game)
%
% Copyright (c) 2016-2017, Ioannis Mavromatis

    % Reshape the generated data per timeslot for better usage later.
    time = round(0:vehicleStruct.simStep:vehicleStruct.simTime,3);
    for nodeIndex = 1:vehicleStruct.numNodes
        [ ~, index ] = ismember(vehicleStruct.vehNode(nodeIndex).V_TIME,time);
        dataGenPerTimeSlot(nodeIndex,1,index) = vehicleStruct.vehNode(nodeIndex).GENDATA;
    end
    
    % Find which vehicles are emergency vehicles
    counterEmer = 1;
    for nodeIndex = 1:vehicleStruct.numNodes
        if vehicleStruct.vehNode(nodeIndex).V_TYPE==1
            emerVeh(counterEmer) = nodeIndex;
            counterEmer = counterEmer + 1;
        end
    end
    
    % Reshape the data rate based on the timeslot length
    calcDataRate = calcDataRate*vehicleStruct.simStep;
    matchingTmp = matchingSetProcessed;
    
    for timeSlot = 1:length(matchingTmp)
        counter = 1;
        counterEmer = 1;
        linkUtilTmp = [];
        dataRateTmp = [];
        regDataAccessTmp = [];
        emerVehLUtilTmp = [];
        dataRateTmpEmer = [];
        regDataAccessEmerTmp = [];
        while ~isempty(matchingTmp{timeSlot}) % Check if there was any stable match during this timeslot
            firstPair = matchingTmp{timeSlot}(1,:); % Find the first pair of the matching game
            index = find(firstPair(1)==matchingTmp{timeSlot}(:,1)); % Take the first vehicle of the pair
            dataExchanged = calcDataRate(firstPair(1), matchingTmp{timeSlot}(index,2),timeSlot);
            dataExchanged = mean(dataExchanged); % Calculate the exchanged data
            dataToTransmit = dataGenPerTimeSlot(firstPair(1),1,timeSlot)*length(index) + sum(dataGenPerTimeSlot(matchingTmp{timeSlot}(index,2),1,timeSlot)); % Calculate the amount of data that will be transmitted during this timeslot
            regDataAccessSum = 0; % Calculate the amount of data tha a vehicle has access to
            for nodes = 1:length(matchingTmp{timeSlot}(index,2))
                regDataAccessSum = regDataAccessSum + vehicleStruct.vehNode(matchingTmp{timeSlot}(index(nodes),2)).REGDATA(timeSlot);
            end
            regDataAccessTmp(counter) = regDataAccessSum;
            linkUtilTmp(counter) = dataToTransmit/dataExchanged; % Calculate the link utilisation
            dataRateTmp(counter) = dataToTransmit; % The amount of exchanged data
            counter = counter + 1;
            
            % Do the same as above for the emergency vehicles only
            if ismember(firstPair(1),emerVeh)
                dataExchangedEmer = calcDataRate(firstPair(1), matchingTmp{timeSlot}(index,2),timeSlot);
                dataExchandataExchangedEmerged = mean(dataExchangedEmer);
                regDataAccessSum = 0;
                for nodes = 1:length(matchingTmp{timeSlot}(index,2))
                    regDataAccessSum = regDataAccessSum + vehicleStruct.vehNode(matchingTmp{timeSlot}(index(nodes),2)).REGDATA(timeSlot);
                end
                regDataAccessEmerTmp(counterEmer) = regDataAccessSum;
                dataToTransmit = dataGenPerTimeSlot(firstPair(1),1,timeSlot)*length(index) + sum(dataGenPerTimeSlot(matchingTmp{timeSlot}(index,2),1,timeSlot));
                emerVehLUtilTmp(counterEmer) = dataToTransmit/dataExchangedEmer;
                dataRateTmpEmer(counterEmer) = dataToTransmit;
                counterEmer = counterEmer + 1;
            end
            
            matchingTmp{timeSlot}(index,:) = [];
            index = find(firstPair(1)==matchingTmp{timeSlot}(:,2));
            matchingTmp{timeSlot}(index,:) = [];
        end
        
        % Find the mean values for all timeslots
        linkUtil(timeSlot) = mean(linkUtilTmp);
        dataExchanged(timeSlot) = mean(dataRateTmp);
        regDataAccess(timeSlot) = mean(regDataAccessTmp);
        
        emerVehLUtil(timeSlot) = mean(emerVehLUtilTmp);
        emerVehDataExchanged(timeSlot) = mean(dataRateTmpEmer);
        emerRegDataAccess(timeSlot) = mean(regDataAccessEmerTmp);
    end
end

