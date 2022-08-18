function [ utility,calcDataRate ] = fitnessFunction(vehicleStruct,linksLOS,closeVehicles,distanceCloseVeh,linkBudget, MCS, utilFuncMode,densityRadius)
%FITNESSFUNCTION Summary of this function goes here
%   I think this calculates the utility function? 

    time = round(0:vehicleStruct.simStep:vehicleStruct.simTime,3);
    for i = 1:length(time)-1
        for nodeIndex = 1:vehicleStruct.numNodes
            for nodeIndex2 = 1:vehicleStruct.numNodes
                if ismember([ nodeIndex nodeIndex2 ],linksLOS{i},'rows') || ismember([ nodeIndex2 nodeIndex ],linksLOS{i},'rows')

                    posIndex = find(closeVehicles{i}{nodeIndex}(:)==nodeIndex2);
                    if ~isempty(posIndex)
                        distance = distanceCloseVeh{i}{nodeIndex}(posIndex);
                        rxPower = linkBudget.TXPOWER + linkBudget.TXGAIN + linkBudget.RXGAIN - linkBudget.ALPHA - 10*linkBudget.PLEXPONENT*log10(distance) - 40*distance/1000;
                        snr = rxPower - linkBudget.NOISEPOWER;

                        modScheme = find(snr>MCS.LINKMARGIN);

                        [ ~, modIndex ] = max(modScheme);
                        MCS.DATARATE(modIndex+1);
                        dataRate = MCS.DATARATE(8-modIndex);
                        utilDataRate = dataRate/MCS.DATARATE(1);
                        
                        if utilFuncMode == 1 % distance
                            utility(nodeIndex,nodeIndex2,i) =...
                                utilDataRate;
                            calcDataRate(nodeIndex,nodeIndex2,i) = dataRate;
                            
                        elseif utilFuncMode == 2 % timeliness/vehicle type/regional data
                            
                            % Second part of timeliness
                            timeliness = vehicleStruct.vehNode(nodeIndex).TIMELINESS(nodeIndex2).VEHICLE(i);
                            distFun = (densityRadius - distance)/densityRadius;
                            sumBoth = timeliness + distFun;
                            w1 = timeliness/sumBoth;
                            w2 = distFun/sumBoth;
                            directionalityDistance = timeliness*w1 + distFun*w2;
                            
                            utility(nodeIndex,nodeIndex2,i) =...
                                vehicleStruct.vehNode(nodeIndex2).REGDATANORM(i) *...
                                vehicleStruct.vehNode(nodeIndex2).V_TYPE *...
                                directionalityDistance *...
                                utilDataRate;                               
                                
                            calcDataRate(nodeIndex,nodeIndex2,i) = dataRate;
                        end
                    else
                        utility(nodeIndex,nodeIndex2,i) = 0;
                        calcDataRate(nodeIndex,nodeIndex2,i) = 0;
                    end
                else
                    utility(nodeIndex,nodeIndex2,i) = 0;
                    calcDataRate(nodeIndex,nodeIndex2,i) = 0;
                end
            end
        end
    end
    
end

