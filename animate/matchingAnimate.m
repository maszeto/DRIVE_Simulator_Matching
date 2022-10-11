function printAnimate(sumo,vehicle,pedestrian,outputMap, xyLinks)
%PRINTANIMATE This function interpolates the vehicle and pedestrian
%  positions at first, plots the map in a figure and prints all the mobility
%  traces overlaid above the map. The mobility traces are update per
%  timestep interval.
%
%  Input  :
%     vehicle     : A structure containing all the information about the
%                   vehicles (type, position at each timeslot, etc.)
%     pedestrian  : A structure containing all the information about the
%                   pedestrians (type, position at each timeslot, etc.)
%     outputMap   : The map structure extracted from the map file or loaded
%                   from the preprocessed folder
%
% Copyright (c) 2019-2020, Ioannis Mavromatis
% email: ioan.mavromatis@bristol.ac.uk    
% email: ioannis.mavromatis@toshiba-bril.com    

    global SIMULATOR
    global MATCHING
    
    if SIMULATOR.map == 0
        mapPrint(outputMap);
        return
    end
    
    timeSteps = 0:vehicle.simStep:vehicle.simTime;
    
    % The linear position interpolation
    vNode = positionInterpolation(vehicle);
    pNode = positionInterpolation(pedestrian);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    clf
    mapPrint(outputMap);
    alpha(0.5)
    
    hold on;
    % Generate the animation of the different links for all vehicles and
    % pedestrians
    [vNodePos,typePos] = generateLineObjects(vNode,vehicle);
    pNodePos = generateLineObjects(pNode,pedestrian);
    
    % The information of the matlab figure
    title('V2V mmWave Simulation')
    xlabel('X (meters)');
    ylabel('Y (meters)');
    axlim = get(gca, 'XLim');    % Get XLim Vector
    aylim = get(gca, 'YLim');    % Get YLim Vector 
    x_txt = min(axlim) + 120;    % Set x-Coordinate Of Text Object
    y_txt = max(aylim) + 20;     % Set y-Coordinate Of Text Object
    ht = text(x_txt, y_txt, cat(2,'Time (sec) = 0'));

    legendString = {};
    for i = 1:length(vehicle.type)
        legendString{i} = sumo.vehicleTypes{vehicle.type(i)};
    end
    legendString{i+1} = 'Pedestrian';
    legend(legendString,'Location','eastoutside')
    
    %Plot the Building IDs
    if MATCHING.verboseMap == 1
        for i = 1:length(outputMap.buildingIncentre)
            %label buildings, 3rd column is x and 2nd column is y for some
            %reason
            text(outputMap.buildingIncentre(i,3),outputMap.buildingIncentre(i,2),int2str(outputMap.buildingIncentre(i,1)));
        end
    end
    
    
    % Update all the vehicle and pedestrian positions per timeslot
    hold off;
    timeIndex = 1;
    
    while timeIndex <= length(timeSteps)-1
        if exist('linkLines', 'var') == 1
            delete(linkLines);
        end
        
        t = timeSteps(timeIndex);
        set(ht,'String',cat(2,'Time (sec) = ',num2str(t,4)));
        if ~isempty(fieldnames(vehicle))
            for nodeIndex = 1:length(vehicle.vehNode)
                vid = vehicle.vehNode(nodeIndex).id;
                set(vNodePos(nodeIndex),'XData',vNode(nodeIndex).v_x(timeIndex),'YData',vNode(nodeIndex).v_y(timeIndex));
                
                if MATCHING.verboseMap == 1
                    if (exist('vehLabels','var') == 0)
                        vehLabels(nodeIndex) = text(vNode(nodeIndex).v_x(timeIndex),vNode(nodeIndex).v_y(timeIndex),int2str(vid));
                    elseif (length(vehLabels) < nodeIndex)
                        vehLabels(nodeIndex) = text(vNode(nodeIndex).v_x(timeIndex),vNode(nodeIndex).v_y(timeIndex),int2str(vid));   
                    else
                        set(vehLabels(nodeIndex),'Position',[vNode(nodeIndex).v_x(timeIndex), vNode(nodeIndex).v_y(timeIndex), 0]);
                    end
                end
            end
            
        end
        if MATCHING.verboseMap == 1
            if ~isempty(xyLinks{timeIndex})
                %plot matches 
                matchLinks = xyLinks{timeIndex};
                X1 = matchLinks(:,1)';
                X2 = matchLinks(:, 3)';
                Y1 = matchLinks(:, 2)';
                Y2 = matchLinks(:, 4)';
                linkLines = line([X1; X2], [Y1; Y2]); 
                for lidx=1:length(linkLines)
                    set(get(get(linkLines(lidx),'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
                end
            end
        end
        if ~isempty(fieldnames(pedestrian))
            for nodeIndex = 1:length(pedestrian.pedNode)
                set(pNodePos(nodeIndex),'XData',pNode(nodeIndex).v_x(timeIndex),'YData',pNode(nodeIndex).v_y(timeIndex));
            end
        end
        
        drawnow;

        
        if MATCHING.interactive == 1
            prompt = "Press enter to continue or enter timestep to jump to: ";
            txt = input(prompt);
            if ~isempty(txt)
                if txt < length(timeSteps)-1
                    timeIndex = txt + 1;
                    
                end
            else
                timeIndex = timeIndex + 1;
            end
        else
            timeIndex = timeIndex + 1;
        end
        
    end
end
