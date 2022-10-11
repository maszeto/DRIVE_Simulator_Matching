function [ matchingSet, matchingSetProcessed ] = matchGame( matchingCapacity,utility )
%MATCHINGGAME Summary of this function goes here
%   Detailed explanation goes here

    [ sortedPL , indexPL ] = sort(utility,2,'descend'); %sorted PL is from the utility function 
    
    %3d matrix, idk why you need a z axis 
    [ xSize, ySize, zSize ] = size(indexPL);
    for x=1:xSize
        for y=1:ySize
            for z=1:zSize
                if sortedPL(x,y,z)==0
                    indexPL(x,y,z)=0;
                end
            end
        end
    end

    %for each timeslot 
    for timeSlot = 1:zSize 
%         fprintf('The timeslot is: %d\n',timeSlot);
        if ~all(all(sortedPL(:,:,timeSlot)==0))
            sortedPLTmp = sortedPL(:,:,timeSlot);
            indexPLTmp = indexPL(:,:,timeSlot);
            for i=1:ySize
                a_iElements{i} = {};
                b_iElements{i} = {};
            end
            a_i = zeros(length(sortedPLTmp),1);
            S = [ 0 0 ];
            posToRun = find(a_i<matchingCapacity);
            posToRun = posToRun';

            while ~isempty(posToRun)
                [ sortedPLTmp,a_i,matchingCapacity,S,indexPLTmp,a_iElements,b_iElements ] = matchingGame.sfPhase1( posToRun,sortedPLTmp,a_i,matchingCapacity,S,indexPLTmp,a_iElements,b_iElements );

                [ a_i,b_i,a_iElements,b_iElements,posToRun ] = matchingGame.findPosToRun( matchingCapacity,indexPLTmp,S );
            end
            
            S = S(2:end,:);
            if ~isempty(S)
                S = sortrows(S,1);
            end

            [ sortedPLTmp,a_i,matchingCapacity,S,indexPLTmp,a_iElements,b_iElements ] = matchingGame.sfPhase2(sortedPLTmp,a_i,matchingCapacity,S,indexPLTmp,a_iElements,b_iElements );

            if ~isempty(S)
                S = sortrows(S,1);
            end
            matchingSet{timeSlot} = S;
        else
            matchingSet{timeSlot} = [];
        end
        matchTmpArray = matchingSet{timeSlot};
        
        matchProcTmpArray = [];
        
        while ~isempty(matchTmpArray)
            pairToCheck = fliplr(matchTmpArray(1,:));
            
            [~,k] = ismember(pairToCheck,matchTmpArray,'rows');
            if k~=0
                matchProcTmpArray = [ matchProcTmpArray ; fliplr(pairToCheck) ; pairToCheck ];
                matchTmpArray(k,:) = [];
            end
            matchTmpArray(1,:) = [];
        end
        if ~isempty(matchProcTmpArray)
            matchProcTmpArray = sortrows(matchProcTmpArray,1);
        end    
        if ~isempty(matchingSet{timeSlot})
            matchingSetProcessed{timeSlot} = matchProcTmpArray;
        else
            matchingSetProcessed{timeSlot} = [];
        end
    end
end

