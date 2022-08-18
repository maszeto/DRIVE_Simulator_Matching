function [ sortedPL,a_i,plCap,S,indexPL,a_iElements,b_iElements ] = sfPhase1( posToRun,sortedPL,a_i,plCap,S,indexPL,a_iElements,b_iElements )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%{
    Desc (From paper):
    During this phase, a sequence of bids takes
    place from one vehicle to the others. These bids are used
    to construct a set S that is an initial list of the potential
    matching pairs as well as to identify and delete pairs that
    cannot belong in a stable matching. The outcome of Phase 1 is a
    reduced Pi for each vi and an increased set S of potential
    matching pairs. 
%}
% Input:
%   posToRun             : Which vehicle (player) we are evaluating
%   sortedPL             : Sorted PL of all players in the matching game
%   a_i                  : 
%   plCap                :
%   S                    :
%   indexPL              : 
%   a_iElements          :
%   b_iElements          :
% 
% Output:
%   sortedPL             : Sorted PL of all players in the matching game
%   a_i                  : 
%   plCap                :
%   S                    :
%   indexPL              : 
%   a_iElements          :
%   b_iElements          :


for i = posToRun
    counter = 1;
    if any(indexPL(i,:))~=0 % If any value is zero in the preference list at i
        while a_i(i,1) < min( plCap(i), nnz(indexPL(i,:)) )

            while ismember([i indexPL(i,counter)],S,'rows')
                counter = counter + 1;
            end
            S = [ S ; i indexPL(i,counter) ];
            
            for j = 1:length(sortedPL)
                a_i(j,1) = sum(S(:,1)==j);
                b_i(j,1) = sum(S(:,2)==j);

                aPos = find(S(:,1)==j);
                bPos = find(S(:,2)==j);
                a_iElements{j} = S(aPos,2);
                b_iElements{j} = S(bPos,1);
            end

            if indexPL(i,counter) ~= 0
                if b_i(indexPL(i,counter)) == plCap(indexPL(i,counter))
                    startValue = find(indexPL(indexPL(i,counter),:)==i)+1;
                    endValue =  find(sortedPL(indexPL(i,counter),:)==0,1,'first')-1;

                    x_l = indexPL(indexPL(i,counter),startValue:endValue);

                    indexForX_L = [];
                    for checkPairs = 1:length(x_l)
                        pairToCheck = [ x_l(checkPairs) indexPL(i,counter) ];
                        [~,k] = ismember(pairToCheck,S,'rows');
                        if k~=0 % something to be done here
                            startValue = startValue + 1;
                            indexForX_L = [ indexForX_L checkPairs ];
%                             x_l = [ x_l(1:checkPairs-1) x_l(checkPairs+1:end) ];
                        end
                    end
                    x_l(indexForX_L) = [];

                    pairToCheck = [ indexPL(indexPL(i,counter),startValue:endValue) indexPL(i,counter) ];
                    
                    if length(pairToCheck)==2
                        [~,k] = ismember(pairToCheck,S,'rows');
                    else 
                        k = 0;
                    end
                    if k==0
                        indexPL(indexPL(i,counter),startValue:endValue) = 0;
                        sortedPL(indexPL(i,counter),startValue:endValue) = 0;  

                        for temp=1:length(x_l)
                            pos = find(indexPL(x_l(temp),:)==indexPL(i,counter));
                            indexPL(x_l(temp),pos) = 0;
                            sortedPL(x_l(temp),pos) = 0;
                        end
                    end
                    
                    [ indexPL, sortedPL ] = sortToRemoveZeros( indexPL, sortedPL );
                
                elseif b_i(indexPL(i,counter)) > plCap(indexPL(i,counter))   
                    startValue = find(indexPL(indexPL(i,counter),:)==i)+1;
                    endValue =  find(sortedPL(indexPL(i,counter),:)==0,1,'first')-1;

                    x_l = indexPL(indexPL(i,counter),startValue:endValue);
                    
                    for checkPairs = 1:length(x_l)
                        pairToCheck = [ x_l(checkPairs) indexPL(i,counter) ];
                        [~,k] = ismember(pairToCheck,S,'rows');
                        if k~=0
                            S(k,:)=[];
                        end
                    end
                    indexPL(indexPL(i,counter),startValue:endValue) = 0;
                    sortedPL(indexPL(i,counter),startValue:endValue) = 0;  
                    for temp=1:length(x_l)
                        pos = find(indexPL(x_l(temp),:)==indexPL(i,counter));
                        indexPL(x_l(temp),pos) = 0;
                        sortedPL(x_l(temp),pos) = 0;
                    end
                    
                    [ indexPL, sortedPL ] = sortToRemoveZeros( indexPL, sortedPL );
                end
            end
            
        end
        counter = counter + 1;
    end
end


end

