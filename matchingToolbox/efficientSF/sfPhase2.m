function [ sortedPL,a_i,plCap,S,indexPL,a_iElements,b_iElements ] = sfPhase2( sortedPL,a_i,plCap,S,indexPL,a_iElements,b_iElements )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    sum = 0;
    for i = 1:length(indexPL)
        d(i,1) = min(plCap(i),nnz(indexPL(i,:)));
        sum = sum + min(plCap(i),nnz(indexPL(i,:)));
    end

    if mod(sum,2)==1
        S = []; % return and report instance unsolvable
    else
        [sortLong, indexLong ] = sortLongFun(d,indexPL);
        stableMatchingFound = 1;
        counterToStop = 0;
        while all(sortLong==1) && length(indexLong)>1
            
            
            for i=1:length(indexPL)
                lengthPL(i,1) = nnz(indexPL(i,:));
            end
            [lengthPLSorted indexlengthPL ] = sort(lengthPL,'descend');
            
            breakLoop = 0;
            stableMatchingFound = 0;
            counter = 1;
            while ~stableMatchingFound && counter<=length(indexlengthPL)
                qMatrix = [];
                pMatrix = [];
                indexPLTmp = indexPL;
                sortedPLTmp = sortedPL;
                nextPlayer = indexlengthPL(counter);

                while ~ismember(nextPlayer,pMatrix) || length(pMatrix)<=1

                    pMatrix = [ pMatrix nextPlayer];

                    if length(pMatrix)>1
                        pairToCheck = [ pMatrix(end) qMatrix(end) ];
                        [~,k] = ismember(pairToCheck,S,'rows');
                        if k~=0
                            S(k,:)=[];
                        end
                        pos = find(indexPLTmp(pairToCheck(1),:)==pairToCheck(2));
                        indexPLTmp(pairToCheck(1),pos) = 0;
                        sortedPLTmp(pairToCheck(1),pos) = 0;
                        pos = find(indexPLTmp(pairToCheck(2),:)==pairToCheck(1));
                        indexPLTmp(pairToCheck(2),pos) = 0;
                        sortedPLTmp(pairToCheck(2),pos) = 0;
                        [ indexPLTmp, sortedPLTmp ] = sortToRemoveZeros( indexPLTmp, sortedPLTmp );
                    end
               
                    qMatrix = [ qMatrix indexPLTmp(nextPlayer,2) ];
                    if qMatrix(end) == 0
%                         nextPlayer = indexPLTmp(qMatrix(end),find(indexPLTmp(qMatrix(end),:)==0,1,'first')-1);
                        breakLoop = 1;
                        break;
%                         qMatrix(end) = indexPLTmp(nextPlayer,1);
                    end
                    if find(indexPLTmp(qMatrix(end),:)==0,1,'first')==1
                        breakLoop = 1;
                        break;
                    end    
                    nextPlayer = indexPLTmp(qMatrix(end),find(indexPLTmp(qMatrix(end),:)==0,1,'first')-1);
                        
                end
                
                [sortLong, indexLong ] = sortLongFun(d,indexPLTmp);
                if any(sortLong~=1) || length(indexLong)==0 || breakLoop == 1
                    counter = counter + 1;
                    breakLoop = 0;
                else
                    stableMatchingFound = 1;
                end
                
            end
            
            indexPL = indexPLTmp;
            sortedPL = sortedPLTmp;
            
            [ a_i,b_i,a_iElements,b_iElements,posToRun ] = findPosToRun( plCap,indexPL,S );
            [ sortedPL,a_i,plCap,S,indexPL,a_iElements,b_iElements ] = sfPhase1( posToRun,sortedPL,a_i,plCap,S,indexPL,a_iElements,b_iElements );
            S = sortrows(S,1);
            counterToStop = counterToStop + 1;
            if counterToStop==20
                S = [];
                break;
            end
            [sortLong, indexLong ] = sortLongFun(d,indexPL);
        end
%         if stableMatchingFound
%             display('Stable Matching Found!')
%         else
%             display('NO Stable Matching Found!')
%         end
    end
end
