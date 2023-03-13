
%code using string names 
% total rsus is the sum of potential rsus at each timestep 
% guh{2,1} = 1
potRSUs = {
    [1,2,3,4,5],
    [1,2,3,4],
    [3,4,6,7],
    [4,5],
    [-1],
    [5,6,7],
    [6,7],
    [8],
    [8,9],
    [8,9]
    };
potRSUs = [{[1]}; potRSUs; {[0]}]; % add a start and stop node

nNum = 0; %vertex num
nNumTime = [];
nNames = []; %vertex names
nIDs = []; %ID vertex corresponds to

for i = 1:size(potRSUs)
    nNum = nNum + length(potRSUs{i});
    nNumTime = [nNumTime, length(potRSUs{i})];
    for j = 1:length(potRSUs{i})
        nNames = [nNames, num2str(potRSUs{i}(j)) + "_{" + num2str(i-1) + "}"];
        nIDs = [nIDs, potRSUs{i}(j)];
    end
end

aDAG = zeros(nNum, nNum); %ajaceny matrix
nNumTimeSum = cumsum(nNumTime);
% populate matrix
aDAGYIndex = 1;
for i = 1:length(potRSUs)
    for j = 1:length(potRSUs{i})
        if(i < length(potRSUs))
            aDAGXIndex = nNumTimeSum(i);
            % current vertex is connected to all others in next time
            for k = 1:length(potRSUs{i+1})
                aDAGXIndex = aDAGXIndex + 1;
                
                if(i ~= length(potRSUs) - 1)
                    aDAG(aDAGYIndex, aDAGXIndex) = rand * -1;%call to get datarate function
                else
                    aDAG(aDAGYIndex, aDAGXIndex) = .00000000000000001; %infinetely small
                end         
            end
        end
        
        aDAGYIndex = aDAGYIndex + 1;
    end
end

sDAG = digraph(aDAG, nNames);
p = plot(sDAG,'EdgeLabel',sDAG.Edges.Weight)
%plot(vDAG);

%finding shortest path
%need potentail RSUs, graph, 
spath = shortestpath(sDAG, "1_{0}","0_{11}",'Method','acyclic');
highlight(p,spath,'EdgeColor','g')
% Convert path to schedulew 
schedule = [];
for i = 1:length(spath)
    
    if( i ~= 1 && i ~= length(spath))
        nIndex = find(spath(i) == nNames);
        id = nIDs(nIndex);
        schedule = [schedule, id];
    end
end
spath
schedule

