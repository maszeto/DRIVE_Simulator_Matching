%% Example file to test the Stable Fixtures Matching Game
% to test the functionality of the algorithm

clear; clc ;

% Looks like this defines the preference list and preference list cap for
% 10 vehicle vehicles
%     STABLE MATCHING EXISTS
%     1  2  3  4  5  6  7  8  9 10 
PL = [ 0  9 10  8  7  0  6  5  0  4 ;...
      10  0  8  9  7  0  0  6  5  0 ;...
       7  6  0  5  4  0 10  9  8  3 ;...
       7  5  9  0 10  0  0  6  8  0 ;...
       7 10  9  6  0  4  8  0  5  3 ;...
       0  0  0  0  7  0 10  8  9  0 ;...
       9  0  7  0  6 10  0  8  0  5 ;...
      10  6  5  9  0  4  8  0  7  0 ;...
       0 10  6  7  9  5  0  8  0  0 ;...
      10  0  7  0  9  0  8  0  0  0 ];
plCap = [ 2 ; 2 ; 2 ; 2 ; 2 ; 2 ; 1 ; 1 ; 1 ; 1 ];


%     STABLE MATCHING EXISTS
%      1  2  3  4  5  6     
% PL = [ 0  4  2  6  3  5 ;...  
%        3  0  5  2  4  6 ;...
%        3  2  0  6  5  3 ;...
%        3  6  2  0  4  5 ;...
%        2  5  4  6  0  3 ;...
%        5  3  2  4  6  0 ];
% plCap = [ 1 ; 1 ; 1 ; 1 ; 1 ; 1 ];


%     NO STABLE MATCHING EXISTS
%      1  2  3  4  5  6 
% PL = [ 0  6  4  5  0  0 ;...  
%        4  0  6  0  5  0 ;...
%        6  4  0  0  0  5 ;...
%        5  0  0  0  6  4 ;...
%        0  5  0  4  0  6 ;...
%        0  0  5  6  4  0 ];
% plCap = [ 2 ; 2 ; 2 ; 2 ; 2 ; 2 ];
     

%     NO STABLE MATCHING EXISTS     
%      1  2  3  4  5  6     
% PL = [ 0  6  3  4  2  5 ;...  
%        4  0  6  2  5  3 ;...
%        6  4  0  2  3  5 ;...
%        2  5  4  0  6  3 ;...
%        1  2  4  3  0  6 ;...
%        3  5  0  6  4  2 ];
% plCap = [ 1 ; 1 ; 1 ; 1 ; 1 ; 1 ];

% Sorts preference lists in descending order across second dimension, this
% is the vehicle ranking the other vehicles but the PL should be generated
% from the fitness function so idk wtf this about
[ sortedPL , indexPL ] = sort(PL,2,'descend');

%loop fills indexPL with 0 if sorted PL is also 0
for i=1:length(indexPL)
    for j=1:length(indexPL)
        if sortedPL(i,j)==0
            indexPL(i,j)=0;
        end
    end
end


for i=1:length(sortedPL)
    a_iElements{i} = {};
    b_iElements{i} = {};
end
a_i = zeros(length(sortedPL),1); %Gives you some zeroes
S = [ 0 0 ];
posToRun = find(a_i<plCap);
posToRun = posToRun'; %Just gives you a 1x10 

while ~isempty(posToRun)
    [ sortedPL,a_i,plCap,S,indexPL,a_iElements,b_iElements ] = sfPhase1( posToRun,sortedPL,a_i,plCap,S,indexPL,a_iElements,b_iElements );

    [ a_i,b_i,a_iElements,b_iElements,posToRun ] = findPosToRun( plCap,indexPL,S );
end

S = S(2:end,:);
if ~isempty(S)
    S = sortrows(S,1);
end


[ sortedPL,a_i,plCap,S,indexPL,a_iElements,b_iElements ] = sfPhase2(sortedPL,a_i,plCap,S,indexPL,a_iElements,b_iElements );

if ~isempty(S)
    S = sortrows(S,1);
end
