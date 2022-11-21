classdef DataProcess
    %DATAPROCESS Summary of this class goes here
    %   Collection of functions to process data from simulation runs
    
    properties
        Property1
    end
    
    methods
        function obj = DataProcess(inputArg1, inputArg2)
            %DATAPROCESS Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
    end
    
    methods(Static)
        function handoverDiff = getHandoverDifference(good,bad)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            handoverDiff = bad - good;
        end

        function totalHandoverImprovement = getTotalHandoverImprovement(good, bad)
            totalGood = sum(good);
            totalBad = sum(bad);
            totalHandoverImprovement = totalBad - totalGood;
        end

        function maxHandoverImprovement = getMaxHandoverImprovement(good, bad)
            maxHandoverImprovement = max(obj.handoverDiff(good, bad));
        end
        
        function handoverPlot = plotHandovers(good, bad)
            handoverPlot = bar(bad,'r');
            hold on;
            bar(good,'b');
            hold off;
        end
        
        function vehicleDataRates = calculateVehicleDataRate(rsuPlan, handoverTypes)
            
        end
        
    end
    
end

