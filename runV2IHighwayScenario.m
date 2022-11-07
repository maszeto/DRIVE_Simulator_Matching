% Load the simulation settings (for further explanation see simSettings.m)
simSettings;

% Add the different modules of the simulator to MATLAB path
setupSimulator;


startTraci(sumo);

traci.poi.add("RSU1", 1262.28,1606.62, [255, 255, 255, 255]);

fprintf("");
traci.close();