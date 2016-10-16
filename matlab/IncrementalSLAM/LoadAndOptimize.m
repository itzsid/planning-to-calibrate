function result = LoadAndOptimize(file)
timerValues = [];
ATEValues = [];
ALEValues = [];
AREValues = [];
MarginalCov = [];
globals
config
    [graph, ground_truth] = gtsam.readG2o(file);
    adjMatrix = AdjMatrix(graph);
    IO = IncrementalOptimization(graph, ground_truth, adjMatrix, OPTIMIZER);
    numFactors = graph.size;
    totalCount = 0;
    computationThreshold = numFactors/5;
    count = 0;
    while(totalCount < numFactors)    
        IO = IO.nextMeasurement;
        tStart = tic;
        IO = IO.optimize;
        timeElapsed = toc(tStart);
        timerValues = [timerValues, timeElapsed];
    
        res = IO.result;
        marginals = IO.marginals;
        graph = IO.graph;
        groundTruth = IO.currentGroundTruth;              
        count = count + 1;
        totalCount = totalCount + 1;
        
    end
    
    result.ATEValues = ATEValues;
    result.ALEValues = ALEValues;
    result.AREValues = AREValues;
    result.MarginalCov = MarginalCov;
    result.timerValues = timerValues;
   
    result.res = IO.result;
    result.marginals = IO.marginals;
    result.graph = IO.graph;
    result.groundTruth = IO.currentGroundTruth;
   
    
    
end
    
    
    
    
    
    
    

