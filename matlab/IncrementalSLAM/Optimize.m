function result = Optimize(graph, ground_truth)
globals
config
 adjMatrix = AdjMatrix(graph);
 bearingRangeOffsetNoise.X = R1_OFFSET_COV_X;
 bearingRangeOffsetNoise.Y = R1_OFFSET_COV_Y;
 bearingRangeOffsetNoise.Theta = R1_OFFSET_COV_THETA;
 
 
 betweenFactorOffsetNoise.X = R2_OFFSET_COV_X;
 betweenFactorOffsetNoise.Y = R2_OFFSET_COV_Y;
 betweenFactorOffsetNoise.Theta = R2_OFFSET_COV_THETA;

 IO = IncrementalOptimization(graph, ground_truth, adjMatrix, bearingRangeOffsetNoise, betweenFactorOffsetNoise, OPTIMIZER);
 numFactors = graph.size;
    totalCount = 0;
  
    count = 0;
    while(totalCount < numFactors)    
        IO = IO.nextMeasurement;
        IO = IO.optimize;
        count = count + 1;
        totalCount = totalCount + 1;     
    end

    result.res = IO.result;
    result.marginals = IO.marginals;
    result.graph = IO.graph;
    result.groundTruth = IO.currentGroundTruth;
   
    
    
end
    
    
    
    
    
    
    

