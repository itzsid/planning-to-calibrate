function adjMatrix = AdjMatrix(graph)
adjMatrix = containers.Map('KeyType', 'uint64','ValueType','any'); %Map containing pose-variables
for i = 0:graph.size - 1
    factor = graph.at(i);
    if isa(factor,'gtsam.BearingRangeTransformFactor2D') || isa(factor,'gtsam.BearingRangeFactor2D') || isa(factor,'gtsam.BetweenTransformFactorPose2') || isa(factor,'gtsam.BetweenFactorPose2')
        pose = factor.keys.at(0);
        if adjMatrix.isKey(pose)
            variables = adjMatrix(pose);
            variables = [variables,i];
            adjMatrix(pose) = variables;
        else
            adjMatrix(pose) = [i];
        end
    end
end

end

