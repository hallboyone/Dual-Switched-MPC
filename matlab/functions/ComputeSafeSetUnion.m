function sys = ComputeSafeSetUnion(sys)
% COMPUTESAFESETUNION adds a disturbance field to each agent in the system that
% equals the union of all the current safe-sets. 

for a_idx = 1:numel(sys)
    sys{a_idx}.W = Polyhedron.emptySet(sys{a_idx}.nx);
    for node = sys{a_idx}.graph.node
        sys{a_idx}.W = sys{a_idx}.W & sys{a_idx}.graph.node{1}.S;
    end
    sys{a_idx}.W.minHRep;
end
end