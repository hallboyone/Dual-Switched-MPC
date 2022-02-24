function x = ExtractFrom(sys, field)
% EXTRACTFROM returns a data structure containing the data specified by the
% string parameter <field>. 

if field == "safe_sets"
    % x{agent}{node}
    x = cell(1, numel(sys));
    for a_idx = 1:numel(sys)
        x{a_idx} = cell(1, sys{a_idx}.graph.numnodes);
        for n_idx = 1:sys{a_idx}.graph.numnodes
            x{a_idx}{n_idx} = sys{a_idx}.graph.node{n_idx}.S;
        end
    end
elseif field == "safe_set_union"
    % Extracts a cell array of the unions of each agent's safe-sets
    % x{agent} = <Union of agent's safe-sets>
    x = cell(1, numel(sys));
    for a_idx = 1:numel(sys)
        U = PolyUnion(Polyhedron([0,0]));
        for node = sys{a_idx}.graph.node
            U.add(sys{a_idx}.graph.node{1}.S);
        end
        x{a_idx} = U.convexHull();
    end
end
end