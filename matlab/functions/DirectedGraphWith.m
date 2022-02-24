function G = DirectedGraphWith(nodes)
G.node = nodes;
G.max_label = VerifyLabelsIn(nodes);
G.numnodes = numel(nodes);
end

function max_label = VerifyLabelsIn(nodes)
% All labels should be consecutive integers between 1 and a max value
max_label = 0;
for i=1:numel(nodes)
    if 0~=mod(nodes{i}.label, 1)
        error("Labels must be integers")
    elseif nodes{i}.label < 1
        error("Labels must be greater than or equal to 1")
    end
    max_label = max(max_label, nodes{i}.label);
end
end