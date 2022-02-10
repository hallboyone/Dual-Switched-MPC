function agent = Agent(graph, modes)
agent.mode = modes;
agent.graph = graph;

agent.nummodes = numel(agent.mode);
agent.nx = modes(1).nx;
agent.nu = modes(1).nu;

if numel(agent.mode) > agent.graph.max_label
    error("Labels of graph are out of range")
end
end