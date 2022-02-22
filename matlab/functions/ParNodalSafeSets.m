function agent = ParNodalSafeSets(agent, W, draw_plot)
node = agent.graph.node;

% Compute the local disturbance bounds for each mode
net_W = cell(1,agent.nummodes);
for m=1:agent.nummodes
    net_W{m} = Polyhedron(zeros(1,agent.nx));
    for i=1:numel(W)
        net_W{m} = net_W{m} + agent.mode{m}.E{i}*W{i};
    end
    net_W{m}.minVRep;
end

% Initalize the safe-sets with the state constraints of the active mode
S = repmat(Polyhedron(zeros(1,agent.nx)), 1, agent.graph.numnodes);
for n=1:agent.graph.numnodes
    S(n) = agent.mode{node{n}.label}.X;
end

% Get next iteration of safe-sets.
S_new = S;
k = 0;
if draw_plot
    figure
end
while true
    %fprintf("Inner Iteration %d\n", k);
    k = k+1;
    % For each safe-set
    parfor (n = 1:agent.graph.numnodes, agent.graph.numnodes)
    %for n = 1:agent.graph.numnodes
        % and each possible successor safe-set
        successors = node{n}.edges;
        for s = successors
            % Update the safe-set with the previewed pre set
            S_new(n) = S_new(n) & PreviewedPreSet(S(s), agent.mode{node{s}.label}, net_W{node{s}.label});
            S_new(n).minVRep();
        end
    end
    % End if converged
    if compareSets(S_new, S, draw_plot)
        break
    end
    S = S_new;
end
agent = InsertInto(agent, "safe_sets", S);
end

function tf = compareSets(S1, S2, createplot)
    tf = true;
    for i=1:numel(S1)
        if createplot
            subplot(1,numel(S1),i);
            plot(S1(i));
            hold on
        end
        if S1(i) ~= S2(i)
            tf = false;
            if ~createplot
                return
            end
        end
    end
    if createplot
        drawnow
    end
end