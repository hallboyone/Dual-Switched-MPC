function S = NodalSafeSets(agent, W)
node = agent.graph.node;

% Initalize the safe-sets with the state constraints
S = repmat(Polyhedron.emptySet(agent.nx), 1, agent.graph.numnodes);
for n=1:agent.graph.numnodes
    S(n) = agent.mode(node(n).label).X;
end

% Get next iteration of safe-sets.
S_new = S;
k = 0;
figure
while true
    fprintf("Inner Iteration %d\n", k);
    k = k+1;
    % For each safe-set
    for n = 1:agent.graph.numnodes
        % and each possible successor safe-set
        successors = node(n).edges;
        for s = successors
            % Update the safe-set with the previewed pre set
            S_new(n) = S_new(n) & PreviewedPreSet(S(s), agent.mode(node(s).label), W(node(s).label));
            S_new(n).minVRep();
        end
    end
    % End if converged
    if compareSets(S_new, S, true)
        break
    end
    S = S_new;
end
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