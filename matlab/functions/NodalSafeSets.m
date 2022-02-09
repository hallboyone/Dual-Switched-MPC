function S = NodalSafeSets(agent, W)
% Initalize the safe-sets with the state constraints
S = repmat(Polyhedron.emptySet(agent.nx), 1, agent.G.numnodes);
for i=1:agent.G.numnodes
    S(i) = agent.M(agent.G.L(i)).X;
end

% Get next iteration of safe-sets.
S_new = S;
k = 0;
while true
    fprintf("Iteration %d\n", k);
    k = k+1;
    % For each safe-set
    for n = 1:numel(S)
        % and each possible successor safe-set
        for s = 1:numel(S) 
            if agent.G.E(n,s)
                % Update the safe-set with the previewed pre set
                S_new(n) = S_new(n) & PreviewedPreSet(S(s), agent.M(agent.G.L(s)), W(agent.G.L(s)));
            end
        end
    end
    % End if converged
    if compareSets(S_new, S)
        break
    end
    S = S_new;
end
end

function tf = compareSets(S1, S2)
    for i=1:numel(S1)
        if S1(i) ~= S2(i)
            tf = false;
            return
        end
    end
    tf = true;
    return
end