function sys = BuildSystem(A, B, X, U, G)
sys = {};
for a_idx = 1:numel(X)
    modes = {};
    for m_idx = 1:G{a_idx}.max_label
        modes{end+1} = Mode(A{a_idx}{m_idx},... 
                            B{a_idx}{m_idx},...
                            X{a_idx}{m_idx},...
                            U{a_idx}{m_idx},...
                            a_idx);
    end
    sys{end+1} = Agent(G{a_idx}, modes);
end
end

function Ac = crossTerms(A, a, m)
Ac = {};
for i=1:size(A,2)
    if i ~= a
        Ac{end+1} = A{a,i}{m};
    end
end
end