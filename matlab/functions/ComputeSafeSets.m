function system = ComputeSafeSets(system, plot_outer, plot_inner, par_inner)
% Run the algorithm to find safe-sets.
k = 0;
S = ExtractFrom(system, "safe_sets");
while true
    % Save the current safe-sets to compare later
    old_S = S;

    % Run this twice to return to a valid state (even steps are valid, odd are
    % invalid)
    for i=1:2
        fprintf("Outer Iteration %d.%d\n", k, 5*(i-1));
        % Extract the union of each agent's safe-sets into a cell array
        safe_set_unions = ExtractFrom(system, "safe_set_union");
        % For each agent, compute its safe-sets using the current safe-set unions
        ppm = ParforProgressbar(numel(system),'showWorkerProgress',true);
        parfor a=1:numel(system)
            system{a} = AgentSafeSets(system{a}, safe_set_unions, plot_inner, par_inner);
            ppm.increment();
        end
        delete(ppm)
    end

    % Save any requested figures
    figs_to_save = ExtractFrom(system, "figs_to_save");
    for i=1:numel(figs_to_save)
        S = system{figs_to_save{i}.a_idx}.graph.node{figs_to_save{i}.n_idx}.S;
        save([figs_to_save{i}.filename, int2str(k)], 'S');
    end

    % Get the newly computed safe-sets and compare with the old safe-sets
    S = ExtractFrom(system, "safe_sets");
    if areEqual(S, old_S, plot_outer)
        break
    end
    k = k+1;
end
end

function is_equal = areEqual(safe_sets, old_sets, plot_tf)
row_len = 0;
for m=1:numel(safe_sets)
    row_len = max(row_len, numel(safe_sets{m}));
end
is_equal = true;
for m=1:numel(safe_sets)
    for i=1:numel(safe_sets{m})
        if plot_tf
            subplot(numel(safe_sets),row_len,(m-1)*row_len+i)
            plot(safe_sets{m}{i}, 'alpha', 0.2)
            xlim([-5.5, 5.5])
            ylim([-5.5, 5.5])
            xlabel("$c_c$",'Interpreter','latex','fontsize',15);
            ylabel("$c_k$",'Interpreter','latex','fontsize',15);
            title(['$\mathcal{S}_{(', num2str(m),',',num2str(i),')}$'],'Interpreter','latex','fontsize',15)
            hold on
        end
        if isempty(old_sets{1}) || (old_sets{m}{i} ~= safe_sets{m}{i})
            is_equal = false;
        end
    end
end
if plot_tf
    drawnow
end
end

