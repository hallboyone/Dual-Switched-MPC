% Title   - dual_switched_safe_sets.m
% Author  - Richard A. Hall (hallboyone@icloud.com)
% Date    - Jan 20, 2022
% Purpose - To compute the safe-set collections of an externally switched
% linear system with two seperate switching signals.

% System dynamics are of the form
%    [x1(t+1)] = A11(s1(t))*x1(t) + A12(s1(t))*x2(t) + B1(s1(t))*u1(t)
%    [x2(t+1)] = A22(s2(t))*x2(t) + A21(s2(t))*x1(t) + B2(s2(t))*u2(t)
% where s1(t) (s2(t)) is a switching signal that map the current time to an
% index between 1 and M1 (M2). 
%
% Our objective is to find two safe-set collections for the x1 states and
% x2 states that are indexed by the respective switching signal.
L1 = [1 1 1 2 2 2 2];
E1 = [0 1 0 0 0 0 0;
      0 0 1 0 0 0 0;
      0 0 1 1 0 0 0;
      0 0 0 0 1 0 0;
      0 0 0 0 0 1 0;
      0 0 0 0 0 0 1;
      1 0 0 0 0 0 1];

L2 = [1 1 1 1 2 2 2];
E2 = [0 1 0 0 0 0 0;
      0 0 1 0 0 0 0;
      0 0 0 1 0 0 0;
      0 0 0 1 1 0 0;
      0 0 0 0 0 1 0;
      0 0 0 0 0 0 1
      1 0 0 0 0 0 1];
G1 = DirectedGraph(E1, L1);
G2 = DirectedGraph(E2, L2);

saved_vars = load('rand_dyn.mat');
A11 = {eye(2) + (0.2*rand(2)-0.1), eye(2) + (0.2*rand(2)-0.1)};
A22 = {eye(2) + (0.2*rand(2)-0.1), eye(2) + (0.2*rand(2)-0.1)};
A12 = {0.1*(0.2*rand(2)-0.1), 0.1*(0.2*rand(2)-0.1)};
A21 = {0.1*(0.2*rand(2)-0.1), 0.1*(0.2*rand(2)-0.1)};
A = saved_vars.A_good;%{A11, A22};
Ac = saved_vars.Ac_good;%{A12, A21};


B1 = {[0.1; 0.1] + (0.2*rand(2,1)-0.1), [0.1; 0] + (0.2*rand(2,1)-0.1)};
B2 = {[0.1; 0.1] + (0.2*rand(2,1)-0.1), [0.1; 0] + (0.2*rand(2,1)-0.1)};
%B = {B1, B2};
B = saved_vars.B_good;

X1 = Polyhedron([eye(2); -eye(2)], [1; 2; 1; 2]);
X2 = Polyhedron([eye(2); -eye(2)], [2; 1; 2; 1]);
X = {X1, X2};
U1 = Polyhedron([1; -1], 10*[0.5; 0.25]);
U2 = Polyhedron([1;-1], 10*[0.25; 0.5]);
U= {U1, U2};

agent = [Agent(G1, [Mode(A11{1}, B1{1}, A12{1}, X1, U1), Mode(A11{2}, B1{2}, A12{2}, X1, U1)]),
          Agent(G2, [Mode(A22{1}, B2{1}, A21{1}, X2, U2), Mode(A22{2}, B2{2}, A21{2}, X2, U2)])];

W = {repmat(Polyhedron.emptySet(agent(1).nx), agent(1).nummodes, 1),
     repmat(Polyhedron.emptySet(agent(2).nx), agent(2).nummodes, 1)};
S = [NodalSafeSets(agent(1), W{1}); NodalSafeSets(agent(2), W{2})];

safe_sets_1 = cell(numel(mindt_1), 1);
for i=1:numel(mindt_1)
    safe_sets_1{i} = cell(mindt_1(i), 1);
    for k=1:mindt_1(i)
        safe_sets_1{i}{k} = X1;
    end
end
safe_sets_2 = cell(numel(mindt_2), 1);
for i=1:numel(mindt_2)
    safe_sets_2{i} = cell(mindt_2(i), 1);
    for k=1:mindt_2(i)
        safe_sets_2{i}{k} = X2;
    end
end
cur_safe_sets = {safe_sets_1, safe_sets_2};

figure
new_safe_sets = cur_safe_sets;
itr = 1;
start_time = get_time();
while true
    cur_safe_sets = new_safe_sets;
    
    % Initalize the new_safe-set variable to be the full state dynamics
    S = cell(2,1);
    W = cell(2,1);
    for node=1:2
        S{node} = convexHullOfUnion(cur_safe_sets{3-node});
        W{node} = {Ac{node}{1}*S{node}, Ac{node}{2}*S{node}};
        
        for mode=1:2
            W{node}{mode}.minHRep();
            for step=1:mindt{node}(mode)
                new_safe_sets{node}{mode}{step} = X{node};
            end
        end
    end
    
    parfor node=1:2
        while true
            old_safe_sets = new_safe_sets{node};
            for mode = 1:2
                for step = 1:mindt{node}(mode)
                    new_safe_sets{node}{mode}{step} = preSetPreviewedDisturbance(...
                        old_safe_sets{mode}{min(step+1, mindt{node}(mode))},...
                        A{node}{mode}, B{node}{mode}, new_safe_sets{node}{mode}{step},...
                        U{node}, W{node}{mode});
                    if step == mindt{node}(mode)
                        new_safe_sets{node}{mode}{step} = preSetPreviewedDisturbance(...
                            old_safe_sets{3-mode}{1},...
                            A{node}{3-mode}, B{node}{3-mode}, new_safe_sets{node}{mode}{step},...
                            U{node}, W{node}{3-mode});
                    end
                end
                new_safe_sets{node}{mode}{step}.minHRep();
                new_safe_sets{node}{mode}{step}.minVRep();
            end
            if(areEqual(new_safe_sets{node},old_safe_sets, 0))
                break
            end
        end
    end
    disp("Finished iteration");
    if mod(itr, 2)
        if itr > 1
            if(areEqual(new_safe_sets{1},odd_safe_sets{1}, 1) && areEqual(new_safe_sets{2},odd_safe_sets{2}, 1))
                break
            end
        end
        odd_safe_sets = new_safe_sets;
    end
    itr = itr + 1;
end
end_time = get_time();
close(gcf)
duration = end_time - start_time;
dur_min = floor(duration/60);
dur_sec = duration - dur_min*60;
fprintf("Finished in %dm, %0.2fs\n", dur_min, dur_sec);

function is_equal = areEqual(safe_sets, old_sets, plot_tf)
row_len = 0;
for m=1:numel(safe_sets)
    row_len = max(row_len, numel(safe_sets{m}));
end
is_equal = true;
    for m=1:2
        for i=1:numel(safe_sets{m})
            if plot_tf
                subplot(2,row_len,(m-1)*row_len+i)
                plot(safe_sets{m}{i})
                xlim([-2.5, 2.5])
                ylim([-2.5, 2.5])
                xlabel("$c_c$",'Interpreter','latex','fontsize',15);
                ylabel("$c_k$",'Interpreter','latex','fontsize',15);
                title(['$\mathcal{S}_{(', num2str(m),',',num2str(i),')}$'],'Interpreter','latex','fontsize',15)
                hold on
            end
            if (old_sets{m}{i} ~= safe_sets{m}{i})
                is_equal = false;
            end
        end
    end
if plot_tf
    drawnow
end
end

function time_sec = get_time()
    t = clock;
    time_sec = [0 0 86400 3600 60 1]*t';
end
% figure
% first_run = true;
% while true
%     for node=1:2
%         if node==1
%             Ac = A12;
%             A = A11;
%             B = B1;
%             U = U1;
%             X = X1;
%             mindt = mindt_1;
%         else
%             Ac = A21;
%             A = A22;
%             B = B2;
%             U = U2;
%             X = X2;
%             mindt = mindt_2;
%         end
%         if first_run
%             W = {Polyhedron.emptySet(2), Polyhedron.emptySet(2)};
%         else
%             S = convexHullOfUnion(safe_sets{3-node});
%             W = {Ac{1}*S, Ac{2}*S};
%             W{1}.minHRep();
%             W{2}.minHRep();
%         end
%         
%         for mode=1:2
%             safe_sets{node}{i} = cell(mindt(mode), 1);
%             for step=1:mindt(mode)
%                 safe_sets{node}{mode}{step} = X;
%             end
%         end
%         
%         while true
%             old_safe_sets = safe_sets{node};
%             for mode = 1:2
%                 mode_mindt = numel(safe_sets{node}{mode});
%                 for step = 1:mode_mindt
%                     safe_sets{node}{mode}{step} = preSetPreviewedDisturbance(...
%                         old_safe_sets{mode}{min(step+1, mode_mindt)},...
%                         A{mode}, B{mode}, safe_sets{node}{mode}{step},...
%                         U, W{mode});
%                     if step == mode_mindt
%                         safe_sets{node}{mode}{step} = preSetPreviewedDisturbance(...
%                             old_safe_sets{3-mode}{1},...
%                             A{3-mode}, B{3-mode}, safe_sets{node}{mode}{step},...
%                             U, W{mode});
%                     end
%                 end
%             end
%             if(areEqual(safe_sets{node},old_safe_sets, 1))
%                 break
%             end
%         end
%     end
%     first_run = false;
% end
% close(gcf)