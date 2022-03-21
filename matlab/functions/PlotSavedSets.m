function PlotSavedSets(base_name, uid)
% stack = dbstack();
% path = which(stack(2).file);
% path = path(1:find(path=='\', 1, 'last'));
% filePattern = fullfile(path, [base_name,'*']);
% 
% files = dir(filePattern);
% sys_idx = []
% for file = files
%     filename = split(file.name, "_" | ".");
%     insertInto(sys_idx, str2int(filename{2}));
% end
% 
% for file = dir(filePattern)'
% disp(file.name)
% end
figure
S = repmat(Polyhedron.emptySet(2), 1, numel(uid));
for i=1:numel(uid)
S(i) = load([base_name, uid{i}]).S;
%subplot(1, numel(uid), i);
%plot(S(i))
end
subplot(1,5,1:3)
plot(S)
ylabel("$\dot{x}$",'Interpreter','latex','fontsize',15);
xlabel("$x$",'Interpreter','latex','fontsize',15);

subplot(1,5,4:5)
plot(S)
xlim([1.75, 2.5])
ylabel("$\dot{x}$",'Interpreter','latex','fontsize',15);
xlabel("$x$",'Interpreter','latex','fontsize',15);

legend(["Iteration 8", "Iteration 2", "Iteration 1"],'Interpreter','latex','fontsize',10);
sgtitle("Safe-sets for the 5th agent's 1st node",'Interpreter','latex','fontsize',15)

end