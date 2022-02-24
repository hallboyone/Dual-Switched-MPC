function M = Mode(A, B, X, U, idx)
M.nx = X.Dim;
M.nu = U.Dim;
M.A = A{idx}; % Dynamics. Cell array with local and cross dynamics.
M.B = B;
M.E = {A{1:idx-1}, zeros(M.nx), A{idx+1:end}};
M.X = X; % State constraints
M.U = U; % Input constraints
end