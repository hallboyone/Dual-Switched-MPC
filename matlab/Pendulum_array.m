num_modes = 3;

num_row = 6;
num_col = 6;
num_elm = num_row*num_col;

A = cell(num_modes, 1);
Mk = zeros(num_elm);
Mc = zeros(num_elm);
K = rand(num_elm, 4);
C = 0.1*rand(num_elm, 4);
for m_idx = 1:num_modes
    m = 5*(1+rand(num_elm, 1))*sqrt(2);
    for n = 1:num_elm
        if n+num_col <= num_elm
            Mk(n,n+num_col) = K(n,1)/m(n);
            Mk(n,n) = Mk(n,n) - K(n,1)/m(n);
            Mc(n,n+num_col) = C(n,1)/m(n);
            Mc(n,n) = Mc(n,n) - C(n,1)/m(n);
        end
        if n-num_col > 0
            Mk(n,n-num_col) = K(n,2)/m(n);
            Mk(n,n) = Mk(n,n) - K(n,2)/m(n);
            Mc(n,n-num_col) = C(n,2)/m(n);
            Mc(n,n) = Mc(n,n) - C(n,2)/m(n);
        end
        if mod(n, num_col) ~= 0
            Mk(n,n+1) = K(n,3)/m(n);
            Mk(n,n) = Mk(n,n) - K(n,3)/m(n);
            Mc(n,n+1) = -C(n,3)/m(n);
            Mc(n,n) = Mc(n,n) - C(n,3)/m(n);
        end
        if mod(n-1, num_col) ~= 0
            Mk(n,n-1) = K(n, 4)/m(n);
            Mk(n,n) = Mk(n,n) - K(n,4)/m(n);
            Mc(n,n-1) = -C(n,4)/m(n);
            Mc(n,n) = Mc(n,n) - C(n,4)/m(n);
        end

    end
    A{m_idx} = remapDynamics([zeros(num_elm), eye(num_elm); Mk, Mc]);
end
A = formatDynamics(A);

function M = remapDynamics(M)
num_elm = size(M,1)/2;
idx_map = floor(1:0.5:num_elm+0.5)+repmat([0, num_elm], 1, num_elm);
M = M(idx_map, :);
M = M(:, idx_map);
end

function formatedA = formatDynamics(A)
num_elm = size(A{1},1)/2;
num_mode = numel(A);

formatedA = cell(num_elm, 1);
for i=1:num_elm
    formatedA{i} = cell(num_mode, 1);
    for m=1:num_mode
        formatedA{i}{m} = cell(num_elm, 1);
        for n=1:num_elm
            formatedA{i}{m}{n} = A{m}(1+2*(m-1):2*m, 1+2*(n-1):2*n);
        end
    end
end
end