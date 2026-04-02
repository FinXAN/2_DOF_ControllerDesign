function C1 = TwoDOFtuningFunction(num,denum,C2,alphas)

arguments
    num (1,:){mustBeNumeric}
    denum (1,:){mustBeNumeric}
    C2 {mustBeA(C2, 'tf')}
    alphas (1,:){mustBeNumeric} = [3,3,3,3];
end

b = num;
a = denum;
P = tf(b,a);

C0 = C2;
q = C0.num{1};
p = C0.den{1};
% c(s) = a(s)p(s) + b(s)q(s)
cpoly1 = conv(a, p);
cpoly2 = conv(b, q);


% 确保数组长度相同
max_len = max(length(cpoly1), length(cpoly2));
cpoly1_padded = [zeros(1, max_len - length(cpoly1)), cpoly1];
cpoly2_padded = [zeros(1, max_len - length(cpoly2)), cpoly2];

% 现在可以相加
cpoly = cpoly1_padded + cpoly2_padded;
r = roots(cpoly);

n = numel(a) - 1;       
m = numel(p) - 1;
[f, h] = fw_split_roots_conjugate_blocks(r, n,m);


%%% To check the thing
assert(norm(imag(f)) < 1e-8);
assert(norm(imag(h)) < 1e-8);
%% Check something

c_rebuilt = conv(f,h);
c_rebuilt = c_rebuilt / c_rebuilt(1);

%% Display the things there
cpoly_n   = cpoly / cpoly(1);
disp(norm(c_rebuilt - cpoly_n));
disp([c_rebuilt(:), cpoly_n(:)]);

%% Failure Assert

assert(norm(c_rebuilt - cpoly_n) < 1e-6);

% 验证：deg f = n, deg h = m
assert(length(f)-1 == n, 'deg f != n');
assert(length(h)-1 == m, 'deg h != m');
M = tf(a,f);
N = tf(b,f);
X = tf(p,h);
Y = tf(q,h);
% 验证
assert(length(f)-1 == n, 'deg f != n');
assert(length(h)-1 == m, 'deg h != m');
 

b_roots = roots(b);
stable_roots = b_roots(real(b_roots) < 0);  
unstable_roots = b_roots(real(b_roots) >= 0); 

% 构建b_s(s)和b_u(s)
b_s = poly(stable_roots);    
b_u = poly(unstable_roots);  
b_u_0 = polyval(b_u, 0);
% ========== 新增：计算k值 ==========
deg_f = length(f) - 1;           % deg f(s)
deg_b_s = length(b_s) - 1;       % deg b_s(s)
min_k = deg_f - deg_b_s;
current_k = length(alphas);
if current_k < min_k
    warning('k值不足: 当前k=%d, 需要k≥%d。自动调整k值。', current_k, min_k);
    % 补充额外的α值
    additional_alphas = 5 * ones(1, min_k - current_k);  % 使用较小的α值
    alphas = [alphas, additional_alphas];
    fprintf('调整后的alphas: %s\n', mat2str(alphas));
end

b_u_0_value = polyval(b_u, 0);  % 这是标量值
denominator_poly = b_s * b_u_0_value;  % b_s(s) * b_u(0)
Q1_base = tf(f, denominator_poly);

% 然后乘以抵消项
for a_i = alphas
    Q1_base = Q1_base * tf(a_i, [1 a_i]);
end
Q1 = minreal(Q1_base);
C1 = minreal(Q1/(X)); % 因为 Q2 是0，C1 按照参数化就是直接设计出来的   
% 9.normlization for elimiatnating pole at (0,0)
[~, den1] = tfdata(C1, 'v');
if den1(end) == 0
    s = tf('s');
    C1 = minreal(C1*s); % remove exactly one integrator;
end
G0 = minreal((P*C1)/(1 + P*C2));
K  = 1/dcgain(G0);
%K = 1/dcgain(C1);
C1 = K*C1;
end



%% AI Root Allocation logic

function [f, h] = fw_split_roots_conjugate_blocks(r, n, m)
tol = 1e-7;

% Snap nearly-real roots to real
mask_real = abs(imag(r)) < tol;
r(mask_real) = real(r(mask_real));

% ---- Build conjugate-safe blocks ----
used = false(size(r));
blocks = {};

for i = 1:numel(r)
	if used(i), continue; end
	ri = r(i);

	if abs(imag(ri)) < tol
		blocks{end+1} = ri; %#ok<AGROW>
		used(i) = true;
	else
		target = conj(ri);
		j = find(~used & abs(r - target) < tol, 1, 'first');
		if isempty(j)
			error('Conjugate root not found for %g%+gi', real(ri), imag(ri));
		end
		blocks{end+1} = [ri; r(j)]; %#ok<AGROW>
		used([i j]) = true;
	end
end

% (Optional) sort blocks by stability (more negative real part first)
key = zeros(1, numel(blocks));
for k = 1:numel(blocks)
	key(k) = mean(real(blocks{k}));
end
[~, idx] = sort(key, 'ascend');
blocks = blocks(idx);

% ---- Allocate blocks to reach exactly n roots (DP/backtracking) ----
block_sizes = cellfun(@numel, blocks);
B = numel(blocks);

% Debug (optional)
% disp(block_sizes);

% dp(i, s) = can we reach sum s using blocks i..B ?
dp = false(B+2, n+1);
dp(B+1, 0+1) = true;

for i = B:-1:1
	for s = 0:n
		keep = dp(i+1, s+1);

		take = false;
		if s >= block_sizes(i)
			take = dp(i+1, (s - block_sizes(i)) + 1);
		end

		dp(i, s+1) = keep || take;
	end
end

if ~dp(1, n+1)
	error('No feasible block assignment to reach deg(f)=%d exactly.', n);
end

% reconstruct a choice
choose = false(1, B);
s = n;
for i = 1:B
	if s >= block_sizes(i) && dp(i+1, (s - block_sizes(i)) + 1)
		choose(i) = true;
		s = s - block_sizes(i);
	end
end
if s ~= 0
	error('Backtracking reconstruction failed.');
end

roots_f = [];
roots_h = [];
for i = 1:B
	if choose(i)
		roots_f = [roots_f; blocks{i}]; %#ok<AGROW>
	else
		roots_h = [roots_h; blocks{i}]; %#ok<AGROW>
	end
end

if numel(roots_f) ~= n
	error('Allocation bug: roots_f count %d, expected %d.', numel(roots_f), n);
end
if numel(roots_h) ~= m
	error('Allocation bug: roots_h count %d, expected %d.', numel(roots_h), m);
end

% ---- Build polynomials ----
f = poly(roots_f);
h = poly(roots_h);

% Clean tiny imaginary coefficient noise
f(abs(imag(f)) < 100*tol) = real(f(abs(imag(f)) < 100*tol));
h(abs(imag(h)) < 100*tol) = real(h(abs(imag(h)) < 100*tol));

if norm(imag(f)) > 1e-6 || norm(imag(h)) > 1e-6
	error('f or h has significant imaginary coefficients.');
end
end

