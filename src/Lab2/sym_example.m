%% Let's define some symbolic variables
syms a b c  % This defines 'a', 'b', and 'c' as symbolic variables

% What happens if I add a concrete number to symbolic variables?
d = sind(a) ^ b / c + 5;

% I get a new symbolic expression!
disp(d)
disp(class(d))

%% But what if I want to evaluate my symoblic expression?
% Use 'subs'
e = subs(d, [a, b, c], [90, 1, 5]);

% Even though I put concrete, floating point numbers into this symbolic
% expression, I still got a symbolic result! MATLAB left it as a fraction,
% as that is the most precise way of displaying the result.
disp(e)
disp(class(e))

% If I want to get the result as a float, I need to cast it.
float_e = double(e)

%% Are Symbolic variables fast? 
%  _   _  ___
% | \ | |/ _ \ 
% |  \| | | | |
% | |\  | |_| |
% |_| \_|\___/ 

% Let's multiply some big arrays to show how slow syms are
normal_mat_1 = rand(20);
sym_mat_1 = sym(normal_mat_1);  % Cast as syms

normal_mat_2 = rand(20);
sym_mat_2 = sym(normal_mat_2);  % Cast as syms

% This function multiplies two symbolic matrices
symfn = @() (sym_mat_1 * sym_mat_2);
timeit(symfn)  % --> 0.0128 s on my 2017 Asus laptop

% This function multiplies two floating-point matrices
float_fn = @() (normal_mat_1 * normal_mat_2);
timeit(float_fn)  % --> 3.434 * 10^-6 seconds
            

