%% This is a section header
% It's different from a normal comment because it starts with two '%'
% Section headers allow you to split your code into different areas to 
% execute using the 'Run Section' button

% Try it out by putting your cursor in this section and hitting 
% 'Run Section'

disp('Running the first section');

%% If you run the first section, nothing below this line will run
disp('Running the second section');

%% Breakpoints
% Breakpoints are the most basic feature of the MATLAB debugger.
% Breakpoints allow you to pause your code execution before a certain line
% of code is run.

% You can create a breakpoint by clicking on a line number in the "gutter"
% on the left of the text editor window
% <------------- Over there

% Try is out
a = 5;
b = 7;
a = a + b;  % drop a breakpoint here
c = a * 7;

%% Now that your code is stopped, you can explore the state of the variables
% At the bottom of the screen is the command window
% The command window allows you to execute MATLAB code that uses variables
% in your current workspace. 

% Try printing out the value of 'a' by typing in 'a'

%% After trying that out, run the highlighted line by hitting 'step'
% Then print out the new value of 'a'
% Try modifying the value of 'a' from the command line and see what happens
% Then hit 'stop' to stop execution of the program

%% Now we're going to learn about some advanced features of step
n = 10;
fibn = fib(n);  % Drop a breakpoint here
fprintf("the %d-th fibonacci number is: %d\n", n, fibn);

% 1. Run this section
% 2. Your code will stop before calculating the fibonacci sequence. You
%    have a few courses of action you can take from here:
%    + Step: This will run the highlighted line of code and stop at the
%            line below it
%    + Step into: this will start to execute the highlighted line, but stop
%                 at the first line of the function it is executing

% Try both of these out!

% Then try dropping a breakpoint inside 'fib' and play around with stepping
% out. Stepping out will run the code until the function returns, then stop
% executing before the result of the function is used for anything.

%% Conditional breakpoints
% Remove all other breakpoints, and add one on the b = fib... line
% Right click on this breakpoint and select "set / modify condition"
% Set a condition that only stops the code when 'a' is even

% Run this section to see how it works
% Hint: You can hit "continue" to run until the next breakpoint is hit
fib(7);

function b = fib(a) 
    % Returns the a-th fibonacci number using tail recursion
    if a <= 1
        b = a;
    else
        % Try dropping a breakpoint on this line and experimenting with
        % stepping in and out!
        % You can actually step in twice on this line, once for each
        % function call
        b = fib(a - 1) + fib(a - 2);
    end
end
