function data = load_data(file_name)

%% Import data from text file
% Script for importing data from the following text file:
%
%    filename: /Users/kani/Documents/vectornav/vn200_test/build/data_20191118/data4.txt
%
% Auto-generated by MATLAB on 18-Nov-2019 14:06:36

%% Setup the Import Options
opts = delimitedTextImportOptions("NumVariables", 16);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["t", "y", "p", "r", "mode", "fix", "error", "p_x", "p_y", "p_z", "v_x", "v_y", "v_z", "a_x", "a_y", "a_z"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
data = readtable(file_name, opts);


%% Clear temporary variables
clear opts

end