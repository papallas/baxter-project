% QUESTION 3 - CODE TO RUN THE PERCEPTRON ALGORITHM ON EACH IRIS SPECIES

% open the data file and read into a cell array of data
file = fopen('colours.data');
C = textscan(file,'%f %f %f %s','Delimiter',',')
fclose(file);

BGRValues = cell2mat(C(1:3))/255;
BGRValues(:,[1,3])= BGRValues(:,[3,1]);

HSVValues = rgb2hsv(BGRValues);

RValues = HSVValues(:,1);
GValues = HSVValues(:,2);
BValues = HSVValues(:,3);

% GREEN VS NON GREEN
% Get the set of setosa values and non-setosa values into two separate sets
X1 = [RValues(1:50), GValues(1:50), BValues(1:50), ones(50,1)];
X2 = [RValues(51:150), GValues(51:150), BValues(51:150), ones(100,1)];

% combine them into one set X
X = [X1; X2];
% create a known outputs matrix of -1s and 1s
Y = [-ones(50,1); ones(100,1)];
size(X)
size(Y)

% create an initial weights matrix with random integers
w = randi(10,1,4);

greenWeights = adaptedStandardPerceptronAlg(X, Y, w, 0.1, 0)

% BLUE VS NON BLUE
% Get the set of setosa values and non-setosa values into two separate sets
X1 = [RValues(51:100), GValues(51:100), BValues(51:100), ones(50,1)];
X2 = [RValues(1:50), GValues(1:50), BValues(1:50), ones(50,1);
      RValues(101:150), GValues(101:150), BValues(101:150), ones(50,1)];

% combine them into one set X
X = [X1; X2];
% create a known outputs matrix of -1s and 1s
Y = [-ones(50,1); ones(100,1)];

% create an initial weights matrix with random integers
w = randi(10,1,4);

blueWeights = adaptedStandardPerceptronAlg(X, Y, w, 0.01, 0)

% RED VS NON RED
% Get the set of setosa values and non-setosa values into two separate sets
X1 = [RValues(101:150), GValues(101:150), BValues(101:150), ones(50,1)];
X2 = [RValues(1:100), GValues(1:100), BValues(1:100), ones(100,1)];

% combine them into one set X
X = [X1; X2];
% create a known outputs matrix of -1s and 1s
Y = [-ones(50,1); ones(150,1)];

% create an initial weights matrix with random integers
w = randi(10,1,4);

redWeights = adaptedStandardPerceptronAlg(X, Y, w, 0.1, 0)
