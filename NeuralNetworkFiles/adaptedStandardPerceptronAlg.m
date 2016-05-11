% function to perform the standard perceptron algorithm on a given set of
% inputs, outputs and initial weights
function [w] = adaptedStandardPerceptronAlg(X, Y, w_init, rate, stopvalue)

w = w_init;
errorfunc = 150;
while errorfunc >  stopvalue % use a generic large iteration size
    for i = 1 : size(X,1)   % cycle through the input values for each iris
      
        % compare the sign of the dot product between the weights and iris
        % values to see if it is less than 0 or not (Y outputs will be 
        % either -1 or 1 to carry out this if function
        if sign(dot(w, X(i,:))) ~= Y(i,:) 
      
            % update the weights in the direction of the sign (add or 
            % subtract)
            w = w + (rate * (X(i,:) * Y(i,:)));
        end
    end
    
    % for all the iris values
    for i = 1 : size(X,1)
        % calculate the dot product of the values with weights
        output = dot(w, X(i,:));
        % assign 0 if classified setosa
        if (output < 0)
            result(i) = 0;
        end
        % assign 1 if classified non-setosa
        if (output >= 0)
            result(i) = 1;
        end
    end

    % calculate the desired result matrix to compare to the calculated one
    desiredResult = [zeros(1,50), ones(1,100)];
    % calculate the error function using the difference between results
    errorfunc = sum(abs(desiredResult - result));
end

end
