function output = BicycleMMMEndpoint(input)
%input is defined behind closed doors but we are told it contains the
%following sub-structures: 'phase', 'auxdata', 'parameters'

output.objective = input.phase(1).integral(1);
end

%the endpoint function defines the cost to be minimized
%a scalar that contains the result of computing the objective function on
%the current call is equal to the integral in phase 1.
