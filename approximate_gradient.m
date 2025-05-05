%save in file named 'approximate_gradient.m'
%template for implementing finite difference approximation
%for the gradient of a function
%INPUTS:
%fun: the mathetmatical function we want to differentiate
%x: the input value of fun that we want to compute the gradient at
%OUTPUTS:
%G: approximation of gradient of fun at x

%x is a vector, fun is a function

function G = approximate_gradient(fun,x) 
    h = 1e-6;
    
    gradient = zeros(length(x), 1);
    for i = 1 : length(x)
        %keep all components but ONE constant 
        h_input = zeros(length(x), 1);
        h_input(i) = h;
        xNewPlus = x + h_input;
        xNewMinus = x - h_input;
        
        gradient(i) = (fun(xNewPlus) - fun(xNewMinus))/(2*h);
    end
    
    G = gradient;

end

