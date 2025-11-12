function [r] = processData(r)
    r = r(1:end-1);
    
    for k = 1:length(r) % indices
        A = char(r(k));
        A(A=='[') = [];
        A(A==']') = [];
        A(A=='(') = [];
        A(A==')') = [];
        A(A==',') = [];
        r(k) = string(A);
    end
end