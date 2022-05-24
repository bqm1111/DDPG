function output = matrixSign(input)
[m,n] = size(input);
output = zeros(m,n);
for i = 1:m
    for j = 1:n
        if input(i,j) >= 0
            output(i,j) = 1;
        else
            output(i,j) = -1;
        end
    end
end

end