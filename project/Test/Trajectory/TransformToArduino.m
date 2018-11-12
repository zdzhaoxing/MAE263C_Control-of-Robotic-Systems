function Arduino_Matrix = TransformToArduino(Time_Theta)
[C_row,C_column] = size(Time_Theta);
Arduino = zeros(C_row,C_column);
for i  = 1:1:C_row
    for j = 1:1:C_column
        a = Time_Theta(i,j);
        Arduino(i,j) = strcat(mat2str(a), ',');
    end
end
Arduino_Matrix = Arduino; 
end