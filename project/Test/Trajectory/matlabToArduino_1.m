clear all;
clc;

fileID = fopen('matlabToArduino_1.txt','w');

% Theta_Time = zeros(6,2);
col = size(Theta_Time,2);
num = 6 * col;


for j = 1:6
    for i = 1:col
        if i==1 && j==1
            fprintf(fileID,'{{%d,',Theta_Time(j,i));
        elseif i==col && j == 6
            fprintf(fileID,'%d}};',Theta_Time(j,i));
        elseif i==col
            fprintf(fileID,'%d},\n',Theta_Time(j,i));
        elseif i==1
            fprintf(fileID,'{%d,',Theta_Time(j,i));
        else
            fprintf(fileID,'%d,',Theta_Time(j,i));
        end
    end
end

fclose(fileID);