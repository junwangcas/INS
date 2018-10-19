function [ navigation_data ] = load_nodes(navigation_data)
f_handle = fopen(navigation_data.file_node_opti,'r');
if ~f_handle
    disp("file open error load nodes");
end
tline = fgetl(f_handle)
id_R = 0;
id_T = 0;
id_V = 0;
while ischar(tline)
    nstrsplit = strsplit(tline,' ');
    if strcmp(nstrsplit{1},'1')
        % bias;
        tliinef = sscanf(tline,'%f');
        navigation_data.bias = tliinef(2:end);
    elseif strcmp(nstrsplit{1},'2')
        %R 
        id_R = id_R +1;
        tliinef = sscanf(tline,'%f');
        navigation_data.R_list(id_R,:) = tliinef(2:end)';
    elseif strcmp(nstrsplit{1},'3')
        % T
        id_T = id_T + 1;
        tliinef = sscanf(tline,'%f');
        navigation_data.T_list(id_T,:) = tliinef(2:end)';
    elseif strcmp(nstrsplit{1},'4')
        %V
        id_V = id_V + 1;
        tliinef = sscanf(tline,'%f');
        navigation_data.V_list(id_V,:) = tliinef(2:end)';
    end
    tline = fgetl(f_handle);
end
fclose(f_handle);
end

