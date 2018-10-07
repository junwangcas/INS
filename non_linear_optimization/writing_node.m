function [ output_args ] = writing_node(navigation_data)
% writing bias;
f_handle = fopen(navigation_data.file_node_init,'w+');
fprintf(f_handle,'%d %f %f %f %f %f %f\n',1,navigation_data.bias);
% writing pose;
id_node = 1;
for i=1:length(navigation_data.R_list)
    id_node = id_node + 1;
    fprintf(f_handle,'%d %f %f %f\n',id_node,navigation_data.R_list(i,:));
    id_node = id_node + 1;
    fprintf(f_handle,'%d %f %f %f\n',id_node,navigation_data.T_list(i,:));
    id_node = id_node + 1;
    fprintf(f_handle,'%d %f %f %f\n',id_node,navigation_data.V_list(i,:));
end
fclose(f_handle);f_handle = fopen(navigation_data.file_node_init,'w+');
end

