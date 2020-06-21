function struct_map(structure)
%STRUCT_MAP Prints to a file the fields of a structure in tree form.
%
%    STRUCTREE(STRUCTURE,FILE_NAME) prints all the fieldnames of the
%    specified structure in a file. By default, the file will be saved 
%    in the Matlab's currrent directory. 

% Variable initialization
level = 1;                                                            
row   = [1 1];

% Structure initialization
structure(level,1).data = structure;                                  

%open file
% TODO

while ~isequal(level, 0)                
    % Check that current structure is really a structure
    flag = isa(structure(level, 1).data, 'struct'); 
    
    if (flag == 1)        
        
        if(row(level)==1)  %do this only if you are accesing this level for the first time
            
            % Get all the field names  
            Temp_names=fieldnames(structure(level,1).data);      

            Temp_size = size(Temp_names);                     %get number of fields
            Directory_size(level) = Temp_size(1);             %store number of fields
            Directory_names(1:Directory_size(level), level) = Temp_names;  %store field names in corresponding level
        end
        Display_name=Directory_names{row(level),level}; %Change field name from cell to arrray
        for index = 1 : level-1      %add necessary tabs
            fprintf('\t |');
        end
        
        var_type = class(structure(level,1).data.(Display_name));
        var_size = size(structure(level,1).data.(Display_name));
       
        fprintf('---');
        fprintf('%s' ,Display_name); 
        fprintf(2, '  %s [%sx%s]\n', var_type, num2str(var_size(1)), num2str(var_size(2))); 

        level = level + 1;                     %Go up one level
        row(level + 1) = 1;                    %Initialize row variable
        structure(level, 1).data = structure(level - 1, 1).data.(Display_name);  %Update current structure
    else
        %If current sutructure is not really a structure go down one level
        level = level - 1;
        
        %Go to next row
        row(level) = row(level) + 1;
        
        feedline = 0;
        
        %Loop if row number exceeds the number of fields
        while (row(level) > Directory_size(level))  
            
            % Go down one level
            level = level - 1;  

            if(feedline == 0)
                fprintf('\n');
                feedline = 1;
            end
            if (level == 0) 
                break;
            end
            row(level + 1) = 1;  
            row(level) = row(level) + 1;
        end
        if (level > 1)
            % Update structure
            structure(level, 1).data = getfield(structure(level - 1, 1).data,Directory_names{row(level - 1), level - 1});
        end 
        
    end
end


