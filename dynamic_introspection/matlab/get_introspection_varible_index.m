function index = get_introspection_varible_index(msgs, variable_name, type)
 if(strcmp(type, 'double'))
   number_variables = size(msgs{1}.doubles, 2);
   for i = 1:number_variables
     if strcmp(msgs{1}.doubles(i).name, variable_name)
       index = i;
       break;
     end
   end
 else
   display('ERROR: Type not supported')
 end
 
end
