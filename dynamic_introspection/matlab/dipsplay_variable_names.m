function dipsplay_variable_names(bag_name)

msgs = read_introspection_bag(bag_name);
msgs{1}.doubles.name

end