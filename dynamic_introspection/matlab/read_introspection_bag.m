function msgs = read_introspection_bag(bag_name)
 bag = ros.Bag.load(bag_name);
 topic1 = '/introspection_data';
 msgs = bag.readAll(topic1);
end
