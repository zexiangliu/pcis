% test_set_minus.m

clear all;
close all;
clc;

%% Constants

%Create an example Where there are overlaps.
a = Polyhedron('lb',[-2 -2],'ub',[4 2]);
b = Polyhedron('lb',[2 -2],'ub',[4 4]);

figure;
subplot(3,1,1)
plot(a)
axis([-3 5 -3 5])

subplot(3,1,2)
plot(b)
axis([-3 5 -3 5])

subplot(3,1,3)
hold on;
plot(a); plot(b,'color','blue')
axis([-3 5 -3 5])

c = PolyUnion([a b]);
c.merge

d = Polyhedron('lb',[1 1],'ub',[3 3]);

figure;
hold on;
plot(a); plot(b,'color','blue')
plot(d,'color','green')
axis([-3 5 -3 5])

g = PolyUnion([a b]);
g.merge
g.add(d);

h = Polyhedron('lb', [3 5], 'ub',[5 7]);

figure;
hold on;
plot(a); plot(h,'color','blue');

j = PolyUnion([a h]);
j.merge;
try
	j.add(b)
catch
	disp('Script failed when trying to add ''b'', a polyhedron that overlaps with')
	disp('''a'' into the union.')
	disp('The ''merge'' operation made the union''s property ''Overlaps'' become 0,')
	disp('and thus we can no longer add polyhedra that overlap with any polyhedra')
	disp('that are already in the union.')
end

k = Polyhedron('lb',[2 2],'ub',[4 5]);

try
	j.add(k)
	disp('Correctly added ''k'' a non-overlapping Polyhedron.')
	plot(j)
catch
	disp('We cannot add Polyhedra like ''k'' to the union.')
end

%% Implementing a Set Minus Operation for a very specific situation:
% We are given a polyhedron, and must find the difference between this and a set of polyhedra given in PolyUnion.

disp(' ')
disp('Starting Set Difference Test.')
disp(' ')

m = Polyhedron('lb',[1 1],'ub',[4 6]);

%Plot the target set and the union
figure;
subplot(3,1,1)
plot(m)
axis([-2.5 5.5 -2.5 7.5 ])

subplot(3,1,2)
plot(j)
axis([-2.5 5.5 -2.5 7.5 ])

subplot(3,1,3)
hold on;
plot(j); plot(m,'color','blue')
axis([-2.5 5.5 -2.5 7.5 ])

%Calculate set differences between m and each individual polyhedron in the union
temp_set_diffs = {};

for i_j = 1:j.Num
	temp_set_diffs{i_j} = m \ j.Set(i_j);
end

%Create all possible combinations of each set.
list1 = [];
temp_list = [1:length(temp_set_diffs{1})]';
list1 = temp_list;

for j = 2:length(temp_set_diffs)
	temp_list = [1:length(temp_set_diffs{j})]';
	list1 = [ repmat(list1,length(temp_list),1) kron(temp_list,ones(size(list1,1),1)) ] ; 
end

%Now intersect for every row in list1
intersect_res1 = {};
figure; hold on;
for i_row = 1:size(list1,1)
	intersect_res1{i_row} = m;
	for i_col = 1:size(list1,2)
		intersect_res1{i_row} = intersect(intersect_res1{i_row},temp_set_diffs{i_col}(list1(i_row,i_col)));
	end

	if intersect_res1{i_row}.isEmptySet
		disp(['Combination #' num2str(i_row) ' results in empty set.' ])
	else
		plot(intersect_res1{i_row})
	end
end
axis([-2.5 5.5 -2.5 7.5 ])

%remove empty sets
intersect_res2 = {};
i_next = 1;
for i_row = 1:length(intersect_res1)
	temp_shrunken_set = intersect_res1{i_row} - 0.001*Polyhedron('lb',[-1 -1],'ub',[1 1]);
	if (~intersect_res1{i_row}.isEmptySet) && ( ~temp_shrunken_set.isEmptySet )
		intersect_res2{i_next} = intersect_res1{i_row};
		i_next = i_next + 1;
	end
end

%Make final set diff
res_diff = PolyUnion([intersect_res2{:}]);

disp('Completed the set difference operation.')
figure;
plot(res_diff)
axis([-2.5 5.5 -2.5 7.5 ])

