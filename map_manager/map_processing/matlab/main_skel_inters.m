%-----------running find_skeleleton_intersation----------------%

clc
clear
close all

filename = '../dataset/ndt.pgm';

I = imread(filename);

input_skeleton_image = bwmorph(I,'thin',inf);
intersecting_pts = find_skel_intersection(input_skeleton_image);
figure,imshow(input_skeleton_image);
hold on; 
plot(intersecting_pts(:,1),intersecting_pts(:,2),'r*');
impixelregion;
manual_initial_guess = [1589, 1734;
                        1418, 1600;
                        1458, 1553;
                        1020, 1171;
                        1063, 1101;
                        832, 871;
                        822, 881;
                        980, 605;
                        977, 598;
                        995, 486;
                        1018, 447;
                        1021, 442;
                        1137, 167;
                        1144, 158;
                        1179, 109;
                        1171, 99];
save ('inters_point_manual.dat', 'manual_initial_guess', '-ascii', '-tabs');
fid = fopen('inters_point_manual.txt','w');
fprintf(fid,'%u %u\n',manual_initial_guess);
fclose(fid);
