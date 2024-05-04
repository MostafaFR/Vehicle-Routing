clc
p = 0.02777;
r = 0.25;
total_packs = 10000;
check = 100;
%while check >= 10
good = 1;
packets = [];
size = 1;
while size <= total_packs

end
size = size + 1;
end
fid = fopen('Loss_Pattern.txt','w');
fprintf(fid, '%d ', packets);
fclose(fid);

%end
p