close all;

agency_all_2 = [];
agency_control_all_2= [];
agency_all_3 = [];
agency_control_all_3 = [];
agency_all_4 = [];
agency_control_all_4 = [];

name_list = ["sugimoto", "kaneko", "ushida", "kato", "mizuno","hanai"];
condition_list = ["2", "3", "4"];
num_list = ["2", "3"];

for name = name_list
    for num = num_list
        data_2 = readmatrix(name+"/agency_2_"+num+".csv");
        data_2 = data_2(:, 2);
        agency_2 = data_2(1:3);
        agency_control_2 = data_2(4:6);
        
        agency_all_2 = [agency_all_2,agency_2];
        agency_control_all_2 = [agency_control_all_2, agency_control_2];
        
        data_3 = readmatrix(name+"/agency_3_"+num+".csv");
        data_3 = data_3(:, 2);
        agency_3 = data_3(1:3);
        agency_control_3 = data_3(4:6);
        
        agency_all_3 = [agency_all_3,agency_3];
        agency_control_all_3 = [agency_control_all_3, agency_control_3];
        
        data_4 = readmatrix(name+"/agency_4_"+num+".csv");
        data_4 = data_4(:, 2);
        agency_4 = data_4(1:3);
        agency_control_4 = data_4(4:6);
        
        agency_all_4 = [agency_all_4,agency_4];
        agency_control_all_4 = [agency_control_all_4, agency_control_4];

    end
end

% agency_control_all_3

score_agency_2 = sum(agency_all_2,"all")/numel(agency_all_2);
score_agency_control_2 = sum(agency_control_all_2,"all")/numel(agency_control_all_2);
score_agency_3 = sum(agency_all_3,"all")/numel(agency_all_3);
score_agency_control_3 = sum(agency_control_all_3,"all")/numel(agency_control_all_3);
score_agency_4 = sum(agency_all_4,"all")/numel(agency_all_4);
score_agency_control_4 = sum(agency_control_all_4,"all")/numel(agency_control_all_4);


agency_2 = [];
agency_all_2 = sum(agency_all_2, 1)/3;
for i = 1:6
    agency_2 = [agency_2, (agency_all_2(i+(i-1))+agency_all_2(i+(i-1)+1))/2];
end

agency_control_2 = [];
agency_control_all_2 = sum(agency_control_all_2, 1)/3;
for i = 1:6
    agency_control_2 = [agency_control_2, (agency_control_all_2(i+(i-1))+agency_control_all_2(i+(i-1)+1))/2];
end

agency_3 = [];
agency_all_3 = sum(agency_all_3, 1)/3;
for i = 1:6
    agency_3 = [agency_3, (agency_all_3(i+(i-1))+agency_all_3(i+(i-1)+1))/2];
end

agency_control_3 = [];
agency_control_all_3 = sum(agency_control_all_3, 1)/3;
for i = 1:6
    agency_control_3 = [agency_control_3, (agency_control_all_3(i+(i-1))+agency_control_all_3(i+(i-1)+1))/2];
end

agency_4 = [];
agency_all_4 = sum(agency_all_4, 1)/3;
for i = 1:6
    agency_4 = [agency_4, (agency_all_4(i+(i-1))+agency_all_4(i+(i-1)+1))/2];
end

agency_control_4 = [];
agency_control_all_4 = sum(agency_control_all_4, 1)/3;
for i = 1:6
    agency_control_4 = [agency_control_4, (agency_control_all_4(i+(i-1))+agency_control_all_4(i+(i-1)+1))/2];
end

std_a_2 = std(agency_2)
std_a_3 = std(agency_3)
std_a_4 = std(agency_4)
std_c_2 = std(agency_control_2)
std_c_3 = std(agency_control_3)
std_c_4 = std(agency_control_4)


x = categorical({'1. Without feedback','2. Vibrotactile feedback','3. Visual feedback'});
x = reordercats(x,{'1. Without feedback','2. Vibrotactile feedback','3. Visual feedback'});
ylim([0 7])
% y = [[6.04, 3.63, 4.75, 4.86], [1.75, 4.875, 4.292, 3.958]];
y = [score_agency_2 score_agency_control_2; score_agency_3 score_agency_control_3; score_agency_4 score_agency_control_4];
% err = [[0.772, 1.16, 1.14, 0.957], [1.02, 0.832, 1.32, 0.92]];
err = [std_a_2 std_c_2; std_a_3 std_c_3; std_a_4 std_c_4;];

bar(x,y);
xtickangle(0)
pbaspect([2 0.8 1])
hold on

ngroups = size(y, 1);
nbars = size(y, 2);
% Calculating the width for each bar group
groupwidth = min(0.8, nbars/(nbars + 1.5));
for i = 1:nbars
    x = (1:ngroups) - groupwidth/2 + (2*i-1) * groupwidth / (2*nbars);
    errorbar(x, y(:,i), err(:,i), 'k', 'linestyle', 'none');
end

hold off

%軸などの設定
fp = {'FontName', 'Times New Roman','FontWeight','bold'} ;
ylim([0 7])
ylabel('Score', fp{:}, 'fontsize', 18) 
ax = gca;
ax.XAxis(1).Color = [0 0 0];
% set(gca,'FontWeight','bold'); 
set(gca,'FontSize',12); 
set(gca,'linewidth',1);
set(gca,'FontName', 'Times New Roman');
set(gca,'FontAngle', 'normal');
% title('Questionnaire',fp{:},'fontsize', 15)
legend('Agency','Agency Control');
box off
