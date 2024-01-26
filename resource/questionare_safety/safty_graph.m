close all;

all_2 = [];
all_3 = [];
all_4 = [];

name_list = ["sugimoto", "kaneko", "ushida", "kato", "mizuno","hanai"];
condition_list = ["2", "3", "4"];
num_list = ["2", "3"];

for name = name_list
    for num = num_list
        data_2 = readmatrix(name+"/safety_2_"+num+".csv");
        data_2 = data_2(:, 2);
        data_2(3) = 8 - data_2(3);
        
        all_2 = [all_2,data_2];
        
        data_3 = readmatrix(name+"/safety_3_"+num+".csv");
        data_3 = data_3(:, 2);
        data_3(3) = 8 - data_3(3);
        
        all_3 = [all_3,data_3];
        
        data_4 = readmatrix(name+"/safety_4_"+num+".csv");
        data_4 = data_4(:, 2);
        data_4(3) = 8 - data_4(3);
        
        all_4 = [all_4,data_4];

    end
end

score_2 = sum(all_2,"all")/numel(all_2)
score_3 = sum(all_3,"all")/numel(all_3)
score_4 = sum(all_4,"all")/numel(all_4)

% data = load('result_all.txt');
data = [score_2, score_3, score_4];
 
%棒グラフ作成
figure();
x = categorical({'1. Without feedback','2. Vibrotactile feedback','3. Visual feedback'});
x = reordercats(x,{'1. Without feedback','2. Vibrotactile feedback','3. Visual feedback'});
y=data(1,:);
bar_graph=bar(x,y, 0.4);
pbaspect([2 1 1])
 
%色分け
% hold on
% bar(x, [y(1) NaN NaN], 'FaceColor',[0.0745, 0.6706, 0.5922]);
% bar(x, [NaN y(2) NaN], 'FaceColor',[0.0824, 0.5216, 0.5882]);
% bar(x, [NaN NaN y(3)], 'FaceColor',[0.0588, 0.3137, 0.4392]);
% hold on
 
%標準偏差
% hold on
% error=data(2,:);
% error = [9.85, 2.8, 3.35, 2.77];
% er = errorbar(x,y,error,error);    
% er.Color = [0 0 0];                            
% er.LineStyle = 'none';  
% hold off
 
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
% title('NASA TLX',fp{:},'fontsize', 15)
box off
