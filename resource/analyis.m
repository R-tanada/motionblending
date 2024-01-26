clear all;
% close all;


name = "kusahuka";
num = "4";
mount = "right";


time = readmatrix(name+"/time_data/"+num+"_"+mount+".csv");
user = readmatrix(name+"/user_data/"+num+"_"+mount+".csv");
model = readmatrix(name+"/auto_data/"+num+"_"+mount+".csv");
robot = readmatrix(name+"/robot_data/"+num+"_"+mount+".csv");

n = numel(time);
s = 0;
user = user(s+1:n+s, :);
model = model(s+1:n+s, :);
robot = robot(s+1:n+s, :);

% size(time)
% size(user)
% size(model)
% size(robot)

norm_user=[];
norm_model=[];
norm_robot=[];
for i = 1:numel(model(:, 1))
    norm_user(i) = norm([user(i, 1) user(i, 3)]);
    norm_model(i) = norm([model(i, 1) model(i, 3)]);
    norm_robot(i) = norm([robot(i, 1) robot(i, 3)]);
end

% time = linspace(0,time(end),numel(user(:, 1)));

flag = 1;
before_val = 0;
ans = 0;
index_list_1 = [];
index_list_2 = [];

model_list = norm_model;
% modelデータの0との境目を検出
for index = 1:numel(model_list)
    ans = before_val * model_list(index);
    before_val = model_list(index);
    if flag == 1
        if ans ~= 0
            flag = 2;
            index_list_1 = [index_list_1, index-1];
        end
    elseif flag == 2
        if ans == 0
            flag = 1;
            index_list_2 = [index_list_2, index-1];
        end
    end
end

% ユーザーデータのオフセットを修正
diff = norm_user(index_list_2(1)+1) - norm_user(index_list_2(1)+2);
diff_list = linspace(diff,0,300);
for j = 1:numel(diff_list)
    norm_user(j+1+index_list_2(1)) = norm_user(j+1+index_list_2(1)) + diff_list(j);
end

diff = norm_user(index_list_2(2)+1) - norm_user(index_list_2(2)+2);
diff_list = linspace(diff,0,300);
for j = 1:numel(diff_list)
    norm_user(j+1+index_list_2(2)) = norm_user(j+1+index_list_2(2)) + diff_list(j);
end

norm_robot(index_list_2(1)+1) = norm_robot(index_list_2(1)+2);
norm_robot(index_list_2(2)+1) = norm_robot(index_list_2(2)+2);

% disp(index_list_1)
% disp(index_list_2)
            
time_1 = time(index_list_1(1):index_list_2(1));
model_1 = norm_model(index_list_1(1):index_list_2(1));
time_2 = time(index_list_1(2):index_list_2(2));
model_2 = norm_model(index_list_1(2):index_list_2(2));

% モデル軌道のリーチング後の軌道を追加,時間も
% diff = model_1(end)-norm_user(index_list_2(1)+2);
% for j = 1:numel(diff_list)
%     model_1 = [model_1, norm_user(j+1+index_list_2(1)) + diff];
% end
% for j = 1:numel(diff_list)
%     time_1 = [model_1, norm_user(j+1+index_list_2(1)) + diff];
% end
% diff = model_2(end)-norm_user(index_list_2(2)+2);
% for j = 1:numel(diff_list)
%     model_2 = [model_2, norm_user(j+1+index_list_2(2)) + diff];
% end

% norm_robot(index_list_2(1)) = norm_robot(index_list_2(1)-1)
% norm_robot(index_list_2(2)) = norm_robot(index_list_2(2)-1)

% disp(model_2)
        
% グラフを作成

% プロット
plot(time, norm_user, 'LineWidth', 2, 'DisplayName', 'user trajectory');
hold on;
% plot(time_1, model_1, 'LineWidth', 2, 'DisplayName', 'model_1 trajectory','Color','r');
% plot(time_2, model_2, 'LineWidth', 2, 'DisplayName', 'model_2 trajectory','Color','r');
plot(time, norm_robot, 'LineWidth', 2, 'DisplayName', 'robot trajectory');

xregion(time(index_list_1(1)),time(index_list_2(1)+300))
xregion(time(index_list_1(2)),time(index_list_2(2)+300))

% pbaspect([1 1 1])

% 軸ラベルの設定
xlabel(name);
ylabel('Normalized norm');

% 凡例の表示
% legend('Location', 'northwest', 'Box', 'off');

% 軸の範囲の調整（必要に応じて）
xlim([time(1) time(end)]);
% ylim([0.05 inf]);

% set(gca,'FontSize',16); 
% set(gca,'linewidth',1);
% set(gca,'FontName', 'Times New Roman');
% set(gca,'FontAngle', 'normal');

