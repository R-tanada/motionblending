% clear all;
close all;

plot_num = 0;
name_list = ["sato","sakurai","oda","nanri","kusahuka","hanai"];
num_list = ["3"];
mount_list = ["right"];

for name = name_list
    plot_num = plot_num + 1;
    disp(name)
    for num = num_list
        for mount = mount_list

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

            % 速度計算
            interval = 30;
            vel_user = [];
            for i = 1:numel(norm_user)-interval
                vel_user(i) = (norm_user(i+interval) - norm_user(i))/(time(i+interval)-time(i));
            end
            vel_model_1 = [];
            for i = 1:numel(model_1)-interval
                vel_model_1(i) = (model_1(i+interval) - model_1(i))/(time_1(i+interval)-time_1(i));
            end
            vel_model_2 = [];
            for i = 1:numel(model_2)-interval
                vel_model_2(i) = (model_2(i+interval) - model_2(i))/(time_2(i+interval)-time_2(i));
            end
            vel_robot = [];
            for i = 1:numel(norm_robot)-interval
                vel_robot(i) = (norm_robot(i+interval) - norm_robot(i))/(time(i+interval)-time(i));;
            end
                    
            % グラフを作成
            % figure;
            subplot(2, 3, plot_num)
            
            % プロット
            % plot(time, norm_user, 'LineWidth', 2, 'DisplayName', 'user trajectory');
            % hold on;
            % plot(time_1, model_1, 'LineWidth', 2, 'DisplayName', 'model_1 trajectory','Color','r');
            % plot(time_2, model_2, 'LineWidth', 2, 'DisplayName', 'model_2 trajectory','Color','r');
            % plot(time, norm_robot, 'LineWidth', 2, 'DisplayName', 'robot trajectory');
            
            % 速度プロット
            plot(time(1:numel(time)-interval), vel_user, 'LineWidth', 2, 'DisplayName', 'user trajectory');
            hold on;
            plot(time_1(1:numel(time_1)-interval), vel_model_1, 'LineWidth', 2, 'DisplayName', 'model_1 trajectory','Color','r');
            plot(time_2(1:numel(time_2)-interval), vel_model_2, 'LineWidth', 2, 'DisplayName', 'model_2 trajectory','Color','r');
            plot(time(1:numel(time)-interval), vel_robot, 'LineWidth', 2, 'DisplayName', 'robot trajectory');
            
            xregion(time(index_list_1(1)),time(index_list_2(1)+300))
            xregion(time(index_list_1(2)),time(index_list_2(2)+300))
            
            % pbaspect([1 1 1])
            
            % 軸ラベルの設定
            xlabel('Normalized time');
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
        end
    end
end

