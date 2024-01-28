clear all;
close all;


name_list = ["sato","sakurai","oda","nanri","kusahuka","hanai"];
num_list = ["2", "3", "4"];
mount_list = ["left", "right"];

for num = num_list
    for mount = mount_list
        figure('Name',num + mount,'NumberTitle','off');
        plot_num = 0;
        for name = name_list
            plot_num = plot_num + 1;
            disp(name)
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
                norm_user(i) = norm([user(i, 1) user(i, 2) user(i, 3)]);
                norm_model(i) = norm([model(i, 1) model(i, 2) model(i, 3)]);
                norm_robot(i) = norm([robot(i, 1) robot(i, 2) robot(i, 3)]);
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

            model_x = model(index_list_1(1):index_list_2(1), 1);
            model_y = model(index_list_1(1):index_list_2(1), 3);
            model_z = model(index_list_1(1):index_list_2(1), 2);

            x_end = model_x(end) - model_x(1);
            x_start = model_x(1);
            normalized_x = [];
            for x = model_x
                diff = x - x_start;
                normalized_x = [normalized_x, diff/x_end];
            end

            y_end = model_y(end) - model_y(1);
            y_start = model_y(1);
            normalized_y = [];
            for y = model_y
                diff = y - y_start;
                normalized_y = [normalized_y, diff/y_end];
            end

            z_end = model_z(end) - model_z(1);
            z_start = model_z(1);
            normalized_z = [];
            for z = model_z
                diff = z - z_start;
                normalized_z = [normalized_z, 1-(diff/z_end)];
            end

            % グラフを作成
            subplot(2, 3, plot_num)
            title('First Subplot')

            % numel(normalized_z)
            % numel(normalized_x)

            
            % プロット
            plot3(normalized_x, normalized_y, normalized_z)
            % plot(time, norm_user, 'LineWidth', 2, 'DisplayName', 'user trajectory');
            hold on;
            % plot(time_1, model_1, 'LineWidth', 2, 'DisplayName', 'model_1 trajectory','Color','r');
            % plot(time_2, model_2, 'LineWidth', 2, 'DisplayName', 'model_2 trajectory','Color','r');
            % plot(time, norm_robot, 'LineWidth', 2, 'DisplayName', 'robot trajectory');
            
            % xregion(time(index_list_1(1)),time(index_list_2(1)+300))
            % xregion(time(index_list_1(2)),time(index_list_2(2)+300))
            
            % pbaspect([1 1 1])
            
            % 軸ラベルの設定
            xlabel('x');
            ylabel('y');
            zlabel('z');
            %
            
            % 凡例の表示
            % legend('Location', 'northwest', 'Box', 'off');
            
            % 軸の範囲の調整（必要に応じて）
            xlim([0 1]);
            ylim([0 1]);
            zlim([0 1]);
            % ylim([0.05 inf]);

            grid on
            
            % set(gca,'FontSize',16); 
            % set(gca,'linewidth',1);
            % set(gca,'FontName', 'Times New Roman');
            % set(gca,'FontAngle', 'normal');
        end
    end
end

