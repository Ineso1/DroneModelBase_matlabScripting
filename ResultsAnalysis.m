classdef ResultsAnalysis

    properties (SetAccess = private)
        set_simulations
        num_simulations
        legends

        xlabel_value
        color_simulations

        standard_subtitles = {'\textbf{(a)}', '\textbf{(b)}', '\textbf{(c)}', '\textbf{(d)}', '\textbf{(e)}', '\textbf{(f)}'};

        norms = struct;

        num_samples

        print_settings = struct;

        line_width = 1.5;
        line_width_ref = 1.5;

        size_figure = struct('width', 20, 'height', 20);
    end

    methods
        function obj = ResultsAnalysis(set_simulations, colors, print_settings)
            if nargin < 3
                print_settings = struct;
                print_settings.save = false;
                print_settings.path = '';
                print_settings.code = '';
            end

            obj.set_simulations = set_simulations;
            obj.num_simulations = length(set_simulations);
            obj.num_samples = length(obj.set_simulations(1).time_array);

            % Compute norms
            obj = obj.computeNorms();

            % For plotting
            obj.legends = cell(1, obj.num_simulations); 

            for i = 1:obj.num_simulations
                obj.legends{i} = obj.set_simulations(i).name;
            end

            obj.color_simulations = colors;
            obj.xlabel_value = 'Time [s]';

            obj.print_settings = print_settings;
            
        end

        function showNormComparison(obj)
            figure(units = "centimeters", Position = [0, 0, obj.size_figure.width, obj.size_figure.height]);

            % Show the control norm
            subplot(3,1,1);
            hold on;
            grid minor;
            for i = 1:obj.num_simulations
                plot(obj.set_simulations(i).time_array, vecnorm(obj.set_simulations(i).torque_array)', 'Color', obj.color_simulations(i), 'LineWidth', obj.line_width);
                
            end

            legend(obj.legends,'interpreter','latex','Location','northoutside','Orientation','horizontal');
            xlabel(obj.xlabel_value,'interpreter','latex');
            ylabel('Torques','interpreter','latex');
            title(obj.standard_subtitles(1),'interpreter','latex');
            %yscale('log');

            % Show the error norm
            subplot(3,1,2);
            hold on;
            grid minor;
            for i = 1:obj.num_simulations
                plot(obj.set_simulations(i).time_array, vecnorm(obj.set_simulations(i).ep_array)', 'Color',  obj.color_simulations(i), 'LineWidth', obj.line_width);
            end
            xlabel(obj.xlabel_value,'interpreter','latex');
            ylabel('Error','interpreter','latex');
            title(obj.standard_subtitles(2),'interpreter','latex');

            % Show thrust value
            subplot(3,1,3);
            hold on;
            grid minor;
            for i = 1:obj.num_simulations
                plot(obj.set_simulations(i).time_array, obj.set_simulations(i).thrust_array, 'Color',  obj.color_simulations(i), 'LineWidth', obj.line_width);
            end
            xlabel(obj.xlabel_value,'interpreter','latex');
            ylabel('Thrust','interpreter','latex');
            title(obj.standard_subtitles(3),'interpreter','latex');

            if obj.print_settings.save
                print(gcf, strcat(obj.print_settings.path, 'general_comparison', obj.print_settings.code, '.pdf'), '-dpdf', '-r500', '-bestfit');
            end

            figure(units = "centimeters", Position = [0, 0, obj.size_figure.width, obj.size_figure.height]);
            titles = {'X rotational coordinate','Y rotational coordinate','Z rotational coordinate'};
            for i = 1:3    
                subplot(3,1,i)     
                hold on;
                grid minor;
                for j = 1:obj.num_simulations
                    plot(obj.set_simulations(j).time_array, obj.set_simulations(j).eq_array(i,:), 'Color',  obj.color_simulations(j), 'LineWidth', obj.line_width);
                end
                xlabel(obj.xlabel_value,'interpreter','latex');
                ylabel(string(titles(i)),'interpreter','latex');
                title(obj.standard_subtitles(i),'interpreter','latex');
                
                if i == 1
                    legend(obj.legends,'interpreter','latex','Location','northoutside','Orientation','horizontal');
                end
            end

            if obj.print_settings.save
                print(gcf, strcat(obj.print_settings.path, 'rotational_error', obj.print_settings.code, '.pdf'), '-dpdf', '-r500', '-bestfit');
            end

            figure(units = "centimeters", Position = [0, 0, obj.size_figure.width, obj.size_figure.height]);
            titles = {'X translational coordinate','Y translational coordinate','Z translational coordinate'};
            for i = 1:3    
                subplot(3,1,i)     
                hold on;
                grid minor;
                for j = 1:obj.num_simulations
                    plot(obj.set_simulations(i).time_array, obj.set_simulations(j).p_array(i,:), 'Color',  obj.color_simulations(j), 'LineWidth', obj.line_width);
                end
                yline(obj.set_simulations(1).p_d_array(i,end), 'k:', 'HandleVisibility','off',  'LineWidth', obj.line_width_ref);
                xlabel(obj.xlabel_value,'interpreter','latex');
                ylabel(titles(i),'interpreter','latex');
                title(obj.standard_subtitles(i),'interpreter','latex');
                
                if i == 1
                    legend(obj.legends,'interpreter','latex','Location','northoutside','Orientation','horizontal');
                end
            end

            if obj.print_settings.save
                print(gcf, strcat(obj.print_settings.path, 'translational_error', obj.print_settings.code, '.pdf'), '-dpdf', '-r500', '-bestfit');
            end


            figure(units = "centimeters", Position = [0, 0, obj.size_figure.width, obj.size_figure.height]); % show norms

            subplot(3,1,1);
            hold on;
            grid minor;
            for i = 1:obj.num_simulations
                plot(obj.set_simulations(i).time_array, obj.norms.iae(:,i), 'Color',  obj.color_simulations(i), 'LineWidth', obj.line_width);
            end
            legend(obj.legends,'interpreter','latex','Location','northoutside','Orientation','horizontal');
            xlabel(obj.xlabel_value,'interpreter','latex');
            ylabel('IAE','interpreter','latex');
            title(obj.standard_subtitles(1),'interpreter','latex');

            subplot(3,1,2);
            hold on;
            grid minor;
            for i = 1:obj.num_simulations
                plot(obj.set_simulations(i).time_array, obj.norms.ise(:,i), 'Color',  obj.color_simulations(i), 'LineWidth', obj.line_width);
            end
            xlabel(obj.xlabel_value,'interpreter','latex');
            ylabel('ISE','interpreter','latex');
            title(obj.standard_subtitles(2),'interpreter','latex');

            subplot(3,1,3);
            hold on;
            grid minor;
            for i = 1:obj.num_simulations
                plot(obj.set_simulations(i).time_array, obj.norms.itae(:,i), 'Color',  obj.color_simulations(i), 'LineWidth', obj.line_width);
            end
            xlabel(obj.xlabel_value,'interpreter','latex');
            ylabel('ITAE','interpreter','latex');
            title(obj.standard_subtitles(3),'interpreter','latex');

            if obj.print_settings.save
                print(gcf, strcat(obj.print_settings.path, 'norms', obj.print_settings.code, '.pdf'), '-dpdf', '-r500', '-bestfit');
            end
           
        end

        function obj = computeNorms(obj)
            % Preallocate the norms
            obj.norms.iae = zeros(obj.num_samples, obj.num_simulations);  % Integrated absolute error
            obj.norms.ise = zeros(obj.num_samples, obj.num_simulations);  % Integrated squared error
            obj.norms.itae = zeros(obj.num_samples, obj.num_simulations); % Integrated time absolute error
            dt = zeros(obj.num_simulations, 1);

            for i = 1:obj.num_simulations
                dt(i) = mean(diff(obj.set_simulations(i).time_array));
            end

            temp = cumsum(abs(obj.set_simulations(1).ep_array(1,:)).*dt(1))';
            size(temp)
            % Compute the norms
            for i = 1:obj.num_simulations
                    obj.norms.iae(:,i) = cumsum(sum(abs(obj.set_simulations(i).ep_array),1).*dt(i))';
                    obj.norms.ise(:,i) = cumsum(sum(obj.set_simulations(i).ep_array.^2,1).*dt(i))';
                    obj.norms.itae(:,i) = cumsum(obj.set_simulations(i).time_array.*sum(abs(obj.set_simulations(i).ep_array),1).*dt(i))';
            end
            
            % summary of computation
            for i = 1:obj.num_simulations
                sprintf('Simulation: %s', obj.set_simulations(i).name)
                sprintf('IAE: %f', obj.norms.iae(end,i))
                sprintf('ISE: %f', obj.norms.ise(end,i))
                sprintf('ITAE: %f', obj.norms.itae(end,i))
            end
            

        end
    end

end
