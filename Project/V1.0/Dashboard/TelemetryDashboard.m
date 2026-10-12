classdef TelemetryDashboard < handle
% TelemetryDashboard - Race telemetry dashboard (single-file programmatic app)
%
%   app = TelemetryDashboard;                  % open, then click "Load file..."
%   app = TelemetryDashboard('session.mat');   % open and load a session
%
% Requires MATLAB R2020b or later. Supports CSV and MAT telemetry files.
%
% MAT input may contain simulator variables (t, lap, lapTime, x, y, s,
% yaw, yawRate, speed, ax, ay, steerAngle, driveForceCmd, brakeForce, ...)
% or channel structs. CSV files should contain equivalent named columns.
%
% Mouse / keyboard:
%   - click a chart or the track  -> move the cursor there
%   - mouse wheel / drag on charts -> zoom / pan (all charts are linked)
%   - mouse wheel / drag on the track -> zoom / pan the circuit
%   - "Fit all" restores the normal scale for the charts and track
%   - click a row of the corner/sector table -> jump to it and zoom on it
%   - Space = play/pause, Left/Right arrows = step -/+ 0.5 s

    %% ------------------------------------------------------------------
    %  Settings (change them, then click "Reload")
    %  ------------------------------------------------------------------
    properties (Access = public)
        Decim              = 10       % display decimation (1 kHz / 10 = 100 Hz)
        WheelBase          = NaN      % [m]      NaN = auto-estimate from data
        UndersteerGradient = NaN      % [s^2/m]  NaN = auto-estimate from data
        SteeringRatio      = 1        % steering-wheel angle / road-wheel angle
        WheelRadius        = NaN      % [m]      NaN = derived from wheelTorque/driveForce
        GearRatio          = NaN      % motor->wheel ratio, NaN = derived from torques
        CornerCurvature    = 1/250    % [1/m] minimum curvature to count as a corner
        CornerMinLength    = 20       % [m]
        CornerMergeGap     = 40       % [m] corners closer than this are merged
        NumSectors         = 3
        FlipGArrow         = false    % false: force felt by the driver, true: car acceleration
    end

    properties (Constant, Access = private)
        BG    = [0.10 0.11 0.13]
        PANEL = [0.15 0.16 0.18]
        BTN   = [0.24 0.26 0.29]
        FG    = [0.92 0.93 0.95]
        MUTED = [0.60 0.62 0.65]
        C1    = [0.30 0.75 1.00]   % selected lap
        CG    = [0.25 0.85 0.45]
        CR    = [1.00 0.35 0.30]
        CY    = [1.00 0.80 0.20]
        G0    = 9.81
    end

    %% ------------------------------------------------------------------
    %  State
    %  ------------------------------------------------------------------
    properties (Access = private)
        % data
        Laps     = {}
        SelIdx   = 1
        RefIdx   = 0
        BestIdx  = 1
        FilePath = ''
        LastDir  = ''
        Delta    = []
        SignAy   = 1
        SignSteer = 1
        Lw       = 2.7
        Kus      = 0
        CornerS  = zeros(0,2)
        TrackLen = 0
        HeadLen  = 10
        BalScale = 10
        TrailN   = 100
        TrackXLim = [-1 1]
        TrackYLim = [-1 1]
        DragAx = []
        DragStartPixel = [0 0]
        DragStartXLim = [0 1]
        DragStartYLim = [0 1]
        DragMoved = false
        % playback
        Tc = 0
        Dir = 1
        Rate = 1
        Playing = false
        Timer
        LastTic
        % ui
        Fig
        LblFile
        DDLap
        DDRef
        DDColor
        DDXAxis
        DDView
        DDRate
        Slider
        LblTime
        AxTrack
        Cbar
        Ax = gobjects(1,5)
        Cur = gobjects(1,5)
        AxG
        AxBal
        AxSV
        AxRpm
        GAccelTxt
        GBrakeTxt
        RpmLeds = gobjects(1,10)
        RpmMax = 1
        LblRpm
        Tbl
        H = struct()
        GDecor = gobjects(0)
        CornerTxt = gobjects(0)
    end

    %% ------------------------------------------------------------------
    methods (Access = public)
        function app = TelemetryDashboard(file)
            app.LastDir = pwd;
            app.buildUI();
            app.Fig.WindowScrollWheelFcn = @(~,e) app.onScroll(e);
            app.Fig.WindowButtonDownFcn = @(~,~) app.onMouseDown();
            app.Fig.WindowButtonMotionFcn = @(~,~) app.onMouseMove();
            app.Fig.WindowButtonUpFcn = @(~,~) app.onMouseUp();
            if nargin > 0 && ~isempty(file)
                app.loadFile(file);
            end
        end

        function closeApp(app)
            app.pausePlayback();
            if ~isempty(app.Timer) && isvalid(app.Timer)
                delete(app.Timer);
            end
            if ~isempty(app.Fig) && isvalid(app.Fig)
                delete(app.Fig);
            end
        end
    end

    %% ------------------------------------------------------------------
    %  UI construction
    %  ------------------------------------------------------------------
    methods (Access = private)

        function buildUI(app)
            C = app.BG;
            app.Fig = uifigure('Name','Telemetry Dashboard','Position',[40 40 1600 920],'Color',C);
            app.Fig.CloseRequestFcn = @(~,~) app.closeApp();
            app.Fig.KeyPressFcn = @(~,e) app.onKey(e);

            root = uigridlayout(app.Fig,[3 1],'RowHeight',{40,'1x',44}, ...
                'Padding',[8 8 8 8],'RowSpacing',6,'BackgroundColor',C);

            % ---------------- top bar ----------------
            tb = uigridlayout(root,[1 12],'ColumnWidth',{95,70,'1x',30,150,30,150,75,130,50,110,70}, ...
                'Padding',[0 0 0 0],'ColumnSpacing',6,'BackgroundColor',C);
            app.mkButton(tb,'Open CSV / MAT...',@(~,~) app.onLoad());
            app.mkButton(tb,'Reload',@(~,~) app.onReload());
            app.LblFile = uilabel(tb,'Text','No file loaded','FontColor',app.MUTED,'FontSize',12);
            app.mkLabel(tb,'Lap');
            app.DDLap = app.mkDropdown(tb,{'-'},@(~,~) app.onLapChange());
            app.mkLabel(tb,'Ref');
            app.DDRef = app.mkDropdown(tb,{'None'},@(~,~) app.onRefChange());
            app.mkLabel(tb,'Track color');
            app.DDColor = app.mkDropdown(tb,{'Speed','Longitudinal G','Lateral G'},@(~,~) app.updateTrackColor());
            app.mkLabel(tb,'X axis');
            app.DDXAxis = app.mkDropdown(tb,{'Distance','Time'},@(~,~) app.onXAxisChange());
            app.DDXAxis.ItemsData = {'Distance','Time'};
            app.mkButton(tb,'Fit all',@(~,~) app.fitView());

            % ---------------- bottom bar ----------------
            bb = uigridlayout(root,[1 6],'ColumnWidth',{80,80,80,90,'1x',380}, ...
                'Padding',[0 0 0 0],'ColumnSpacing',6,'BackgroundColor',C);
            app.mkButton(bb,'Reverse',@(~,~) app.startPlayback(-1));
            app.mkButton(bb,'Pause',@(~,~) app.pausePlayback());
            app.mkButton(bb,'Play',@(~,~) app.startPlayback(1));
            app.DDRate = app.mkDropdown(bb,{'0.1x','0.25x','0.5x','1x','2x','5x','10x'},@(~,~) app.onRate());
            app.DDRate.ItemsData = [0.1 0.25 0.5 1 2 5 10];
            app.DDRate.Value = 1;
            app.Slider = uislider(bb,'Limits',[0 1],'Value',0,'MajorTicks',[],'MinorTicks',[], ...
                'ValueChangingFcn',@(~,e) app.onSlide(e.Value), ...
                'ValueChangedFcn',@(s,~) app.onSlide(s.Value));
            app.LblTime = uilabel(bb,'Text','','FontColor',app.FG,'FontSize',12,'HorizontalAlignment','right');

            % ---------------- main area ----------------
            main = uigridlayout(root,[1 3],'ColumnWidth',{'1.0x','1.8x',350}, ...
                'Padding',[0 0 0 0],'ColumnSpacing',8,'BackgroundColor',C);
            tb.Layout.Row = 1; main.Layout.Row = 2; bb.Layout.Row = 3;   % fix: explicit rows

            % left: track + table
            lg = uigridlayout(main,[2 1],'RowHeight',{'1.25x','1x'},'Padding',[0 0 0 0], ...
                'RowSpacing',8,'BackgroundColor',C);
            app.AxTrack = uiaxes(lg);
            app.styleAxes(app.AxTrack);
            disableDefaultInteractivity(app.AxTrack);
            tg = uigridlayout(lg,[2 1],'RowHeight',{26,'1x'},'Padding',[0 0 0 0], ...
                'RowSpacing',4,'BackgroundColor',C);
            app.DDView = app.mkDropdown(tg,{'Corners','Sectors'},@(~,~) app.updateTable());
            app.Tbl = uitable(tg,'RowName',{},'ColumnName',{},'FontSize',11, ...
                'BackgroundColor',[0.16 0.17 0.19; 0.19 0.20 0.22],'ForegroundColor',app.FG, ...
                'CellSelectionCallback',@(~,e) app.onTableSelect(e));

            % center: stacked charts
            cg = uigridlayout(main,[5 1],'RowHeight',{'1.2x','1x','1x','1x','1x'}, ...
                'Padding',[0 0 0 0],'RowSpacing',4,'BackgroundColor',C);
            for k = 1:5
                app.Ax(k) = uiaxes(cg);
                app.styleAxes(app.Ax(k));
                if k < 5, app.Ax(k).XTickLabel = {}; end
            end
            linkaxes(app.Ax,'x');
            for k = 1:5
                disableDefaultInteractivity(app.Ax(k));
            end

            % right: instruments
            rg = uigridlayout(main,[4 1],'RowHeight',{'0.8x','1.1x','0.55x','1x'}, ...
                'Padding',[0 0 0 0],'RowSpacing',6,'BackgroundColor',C);
            gg = uigridlayout(rg,[2 1],'RowHeight',{'1x',30},'ColumnWidth',{'1x'}, ...
                'Padding',[0 0 0 0],'RowSpacing',0,'BackgroundColor',C);
            app.AxRpm = uiaxes(gg);
            app.styleAxes(app.AxRpm);
            app.AxRpm.XLim = [0 10]; app.AxRpm.YLim = [0 1];
            app.AxRpm.XTick = []; app.AxRpm.YTick = [];
            app.AxRpm.XGrid = 'off'; app.AxRpm.YGrid = 'off';
            app.AxRpm.Box = 'off';
            disableDefaultInteractivity(app.AxRpm);
            title(app.AxRpm,'RPM','Color',app.MUTED,'FontWeight','normal');
            ledColors = [repmat([0.25 0.85 0.45],5,1); ...
                repmat([1.00 0.80 0.20],3,1); repmat([1.00 0.35 0.30],2,1)];
            for k = 1:10
                app.RpmLeds(k) = rectangle(app.AxRpm,'Position',[k-0.86 0.28 0.72 0.42], ...
                    'Curvature',0.15,'FaceColor',app.PANEL,'EdgeColor',ledColors(k,:),'LineWidth',1);
            end
            app.LblRpm = uilabel(gg,'Text','0 rpm','FontSize',20,'FontWeight','bold', ...
                'FontColor',app.FG,'HorizontalAlignment','center');
            app.AxG = uiaxes(rg);   app.styleAxes(app.AxG);   disableDefaultInteractivity(app.AxG);
            app.AxBal = uiaxes(rg); app.styleAxes(app.AxBal); disableDefaultInteractivity(app.AxBal);
            app.AxSV = uiaxes(rg);  app.styleAxes(app.AxSV);

            app.initPlots();
        end

        function b = mkButton(app, parent, txt, cb)
            b = uibutton(parent,'push','Text',txt,'ButtonPushedFcn',cb, ...
                'BackgroundColor',app.BTN,'FontColor',app.FG);
        end

        function l = mkLabel(app, parent, txt)
            l = uilabel(parent,'Text',txt,'FontColor',app.FG,'HorizontalAlignment','right');
        end

        function d = mkDropdown(app, parent, items, cb)
            d = uidropdown(parent,'Items',items,'ValueChangedFcn',cb, ...
                'BackgroundColor',app.BTN,'FontColor',app.FG);
        end

        function styleAxes(app, ax)
            ax.Color = app.PANEL;
            ax.BackgroundColor = app.PANEL;
            ax.XColor = app.FG; ax.YColor = app.FG;
            ax.GridColor = app.FG; ax.GridAlpha = 0.15;
            ax.XGrid = 'on'; ax.YGrid = 'on';
            ax.Box = 'on'; ax.FontSize = 10;
            ax.Toolbar.Visible = 'off';
            hold(ax,'on');
        end

        function mkLegend(app, ax, handles)
            legend(ax,handles,'Location','northeast','TextColor',app.FG,'Color',app.PANEL, ...
                'EdgeColor','none','FontSize',9,'AutoUpdate','off');
        end

        function initPlots(app)
            F = app.FG; H = struct(); %#ok<*PROP>

            % ---- track ----
            ax = app.AxTrack;
            title(ax,'Track','Color',F,'FontWeight','normal');
            ax.XTick = []; ax.YTick = []; ax.XGrid = 'off'; ax.YGrid = 'off';
            H.trkRef = plot(ax,NaN,NaN,'-','Color',[0.5 0.5 0.5],'LineWidth',1,'PickableParts','none');
            H.trk = scatter(ax,0,0,9,0,'filled','PickableParts','none');
            H.head = plot(ax,NaN,NaN,'-','Color','w','LineWidth',2,'PickableParts','none');
            H.car = plot(ax,NaN,NaN,'o','MarkerSize',10,'MarkerFaceColor','w', ...
                'MarkerEdgeColor','k','PickableParts','none');
            colormap(ax,'turbo');
            app.Cbar = colorbar(ax,'Color',F);

            % ---- stacked charts ----
            names = {'Speed (km/h)','Throttle / Brake (%)','Steering (deg)','Yaw rate (deg/s)','Delta vs ref (s)'};
            for k = 1:5
                ylabel(app.Ax(k),names{k},'Color',F);
            end
            xlabel(app.Ax(5),'Distance (m)','Color',F);
            mk = @(a,c,varargin) plot(a,NaN,NaN,'Color',c,'LineWidth',1.4,'PickableParts','none',varargin{:});

            H.spdR = mk(app.Ax(1),[0.6 0.6 0.6],'DisplayName','Ref');
            H.spd  = mk(app.Ax(1),app.C1,'DisplayName','Lap');
            H.thrR = mk(app.Ax(2),app.CG,'LineStyle',':');
            H.brkR = mk(app.Ax(2),app.CR,'LineStyle',':');
            H.thr  = mk(app.Ax(2),app.CG,'DisplayName','Throttle');
            H.brk  = mk(app.Ax(2),app.CR,'DisplayName','Brake');
            H.strR = mk(app.Ax(3),[0.6 0.6 0.6]);
            H.str  = mk(app.Ax(3),app.C1);
            H.yawE = mk(app.Ax(4),app.CY,'LineStyle','--','DisplayName','Expected from steering');
            H.yaw  = mk(app.Ax(4),app.C1,'DisplayName','Actual');
            H.dlt  = mk(app.Ax(5),app.CY);
            yline(app.Ax(5),0,'Color',[0.6 0.6 0.6],'PickableParts','none');
            for k = 1:5
                app.Cur(k) = xline(app.Ax(k),0,'Color',[1 1 1],'LineWidth',1.2,'PickableParts','none');
            end
            app.mkLegend(app.Ax(1),[H.spd H.spdR]);
            app.mkLegend(app.Ax(2),[H.thr H.brk]);
            app.mkLegend(app.Ax(4),[H.yaw H.yawE]);
            app.Ax(2).YLim = [-5 105];

            % ---- G circle ----
            ax = app.AxG;
            ax.XTick = []; ax.YTick = []; ax.XGrid = 'off'; ax.YGrid = 'off';
            H.garr = quiver(ax,0,0,0,0,0,'Color',app.CY,'LineWidth',3,'MaxHeadSize',0.9,'PickableParts','none');
            H.gdot = plot(ax,0,0,'o','MarkerFaceColor',app.CY,'MarkerEdgeColor','none', ...
                'MarkerSize',9,'PickableParts','none');
            title(ax,'G-force','Color',F,'FontWeight','normal');

            % ---- balance indicator ----
            ax = app.AxBal;
            ax.XTick = []; ax.YTick = []; ax.XGrid = 'off'; ax.YGrid = 'off';
            ax.XLim = [-1 1]; ax.YLim = [0 1];
            db = 0.15;
            patch(ax,[-1 -db -db -1],[0.25 0.25 0.7 0.7],[0.25 0.45 0.85],'FaceAlpha',0.6,'EdgeColor','none','PickableParts','none');
            patch(ax,[-db db db -db],[0.25 0.25 0.7 0.7],[0.25 0.7 0.4],'FaceAlpha',0.6,'EdgeColor','none','PickableParts','none');
            patch(ax,[db 1 1 db],[0.25 0.25 0.7 0.7],[0.95 0.55 0.2],'FaceAlpha',0.6,'EdgeColor','none','PickableParts','none');
            text(ax,-0.55,0.1,'UNDERSTEER','Color',F,'FontSize',9,'HorizontalAlignment','center','PickableParts','none');
            text(ax,0.55,0.1,'OVERSTEER','Color',F,'FontSize',9,'HorizontalAlignment','center','PickableParts','none');
            H.balMark = plot(ax,[0 0],[0.15 0.8],'-','Color','w','LineWidth',5,'PickableParts','none');
            title(ax,'Handling balance','Color',F,'FontWeight','normal');

            % ---- steering vs throttle ----
            ax = app.AxSV;
            title(ax,'Steering / throttle (last second)','Color',F,'FontWeight','normal');
            xlabel(ax,'Steering (+ left, deg)','Color',F); ylabel(ax,'Throttle (%)','Color',F);
            ax.YLim = [-5 105];
            xline(ax,0,'Color',[0.55 0.57 0.60],'LineStyle',':','PickableParts','none');
            yline(ax,50,'Color',[0.55 0.57 0.60],'LineStyle',':','PickableParts','none');
            H.sv = plot(ax,NaN,NaN,'.','Color',[0.5 0.55 0.6],'MarkerSize',4,'PickableParts','none');
            H.svTrail = plot(ax,NaN,NaN,'-','Color',app.C1,'LineWidth',1.5,'PickableParts','none');
            H.svDot = plot(ax,NaN,NaN,'o','MarkerFaceColor',app.C1,'MarkerEdgeColor','w', ...
                'MarkerSize',8,'PickableParts','none');

            app.H = H;
            app.buildGCircle(2);
        end

        function buildGCircle(app, gm)
            ax = app.AxG; H = app.H;
            delete(app.GDecor); app.GDecor = gobjects(0);
            th = linspace(0,2*pi,181);
            gray = [0.40 0.42 0.45];
            for r = 0.5:0.5:gm
                app.GDecor(end+1) = plot(ax,r*cos(th),r*sin(th),'-','Color',gray,'PickableParts','none');
                app.GDecor(end+1) = text(ax,r*0.7071,r*0.7071,sprintf('%.1f g',r),'Color',app.MUTED, ...
                    'FontSize',8,'PickableParts','none');
            end
            app.GDecor(end+1) = plot(ax,[-gm gm],[0 0],'-','Color',gray,'PickableParts','none');
            app.GDecor(end+1) = plot(ax,[0 0],[-gm gm],'-','Color',gray,'PickableParts','none');
            o = 0.9*gm;
            app.GAccelTxt = text(ax,0,o,'ACCEL','Color',app.MUTED,'FontSize',8,'HorizontalAlignment','center','PickableParts','none');
            app.GBrakeTxt = text(ax,0,-o,'BRAKE','Color',app.MUTED,'FontSize',8,'HorizontalAlignment','center','PickableParts','none');
            app.GDecor(end+1:end+2) = [app.GAccelTxt app.GBrakeTxt];
            app.GDecor(end+1) = text(ax,-o,0,'L','Color',app.MUTED,'FontSize',9,'HorizontalAlignment','center','PickableParts','none');
            app.GDecor(end+1) = text(ax,o,0,'R','Color',app.MUTED,'FontSize',9,'HorizontalAlignment','center','PickableParts','none');
            ax.XLim = 1.05*gm*[-1 1]; ax.YLim = 1.05*gm*[-1 1];
            axis(ax,'equal');
            uistack([H.garr H.gdot],'top');
        end
    end

    %% ------------------------------------------------------------------
    %  File loading and data preparation
    %  ------------------------------------------------------------------
    methods (Access = private)

        function onLoad(app)
            [f,p] = uigetfile({'*.csv;*.mat','Telemetry files (*.csv, *.mat)'; ...
                '*.csv','CSV files (*.csv)';'*.mat','MAT-files (*.mat)'}, ...
                'Select a telemetry session',app.LastDir);
            if isequal(f,0), return; end
            app.LastDir = p;
            app.loadFile(fullfile(p,f));
        end

        function onReload(app)
            if ~isempty(app.FilePath), app.loadFile(app.FilePath); end
        end

        function loadFile(app, file)
            app.pausePlayback();
            dlg = uiprogressdlg(app.Fig,'Title','Loading','Message','Reading session...','Indeterminate','on');
            cleanup = onCleanup(@() close(dlg)); %#ok<NASGU>
            try
                D = readTelemetryData(file);
                dlg.Message = 'Processing laps...';
                app.prepareSession(D);
            catch ME
                uialert(app.Fig,ME.message,'Could not load session');
                return
            end
            app.FilePath = file;
            [~,name,ext] = fileparts(file);
            n = numel(app.Laps);
            app.LblFile.Text = sprintf('%s%s  -  %d lap(s)', name, ext, n);
            app.Fig.Name = ['Telemetry Dashboard - ' name ext];
            app.populateDropdowns();
            app.applySessionLimits();
            app.selectLap(app.SelIdx, 0);
        end

        function prepareSession(app, D)
            v = D.speed;

            % ---- sign conventions (relative to yaw rate) ----
            s1 = sign(sum(D.ay .* v .* D.yawRate));        app.SignAy    = s1 + (s1 == 0);
            s2 = sign(sum(D.steerAngle .* v .* D.yawRate)); app.SignSteer = s2 + (s2 == 0);

            % ---- normalisation of pedals ----
            N.thrMax = max(max(D.driveForceCmd), eps);
            N.brkMax = max(max(abs(D.brakeForce)), eps);

            % ---- motor RPM (derived from torques unless given) ----
            R = app.WheelRadius;
            if isnan(R) && isfield(D,'wheelTorque') && isfield(D,'driveForce')
                m = abs(D.driveForce) > 100;
                if any(m), R = median(abs(D.wheelTorque(m) ./ D.driveForce(m))); end
            end
            if isnan(R) || R < 0.15 || R > 0.7, R = 0.33; end
            G = app.GearRatio;
            if isnan(G) && isfield(D,'motorTorque') && isfield(D,'wheelTorque')
                m = D.motorTorque > 1 & D.wheelTorque > 1;
                if any(m), G = median(D.wheelTorque(m) ./ D.motorTorque(m)); end
            end
            if ~isnan(G) && G <= 0, G = NaN; end
            N.R = R; N.G = G;

            % ---- neutral steering response: yaw rate = v*delta / (L + K*v^2) ----
            d = app.SignSteer * D.steerAngle / app.SteeringRatio;
            y = v .* d ./ D.yawRate;
            m = abs(D.yawRate) > 0.02 & abs(d) > 0.003 & v > 5 & abs(D.ay) < 0.3*max(abs(D.ay));
            m = m & y > 0.5 & y < 10;
            Lw = app.WheelBase; K = app.UndersteerGradient;
            if isnan(Lw) && isnan(K) && nnz(m) > 50
                p = [ones(nnz(m),1) v(m).^2] \ y(m);
                if p(1) > 1.5 && p(1) < 4.5, Lw = p(1); K = p(2); end
            end
            if isnan(Lw), Lw = 2.7; end
            if isnan(K),  K  = 0;   end
            app.Lw = Lw; app.Kus = K;

            % ---- split into laps (decimated) ----
            laps = {};
            ids = unique(D.lap(~isnan(D.lap)));
            for i = 1:numel(ids)
                idx = find(D.lap == ids(i));
                if numel(idx) < 20, continue; end
                keep = unique([1:app.Decim:numel(idx), numel(idx)]);
                laps{end+1} = app.buildLap(D, idx(keep), N); %#ok<AGROW>
            end
            if isempty(laps), error('No usable lap found in the file.'); end

            lens = cellfun(@(l) l.s(end), laps);
            for k = 1:numel(laps), laps{k}.complete = lens(k) >= 0.97*max(lens); end
            times = cellfun(@(l) l.tl(end), laps);
            times(~cellfun(@(l) l.complete, laps)) = inf;
            [~,best] = min(times);

            app.Laps = laps;
            app.BestIdx = best;
            app.TrackLen = laps{best}.s(end);
            app.RefIdx = best;
            sel = find((1:numel(laps)) ~= best, 1);
            if isempty(sel), sel = best; end
            app.SelIdx = sel;
        end

        function Lp = buildLap(app, D, idx, N)
            n = numel(idx); r = (1:n)'*1e-9;
            Lp.id = D.lap(idx(1));
            tl = D.lapTime(idx) - D.lapTime(idx(1));
            if any(diff(tl) < 0), tl = D.t(idx) - D.t(idx(1)); end
            Lp.tl = tl + r;
            Lp.s  = cummax(D.s(idx) - D.s(idx(1))) + r;
            Lp.x = D.x(idx); Lp.y = D.y(idx); Lp.yaw = D.yaw(idx);
            v = D.speed(idx);
            Lp.vk = v*3.6;
            Lp.thr = 100*max(D.driveForceCmd(idx),0)/N.thrMax;
            Lp.brk = 100*abs(D.brakeForce(idx))/N.brkMax;
            Lp.steer = D.steerAngle(idx)*180/pi;
            Lp.glon = D.ax(idx)/app.G0;
            Lp.glat = -app.SignAy*D.ay(idx)/app.G0;     % negative = left turn on the G circle
            Lp.rA = D.yawRate(idx)*180/pi;
            Lp.rpm = v/N.R*N.G*30/pi;

            % expected yaw rate from steering (deg/s) and balance error
            dEff = app.SignSteer*Lp.steer/app.SteeringRatio;
            rE = v.*dEff ./ max(app.Lw + app.Kus*v.^2, 0.5*app.Lw);
            rA = Lp.rA;
            dirS = sign(rE);
            opp = (rA.*rE < 0) & (abs(rA) > abs(rE));
            swap = opp | abs(rE) < 1;
            dirS(swap) = sign(rA(swap));
            valid = abs(rE) > 1 | abs(rA) > 3;
            e = zeros(n,1);
            e(valid) = dirS(valid).*(rA(valid) - rE(valid));   % <0 understeer, >0 oversteer
            Lp.rE = rE;
            Lp.e = movmean(e,15);
            Lp.bvalid = movmean(double(valid),15) > 0.5;
            Lp.complete = true;
        end

        function populateDropdowns(app)
            n = numel(app.Laps); labels = cell(1,n);
            for k = 1:n
                Lp = app.Laps{k}; tag = '';
                if ~Lp.complete, tag = ' (incomplete)';
                elseif k == app.BestIdx, tag = ' (best)'; end
                labels{k} = sprintf('Lap %d  -  %s%s', Lp.id, fmtTime(Lp.tl(end)), tag);
            end
            app.DDLap.Items = labels; app.DDLap.ItemsData = 1:n;
            app.DDLap.Value = app.SelIdx;
            app.DDRef.Items = [{'None'}, labels]; app.DDRef.ItemsData = 0:n;
            app.DDRef.Value = app.RefIdx;
        end

        function applySessionLimits(app)
            laps = app.Laps;
            vmax   = max(cellfun(@(l) max(l.vk), laps));
            smax   = max(cellfun(@(l) max(abs(l.steer)), laps));
            rmax   = max(cellfun(@(l) max(abs([l.rA; l.rE])), laps));
            gmax   = max(cellfun(@(l) max(hypot(l.glat,l.glon)), laps));
            rpmmax = max(cellfun(@(l) max(l.rpm), laps));
            X = cell2mat(cellfun(@(l) [min(l.x) max(l.x) min(l.y) max(l.y)], laps(:), 'UniformOutput', false));

            vtop = max(50, ceil(vmax/50)*50);
            app.Ax(1).YLim = [0 vtop];
            app.Ax(3).YLim = max(1,ceil(smax*1.1))*[-1 1];
            app.Ax(4).YLim = max(5,ceil(rmax*1.1))*[-1 1];
            app.AxSV.XLim = app.Ax(3).YLim;

            if isnan(rpmmax), rtop = 1000; else, rtop = max(1000,ceil(rpmmax/1000)*1000); end
            app.RpmMax = rtop;

            rg = [max(X(:,2))-min(X(:,1)), max(X(:,4))-min(X(:,3))];
            pad = 0.05*max(rg);
            app.AxTrack.XLim = [min(X(:,1)) max(X(:,2))] + pad*[-1 1];
            app.AxTrack.YLim = [min(X(:,3)) max(X(:,4))] + pad*[-1 1];
            axis(app.AxTrack,'equal');
            app.TrackXLim = app.AxTrack.XLim;
            app.TrackYLim = app.AxTrack.YLim;
            app.HeadLen = 0.03*max(rg);

            app.buildGCircle(max(1,ceil(gmax*2)/2));

            Bp = app.Laps{app.BestIdx};
            ae = sort(abs(Bp.e(Bp.bvalid)));
            if isempty(ae), app.BalScale = 10;
            else, app.BalScale = max(5, ae(max(1,round(0.98*numel(ae))))); end

            app.buildCorners();
        end

        function C = detectCorners(app, Lp)
            kap = movmean(abs(Lp.rA)*pi/180 ./ max(Lp.vk/3.6,1), 25);
            m = kap > app.CornerCurvature;
            d = diff([0; m(:); 0]);
            i0 = find(d == 1); i1 = find(d == -1) - 1;
            if isempty(i0), C = zeros(0,2); return; end
            S = [Lp.s(i0), Lp.s(i1)];
            out = S(1,:);
            for k = 2:size(S,1)
                if S(k,1) - out(end,2) < app.CornerMergeGap
                    out(end,2) = S(k,2);
                else
                    out(end+1,:) = S(k,:); %#ok<AGROW>
                end
            end
            C = out((out(:,2)-out(:,1)) >= app.CornerMinLength, :);
        end

        function buildCorners(app)
            delete(app.CornerTxt); app.CornerTxt = gobjects(0);
            Bp = app.Laps{app.BestIdx};
            app.CornerS = app.detectCorners(Bp);
            for c = 1:size(app.CornerS,1)
                idx = find(Bp.s >= app.CornerS(c,1) & Bp.s <= app.CornerS(c,2));
                [~,j] = min(Bp.vk(idx)); k = idx(j);
                app.CornerTxt(end+1) = text(app.AxTrack,Bp.x(k),Bp.y(k),sprintf(' T%d',c), ...
                    'Color',app.FG,'FontSize',9,'FontWeight','bold','PickableParts','none');
            end
        end
    end

    %% ------------------------------------------------------------------
    %  Lap display, cursor, tables
    %  ------------------------------------------------------------------
    methods (Access = private)

        function selectLap(app, idx, tStart)
            if nargin < 3, tStart = 0; end
            app.SelIdx = idx;
            app.DDLap.Value = idx;
            Lp = app.Laps{idx};
            app.TrailN = max(5, round(1/median(diff(Lp.tl))));
            app.Slider.Value = 0;
            app.Slider.Limits = [0 max(Lp.tl(end),0.01)];
            app.Slider.MajorTicks = []; app.Slider.MinorTicks = [];
            app.refreshLap();
            app.setCursor(tStart);
        end

        function refreshLap(app)
            Lp = app.Laps{app.SelIdx}; H = app.H;
            x = app.graphX(Lp);
            set(H.spd,'XData',x,'YData',Lp.vk,'DisplayName',sprintf('Lap %d',Lp.id));
            set(H.thr,'XData',x,'YData',Lp.thr);
            set(H.brk,'XData',x,'YData',Lp.brk);
            set(H.str,'XData',x,'YData',Lp.steer);
            set(H.yaw,'XData',x,'YData',Lp.rA);
            set(H.yawE,'XData',x,'YData',Lp.rE);
            set(H.sv,'XData',app.SignSteer*Lp.steer(1:3:end),'YData',Lp.thr(1:3:end));
            if app.RefIdx > 0
                Rp = app.Laps{app.RefIdx};
                xr = app.graphX(Rp);
                set(H.spdR,'XData',xr,'YData',Rp.vk,'DisplayName',sprintf('Ref: lap %d',Rp.id));
                set(H.thrR,'XData',xr,'YData',Rp.thr);
                set(H.brkR,'XData',xr,'YData',Rp.brk);
                set(H.strR,'XData',xr,'YData',Rp.steer);
                set(H.trkRef,'XData',Rp.x,'YData',Rp.y);
                app.Delta = Lp.tl - interp1(Rp.s,Rp.tl,Lp.s,'linear',NaN);
            else
                set([H.spdR H.thrR H.brkR H.strR H.trkRef],'XData',[],'YData',[]);
                app.Delta = nan(size(Lp.s));
            end
            set(H.dlt,'XData',x,'YData',app.Delta);
            if strcmp(app.DDXAxis.Value,'Time')
                xlabel(app.Ax(5),'Lap time (s)','Color',app.FG);
            else
                xlabel(app.Ax(5),'Distance (m)','Color',app.FG);
            end
            app.updateTrackColor();
            app.updateTable();
            app.fitGraphs();
        end

        function fitView(app)
            if isempty(app.Laps), return; end
            app.fitGraphs();
            app.AxTrack.XLim = app.TrackXLim;
            app.AxTrack.YLim = app.TrackYLim;
            axis(app.AxTrack,'equal');
        end

        function fitGraphs(app)
            if isempty(app.Laps), return; end
            x = app.graphX(app.Laps{app.SelIdx});
            app.Ax(1).XLim = [0 max(x(end),1)];
        end

        function x = graphX(app, Lp)
            if strcmp(app.DDXAxis.Value,'Time')
                x = Lp.tl;
            else
                x = Lp.s;
            end
        end

        function x = graphXAtDistance(app, Lp, s)
            if strcmp(app.DDXAxis.Value,'Time')
                x = interp1(Lp.s,Lp.tl,s,'linear','extrap');
            else
                x = s;
            end
        end

        function updateTrackColor(app)
            if isempty(app.Laps), return; end
            Lp = app.Laps{app.SelIdx}; H = app.H;
            switch app.DDColor.Value
                case 'Speed'
                    c = Lp.vk; lim = [min(c) max(c)]; lab = 'km/h';
                case 'Longitudinal G'
                    c = Lp.glon; mm = max(abs(c)); lim = [-mm mm]; lab = 'g (neg. = braking)';
                otherwise
                    c = abs(Lp.glat); lim = [0 max(c)]; lab = '|lateral g|';
            end
            if lim(2) <= lim(1), lim(2) = lim(1) + 1; end
            set(H.trk,'XData',Lp.x,'YData',Lp.y,'CData',c);
            app.AxTrack.CLim = lim;
            app.Cbar.Label.String = lab;
            app.Cbar.Label.Color = app.FG;
        end

        function setCursor(app, tc)
            if isempty(app.Laps), return; end
            Lp = app.Laps{app.SelIdx}; H = app.H;
            tc = min(max(tc,0),Lp.tl(end)); app.Tc = tc;
            i = find(Lp.tl >= tc,1);
            if isempty(i), i = numel(Lp.tl); end
            if i > 1 && (tc - Lp.tl(i-1)) < (Lp.tl(i) - tc), i = i - 1; end

            x = app.graphX(Lp);
            for k = 1:5, app.Cur(k).Value = x(i); end

            % track
            H.car.XData = Lp.x(i); H.car.YData = Lp.y(i);
            H.head.XData = Lp.x(i) + [0 app.HeadLen*cos(Lp.yaw(i))];
            H.head.YData = Lp.y(i) + [0 app.HeadLen*sin(Lp.yaw(i))];

            % gauges
            vk = Lp.vk(i);
            rp = Lp.rpm(i);
            if isnan(rp)
                app.LblRpm.Text = 'n/a';
            else
                app.LblRpm.Text = sprintf('%.0f rpm',rp);
            end
            for k = 1:numel(app.RpmLeds)
                if ~isnan(rp) && rp >= k*app.RpmMax/numel(app.RpmLeds)
                    app.RpmLeds(k).FaceColor = app.RpmLeds(k).EdgeColor;
                else
                    app.RpmLeds(k).FaceColor = app.PANEL;
                end
            end

            % G circle
            accelPos = app.GAccelTxt.Position;
            brakePos = app.GBrakeTxt.Position;
            if app.FlipGArrow
                gx = Lp.glat(i); gy = Lp.glon(i);
                accelPos(2) = abs(accelPos(2));
            else
                gx = -Lp.glat(i); gy = -Lp.glon(i);
                accelPos(2) = -abs(accelPos(2));
            end
            brakePos(2) = -accelPos(2);
            app.GAccelTxt.Position = accelPos;
            app.GBrakeTxt.Position = brakePos;
            H.garr.UData = gx; H.garr.VData = gy;
            H.gdot.XData = gx; H.gdot.YData = gy;
            app.AxG.Title.String = sprintf('G-force: %.2f g',hypot(gx,gy));

            % handling balance
            if Lp.bvalid(i)
                eN = max(-1,min(1,Lp.e(i)/app.BalScale));
                if eN < -0.15,     st = 'UNDERSTEER'; col = [0.45 0.65 1.0];
                elseif eN > 0.15,  st = 'OVERSTEER';  col = [1.0 0.65 0.25];
                else,              st = 'NEUTRAL';    col = app.CG; end
                txt = sprintf('Handling balance: %s (%+.1f deg/s)',st,Lp.e(i));
            else
                eN = 0; col = app.MUTED; txt = 'Handling balance: - (not cornering)';
            end
            H.balMark.XData = [eN eN];
            app.AxBal.Title.String = txt; app.AxBal.Title.Color = col;

            % steering vs throttle
            j = max(1,i - app.TrailN);
            H.svTrail.XData = app.SignSteer*Lp.steer(j:i); H.svTrail.YData = Lp.thr(j:i);
            H.svDot.XData = app.SignSteer*Lp.steer(i);     H.svDot.YData = Lp.thr(i);

            % readout
            dl = app.Delta(i);
            if isnan(dl), ds = '-'; else, ds = sprintf('%+.3f s',dl); end
            app.LblTime.Text = sprintf('Lap %d | %s / %s | %.0f m | Delta %s', ...
                Lp.id,fmtTime(tc),fmtTime(Lp.tl(end)),Lp.s(i),ds);
            app.Slider.Value = tc;
            drawnow limitrate
        end

        function updateTable(app)
            if isempty(app.Laps), return; end
            Lp = app.Laps{app.SelIdx}; Rp = [];
            if app.RefIdx > 0, Rp = app.Laps{app.RefIdx}; end
            if strcmp(app.DDView.Value,'Corners')
                [rows,cols,dv] = app.cornerRows(Lp,Rp); dcol = 9;
            else
                [rows,cols,dv] = app.sectorRows(Lp,Rp); dcol = 6;
            end
            app.Tbl.Data = {};
            app.Tbl.ColumnName = cols;
            app.Tbl.Data = rows;
            removeStyle(app.Tbl);
            for r = 1:numel(dv)
                if isnan(dv(r)), continue; end
                if dv(r) < -0.0005, col = app.CG;
                elseif dv(r) > 0.0005, col = app.CR;
                else, continue; end
                addStyle(app.Tbl,uistyle('FontColor',col),'cell',[r dcol]);
            end
        end

        function t = segTime(~, L, s0, s1)
            % time spent between distances s0 and s1 on lap L (NaN if not covered)
            if L.s(end) < s1 && ~L.complete, t = NaN; return; end
            s1 = min(s1,L.s(end));
            t = interp1(L.s,L.tl,s1,'linear','extrap') - interp1(L.s,L.tl,s0,'linear','extrap');
        end

        function [rows, cols, dv] = cornerRows(app, Lp, Rp)
            cols = {'Corner','Start (m)','Entry (km/h)','Apex (km/h)','Exit (km/h)', ...
                    'Brake pt (m)','Max lat G','Time (s)','Delta (s)'};
            C = app.CornerS; n = size(C,1);
            rows = cell(n,9); dv = nan(n,1);
            for c = 1:n
                s0 = C(c,1); s1 = C(c,2); s1e = min(s1,Lp.s(end));
                mk = Lp.s >= s0 & Lp.s <= s1e;
                tc = app.segTime(Lp,s0,s1);
                if isnan(tc) || nnz(mk) < 2
                    rows(c,:) = [{sprintf('T%d',c),sprintf('%.0f',s0)}, repmat({'-'},1,7)];
                    continue
                end
                sk = Lp.s(mk); vk = Lp.vk(mk);
                [vmin,im] = min(vk); sApex = sk(im);
                entry = interp1(Lp.s,Lp.vk,s0,'linear','extrap');
                exitv = interp1(Lp.s,Lp.vk,s1e,'linear','extrap');
                sPrev = -inf; if c > 1, sPrev = C(c-1,2); end
                w = find(Lp.s >= max(s0-250,sPrev) & Lp.s <= sApex & Lp.brk > 5,1);
                if isempty(w), bp = '-'; else, bp = sprintf('%.0f',s0 - Lp.s(w)); end
                if isempty(Rp)
                    dstr = '-';
                else
                    d = tc - app.segTime(Rp,s0,s1); dv(c) = d;
                    if isnan(d), dstr = '-'; else, dstr = sprintf('%+.3f',d); end
                end
                rows(c,:) = {sprintf('T%d',c),sprintf('%.0f',s0),sprintf('%.0f',entry), ...
                    sprintf('%.0f',vmin),sprintf('%.0f',exitv),bp, ...
                    sprintf('%.2f',max(abs(Lp.glat(mk)))),sprintf('%.3f',tc),dstr};
            end
        end

        function [rows, cols, dv] = sectorRows(app, Lp, Rp)
            cols = {'Sector','Start (m)','End (m)','Time (s)','Ref (s)','Delta (s)', ...
                    'Avg (km/h)','Max (km/h)','Min (km/h)'};
            n = app.NumSectors; ed = linspace(0,app.TrackLen,n+1);
            rows = cell(n,9); dv = nan(n,1);
            for k = 1:n
                s0 = ed(k); s1 = ed(k+1);
                head = {sprintf('S%d',k),sprintf('%.0f',s0),sprintf('%.0f',s1)};
                tc = app.segTime(Lp,s0,s1);
                mk = Lp.s >= s0 & Lp.s <= min(s1,Lp.s(end));
                if isnan(tc) || nnz(mk) < 2
                    rows(k,:) = [head, repmat({'-'},1,6)]; continue
                end
                if isempty(Rp), tr = NaN; else, tr = app.segTime(Rp,s0,s1); end
                if isnan(tr), rs = '-'; ds = '-';
                else, rs = sprintf('%.3f',tr); dv(k) = tc - tr; ds = sprintf('%+.3f',tc-tr); end
                rows(k,:) = [head,{sprintf('%.3f',tc),rs,ds,sprintf('%.0f',mean(Lp.vk(mk))), ...
                    sprintf('%.0f',max(Lp.vk(mk))),sprintf('%.0f',min(Lp.vk(mk)))}];
            end
        end
    end

    %% ------------------------------------------------------------------
    %  Interaction and playback
    %  ------------------------------------------------------------------
    methods (Access = private)

        function onLapChange(app)
            app.selectLap(app.DDLap.Value,0);
        end

        function onXAxisChange(app)
            if isempty(app.Laps), return; end
            app.refreshLap();
            app.setCursor(app.Tc);
        end

        function onRefChange(app)
            app.RefIdx = app.DDRef.Value;
            if isempty(app.Laps), return; end
            app.refreshLap(); app.setCursor(app.Tc);
        end

        function onRate(app)
            app.Rate = app.DDRate.Value;
        end

        function onSlide(app, val)
            app.pausePlayback();
            app.setCursor(val);
        end

        function onAxesClick(app, ax)
            if isempty(app.Laps), return; end
            p = ax.CurrentPoint(1,1);
            Lp = app.Laps{app.SelIdx};
            [~,i] = min(abs(app.graphX(Lp) - p));
            app.pausePlayback(); app.setCursor(Lp.tl(i));
        end

        function onTrackClick(app)
            if isempty(app.Laps), return; end
            p = app.AxTrack.CurrentPoint(1,1:2);
            Lp = app.Laps{app.SelIdx};
            [~,i] = min((Lp.x - p(1)).^2 + (Lp.y - p(2)).^2);
            app.pausePlayback(); app.setCursor(Lp.tl(i));
        end

        function onScroll(app, e)
            if isempty(app.Laps), return; end
            ax = app.axesAtPointer();
            if isempty(ax), return; end
            count = e.VerticalScrollCount;
            if count == 0, return; end
            factor = 1.2^count;
            if any(ax == app.Ax)
                lim = ax.XLim;
                point = ax.CurrentPoint(1,1);
                span = diff(lim);
                x = app.graphX(app.Laps{app.SelIdx});
                full = max(x(end),1);
                newSpan = min(max(span*factor,full/1000),full*20);
                anchor = min(max((point-lim(1))/span,0),1);
                newLim = point - anchor*newSpan + [0 newSpan];
                ax.XLim = newLim;
            elseif ax == app.AxTrack
                point = ax.CurrentPoint(1,1:2);
                xlim = ax.XLim; ylim = ax.YLim;
                xspan = diff(xlim); yspan = diff(ylim);
                fullX = diff(app.TrackXLim); fullY = diff(app.TrackYLim);
                newXSpan = min(max(xspan*factor,fullX/1000),fullX*20);
                newYSpan = min(max(yspan*factor,fullY/1000),fullY*20);
                anchorX = min(max((point(1)-xlim(1))/xspan,0),1);
                anchorY = min(max((point(2)-ylim(1))/yspan,0),1);
                ax.XLim = point(1) - anchorX*newXSpan + [0 newXSpan];
                ax.YLim = point(2) - anchorY*newYSpan + [0 newYSpan];
                axis(ax,'equal');
            end
        end

        function onMouseDown(app)
            ax = app.axesAtPointer();
            if isempty(ax) || (~any(ax == app.Ax) && ax ~= app.AxTrack)
                app.DragAx = [];
                return
            end
            app.DragAx = ax;
            app.DragStartPixel = app.Fig.CurrentPoint;
            app.DragStartXLim = ax.XLim;
            app.DragStartYLim = ax.YLim;
            app.DragMoved = false;
        end

        function onMouseMove(app)
            if isempty(app.DragAx) || ~isvalid(app.DragAx), return; end
            pixel = app.Fig.CurrentPoint;
            delta = pixel - app.DragStartPixel;
            if ~app.DragMoved && norm(delta) < 4, return; end
            app.DragMoved = true;
            bounds = getpixelposition(app.DragAx,true);
            if bounds(3) <= 0 || bounds(4) <= 0, return; end
            xlim = app.DragStartXLim - delta(1)*diff(app.DragStartXLim)/bounds(3);
            app.DragAx.XLim = xlim;
            if app.DragAx == app.AxTrack
                ylim = app.DragStartYLim - delta(2)*diff(app.DragStartYLim)/bounds(4);
                app.DragAx.YLim = ylim;
                axis(app.DragAx,'equal');
            end
        end

        function onMouseUp(app)
            ax = app.DragAx;
            moved = app.DragMoved;
            app.DragAx = [];
            app.DragMoved = false;
            if moved || isempty(ax) || ~isvalid(ax), return; end
            if any(ax == app.Ax)
                app.onAxesClick(ax);
            elseif ax == app.AxTrack
                app.onTrackClick();
            end
        end

        function ax = axesAtPointer(app)
            ax = [];
            obj = hittest(app.Fig);
            while ~isempty(obj)
                if isa(obj,'matlab.ui.control.UIAxes')
                    ax = obj;
                    return
                end
                if ~isprop(obj,'Parent'), return; end
                obj = obj.Parent;
            end
        end

        function onTableSelect(app, e)
            if isempty(e.Indices) || isempty(app.Laps), return; end
            r = e.Indices(1,1);
            if strcmp(app.DDView.Value,'Corners')
                if r > size(app.CornerS,1), return; end
                s0 = app.CornerS(r,1); s1 = app.CornerS(r,2);
                xl = [max(0,s0-80) s1+80];
            else
                ed = linspace(0,app.TrackLen,app.NumSectors+1);
                s0 = ed(r); s1 = ed(r+1); xl = [s0 s1];
            end
            Lp = app.Laps{app.SelIdx};
            [~,i] = min(abs(Lp.s - s0));
            app.Ax(1).XLim = [app.graphXAtDistance(Lp,xl(1)),app.graphXAtDistance(Lp,xl(2))];
            app.pausePlayback(); app.setCursor(Lp.tl(i));
        end

        function onKey(app, e)
            switch e.Key
                case 'space'
                    if app.Playing, app.pausePlayback(); else, app.startPlayback(1); end
                case 'leftarrow'
                    app.pausePlayback(); app.setCursor(app.Tc - 0.5);
                case 'rightarrow'
                    app.pausePlayback(); app.setCursor(app.Tc + 0.5);
            end
        end

        function startPlayback(app, dirn)
            if isempty(app.Laps), return; end
            T = app.Laps{app.SelIdx}.tl(end);
            app.Dir = dirn;
            if dirn > 0 && app.Tc >= T - 1e-6, app.Tc = 0; end
            if dirn < 0 && app.Tc <= 1e-6, app.Tc = T; end
            app.LastTic = tic;
            app.Playing = true;
            if isempty(app.Timer) || ~isvalid(app.Timer)
                app.Timer = timer('ExecutionMode','fixedRate','Period',0.05,'BusyMode','drop', ...
                    'TimerFcn',@(~,~) app.safeTick());
            end
            if strcmp(app.Timer.Running,'off'), start(app.Timer); end
        end

        function pausePlayback(app)
            app.Playing = false;
            if ~isempty(app.Timer) && isvalid(app.Timer) && strcmp(app.Timer.Running,'on')
                stop(app.Timer);
            end
        end

        function safeTick(app)
            try
                app.tick();
            catch ME
                app.pausePlayback();
                warning('Playback stopped: %s',ME.message);
            end
        end

        function tick(app)
            if ~app.Playing, return; end
            if isempty(app.Fig) || ~isvalid(app.Fig), app.pausePlayback(); return; end
            dt = toc(app.LastTic); app.LastTic = tic;
            T = app.Laps{app.SelIdx}.tl(end);
            tn = app.Tc + app.Dir*app.Rate*dt;
            if tn >= T && app.Dir > 0
                if app.SelIdx < numel(app.Laps), app.selectLap(app.SelIdx+1,0); return; end
                app.setCursor(T); app.pausePlayback(); return
            elseif tn <= 0 && app.Dir < 0
                if app.SelIdx > 1, app.selectLap(app.SelIdx-1,Inf); return; end
                app.setCursor(0); app.pausePlayback(); return
            end
            app.setCursor(tn);
        end
    end
end

%% ----------------------------------------------------------------------
function str = fmtTime(t)
    if isnan(t), str = '-'; return; end
    m = floor(t/60);
    str = sprintf('%d:%06.3f',m,t - 60*m);
end
