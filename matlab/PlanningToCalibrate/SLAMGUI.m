function varargout = SLAMGUI(varargin)
% SLAMGUI MATLAB code for SLAMGUI.fig
%      SLAMGUI, by itself, creates a new SLAMGUI or raises the existing
%      singleton*.
%
%      H = SLAMGUI returns the handle to a new SLAMGUI or the handle to
%      the existing singleton*.
%
%      SLAMGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SLAMGUI.M with the given input arguments.
%
%      SLAMGUI('Property','Value',...) creates a new SLAMGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SLAMGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SLAMGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SLAMGUI

% Last Modified by GUIDE v2.5 13-Jan-2015 17:39:22

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @SLAMGUI_OpeningFcn, ...
    'gui_OutputFcn',  @SLAMGUI_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before SLAMGUI is made visible.
function SLAMGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SLAMGUI (see VARARGIN)

% Initialize constants
addpath('/usr/local/gtsam_toolbox');
addpath('../IncrementalSLAM');
import gtsam.*
initialize_constants(handles.axes1);

set(0,'defaultLineLineWidth',2);   % set the default line width to lw
set(0, 'defaultLineLineWidth', 2);
set(0,'DefaultFigureWindowStyle','normal');


% Choose default command line output for SLAMGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SLAMGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SLAMGUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in add_landmarks.
function add_landmarks_Callback(hObject, eventdata, handles)
% hObject    handle to add_landmarks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure withpath handles and user data (see GUIDATA)
globals
[landmarks, landmark_values] = initialize_landmarks(handles.axes1);
USER_DATA.landmarks = landmarks;
USER_DATA.landmark_values = landmark_values;

% --- Executes on button press in add_trajectory.
function add_trajectory_Callback(hObject, eventdata, handles)
% hObject    handle to add_trajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
globals
path=initialize_poses(handles.axes1, handles.estimatedGraph);
[temp,n_path]=size(path);
USER_DATA.path = path;


% --- Executes on button press in data_associate.
function data_associate_Callback(hObject, eventdata, handles)
% hObject    handle to data_associate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
globals
disp('Data Associating....');
obs = data_associate(USER_DATA.xtrue, USER_DATA.landmarks);
USER_DATA.observations = obs;



% --- Executes on button press in visualize_DA.
function visualize_DA_Callback(hObject, eventdata, handles)
% hObject    handle to visualize_DA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp('Visualizing Data Association....');
globals
graph = USER_DATA.graph;
ground_truth = USER_DATA.ground_truth;
axes(handles.axes1);
%cla;
hold on;
for factor = 0:graph.nrFactors -1
    if strcmp(class(graph.at(factor)), 'gtsam.BearingRangeFactor2D')
        pose_symbol=graph.at(factor).keys.at(0); % Pose 1
        landmark_symbol=graph.at(factor).keys.at(1);
        landmark = ground_truth.at(landmark_symbol);
        pose = ground_truth.at(pose_symbol);
        line([landmark.x, pose.x],[landmark.y, pose.y])
    end
end
disp('Done');

% --- Executes on button press in save_FG.
function save_FG_Callback(hObject, eventdata, handles)
% hObject    handle to save_FG (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
globals
[filename, path] = uiputfile('*.*');
filepath = [path,'/',filename];
fprintf('Saving graph at %s....',filepath);
gtsam.writeG2o(USER_DATA.graph, USER_DATA.ground_truth,filepath);
disp('Done');

% --- Executes on button press in visualize_FG.
function visualize_FG_Callback(hObject, eventdata, handles)
% hObject    handle to visualize_FG (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp('Visualizing Factor Graph....');
globals
graph = USER_DATA.graph;
ground_truth = USER_DATA.ground_truth;
axes(handles.axes1);
%cla;
hold on;
%title('PLOT');
%gtsam.plot2DTrajectory(initialC, 'r-'); axis equal
gtsam.plot2DTrajectory(ground_truth, 'g');axis equal
gtsam.plot2DPoints(ground_truth, 'b');
disp('Done');


% --- Executes on button press in simulate_controls.
function simulate_controls_Callback(hObject, eventdata, handles)
% hObject    handle to simulate_controls (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp('Simulating Controls....');
globals
path = USER_DATA.path;
[xtrue, utrue, uz] = simulate_controls(path, handles.axes1);
USER_DATA.xtrue = xtrue;
USER_DATA.utrue = utrue;
USER_DATA.uz = uz;
USER_DATA.path = path;



% --- Executes on button press in random_add.
function random_add_Callback(hObject, eventdata, handles)
% hObject    handle to random_add (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
globals
prompt = {'Enter Number of Landmarks'};
dlg_title = 'Input';
num_lines = 1;
ans = inputdlg(prompt, dlg_title, num_lines);
num_landmarks = str2num(ans{:});
landmarks = [];
landmark_values = gtsam.Values;

for i = 1:num_landmarks
    x = rand*WORLD_SIZE;
    y = rand*WORLD_SIZE;
    landmarks = [landmarks; [x y]];
    point = gtsam.Point2(x,y);
    landmark_values.insert(gtsam.symbol('l',i), point);
    
    
end
USER_DATA.landmarks = landmarks;
USER_DATA.landmark_values = landmark_values;
axes(handles.axes1);
plot(landmarks(:,1), landmarks(:,2),'go')



% --- Executes on button press in generate_FG.
function generate_FG_Callback(hObject, eventdata, handles)
% hObject    handle to generate_FG (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Generate factor graph
disp('Generating Factor Graph....');
generateValues
generate_factor_graph
disp('Done');


% --- Executes on button press in add_trajectory.
function bruteForce_Callback(hObject, eventdata, handles)
% hObject    handle to add_trajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
globals
[path,dTD]=randomWalk(handles.axes1, handles.estimatedGraph, handles.covAx1,handles.errAx1);
[temp,n_path]=size(path);
USER_DATA.path = path;


% --- Executes on button press in planningInBeliefSpace.
function planningInBeliefSpace_Callback(hObject, eventdata, handles)
% hObject    handle to planningInBeliefSpace (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
globals
[path,dTD,dTE]=planningInBeliefSpace(handles.axes1, handles.estimatedGraph, handles.covAx1, handles.errAx1);
[temp,n_path]=size(path);
USER_DATA.path = path;



% --- Executes on button press in save_figures.
function save_figures_Callback(hObject, eventdata, handles)
% hObject    handle to save_figures (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
globals
handler = {handles.axes1, handles.estimatedGraph};
[filename, path] = uiputfile('*.fig');
filepath = [path,'/',filename];
savingFig = figure;
set(savingFig,'Position',[1681 50 1600 700])
c1 = copyobj(handles.axes1, savingFig);
set(c1,'Position',[.02 .05 .45 .9],'xtick',[],'ytick',[]);
c2 = copyobj(handles.estimatedGraph, savingFig);
set(c2,'Position',[.35 .05 .8 .9],'xtick',[],'ytick',[],'box','on');
set(get(c2,'title'),'String',[])
fprintf('Saving figures at %s....',filepath);
savefig(savingFig,filepath)


% --- Executes on button press in runExp.
function runExp_Callback(hObject, eventdata, handles)
% hObject    handle to runExp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
globals
initialize_constants


% prompt = {'Enter Number of Runs'};
% dlg_title = 'Input';
% num_lines = 1;
% ans = inputdlg(prompt, dlg_title, num_lines);
% num_runs = str2num(ans{:});

% prompt2 = {'Enter Scenario Number'};
% dlg_title2 = 'Input';
% num_lines2 = 1;
% ans = inputdlg(prompt2, dlg_title2, num_lines2);
% scNum = str2num(ans{:});

for scNum = 3
    
    axes(handles.axes1)
    cla
    
    landmarks = [];
    landmark_values = gtsam.Values;
    
    switch scNum
        case 1
            landmarks = [175 250; 275 250; 225 250+(sqrt(3)/2*100)];
            optStart = [225 250+(sqrt(3)/4*100)];
            
        case 2
            landmarks = [150 250; 170 250; 190 250; 210 250; 210 270; 210 290; 210 310];
            optStart = [160 300];
        case 3
            x = [.1:.2:1]*WORLD_SIZE;
            y = [.1:.2:1]*WORLD_SIZE;
            
            [X,Y] = meshgrid(x,y);

            x = reshape(X,numel(X),1);
            y = reshape(Y,numel(Y),1);
            
            landmarks = [x y];
            optStart = [275 275];
        case 4
            landmarks = [250 250; 300 250];
            optStart = [250 200];            
            
    end
    
    
    
    for i = 1:length(landmarks)
        xs = landmarks(i,1);
        ys = landmarks(i,2);
        point = gtsam.Point2(xs,ys);
        landmark_values.insert(gtsam.symbol('l',i), point);
    end
    
    USER_DATA.landmarks = landmarks;
    USER_DATA.landmark_values = landmark_values;
    axes(handles.axes1);
    plot(landmarks(:,1), landmarks(:,2),'go')
    
    covT1_1 = zeros(num_monte_carlo_runs,8);
    covT1_2 = zeros(num_monte_carlo_runs,8);
    errT1_1= zeros(num_monte_carlo_runs,8);
    errT1_2 = zeros(num_monte_carlo_runs,8);
    
    covT2_1 = zeros(num_monte_carlo_runs,8);
    covT2_2 = zeros(num_monte_carlo_runs,8);
    errT2_1 = zeros(num_monte_carlo_runs,8);
    errT2_2 = zeros(num_monte_carlo_runs,8);
    
     %for n=1:num_monte_carlo_runs
     %[path,dTD1,dTE1]=randomWalk(handles.axes1, handles.estimatedGraph, handles.covAx1,handles.errAx1, optStart);
     %[temp,n_path]=size(path);
     %USER_DATA.path = path;
     
     %covT1_1(n,1:length(dTD1)) = dTD1.sensor1;
     %covT1_2(n,1:length(dTD1)) = dTD1.sensor2;
     %errT1_1(n,1:length(dTE1)) = dTE1.sensor1;
     %errT1_2(n,1:length(dTE1)) = dTE1.sensor2;
     %end
    
  
   % fileName = strcat(['Random_cov_Sensor_1_Scene' num2str(scNum) '_' num2str(num_runs) 'runs.csv']);
   % csvwrite(fileName,covT1_1);
    
   % fileName = strcat(['Random_err_Sensor_1_Scene' num2str(scNum) '_' num2str(num_runs) 'runs.csv']);
   % csvwrite(fileName,errT1_1);

    %fileName = strcat(['Random_cov_Sensor_2_Scene' num2str(scNum) '_' num2str(num_runs) 'runs.csv']);
    %csvwrite(fileName,covT1_2);
    
    %fileName = strcat(['Random_err_Sensor_2_Scene' num2str(scNum) '_' num2str(num_runs) 'runs.csv']);
    %csvwrite(fileName,errT1_2);
       
     
    for n=1:num_monte_carlo_runs
        [path,dTD2,dTE2]=planningInBeliefSpace(handles.axes1, handles.estimatedGraph, handles.covAx1, handles.errAx1, optStart);
        [temp,n_path]=size(path);
        USER_DATA.path = path;
        
        covT2_1(n,1:length(dTD2)) = dTD2.sensor1;
        errT2_1(n,1:length(dTE2)) = dTE2.sensor1;
        covT2_2(n,1:length(dTD2)) = dTD2.sensor2;
        errT2_2(n,1:length(dTE2)) = dTE2.sensor2;
    end
    
    fileName = strcat(['Belief_cov_Sensor_1_Scene' num2str(scNum) '_' num2str(num_runs) 'runs.csv']);
    csvwrite(fileName,covT2_1);
    
    fileName = strcat(['Belief_err_Sensor_1_Scene' num2str(scNum) '_' num2str(num_runs) 'runs.csv']);
    csvwrite(fileName,errT2_1);

    fileName = strcat(['Belief_cov_Sensor_2_Scene' num2str(scNum) '_' num2str(num_runs) 'runs.csv']);
    csvwrite(fileName,covT2_2);
    
    fileName = strcat(['Belief_err_Sensor_2_Scene' num2str(scNum) '_' num2str(num_runs) 'runs.csv']);
    csvwrite(fileName,errT2_2);
    
end
