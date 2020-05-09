function varargout = ControlGUI(varargin)
% CONTROLGUI MATLAB code for ControlGUI.fig
%      CONTROLGUI, by itself, creates a new CONTROLGUI or raises the existing
%      singleton*.
%
%      H = CONTROLGUI returns the handle to a new CONTROLGUI or the handle to
%      the existing singleton*.
%
%      CONTROLGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CONTROLGUI.M with the given input arguments.
%
%      CONTROLGUI('Property','Value',...) creates a new CONTROLGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ControlGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ControlGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ControlGUI

% Last Modified by GUIDE v2.5 09-May-2020 14:54:13

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ControlGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @ControlGUI_OutputFcn, ...
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

% --- Executes just before ControlGUI is made visible.
function ControlGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ControlGUI (see VARARGIN)

% Choose default command line output for ControlGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using ControlGUI.
if strcmp(get(hObject,'Visible'),'off')
    plot(rand(5));
end

% UIWAIT makes ControlGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ControlGUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes1);
cla;

popup_sel_index = get(handles.popupmenu1, 'Value');
sim('myCRP14_V2')
switch popup_sel_index
    case 1
        plot(q1.Time, q1.Data, 'b');
        hold on
        plot(q1.Time, sin(0.5*q1.Time), 'r');
    case 2
        plot(sin(1:0.01:25.99));
    case 3
        bar(1:.5:10);
    case 4
        plot(membrane);
    case 5
        surf(peaks);
end


% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
     set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'q1', 'plot(sin(1:0.01:25))', 'bar(1:.5:10)', 'plot(membrane)', 'surf(peaks)'});


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function P0_Callback(hObject, eventdata, handles)
% hObject    handle to P0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of P0 as text
%        str2double(get(hObject,'String')) returns contents of P0 as a double


% --- Executes during object creation, after setting all properties.
function P0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to P0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pt_Callback(hObject, eventdata, handles)
% hObject    handle to Pt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pt as text
%        str2double(get(hObject,'String')) returns contents of Pt as a double


% --- Executes during object creation, after setting all properties.
function Pt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Tc1_Callback(hObject, eventdata, handles)
% hObject    handle to Tc1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Tc1 as text
%        str2double(get(hObject,'String')) returns contents of Tc1 as a double


% --- Executes during object creation, after setting all properties.
function Tc1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Tc1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Tc2_Callback(hObject, eventdata, handles)
% hObject    handle to Tc2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Tc2 as text
%        str2double(get(hObject,'String')) returns contents of Tc2 as a double


% --- Executes during object creation, after setting all properties.
function Tc2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Tc2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in beginbutton.
function beginbutton_Callback(hObject, eventdata, handles)
T_get = get(handles.T, 'string');
T_num = str2num(char(T_get));
%获得输入的初始角度数组，并转为num数组
th0_get = get(handles.th0, 'string');
th0_num = str2num(char(th0_get));

pos0_get = get(handles.pos0, 'string');
pos0_num = str2num(char(pos0_get));
%获得输入的起点位置数组，并转为num数组
p0_get = get(handles.P0, 'string');
p0_num = str2num(char(p0_get));
%获得输入的终点位置数组，并转为num数组
pt_get = get(handles.Pt, 'string');
pt_num = str2num(char(pt_get));
%获得输入的轨迹规划收敛时间，并转为num数组
tc1_get = get(handles.Tc1, 'string');
tc1_num = str2num(char(tc1_get));
%获得输入的轨迹跟踪规划收敛时间，并转为num数组
tc2_get = get(handles.Tc2, 'string');
tc2_num = str2num(char(tc2_get));
set(handles.tip, 'string', '正在进行轨迹规划，请稍后......');
[sum_v, v, lx, ly, lz, vx, vy, vz] = GUILineTrajectorMake(T_num, p0_num, pt_num);
%示波器绘图
axes(handles.traj);
plot3(lx, ly, lz, 'LineWidth',1,'color','k');
set(gca,'ZLim',[min(lz)-(max(lz)-min(lz))/10 max(lz)+(max(lz)-min(lz))/10]);
axis square
grid on;
text(lx(1),ly(1),lz(1)-0.005,['$$P_0(x_0, y_0, z_0)$$'],'FontName','黑体','FontSize',12, 'Interpreter','latex')  %标出起点和终点坐标
text(lx(101),ly(101),lz(101)-0.005,['$$P_f(x_f, y_f, z_f)$$'],'FontName','黑体','FontSize',12, 'Interpreter','latex')
T2 = 0:0.1:T_num;

axes(handles.velocity);
plot(T2, sum_v,'--','LineWidth',1.5,'color','k');
hold on;
plot(T2, v,'LineWidth',1.5,'color','blue');
hold on;
legend('位移曲线','速度曲线');

set(handles.tip, 'string', '正在将轨迹转为各个关节，请稍后......');
[th,xc,yc,zc] = GUITrajControl(pos0_num, th0_num, p0_num, pt_num, lx, ly, lz, vx, vy, vz, T_num, tc1_num);


axes(handles.angle);
plot(T2, th(1, :),'LineWidth',1.5,'color','k');
hold on;
plot(T2, th(2, :),'LineWidth',1.5,'color','blue');
hold on;
plot(T2, th(3, :),'LineWidth',1.5,'color','green');
hold on;
plot(T2, th(4, :),'LineWidth',1.5,'color','red');
hold on
plot(T2, th(5, :),'LineWidth',1.5,'color','cyan');
hold on;
plot(T2, th(6, :),'LineWidth',1.5,'color','yellow');
legend('第1关节轨迹', '第2关节轨迹', '第3关节轨迹', '第4关节轨迹', '第5关节轨迹', '第6关节轨迹');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度值','FontName','黑体','FontSize',12);
set(gca,'XLim',[min(T2) max(T2)]);
grid on;
set(handles.tip, 'string', '正在进行轨迹跟踪控制，请稍后......');
sim('myCRP14_Trajectory');

set(handles.tip, 'string', '轨迹跟踪完成！');
axes(handles.err);
plot(e1.Time, e1.Data,'LineWidth',1.5,'color','k');
hold on;
plot(e2.Time, e2.Data,'LineWidth',1.5,'color','blue');
hold on;
plot(e3.Time, e3.Data,'LineWidth',1.5,'color','green');
hold on;
plot(e4.Time, e4.Data,'LineWidth',1.5,'color','red');
hold on
plot(e5.Time, e5.Data,'LineWidth',1.5,'color','cyan');
hold on;
plot(e6.Time, e6.Data,'LineWidth',1.5,'color','yellow');
legend('第1关节跟踪误差', '第2关节跟踪误差', '第3关节跟踪误差', '第4关节跟踪误差', '第5关节跟踪误差', '第6关节跟踪误差');
set(gca,'XLim',[min(T2) max(T2)]);
grid on;


% --- Executes on button press in endbutton.
function endbutton_Callback(hObject, eventdata, handles)
set(handles.T, 'string', 'T');
set(handles.th0, 'string', '[th1; th2; ... ; thn]');
set(handles.P0, 'string','[x0; y0; z0]');
set(handles.Pt, 'string', '[xt; yt; zt]');
set(handles.Tc1, 'string', 'Tc1');
set(handles.Tc2, 'string', 'Tc2');
set(handles.tip, 'string', '等待开始');
set(handles.pos0, 'string', '请进行正运动学求解');
cla(handles.traj,'reset');
cla(handles.angle,'reset');
cla(handles.velocity,'reset');
cla(handles.err,'reset');



function th0_Callback(hObject, eventdata, handles)
% hObject    handle to th0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of th0 as text
%        str2double(get(hObject,'String')) returns contents of th0 as a double


% --- Executes during object creation, after setting all properties.
function th0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to th0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function traj_CreateFcn(hObject, eventdata, handles)
% hObject    handle to traj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate traj


% --- Executes during object creation, after setting all properties.
function text3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function tip_Callback(hObject, eventdata, handles)
% hObject    handle to tip (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tip as text
%        str2double(get(hObject,'String')) returns contents of tip as a double


% --- Executes during object creation, after setting all properties.
function tip_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tip (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T_Callback(hObject, eventdata, handles)
% hObject    handle to T (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T as text
%        str2double(get(hObject,'String')) returns contents of T as a double


% --- Executes during object creation, after setting all properties.
function T_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in FK.
function FK_Callback(hObject, eventdata, handles)
th_get = get(handles.th0, 'string');
th0 = str2num(char(th_get));
Tp = myFKSolver(th0);
str = [num2str(Tp(1, 4)), ', ' ,num2str(Tp(2, 4)), ', ',num2str(Tp(3, 4))];
set(handles.pos0, 'string', str);



function pos0_Callback(hObject, eventdata, handles)
% hObject    handle to pos0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pos0 as text
%        str2double(get(hObject,'String')) returns contents of pos0 as a double


% --- Executes during object creation, after setting all properties.
function pos0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pos0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
