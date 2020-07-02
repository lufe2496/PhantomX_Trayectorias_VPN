function varargout = P_intermedio(varargin)
% P_INTERMEDIO MATLAB code for P_intermedio.fig
%      P_INTERMEDIO, by itself, creates a new P_INTERMEDIO or raises the existing
%      singleton*.
%
%      H = P_INTERMEDIO returns the handle to a new P_INTERMEDIO or the handle to
%      the existing singleton*.
%
%      P_INTERMEDIO('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in P_INTERMEDIO.M with the given input arguments.
%
%      P_INTERMEDIO('Property','Value',...) creates a new P_INTERMEDIO or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before P_intermedio_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to P_intermedio_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help P_intermedio

% Last Modified by GUIDE v2.5 28-Jun-2020 12:06:34

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @P_intermedio_OpeningFcn, ...
                   'gui_OutputFcn',  @P_intermedio_OutputFcn, ...
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


% --- Executes just before P_intermedio is made visible.
function P_intermedio_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to P_intermedio (see VARARGIN)

% Choose default command line output for P_intermedio
handles.output = hObject;

%Variables del Robot
handles.q = zeros(4,1);
handles.pos = zeros(3,1);
%Parametros del Robot
l1 = 1.05;
l2 = 1.05;
l3 = 1.1;
base = 1.37;
q1=0;  q2=0;  q3=0;  q4=0;
          %theta,    d ,   a,    alpha, junta , offset ,  modif, lims
L(1)=Link([q1       0,        0,        0,    0,  pi/2],  'modified');
L(2)=Link([q2       0,        0,     pi/2,    0,  pi/2],  'modified');
L(3)=Link([q3       0,       l1,        0,    0],  'modified');
L(4)=Link([q4       0,       l2,        0,    0],  'modified');
handles.l=L;
%Construccion del Robot

Phantom=SerialLink(L,'name', 'Robot Phantom');
Phantom.tool = transl(l3,0,0)*trotx(-pi/2)*troty(pi/2);
Phantom.base = transl(0,0,base);

%Objeto robot entre funciones
handles.robot = Phantom;
handles.q(1)=q1;
handles.q(2)=q2;
handles.q(3)=q3;
handles.q(4)=q4;

%Plot del robot
axes(handles.axes1);
Phantom.plot([q1 q2 q3 q4],'scale',0.8);
hold on
trplot(eye(4),'rgb','arrow','length',1.2,'frame','0');
axis([-4.00 4.00 -4.00 4.00 -0.50 5.00])

rosinit

handles.joint(1)=rospublisher("/Phantom_sim/joint1_position_controller/command","std_msgs/Float64");
handles.joint(2)=rospublisher("/Phantom_sim/joint2_position_controller/command","std_msgs/Float64");
handles.joint(3)=rospublisher('/Phantom_sim/joint3_position_controller/command','std_msgs/Float64');
handles.joint(4)=rospublisher('/Phantom_sim/joint4_position_controller/command','std_msgs/Float64');
handles.joint(5)=rospublisher('/Phantom_sim/joint5_position_controller/command','std_msgs/Float64');
handles.joint(6)=rospublisher('/Phantom_sim/joint6_position_controller/command','std_msgs/Float64');
handles.cam = rossubscriber('/Phantom_sim/mybot/camera/image_raw/compressed');
handles.joint_msg = rosmessage("std_msgs/Float64");
qi= [q1 q2 q3 q4];
for j=1 : 6
    qp=[qi 0.01 0.01];            
    handles.joint_msg.Data = qp(j);
    send(handles.joint(j),handles.joint_msg);

end


%Matriz de transformacion homogenea T0_tool (Cinematica directa)
T = handles.robot.fkine(handles.q);

%Obtener posicion
handles.pos = T(1:3,4);
handles.txt_X.String = num2str(handles.pos(1));
handles.txt_Y.String = num2str(handles.pos(2));
handles.txt_Z.String = num2str(handles.pos(3));
%Cargar imagen Camara
% axes(handles.Panel);
% t=timer('ExecutionMode','fixedRate' ,'TimerFcn', {@TimeCamera},'Period',0.1,'TasksToExecute', 100000);
% start(t);
axes(handles.Panel);
databad = receive(handles.cam);
img = readImage(databad);
imshow(img);
% Update handles structure
guidata(hObject, handles);



% UIWAIT makes P_intermedio wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = P_intermedio_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pbtn_StartA.
function pbtn_StartA_Callback(hObject, eventdata, handles)
% hObject    handle to pbtn_StartA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%L=handles.l;
%sto=handles.st;

handles.stop=0;
%%%%%%  POSICIONES Y POSES PREDEFINIDAS   %%%%%%
place1 = [2.15 2.0];
pose1 = [-90 tan(place1(2)/place1(1)) -90]; % -90 -90 por ubicarse en el primer cuadrante X-Y 

place2 = [2.15 0]; %DEJAR y=0. 
pose2 = [0 90 -179];% 0 90 -179 por ubicarse en y=0
%No se puede poner ni 180 ni -180 en R. ikunc no lo resuelve. Por eso toca
%-179

place3 = [2.15 -2.0];
pose3 = [90 tan(place3(2)/place3(1)) 90]; % 90 90 por ubicarse en el cuarto cuadrante X-Y 

pick = [-2.15 0]; %DEJAR y=0
posepick = [0 -90 0]; % 0 -90 0 por ubicarse en y=0

up = 2.0;
down = 0.32; %Altura mínima

home = [0 0 4.57 0 0 90]; 

place1down = [place1 down pose1];
place2down = [place2 down pose2];
place3down = [place3 down pose3];
pickdown = [pick down posepick];

wait = [0 -2.15 up 90 0 90]; 

place1up = [place1 up pose1];
place2up = [place2 up pose2];
place3up = [place3 up pose3];
pickup = [pick up posepick];

home_MTH       = MTH(home);
place1down_MTH = MTH(place1down);
place2down_MTH = MTH(place2down);
place3down_MTH = MTH(place3down);
pickdown_MTH   = MTH(pickdown);
wait_MTH       = MTH(wait);
place1up_MTH   = MTH(place1up);
place2up_MTH   = MTH(place2up);
place3up_MTH   = MTH(place3up);
pickup_MTH     = MTH(pickup);

invhome = handles.robot.ikunc(home_MTH);

invwait = handles.robot.ikunc(wait_MTH,invhome);

invplace1up = handles.robot.ikunc(place1up_MTH,invwait);
invplace2up = handles.robot.ikunc(place2up_MTH,invwait);
invplace3up = handles.robot.ikunc(place3up_MTH,invwait);
invpickup = handles.robot.ikunc(pickup_MTH,invwait);

invplace1down = handles.robot.ikunc(place1down_MTH,invplace1up);
invplace2down = handles.robot.ikunc(place2down_MTH,invplace2up);
invplace3down = handles.robot.ikunc(place3down_MTH,invplace3up);
invpickdown = handles.robot.ikunc(pickdown_MTH,invpickup);


t = [0:0.01:0.2]; %Con esto se configura la cantidad de posiciones 
% entre movimientos y el tiempo total que dura cada desplazamiento


home2wait = jtraj(invhome,invwait,t);
wait2home = jtraj(invwait,invhome,t);

wait2pickup = jtraj(invwait,invpickup,t);
pickup2down = jtraj(invpickup,invpickdown,t);
pickdown2up = jtraj(invpickdown,invpickup,t);
pickup2wait = jtraj(invpickup,invwait,t);

wait2place1up = jtraj(invwait,invplace1up,t);
place1up2down = jtraj(invplace1up,invplace1down,t);
place1down2up = jtraj(invplace1down,invplace1up,t);
place1up2wait = jtraj(invplace1up,invwait,t);

wait2place2up = jtraj(invwait,invplace2up,t);
place2up2down = jtraj(invplace2up,invplace2down,t);
place2down2up = jtraj(invplace2down,invplace2up,t);
place2up2wait = jtraj(invplace2up,invwait,t);


wait2place3up = jtraj(invwait,invplace3up,t);
place3up2down = jtraj(invplace3up,invplace3down,t);
place3down2up = jtraj(invplace3down,invplace3up,t);
place3up2wait = jtraj(invplace3up,invwait,t);


%%% Evaluar que trayectoria Seguir  %%%%%
Trayectoria = get(handles.Place123,'Value');

%Plotear
ejes = [-4.00 4.00 -4.00 4.00 -0.50 5.00];
t = 0; %Tiempo de espera en la animación de cada movimiento
trplot(eye(4),'length',20,'rgb')
hold on
axis(ejes)
axes(handles.axes1);
handles.robot.plot(invhome,'workspace',ejes,'scale',0.7,'jvec')


pause(t*3)

trac1=[home2wait; wait2pickup; pickup2down; pickdown2up; pickup2wait; wait2place1up; place1up2down; place1down2up; place1up2wait; wait2home];
trac2=[home2wait; wait2pickup; pickup2down; pickdown2up; pickup2wait; wait2place2up; place2up2down; place2down2up; place2up2wait; wait2home];
trac3=[home2wait; wait2pickup; pickup2down; pickdown2up; pickup2wait; wait2place3up; place3up2down; place3down2up; place3up2wait; wait2home];

if Trayectoria == 2
    for i=1 :11
        if i==1
            handles.robot.animate(home2wait);
        elseif i==2
            handles.robot.animate(wait2pickup);
        elseif i==3
            handles.robot.animate(pickup2down);
        elseif i==4
            handles.robot.animate(pickdown2up);    
        elseif i==5
            handles.robot.animate(pickup2wait);
        elseif i==6
            handles.robot.animate(wait2place1up);  
        elseif i==7
            handles.robot.animate(place1up2down);
        elseif i==8
            handles.robot.animate(place1down2up);
        elseif i==9
            handles.robot.animate(place1up2wait);
        elseif i==10
            handles.robot.animate(wait2home);
        end
        fr=0.02;
        fl=0.02;
        contador=20*(i-1)+1;
        for w=contador : contador+20
            for j=1: 6
                if i==4
                    fr=0.015;
                    fl=0.015; 
                end
                qp=[trac1(w,:) fr fl];            
                handles.joint_msg.Data = qp(j);
                send(handles.joint(j),handles.joint_msg);                
            end
            pause(0.1)
        end
        
        axes(handles.Panel);
        databad = receive(handles.cam);
        img = readImage(databad);
        imshow(img);
        pause(1)
        %Matriz de transformacion homogenea T0_tool (Cinematica directa)
        for k=1 :4    
            handles.q(k) =qp(k);
        end
        T = handles.robot.fkine(handles.q);
        %Obtener posicion
        handles.pos = T(1:3,4);
        handles.txt_X.String = num2str(handles.pos(1));
        handles.txt_Y.String = num2str(handles.pos(2));
        handles.txt_Z.String = num2str(handles.pos(3));
        qe=qp*180/pi;
        handles.txt_q1.String = num2str(qe(1));
        handles.txt_q2.String = num2str(qe(2));
        handles.txt_q3.String = num2str(qe(3));
        handles.txt_q4.String = num2str(qe(4));
        
    end
elseif Trayectoria== 3
    for i=1 :11
        if i==1
            handles.robot.animate(home2wait);
        elseif i==2
            handles.robot.animate(wait2pickup);
        elseif i==3
            handles.robot.animate(pickup2down);
        elseif i==4
            handles.robot.animate(pickdown2up);    
        elseif i==5
            handles.robot.animate(pickup2wait);
        elseif i==6
            handles.robot.animate(wait2place2up);  
        elseif i==7
            handles.robot.animate(place2up2down);
        elseif i==8
            handles.robot.animate(place2down2up);
        elseif i==9
            handles.robot.animate(place2up2wait);  
        elseif i==10
            handles.robot.animate(wait2home);
        end
        fr=0.02;
        fl=0.02;
        contador=20*(i-1)+1;
        for w=contador : contador+20
            for j=1: 6
                if i==4
                    fr=0;
                    fl=0; 
                end
                qp=[trac2(w,:) fr fl];            
                handles.joint_msg.Data = qp(j);
                send(handles.joint(j),handles.joint_msg);                
            end 
            pause(0.1)
        end
        axes(handles.Panel);
        databad = receive(handles.cam);
        img = readImage(databad);
        imshow(img);
   
        %Matriz de transformacion homogenea T0_tool (Cinematica directa)
        for k=1 :4    
            handles.q(k) =qp(k);
        end
        T = handles.robot.fkine(handles.q);
        %Obtener posicion
        handles.pos = T(1:3,4);
        handles.txt_X.String = num2str(handles.pos(1));
        handles.txt_Y.String = num2str(handles.pos(2));
        handles.txt_Z.String = num2str(handles.pos(3));
        qe=qp*180/pi;
        handles.txt_q1.String = num2str(qe(1));
        handles.txt_q2.String = num2str(qe(2));
        handles.txt_q3.String = num2str(qe(3));
        handles.txt_q4.String = num2str(qe(4));
    end
elseif Trayectoria== 4
    for i=1 :11
        if i==1
            handles.robot.animate(home2wait);
        elseif i==2
            handles.robot.animate(wait2pickup);
        elseif i==3
            handles.robot.animate(pickup2down);
        elseif i==4
            handles.robot.animate(pickdown2up);    
        elseif i==5
            handles.robot.animate(pickup2wait);
        elseif i==6
            handles.robot.animate(wait2place3up);  
        elseif i==7
            handles.robot.animate(place3up2down);
        elseif i==8
            handles.robot.animate(place3down2up);
        elseif i==9
            handles.robot.animate(place3up2wait);  
        elseif i==10
            handles.robot.animate(wait2home);
        end
        fr=0.02;
        fl=0.02;
        contador=20*(i-1)+1;
        for w=contador : contador+20
            for j=1: 6
                if i==4
                    fr=0;
                    fl=0; 
                end
                qp=[trac3(w,:) fr fl];            
                handles.joint_msg.Data = qp(j);
                send(handles.joint(j),handles.joint_msg);                
            end
            pause(0.1)
        end
        axes(handles.Panel);
        databad = receive(handles.cam);
        img = readImage(databad);
        imshow(img);
        
        %Matriz de transformacion homogenea T0_tool (Cinematica directa)
        for k=1 :4    
            handles.q(k) =qp(k);
        end
        T = handles.robot.fkine(handles.q);
        %Obtener posicion
        handles.pos = T(1:3,4);
        handles.txt_X.String = num2str(handles.pos(1));
        handles.txt_Y.String = num2str(handles.pos(2));
        handles.txt_Z.String = num2str(handles.pos(3));
        qe=qp*180/pi;
        handles.txt_q1.String = num2str(qe(1));
        handles.txt_q2.String = num2str(qe(2));
        handles.txt_q3.String = num2str(qe(3));
        handles.txt_q4.String = num2str(qe(4));

    end      
end

pause(t)
   
% --- Executes on button press in pbtn_StarR.

function pbtn_StopA_Callback(hObject, eventdata, handles)
% hObject    handle to pbtn_StopA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% --- Executes on button press in pbtn_StopR.

handles.stop=1;

function pbtn_StopR_Callback(hObject, eventdata, handles)
% hObject    handle to pbtn_StopR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.stop=1;



% --- Executes on selection change in list_place.
function list_place_Callback(hObject, eventdata, handles)
% hObject    handle to list_place (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns list_place contents as cell array
%        contents{get(hObject,'Value')} returns selected item from list_place


% --- Executes during object creation, after setting all properties.
function list_place_CreateFcn(hObject, eventdata, handles)
% hObject    handle to list_place (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in Place123.
function Place123_Callback(hObject, eventdata, handles)
% hObject    handle to Place123 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns Place123 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Place123


% --- Executes during object creation, after setting all properties.
function Place123_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Place123 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pbtn_startR.
function pbtn_startR_Callback(hObject, eventdata, handles)
% hObject    handle to pbtn_startR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
r=0;
d=0;
while true
    
    joydata = rossubscriber('/joy');
    h=receive(joydata);
    s= h.Axes(2);
    e= h.Axes(1);
    d=double(d+e);
    r=r+s;
    
    x=0+(d*0.2);
    y=-2.15;
    z=double(2+r*0.2);
    wait = [x y z 90 0 90];
    tray= MTH(wait);
    invhome = handles.robot.ikunc(tray);
    %Plotear
    ejes = [-4.00 4.00 -4.00 4.00 -0.50 5.00];
    t = 0; %Tiempo de espera en la animación de cada movimiento
    trplot(eye(4),'length',20,'rgb')
    hold on
    axis(ejes)
    axes(handles.axes1);
    handles.robot.plot(invhome,'workspace',ejes,'scale',0.7,'jvec')
    %Matriz de transformacion homogenea T0_tool (Cinematica directa)

    for j=1 : 6
        qp=[invhome 0.02 0.02];            
        handles.joint_msg.Data = qp(j);
        send(handles.joint(j),handles.joint_msg);
        
        
    end
    axes(handles.Panel);
    databad = receive(handles.cam);
    img = readImage(databad);
    imshow(img);
    pause(0.1)
        
    for i=1 :4    
        handles.q(i) =invhome(i);
    end
    T = handles.robot.fkine(handles.q);
    %Obtener posicion
    handles.pos = T(1:3,4);
    handles.txt_X.String = num2str(handles.pos(1));
    handles.txt_Y.String = num2str(handles.pos(2));
    handles.txt_Z.String = num2str(handles.pos(3));
    qe=qp*180/pi;
    handles.txt_q1.String = num2str(qe(1));
    handles.txt_q2.String = num2str(qe(2));
    handles.txt_q3.String = num2str(qe(3));
    handles.txt_q4.String = num2str(qe(4));
    
end



function txt_q1_Callback(hObject, eventdata, handles)
% hObject    handle to txt_q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_q1 as text
%        str2double(get(hObject,'String')) returns contents of txt_q1 as a double


% --- Executes during object creation, after setting all properties.
function txt_q1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_q2_Callback(hObject, eventdata, handles)
% hObject    handle to txt_q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_q2 as text
%        str2double(get(hObject,'String')) returns contents of txt_q2 as a double


% --- Executes during object creation, after setting all properties.
function txt_q2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_q3_Callback(hObject, eventdata, handles)
% hObject    handle to txt_q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_q3 as text
%        str2double(get(hObject,'String')) returns contents of txt_q3 as a double


% --- Executes during object creation, after setting all properties.
function txt_q3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_q4_Callback(hObject, eventdata, handles)
% hObject    handle to txt_q4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_q4 as text
%        str2double(get(hObject,'String')) returns contents of txt_q4 as a double


% --- Executes during object creation, after setting all properties.
function txt_q4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_q4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_X_Callback(hObject, eventdata, handles)
% hObject    handle to txt_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_X as text
%        str2double(get(hObject,'String')) returns contents of txt_X as a double


% --- Executes during object creation, after setting all properties.
function txt_X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_Y_Callback(hObject, eventdata, handles)
% hObject    handle to txt_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_Y as text
%        str2double(get(hObject,'String')) returns contents of txt_Y as a double


% --- Executes during object creation, after setting all properties.
function txt_Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_Z_Callback(hObject, eventdata, handles)
% hObject    handle to txt_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_Z as text
%        str2double(get(hObject,'String')) returns contents of txt_Z as a double


% --- Executes during object creation, after setting all properties.
function txt_Z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
