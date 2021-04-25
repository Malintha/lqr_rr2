function varargout = GUI_RR2_ID(varargin)
% GUI_RR2_ID MATLAB code for GUI_RR2_ID.fig
%      GUI_RR2_ID, by itself, creates a new GUI_RR2_ID or raises the existing
%      singleton*.
%
%      H = GUI_RR2_ID returns the handle to a new GUI_RR2_ID or the handle to
%      the existing singleton*.
%
%      GUI_RR2_ID('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_RR2_ID.M with the given input arguments.
%
%      GUI_RR2_ID('Property','Value',...) creates a new GUI_RR2_ID or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_RR2_ID_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_RR2_ID_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES
% Edit the above text to modify the response to help GUI_RR2_ID
% Last Modified by GUIDE v2.5 11-Apr-2014 11:40:55
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_RR2_ID_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_RR2_ID_OutputFcn, ...
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
% --- Executes just before GUI_RR2_ID is made visible.
function GUI_RR2_ID_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI_RR2_ID (see VARARGIN)
% Choose default command line output for GUI_RR2_ID
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);
% UIWAIT makes GUI_RR2_ID wait for user response (see UIRESUME)
% uiwait(handles.figure1);
% --- Outputs from this function are returned to the command line.
function varargout = GUI_RR2_ID_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get default command line output from handles structure
varargout{1} = handles.output;
% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
if get(handles.mass1_1,'Value')>0
    m1=get(handles.mass1_1,'UserData');
end
if get(handles.mass1_2,'Value')>0
    m1=get(handles.mass1_2,'UserData');
end
if get(handles.mass1_3,'Value')>0
    m1=get(handles.mass1_3,'UserData');
end
if get(handles.mass2_1,'Value')>0
    m2=get(handles.mass2_1,'UserData');
end
if get(handles.mass2_2,'Value')>0
    m2=get(handles.mass2_2,'UserData');
end
if get(handles.mass2_3,'Value')>0
    m2=get(handles.mass2_3,'UserData');
end
if get(handles.length1_1,'Value')>0
    l1=get(handles.length1_1,'UserData');
end
if get(handles.length1_2,'Value')>0
    l1=get(handles.length1_2,'UserData');
end
if get(handles.length1_3,'Value')>0
    l1=get(handles.length1_3,'UserData');
end
if get(handles.length2_1,'Value')>0
    l2=get(handles.length2_1,'UserData');
end
if get(handles.length2_2,'Value')>0
    l2=get(handles.length2_2,'UserData');
end
if get(handles.length2_3,'Value')>0
    l2=get(handles.length2_3,'UserData');
end
omega=get(handles.popupmenu1,'UserData');
tvec=linspace(0,100,1001);
[tau1,tau2,M1,M2,V1,V2,G1,G2]=RR2_ID(omega,m1,m2,l1,l2)
if get(handles.tau_inert,'Value')>0
    axes(handles.axes1)
    plot(tvec,M1,'--r')
    axes(handles.axes2)
    plot(tvec,M2,'--r')
    
    
end
if get(handles.tau_cor,'Value')>0
    axes(handles.axes1)
    plot(tvec,V1,'--b')
    axes(handles.axes2)
    plot(tvec,V2,'--b')
    
    
end
if get(handles.tau_grav,'Value')>0
    axes(handles.axes1)
    plot(tvec,G1,'--g')
    axes(handles.axes2)
    plot(tvec,G2,'--g')
    
    
end
if get(handles.tau_tot,'Value')>0
    axes(handles.axes1)
    plot(tvec,tau1,'k')
    axes(handles.axes2)
    plot(tvec,tau2,'k')
    
    
end
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
str=get(hObject,'String');
val=get(hObject,'Value');
switch str{val}
    case 'Slow'
        w1=1*(pi/180);
        w2=3*(pi/180);
        omega=[w1;w2];
        set(handles.popupmenu1,'UserData',omega)
    case 'Medium'
        w1=5*(pi/180);
        w2=15*(pi/180);
        omega=[w1;w2];
        set(handles.popupmenu1,'UserData',omega);
    case 'Fast'
        w1=25*(pi/180);
        w2=75*(pi/180);
        omega=[w1;w2];
        set(handles.popupmenu1,'UserData',omega);
        
end
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
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
% --- Executes on button press in mass1_1.
function mass1_1_Callback(hObject, eventdata, handles)
set(handles.mass1_1,'UserData',0.1);
% hObject    handle to mass1_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of mass1_1
% --- Executes on button press in mass1_2.
function mass1_2_Callback(hObject, eventdata, handles)
set(handles.mass1_2,'UserData',1);
% hObject    handle to mass1_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of mass1_2
% --- Executes on button press in mass1_3.
function mass1_3_Callback(hObject, eventdata, handles)
set(handles.mass1_3,'UserData',10);
% hObject    handle to mass1_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of mass1_3
% --- Executes on button press in mass2_1.
function mass2_1_Callback(hObject, eventdata, handles)
set(handles.mass2_1,'UserData',0.1);
% hObject    handle to mass2_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of mass2_1
% --- Executes on button press in mass2_2.
function mass2_2_Callback(hObject, eventdata, handles)
set(handles.mass2_2,'UserData',1);
% hObject    handle to mass2_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of mass2_2
% --- Executes on button press in mass2_3.
function mass2_3_Callback(hObject, eventdata, handles)
set(handles.mass2_3,'UserData',10);
% hObject    handle to mass2_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of mass2_3
% --- Executes on button press in length1_1.
function length1_1_Callback(hObject, eventdata, handles)
set(handles.length1_1,'UserData',0.1);
% hObject    handle to length1_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of length1_1
% --- Executes on button press in length1_2.
function length1_2_Callback(hObject, eventdata, handles)
set(handles.length1_2,'UserData',1);
% hObject    handle to length1_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of length1_2
% --- Executes on button press in length1_3.
function length1_3_Callback(hObject, eventdata, handles)
set(handles.length1_3,'UserData',10);
% hObject    handle to length1_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of length1_3
% --- Executes on button press in length2_1.
function length2_1_Callback(hObject, eventdata, handles)
set(handles.length2_1,'UserData',0.1);
% hObject    handle to length2_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of length2_1
% --- Executes on button press in length2_2.
function length2_2_Callback(hObject, eventdata, handles)
set(handles.length2_2,'UserData',1);
% hObject    handle to length2_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of length2_2
% --- Executes on button press in length2_3.
function length2_3_Callback(hObject, eventdata, handles)
set(handles.length2_3,'UserData',10);
% hObject    handle to length2_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of length2_3
% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: place code in OpeningFcn to populate axes1
% --- Executes on button press in tau_inert.
function tau_inert_Callback(hObject, eventdata, handles)
% hObject    handle to tau_inert (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of tau_inert
% --- Executes on button press in tau_cor.
function tau_cor_Callback(hObject, eventdata, handles)
% hObject    handle to tau_cor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of tau_cor
% --- Executes on button press in tau_grav.
function tau_grav_Callback(hObject, eventdata, handles)
% hObject    handle to tau_grav (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of tau_grav
% --- Executes on button press in tau_tot.
function tau_tot_Callback(hObject, eventdata, handles)
% hObject    handle to tau_tot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of tau_tot
% --- Executes during object creation, after setting all properties.
function axes4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
axes(hObject)
imshow('EOM.png')
% Hint: place code in OpeningFcn to populate axes4