close all; clear; clc

[tmp, name] = dos("echo $USER");
name = erase(name,newline);
catkin_dir = '/home/%s/catkin_ws/src/mavros/matlab_msg_gen_ros1/glnxa64/install/m';

addpath(sprintf(catkin_dir, name))
addpath('gui')
% rosinit
gui();
%%
global desiredRate
desiredRate= 5;
loopTime =20;
rate = rosrate(desiredRate);
rate.OverrunAction = 'slip';

odomsub =rossubscriber('/mavros/local_position/odom','nav_msgs/Odometry');
imusub =rossubscriber('/mavros/imu/data','sensor_msgs/Imu');

depthsub =rossubscriber('/camera/depth/image_raw','sensor_msgs/Image');
imagesub =rossubscriber('/camera/rgb/image_raw','sensor_msgs/Image');

pause(1);
[setpub,setmsg] = rospublisher('/mavros/setpoint_position/local','geometry_msgs/PoseStamped');

K=([565.6008952774197, 0.0, 320.5; 0.0, 565.6008952774197, 240.5; 0.0, 0.0, 1.0]);


for i=1:15
    setmsg.Pose.Position.X = 0;
    setmsg.Pose.Position.Y = 0;
    setmsg.Pose.Position.Z = 1;
    send(setpub,setmsg);
    pause(0.1);
    
    if rem(i,5)==0
        arming = rossvcclient('mavros/cmd/arming');
        testreq1 = rosmessage(arming);
        testreq1.Value=1;
        response1 = call(arming,testreq1,'Timeout',2);
        if testreq1.Value==1
            disp('Arming enabled');
        else
            disp('Arming failed');
            
        end
        
        set_mode = rossvcclient('mavros/set_mode');
        testreq2 = rosmessage(set_mode);
        testreq2.CustomMode='OFFBOARD';
        response2 = call(set_mode,testreq2,'Timeout',2);
        if testreq2.CustomMode=='OFFBOARD'
            disp('Offboard enabled');
        else
            disp('Offboard failed');
            
        end
    end
    
end


xf=[5;3];

xfz=1;

xd(:,1)=[0;0];
xdz=1;

reset(rate);
X=[];
Y=[];
Z=[];
VX=[];
VY=[];
VZ=[];

handles = guidata(gui);

for i = 1:desiredRate*loopTime
    
    time = rate.TotalElapsedTime;
    fprintf('Iteration: %d - Time Elapsed: %f\n',i,time)
    
    state = receive(odomsub);
    imu = receive(imusub);
    
    quat=[imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z];
    eul = quat2eul(quat);
    eul=[-eul(1) -eul(2) eul(3)];
    Rot=eul2rotm(eul);
    image_msg=receive(imagesub);
    depth_msg=receive(depthsub);
    
    [img,im_alpha] = readImage(image_msg);
    [dep,dp_alpha] = readImage(depth_msg);
    
    x=state.Pose.Pose.Position.X;
    X=[X;x];
    y=state.Pose.Pose.Position.Y;
    Y=[Y;y];
    z=state.Pose.Pose.Position.Z;
    Z=[Z;z];
    vx=state.Twist.Twist.Linear.X;
    VX=[VX;vx];
    vy=state.Twist.Twist.Linear.Y;
    VY=[VY;vy];
    vz=state.Twist.Twist.Linear.Z;
    VZ=[VZ;vz];
    
    trans=[x;y;z];
    
    transxy=[x;y];
    
    set(handles.posX, 'String', x);
    set(handles.posY, 'String', y);
    set(handles.posZ, 'String', z);
    set(handles.velX, 'String', vx);
    set(handles.velY, 'String', vy);
    set(handles.velZ, 'String', vz);
    
    % resize image
    scale=1/1;
    re_img=imresize(img,scale);
    re_dep=imresize(dep,scale);
    
    cloud=[];
    dep_mm=re_dep*1000;
    Sd=size(dep_mm);
    [pX, pY]=meshgrid(1:Sd(2),1:Sd(1));
    
    pX=pX-K(1,3)*scale+0.5;
    pY=pY-K(2,3)*scale+0.5;
    
    xDf=double(dep_mm/(K(1,1)*scale));
    yDf=double(dep_mm/(K(2,2)*scale));
    
    pX=pX.*xDf;
    pY=pY.*yDf;
    
    
    pXY=cat(3,pX,pY);
    
    cloud_nan=cat(3,pXY,dep_mm);
    cloud_nan=reshape(cloud_nan,[],3)/1000;
    %     tic
    cloud = rmmissing(cloud_nan);
    %     toc
    n=length(cloud);
    cam_eul=[pi/2+pi 0 pi/2+pi];
    rotcam=eul2rotm(cam_eul);
    
    cloud_affine=[];
    cloud_affine=([Rot trans]*[rotcam zeros(3,1);0 0 0 1]*[cloud';ones(1,n)])';
    
    ptCloud=pointCloud(cloud_affine);
    ptCloud_d=pcdownsample(ptCloud,'gridAverage',0.1);
    
    [groundPtsIdx,nonGroundPtCloud,groundPtCloud] = segmentGroundSMRF(ptCloud_d);
    
    ptCloud_obs = rmmissing(nonGroundPtCloud.Location);
    
    x_mean=mean(ptCloud_obs(:,1));
    set(handles.objX, 'String', x_mean);
    y_mean=mean(ptCloud_obs(:,2));
    set(handles.objY, 'String', y_mean);
    z_mean=mean(ptCloud_obs(:,3));
    set(handles.objZ, 'String', z_mean);
    temp=[x_mean;y_mean;z_mean];
    
    if  ~any(isnan(temp), 'all')
        %     obs=[x_mean;y_mean;z_mean];
        obs=[x_mean;y_mean];
    else
        obs=[-100;-100];
    end
    
    % potential field
    if i>1 && z>0.5
        [goal,vd]=potential(transxy,xd(:,i-1),xf,obs);
        
    else
        goal=xd(:,1);
    end
    
    
    xd(:,i)=goal;
    
    plot3(X,Y,Z,'LineWidth',2,'Color', 'b','parent',handles.axes1); hold( handles.axes1, 'on' )
    plot3(xd(1,1:i),xd(2,1:i),xfz*ones(size(xd(2,1:i))),'LineWidth',2,'Color', 'r','parent',handles.axes1);
    plot3(ptCloud_obs(:,1),ptCloud_obs(:,2),ptCloud_obs(:,3),'ok','MarkerSize',1,'parent',handles.axes1);
    %     plot3(ptCloud_d.Location(:,1),ptCloud_d.Location(:,2),ptCloud_d.Location(:,3),'ok','MarkerSize',1,'parent',handles.axes1);
    
    hold( handles.axes1, 'off' );    grid(handles.axes1,'on');
    axis(handles.axes1,[-2 10 -4 4 -1 3]);
    xlabel(handles.axes1,'x');
    ylabel(handles.axes1,'y');
    zlabel(handles.axes1,'z');
    
    imshow(img,'Parent',handles.axes2);
    imagesc(dep,'Parent',handles.axes3);
    
    
    pX=[];
    pY=[];
    pZ=[];
    
    setmsg.Pose.Position.X = xd(1,i);
    setmsg.Pose.Position.Y = xd(2,i);
    setmsg.Pose.Position.Z = xfz;
    
    send(setpub,setmsg);
    waitfor(rate);
    
    
end
rosshutdown

function [xd,vd]=potential(x,xd_pre,xf,obs)
global desiredRate ;
r_rho=1.0; % infludence of the obstacle
eps=0.5;
dphi_r=[0;0];
dt=1/desiredRate;
a=0.3 * 3; % velocity limits
k=0.1; % potential gain

dphi_a=-(x-xf);
r_obs=norm(x-obs);
r_goal=norm(x-xf);

n=2;
if r_obs<r_rho
    dphi_r=k*(1/(r_obs-eps)-1/r_rho)*(1/(r_obs^3))*r_goal^n*(x-obs) ...
        -k/2*n*(1/(r_obs-eps)-1/r_rho)^2*r_goal^(n-1)*(x-xf);
else
    dphi_r=0;
end


dphi_p=dphi_a+dphi_r;
dphi_p=a*dphi_p/(norm(dphi_p));

vd=dphi_p;
xd=xd_pre+dphi_p*dt;

end