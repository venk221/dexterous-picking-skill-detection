classdef panda_whisperer < handle
    %PANDA WHISPERER tests accessing data from Panda using a class
    %   class-based approach to communicating with Panda arm

    properties
        % fields for setting up publishers and subscribers
        num_joints
        num_fingers
        dq_topic
        finger_topic
        joint_states_topic
        joint_subscriber
        dq_publisher
        dq_msg
        finger_publisher
        finger_msg
        left_ajenyedy

        % camera pose and focal lengths
        c
        varphi

        % feature error vector data
        left_e_subscriber
        right_e_subscriber
        left_em_subscriber
        right_em_subscriber
        e_
        r_em
        l_em

        % actual joint angles and velocities
        qa
        dqa
        % actual finger joint states
        fqa 
        fdqa

        % jacobian definition variables
        q
        tftree
        lhtm
        rhtm
        ldedqf
        rdedqf
        lp_camf
        rp_camf
               
        % controller variables
        dq

% dq

        % stop flag
%         end_flag
%         end_flag_subscriber

    end

    methods
        function obj = panda_whisperer(first_, loadHTM)
            % Construct an instance of Panda Whisperer
            
            % if first panda_whisperer created, start up ros
            if(first_)
                rosshutdown
                rosinit
            end
            
            % define symbolic q
            syms q1 q2 q3 q4 q5 q6 q7 q8 q9
            obj.q = [q1; q2; q3; q4; q5; q6; q7; q8; q9];

            % acquire the tf transform tree
            fprintf("Acquiring TF tree\n");
            obj.tftree = rostf;
            pause(0.5)

            % set camera poses and focal lengths
            obj.c = zeros(6,2);
            obj.varphi = [6.5; 6.5]; % overwritten in get rosparams

            % acquire homogeneous transform matrices for left and right cam
            fprintf("Setting up Jacobians\n");
            if loadHTM
                load('RTHM.mat');
                load('LTHM.mat');
                load('LDEDQF.mat');
                load('RDEDQF.mat');
                load('RP_CAMF.mat');
                load('LP_CAMF.mat');
                obj.lhtm = RHTM;
                obj.rhtm = LHTM; 
                obj.ldedqf = LDEDQF;
                obj.rdedqf = RDEDQF;
                obj.rp_camf = RP_CAMF;
                obj.lp_camf = LP_CAMF;
            else
                [obj.lhtm, obj.rhtm] = getPandaHTM(obj.q, obj.tftree);
                obj.setupJacobian();
            end
            % obj.setupJacobian();
            fprintf("Finished setting up Jacobians\n");

            % instantiate pub/sub framework to default values
            obj.num_joints = 0;
            obj.num_fingers = 0;
            obj.dq_topic = string.empty;
            obj.finger_topic = string.empty;
            obj.joint_states_topic = [];
            obj.joint_subscriber = [];
            obj.dq_publisher = [];
            obj.dq_msg = [];
            obj.finger_publisher = [];
            obj.finger_msg = [];
            
            % instantiate data values to zero
            obj.qa = [];
            obj.dqa = [];
            obj.fqa = [];
            obj.fdqa = [];
            obj.dq = zeros(7,1);
            obj.e_ = zeros(4,1);
            obj.r_em = 0;
            obj.l_em = 0;

        end
        
        function getRosParams(obj)        
            % source topics from rosparam server and set up publishers
            
            % acquire number of joints and fingers on robot
            obj.num_joints = rosparam("get", "/vs_numjoints");
            obj.num_fingers = rosparam("get", "/vs_numfingers");
            
            % set up joint velocity control publishers and instantiate msg
            for i = 1:obj.num_joints
                obj.dq_topic(i) = rosparam("get", "/dq"+i+"_topic");
                if i == 1
                    obj.dq_publisher = rospublisher(obj.dq_topic(i),"std_msgs/Float64","DataFormat","struct");
                    obj.dq_msg = rosmessage(obj.dq_publisher(i));
                    obj.dq_msg.Data = 0.0;
                else
                    obj.dq_publisher(i) = rospublisher(obj.dq_topic(i),"std_msgs/Float64","DataFormat","struct");
                    obj.dq_msg(i) = rosmessage(obj.dq_publisher(i));
                    obj.dq_msg(i).Data = 0.0;
                end
            end

            % set up finger control publishers and instantiate msgs
            for i = 1:obj.num_fingers
                obj.finger_topic(i) = rosparam("get", "/finger"+i+"_topic");
                if i == 1
                    obj.finger_publisher = rospublisher(obj.finger_topic(i),"std_msgs/Float64","DataFormat","struct");
                    obj.finger_msg = rosmessage(obj.finger_publisher(i));
                else
                    obj.finger_publisher(i) = rospublisher(obj.finger_topic(i),"std_msgs/Float64","DataFormat","struct");
                    obj.finger_msg(i) = rosmessage(obj.finger_publisher(i));
                end                
            end

            % subscribe to joint states topic
            obj.joint_states_topic = rosparam("get", "/joint_states_topic");
            obj.joint_subscriber = rossubscriber(obj.joint_states_topic ,@obj.jointStateCallback,"DataFormat","struct");

            % get camera focal length from param server:
            obj.varphi(1) = rosparam("get", "/camera_focal_length");
            obj.varphi(2) = rosparam("get", "/camera_focal_length");
        
            % set up error vector subscribers
            obj.left_e_subscriber = rossubscriber("/Error_Left", @obj.leftErrorCallback,"DataFormat","struct");
            obj.right_e_subscriber = rossubscriber("/Error_Right", @obj.rightErrorCallback,"DataFormat","struct");
            obj.left_em_subscriber = rossubscriber("/vs_lemp", @obj.leftEmCallback,"DataFormat","struct");
            obj.right_em_subscriber = rossubscriber("/vs_remp", @obj.rightEmCallback,"DataFormat","struct");
        end

        function setupJacobian(obj)
            % image jacobian
            [ldedp, rdedp] = de_dp_(obj.c, obj.varphi);
            
            % manipulator jacobian
            ldpdq = dp_dq(obj.q, obj.lhtm);
            rdpdq = dp_dq(obj.q, obj.rhtm);
            
            % composed jacobian: change in error to change in joint angles
            ldedq = ldedp*ldpdq;
            rdedq = rdedp*rdpdq;

            % set up MATLAB functions to use symbolic jacobians efficiently
            obj.ldedqf = matlabFunction(ldedq);
            obj.rdedqf = matlabFunction(rdedq);
            
            % set up MATLAB function for calculating FK endeff position
            obj.lp_camf = matlabFunction(obj.lhtm(1:3,4));
            obj.rp_camf = matlabFunction(obj.rhtm(1:3,4));
        end

        function [lp_cam, rp_cam] = getFK(obj)
            lp_cam = obj.lp_camf(obj.qa(1),obj.qa(2),obj.qa(3),obj.qa(4),obj.qa(5),obj.qa(6),obj.qa(7));
            rp_cam = obj.rp_camf(obj.qa(1),obj.qa(2),obj.qa(3),obj.qa(4),obj.qa(5),obj.qa(6),obj.qa(7));
        end

        function jointStateCallback(obj, ~, msg)
            % actual Panda joint angles and velocities
            for i = 1:(obj.num_fingers+obj.num_joints)
                if i < 3
                    obj.fqa(i) = msg.Position(i);
                    obj.fdqa(i) = msg.Velocity(i);
                else
                    obj.qa(i-2) = msg.Position(i);
                    obj.dqa(i-2) = msg.Velocity(i);
                end
            end
        end

        function leftErrorCallback(obj, ~, msg)
            % obtain feature error vector from left camera in pixels
            obj.e_(1) = msg.Position.X;
            obj.e_(2) = msg.Position.Y;
        end

        function rightErrorCallback(obj, ~, msg)
            % obtain feature error vector from right camera in pixels
            obj.e_(3) = msg.Position.X;
            obj.e_(4) = msg.Position.Y;
        end

        function leftEmCallback(obj, ~, msg)
            % obtain feature error vector magnitude from left camera in pix
            obj.l_em = msg.Data;
        end
        
        function rightEmCallback(obj, ~, msg)
            % obtain feature error vector magnitude from right camera in pix
            obj.r_em = msg.Data;
        end

        function updateController(obj)
            % visual servoing controller, produces dq for publishing
            
            % update position of arm
            lp_cam = obj.lp_camf(obj.qa(1),obj.qa(2),obj.qa(3),obj.qa(4),obj.qa(5),obj.qa(6),obj.qa(7));
            rp_cam = obj.rp_camf(obj.qa(1),obj.qa(2),obj.qa(3),obj.qa(4),obj.qa(5),obj.qa(6),obj.qa(7));
            % substitute values into Jacobians
            ldedq = obj.ldedqf(obj.qa(1),obj.qa(2),obj.qa(3),obj.qa(4),obj.qa(5),obj.qa(6),obj.qa(7),lp_cam(1),lp_cam(2),lp_cam(3));
            rdedq = obj.rdedqf(obj.qa(1),obj.qa(2),obj.qa(3),obj.qa(4),obj.qa(5),obj.qa(6),obj.qa(7),rp_cam(1),rp_cam(2),rp_cam(3));
            % unify left and right Jacobians 
            dedq = [ldedq; rdedq];

            % visual servoing gains
            lambda = 0.001;
            
            % generate desired velocities using visual servoing controller
            obj.dq = -lambda * (pinv(dedq)*obj.e_);

            % @TODO: obey joint limits and velocity limits

            % publish desired velocities
            obj.publishDesiredVelocities();

        end
        
        function publishDesiredVelocities(obj)
            % publish dq values to Panda velocity controllers
            for i = 1:obj.num_joints
                obj.dq_msg(i).Data = obj.dq(i);
                send(obj.dq_publisher(i),obj.dq_msg(i));
            end
        end

        function setDq(obj, dq)
            % set desired joint velocities
            for i = 1:obj.num_joints
                obj.dq(i) = dq(i);
            end
        end
    end
end