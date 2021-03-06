classdef FEpVrepRobot < DQ_VrepRobot
    
    properties
        joint_names;
        base_frame_name;
        base_frame;
    end
    
    methods 
        function obj = FEpVrepRobot(robot_name,vrep_interface)
            
            obj.robot_name = robot_name;
            obj.vrep_interface = vrep_interface;
            
            splited_name = strsplit(robot_name,'#');
            robot_label = splited_name{1};
            if ~strcmp(robot_label,'UR10')
                error('UR10')
            end
            if length(splited_name) > 1
                robot_index = splited_name{2};
            else
                robot_index = '';
            end
            
            %Initialize joint names and base frame
            obj.joint_names = {};
            for i=1:6
                current_joint_name = {robot_label,'_joint',int2str(i),robot_index};
                obj.joint_names{i} = strjoin(current_joint_name,'');
            end
            obj.base_frame_name = obj.joint_names{1};
        end            
   
        function send_q_to_vrep(obj,q)
            obj.vrep_interface.set_joint_positions(obj.joint_names,q)
        end
        
        function q = get_q_from_vrep(obj)
            q = obj.vrep_interface.get_joint_positions(obj.joint_names,65536);
        end
        
        
        
        function kin = kinematics(obj)
            %Standard D-H of FE Panda
            FEp_DH_theta = [0, 0, 0, 0, 0, 0];            
            FEp_DH_d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922];
            FEp_DH_a = [0, -0.612, -0.5723, 0, 0, 0];
            FEp_DH_alpha = [pi/2, 0 , 0, pi/2, -pi/2, 0];
            
%             FEp_DH_theta = [0, 0, 0, 0, 0, 0];            
%             FEp_DH_d = [0.1807, 0, 0, 0.17415, 0.1199, 0.1166];
%             FEp_DH_a = [0, -0.6127, -0.5715, 0, 0, 0];
%             FEp_DH_alpha = [pi/2, 0 , 0, pi/2, -pi/2, 0];

%             FEp_DH_theta=[0, 0, 0, 0, 0, 0, 0];
%             FEp_DH_d = [0.333, 0, 0.316, 0, 0.384, 0, 0];
%             FEp_DH_a = [0, 0, 0, 0.0825, -0.0825, 0, 0.088];
%             FEp_DH_alpha = [0, -pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2];
            FEp_DH_matrix = [FEp_DH_theta;
                            FEp_DH_d;
                            FEp_DH_a;
                            FEp_DH_alpha];
            
            kin = DQ_SerialManipulator(FEp_DH_matrix, 'standard');
            % We set the transformation from the world frame to the robot
            % base frame. Therefore, the end-effector pose is given by
%             transfW_rf = [[-1 0 0; 0 1 0; 0 0 -1], [0,0,1]'; 0 0 0 1];
%             pose_effector = MatrixToDQuaternion(transfW_rf)*fkm(q);
%             kin.set_reference_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
%               kin.set_reference_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
%               kin.set_base_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            % kin.set_effector(1+0.5*DQ.E*DQ.k*0.1070);
%             kin.set_effector(1+0.5*DQ.E*DQ.k*0.1070);
        end
    end
end