from xml.dom import minidom
from logging import warning

class urdf:

    def __init__ (self, links, joints_types, joint_axis, rpy):
        """
        :param links: array of links lengths
        :param joints_types: array of joints types, can be 'prismatic' or 'revolute'
        :param joint_axis: array of axis according to joints motion
        :param rpy: array of the rotation around fixed axis
        default data will be UR5 data.
        """

        if links is None:
            links=['0.089159', '0.425', '0.392', '0.10915', '0.09465', '0.0823']
        if joints_types is None:
            joints_types=['revolute', 'revolute', 'revolute', 'revolute', 'revolute', 'revolute']
        if joint_axis is None:
            joint_axis=['z', 'y', 'y', 'y', 'z', 'y']
        if rpy is None:
            rpy=[]

        self.links = links
        self.joint_types = joints_types
        self.joint_axis= joint_axis
        self.axis = self.Axis(self.joint_types, joint_axis)
        self.links_number = len(self.links)
        self.rpy = rpy
        self.links_mass = self.calc_mass()

    def calc_origin(self, n):
        """
            calculate the origin of this joint according to the previous joint
        """
        if self.joint_data[n-1] == "revolute":
            if self.axis[n - 1] == '0 0 1':  # roll
                if self.rpy[n-1] == ['0 ', '0 ', '0 ']:  # links in the same direction
                    return "0 0 ${link" + str(n-1) + "_length}" # v
                elif self.rpy[n-1] == ['${1/2*pi} ', '0 ', '0 ']:
                    return "0 -${link" + str(n-1) + "_radius} ${link" + str(n-1) + "_length}"
                elif self.rpy[n-1] == ['0 ', '${pi/2} ', '0 ']:
                    return "0 0 ${link" + str(n) + "_radius + link" + str(n - 1) + "_length}"
                else:  # the links are perpendiculars
                    return "0 ${link" + str(n-1) + "_radius} ${link" + str(n-1) + "_length}"
            else:  # pitch
                if self.rpy[n-1] == ['0 ', '0 ', '0 ']:  # around y: links are in the same direction
                    return "0 ${link" + str(n-1) + "_radius+link" + str(n) + "_radius} ${link" + str(n-1) + "_length}"
                elif self.rpy[n-1] == ['0 ', '0 ', '${-pi/2} ']:  # around x: links are not in the same direction
                    return " ${link" + str(n-1) + "_radius+link" + str(n) + "_radius} 0 ${link" + str(n-1) + "_length}"
                else:  # round x:  the links are perpendiculars
                    return "0 0 ${link" + str(n-1) + "_length + link" + str(n) + "_radius}"
        else:  # prismatic
            if self.rpy[n-1] == ['0 ', '0 ', '0 ']:  # links in the same direction
                return "0 0 ${link" + str(n - 1) + "_length}"
            else:  # the links are perpendiculars
                return "0 0 ${link" + str(n-1) + "_length + link" + str(n) + "_radius}"

    def Axis(self, joint_types,joints_axis):
        """
            defining <axis> for each joint- Axis of rotation
        """
        Origin_axis = []
        for i in range(0, len(joint_types)):
            if joints_axis[i] == 'x':
                 Origin_axis.append('1 0 0')
            elif joints_axis[i] == 'y':
                   Origin_axis.append('0 1 0')
            elif joints_axis[i] == 'z':
                   Origin_axis.append('0 0 1')
            else:
                warning('wrong input.' + joints_axis[i] + ' entered. returning [0 0 0] ')
                Origin_axis.append('0 0 0')
            print(Origin_axis)
        return Origin_axis

    def calc_mass(self):
        """
            defining mass value for each link
        """
        cumulative_length = 0
        cumulative_weight = 0
        link_mass = []
        for l in self.links:
            cumulative_length += float(l)
            mass = cumulative_length*7.3149 + 1.1755 - cumulative_weight
            link_mass.append(str(mass))
            cumulative_weight += mass
        return link_mass

    def joint_creation(self, joint_number):
        joint_number = joint_number + 1
        jointname = 'joint' + str(joint_number)
        jtype = self.joint_types[joint_number]
        rpy = '0 0 0'
        xyz = '0 0 0'

        joint = ''' <!--  joint ''' + str(joint_number) + '''	-->
            <joint name="${prefix}''' + jointname + '''" type="${''' + jtype + '''}">
                <parent link="${prefix}link''' + str(joint_number - 1) + '''"/>
                <child link="${prefix}link''' + str(joint_number) + '''" />
                <origin xyz="''' + xyz + '''" rpy="''' + rpy + '''"/>
                <axis xyz="''' + self.axis[joint_number] + '''"/>
                <dynamics damping="0.7" />
            </joint> \n
            '''
        return joint

    def link_creation(self, link_number):
        link_number = link_number + 1
        linkname = 'link' + str(link_number)

        if link_number == 1:
            link = ''' <!-- link''' + str(link_number) + '''	-->
            <link name="${prefix}''' + linkname + '''">
                <collision>
                    <origin xyz="0 0 ${''' + linkname + '''_length / 2 }" rpy="0 0 0" />
                        <geometry>
                            <cylinder radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}"/>
                         </geometry>
                </collision>
            <visual>
                <origin xyz="0 0 ${''' + linkname + '''_length / 2}" rpy="0 0 0" />
                    <geometry>
                        <cylinder radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}"/>
                    </geometry>
                    <material name="LightGrey">
                        <color rgba="0.7 0.7 0.7 1.0"/>
                    </material>
            </visual>

            <xacro:solid_cylinder_inertia radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}" mass="${''' + linkname + '''_mass}">
            <origin xyz="0.0 0.0 ${''' + linkname + '''_length / 2 }" rpy="0 0 0" />
            </xacro:cylinder_inertial>
            </link> \n
                '''
        else:
            link = ''' <!-- link''' + str(link_number) + '''	-->
            <link name="${prefix}''' + linkname + '''">
                <collision>
                    <origin xyz="0 0 ${''' + linkname + '''_length / 2 - offset''' + link_number + '''}" rpy="0 0 0" />
                        <geometry>
                            <cylinder radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}"/>
                         </geometry>
                </collision>
            <visual>
                <origin xyz="0 0 ${''' + linkname + '''_length / 2 - offset''' + link_number + '''}" rpy="0 0 0" />
                    <geometry>
                        <cylinder radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}"/>
                    </geometry>
                    <material name="LightGrey">
                        <color rgba="0.7 0.7 0.7 1.0"/>
                    </material>
            </visual>
            
            <xacro:solid_cylinder_inertia radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}" mass="${''' + linkname + '''_mass}">
            <origin xyz="0.0 0.0 ${''' + linkname + '''_length / 2 }" rpy="0 0 0" />
            </xacro:cylinder_inertial>
            </link> \n
                '''
        return link

    def urdf_creation(self):
        head = '''<?xml version="1.0"?>
    <robot xmlns:xacro="http://wiki.ros.org/xacro"  name="arm">
        
        <!-- the inertial matrix of a solid cylinder-->
        <xacro:macro name="solid_cylinder_inertia" params="radius length mass *origin">
            <inertial>
                <mass value="${mass}" />
                <xacro:insert_block name="origin" />
                <inertia ixx="${0.0833333 * mass * (3 * radius * radius + length * length)}" ixy="0.0" ixz="0.0"
                 iyy="${0.0833333 * mass * (3 * radius * radius + length * length)}" iyz="0.0"
                 izz="${0.5 * mass * radius * radius}" />
            </inertial>
        </xacro:macro>
        
        <xacro:property name="camera_link" value="0.02" /> <!-- Size of square 'camera' box -->
        <xacro:property name="pi" value="3.1415926535897931"/>

        <xacro:property name="prefix" value="roni_"/>
   
        '''

        inertia = '''
        <xacro:property name="base_length" value="3.25"/>
        <xacro:property name="base_radius" value="0.060" />
        <xacro:property name="link0_radius" value="0.060" /> 
          
            <!-- Inertia parameters -->
        <xacro:property name="base_mass" value="1.0" />
        <xacro:property name="link0_mass" value="40.7" />
        
        <xacro:property name="link1_length" value="''' + self.links[0] + '''" /> 
        <xacro:property name="link2_length" value="''' + self.links[1] + '''" />
        <xacro:property name="link3_length" value="''' + self.links[2] + '''" />
        <xacro:property name="link4_length" value="''' + self.links[3] + '''" />
        <xacro:property name="link5_length" value="''' + self.links[4] + '''" />
        <xacro:property name="link6_length" value="''' + self.links[5] + '''" />

        
        <xacro:property name="link1_mass" value="''' + self.links_mass[0] + '''" /> 
        <xacro:property name="link2_mass" value="''' + self.links_mass[1] + '''" />
        <xacro:property name="link3_mass" value="''' + self.links_mass[2] + '''" />
        <xacro:property name="link4_mass" value="''' + self.links_mass[3] + '''" />
        <xacro:property name="link5_mass" value="''' + self.links_mass[4] + '''" />
        <xacro:property name="link6_mass" value="''' + self.links_mass[5] + '''" />
    
        <!-- Radius for each links- UR5 parameters- Do not change -->
        <xacro:property name="link1_radius" value="0.06" />
        <xacro:property name="link2_radius" value="0.054" />
        <xacro:property name="link3_radius" value="0.06" />
        <xacro:property name="link4_radius" value="0.04" />
        <xacro:property name="link5_radius" value="0.045" />
        <xacro:property name="link6_radius" value="0.045" /> 

        xacro:property name="offset1" value="${link1_radius/2}" /> <!-- Space btw top of the link and the each joint -->
        <xacro:property name="offset2" value="${link2_radius/2}" /> <!-- Space btw top of the link and the each joint -->
        <xacro:property name="offset3" value="${link3_radius/2}" /> <!-- Space btw top of beam and the each joint -->
        <xacro:property name="offset4" value="${link4_radius/2}" /> <!-- Space btw top of beam and the each joint -->
        <xacro:property name="offset5" value="${link5_radius/2}" /> <!-- Space btw top of beam and the each joint -->
        <xacro:property name="offset6" value="${link6_radius/2}" /> <!-- Space btw top of beam and the each joint --> 
        
        
        <!-- used for fixing robot to Gazebo 'base_link -->
        <link name="world"/>
        
        <!--  joint 0	-->
        <joint name="fixed" type="fixed">
        <parent link="world"/>
        <child link="${prefix}link1"/>
        </joint>

        '''

        data = ''
        for i in range(1,len(self.links)+1):
            data = data + self.link_creation(i) + self.joint_creation(i)

        end_file = ''' 
        <!-- hokuyo joint	-->
        <joint name="hokuyo_joint" type="fixed">
            <axis xyz="0 1 0" />
            <origin xyz="0 0 ${link6_length - offset6/2}" rpy="0 0 0"/>
            <parent link="${prefix}link6"/>
            <child link="hokuyo_link"/>
        </joint>
    
        <!-- Hokuyo Laser -->
        <link name="hokuyo_link">
            <collision>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                    <geometry>
                        <box size="0.1 0.1 0.1"/>
                    </geometry>
            </collision>
    
            <visual>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                    <geometry>
                        <mesh filename="package://rrbot_description/meshes/hokuyo.dae"/>
                    </geometry>
            </visual>
    
            <inertial>
                <mass value="1e-5" />
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
            </inertial>
        </link>
        
    </robot>'''

        return head + inertia + self.joints + self.links + end_file

    def urdf_creation_final(self):
        data = '''
                <?xml version="1.0"?>
        <robot xmlns:xacro="http://wiki.ros.org/xacro"  name="arm" >
        
          <!-- common stuff -->
          <xacro:include filename="$(find man_gazebo)/urdf/common.gazebo.xacro" />
         <xacro:include filename="$(find man_gazebo)/urdf/6dof/transmission_6dof.xacro" />
          <xacro:include filename="$(find man_gazebo)/urdf/gazebo.xacro" />
        
        <link name="world" />
        
          <joint name="world_joint" type="fixed">
            <parent link="world" />
            <child link = "base_link" />
            <origin xyz="0 0 0" rpy="0.0 0.0 0.0" />
          </joint>
        
        
          <xacro:macro name="cylinder_inertial" params="radius length mass *origin">
            <inertial>
              <mass value="${mass}" />
              <xacro:insert_block name="origin" />
              <inertia ixx="${0.0833333 * mass * (3 * radius * radius + length * length)}" ixy="0.0" ixz="0.0"
                iyy="${0.0833333 * mass * (3 * radius * radius + length * length)}" iyz="0.0"
                izz="${0.5 * mass * radius * radius}" />
            </inertial>
          </xacro:macro>
        
        <xacro:macro name="joint_limit" params="joint_type link_length ">
            <xacro:if value="${joint_type == 'revolute'}"  >
                <xacro:property name="joint_upper_limit" value="${pi}" /> 
                <xacro:property name="joint_lower_limit" value="${-pi}" /> 
            </xacro:if>
            <xacro:unless value="${joint_type == 'revolute'}"  >
                <xacro:property name="joint_upper_limit" value="${link_length}" /> 
                <xacro:property name="joint_lower_limit" value="${-link_length}" />  
            </xacro:unless>
            <limit lower="${joint_lower_limit}" upper="${joint_upper_limit}" effort="150.0" velocity="3.15"/>
        </xacro:macro>
        
        <xacro:macro name="arm_robot" params="prefix ">
            
            <xacro:property name="joint0_type" value="prismatic" /> 
            <xacro:property name="joint1_type" value="revolute" /> 
            <xacro:property name="joint2_type" value="revolute" /> 
            <xacro:property name="joint3_type" value="revolute" /> 
            <xacro:property name="joint4_type" value="revolute" /> 
            <xacro:property name="joint5_type" value="revolute" /> 
            <xacro:property name="joint6_type" value="revolute" /> 
        
            <xacro:property name="joint0_axe" value="0 1 0" /> 
            <xacro:property name="joint1_axe" value="0 0 1" /> 
            <xacro:property name="joint2_axe" value="0 1 0" /> 
            <xacro:property name="joint3_axe" value="0 1 0" /> 
            <xacro:property name="joint4_axe" value="0 1 0" /> 
            <xacro:property name="joint5_axe" value="0 1 0" /> 
            <xacro:property name="joint6_axe" value="0 0 1" /> 
        
            
            <xacro:property name="link0_length" value="0.25" />
          <xacro:property name="base_length"  value="11" /> 
          <xacro:property name="base_height"  value="0.5" />
            <xacro:property name="link1_length" value="0.25" /> 
            <xacro:property name="link2_length" value="0.5" /> 
            <xacro:property name="link3_length" value="0.4" /> 
            <xacro:property name="link4_length" value="0.07" /> 
            <xacro:property name="link5_length" value="0.1" /> 
            <xacro:property name="link6_length" value="0.02" /> 
        
            <!-- Inertia parameters -->
            <xacro:property name="base_mass" value="44" /> 
            <xacro:property name="link0_mass" value="7" />
            <xacro:property name="link1_mass" value="3.7" />
            <xacro:property name="link2_mass" value="8.393" />
            <xacro:property name="link3_mass" value="2.275" />
            <xacro:property name="link4_mass" value="1.219" />
            <xacro:property name="link5_mass" value="1.219" />
            <xacro:property name="link6_mass" value="0.1879" />  
        
            <xacro:property name="base_radius" value="0.060" />
              <xacro:property name="link0_radius" value="0.060" /> 
            <xacro:property name="link1_radius" value="0.060" /> 
            <xacro:property name="link2_radius" value="0.060" />   
            <xacro:property name="link3_radius" value="0.060" />  
            <xacro:property name="link4_radius" value="0.040" />      
            <xacro:property name="link5_radius" value="0.030" />   
            <xacro:property name="link6_radius" value="0.025" /> 
        
        
            <!--   Base Link -->
            <link name="${prefix}base_link" >
              <visual>
                    <origin xyz="0 0 ${base_height/2}" rpy="0 0 0" /> 
                <geometry>
                        <box size="${base_height} ${base_length} ${base_height}"/> 
                </geometry>
              </visual>
              <collision>
                     <origin xyz="0 0 ${base_height/2}" rpy="0 0 0" /> 
                <geometry>
                        <box size="${base_length} ${base_length} ${base_height}"/> 	
                </geometry>
              </collision>
              <inertial>
                <mass value="${base_mass}" />
                <origin xyz="0.0 0.0 ${base_height/2}" />
                <inertia ixx="436.5" ixy="0"  ixz="0"
                                  iyy="1.8" iyz="0"
                                          izz="436.5"/>
              </inertial>
             
            </link>
        
        <!--  joint 0	-->
            <joint name="${prefix}joint0" type="${joint0_type}">
              <parent link="${prefix}base_link" />
              <child link = "${prefix}link0" />
              <origin xyz="0.0 0 ${base_height + link0_radius+0.01}" rpy="0 0.0 0" />
              <axis xyz="${joint0_axe}" />
              <xacro:joint_limit joint_type="${joint0_type}" link_length="${base_length/2}"/>
              <dynamics damping="0.0" friction="0.0"/>
            </joint>
        
        <!--  link 0 -->
            <link name="${prefix}link0">
              <visual>
                <origin xyz="0 0 ${link0_radius} " rpy="0 0 0" /> 
                <geometry>
                    <cylinder radius="${link0_radius}" length="${link0_length}"/>	 
                </geometry>
              </visual>
              <collision>
                 <origin xyz="0 0 ${link0_radius}" rpy="0 0 0" /> 
                <geometry>
                    <cylinder radius="${link0_radius}" length="${link0_length}"  mass="${link0_mass}"/>
                </geometry>
              </collision>
              <xacro:cylinder_inertial radius="${link0_radius}" length="${link0_length}" mass="${link0_mass}">
                <origin xyz="0.0 0.0 ${link0_radius}" rpy="0 0 0" />
              </xacro:cylinder_inertial>
            </link> 
        
        
        <!--  joint 1	-->
            <joint name="${prefix}joint1" type="${joint1_type}">
              <parent link="${prefix}link0" />
              <child link = "${prefix}link1" />
              <origin xyz="0.0 0.0 ${link0_length}" rpy="${pi/2} 0 0" />
              <axis xyz="${joint1_axe}" />
              <xacro:joint_limit joint_type="${joint1_type}" link_length="${link1_length}"/>
              <dynamics damping="0.0" friction="0.0"/>
            </joint>
        <!--  link 1  -->
            <link name="${prefix}link1">
              <visual>
                <origin xyz="0 0 ${link1_length/2} " rpy="0 0 0" /> 
                <geometry>
                    <cylinder radius="${link1_radius}" length="${link1_length}"/>	 
                </geometry>
              </visual>
              <collision>
                 <origin xyz="0 0 ${link1_length/2}" rpy="0 0 0" /> 
                <geometry>
                    <cylinder radius="${link1_radius}" length="${link1_length}"  mass="${link1_mass}"/>
                </geometry>
              </collision>
              <xacro:cylinder_inertial radius="${link1_radius}" length="${link1_length}" mass="${link1_mass}">
                <origin xyz="0.0 0.0 ${link1_length/2}" rpy="0 0 0" />
              </xacro:cylinder_inertial>
            </link>
        <!--  joint 2	-->
            <joint name="${prefix}joint2" type="${joint2_type}">
              <parent link="${prefix}link1" />
              <child link = "${prefix}link2" />
              <origin xyz="0.0 ${link1_radius * 2.0} ${link1_length }" rpy="0.0 ${pi/2.0} 0.0" />
              <axis xyz="${joint2_axe}" />
              <xacro:joint_limit joint_type="${joint2_type}" link_length="${link2_length}"/>
              <dynamics damping="0.0" friction="0.0"/>
            </joint>
        <!-- link 2	-->
            <link name="${prefix}link2">
              <visual>
                <origin xyz="0 0 ${link2_length / 2}" rpy="0 0 0" />
                <geometry>
                    <cylinder radius="${link2_radius}" length="${link2_length}"/> 
                </geometry>
              </visual>
              <collision>
                <origin xyz="0 0 ${link2_length / 2 }" rpy="0 0 0" />
                <geometry>
                <cylinder radius="${link2_radius}" length="${link2_length}"/>
                </geometry>
              </collision>
              <xacro:cylinder_inertial radius="${link2_radius}" length="${link2_length}" mass="${link2_mass}">
                <origin xyz="0.0 0.0 ${link2_length / 2 }" rpy="0 0 0" />
              </xacro:cylinder_inertial>
            </link>
        <!--  joint 3	-->
            <joint name="${prefix}joint3" type="${joint3_type}">
              <parent link="${prefix}link2" />
              <child link = "${prefix}link3" />
              <origin xyz="0.0 ${link2_radius*2.0} ${link2_length}" rpy="0.0 0.0 0.0" />
              <axis xyz="${joint3_axe}" />
              <xacro:joint_limit joint_type="${joint3_type}" link_length="${link3_length}"/>
              <dynamics damping="0.0" friction="0.0"/>
            </joint>
        
        <!--  link 3 -->  
            <link name="${prefix}link3">
              <visual>
                <origin xyz="0 0 ${link3_length / 2 }" rpy="0 0 0" />
                <geometry>
                <cylinder radius="${link3_radius}" length="${link3_length}"/>
                </geometry>
              </visual>
              <collision>
                <origin xyz="0 0 ${link3_length / 2 }" rpy="0 0 0" />
                <geometry>
                    <cylinder radius="${link3_radius}" length="${link3_length}"/>
                </geometry>
              </collision>
              <xacro:cylinder_inertial radius="${link3_radius}" length="${link3_length}" mass="${link3_mass}">
                <origin xyz="0 0 ${link3_length / 2 }" rpy="0 0 0" />
              </xacro:cylinder_inertial>
            </link>
        <!--  joint 4	-->
            <joint name="${prefix}joint4" type="${joint4_type}">
              <parent link="${prefix}link3" />
              <child link = "${prefix}link4" />
              <origin xyz="0 -${link4_radius + link3_radius } ${link3_length}" rpy="0.0 0 0.0" />
              <axis xyz="${joint4_axe} " />
              <xacro:joint_limit joint_type="${joint4_type}" link_length="${link4_length}"/>
              <dynamics damping="0.0" friction="0.0"/>
            </joint>
        
        <!-- link 4 -->
            <link name="${prefix}link4">
              <visual>
                <origin xyz="0 0 ${link4_length/2}" rpy="0 0 0" />
                <geometry>
                       <cylinder radius="${link4_radius}" length="${link4_length}"/>
                </geometry>
              </visual>
              <collision>
                <origin xyz="0 0 ${link4_length/2}" rpy="0 0 0" />
                <geometry>
                       <cylinder radius="${link4_radius}" length="${link4_length}"/>	
                </geometry>
              </collision>
              <xacro:cylinder_inertial radius="${link4_radius}" length="${link4_length}" mass="${link4_mass}">
                <origin xyz="0 0 ${link4_length/2}" rpy="0 0 0" />
              </xacro:cylinder_inertial>
            </link>
        <!--  joint 5	-->
            <joint name="${prefix}joint5" type="${joint5_type}">
              <parent link="${prefix}link4" />
              <child link = "${prefix}link5" />
              <origin xyz="0 0 ${link4_length+link5_radius}" rpy=" ${-pi/2} 0 0" />
              <axis xyz="${joint5_axe}" />
              <xacro:joint_limit joint_type="${joint5_type}" link_length="${link5_length}"/>
              <dynamics damping="0.0" friction="0.0"/>
            </joint>
        
        <!-- link 5 -->
            <link name="${prefix}link5">
              <visual>
                <origin xyz=" 0 0 ${link5_length/2} " rpy="0 0 0" />
                <geometry>
                <cylinder radius="${link5_radius}" length="${link5_length}"/>
                </geometry>
              </visual>
              <collision>
                <origin xyz="0 0 ${link5_length/2} " rpy="0 0 0" />
                <geometry>
                <cylinder radius="${link5_radius}" length="${link5_length}"/>
                </geometry>
              </collision>
              <xacro:cylinder_inertial radius="${link5_radius}" length="${link5_length}" mass="${link5_mass}">
                <origin xyz="0 0 ${link5_length/2}" rpy="0 0 0" />
              </xacro:cylinder_inertial>
            </link>
        <!--  joint 6	-->
            <joint name="${prefix}joint6" type="${joint6_type}">
              <parent link="${prefix}link5" />
              <child link = "${prefix}link6" />
              <origin xyz="0 0 ${link5_length } " rpy="0 0 0" />
              <axis xyz="${joint6_axe}" />
              <xacro:joint_limit joint_type="${joint6_type}" link_length="${link6_length}"/>
              <dynamics damping="0.0" friction="0.0"/>
            </joint>
        <!-- link 6 -->
            <link name="${prefix}link6">
              <visual>
                <origin xyz="0 0 ${link6_length/2}" rpy="0 0 0" />
                <geometry>
                <cylinder radius="${link6_radius}" length="${link6_length}"/>
                </geometry>
              </visual>
              <collision>
                <origin xyz="0 0 ${link6_length/2}" rpy="0 0 0" />
                <geometry>
                <cylinder radius="${link6_radius}" length="${link6_length}"/>
                </geometry>
              </collision>
              <xacro:cylinder_inertial radius="${link6_radius}" length="${link6_length}" mass="${link6_mass}">
                <origin xyz="0 0 ${link6_length/2}" rpy="0 0 0" />
              </xacro:cylinder_inertial>
            </link>
        
                 <!-- fake joint - the rotation about z axe of the camera is not important -->
                <joint name="fake_joint" type="revolute">
              <parent link="${prefix}link6" />
              <child link = "camera_link" />
              <origin xyz="0.0  0.0 ${link6_length}" rpy="0.0 0.0 0" />
              <axis xyz="0 0 1"/>
              <xacro:joint_limit joint_type="revolute" link_length="0.1"/>
              <dynamics damping="0.0" friction="0.0"/>
                </joint>
        
            <!-- camera link -->
                <link name="camera_link">
                  <collision>
                    <geometry>
                      <box size="0.01 0.01 0.01"/>
                    </geometry>
                    <origin rpy="0 0 0" xyz="0 0 0.005"/>
                  </collision>
                  <xacro:cylinder_inertial radius="0.01" length="0.01" mass="0.01">
                    <origin xyz="0.0 0.0 0.005" rpy="0 0 0" />
                  </xacro:cylinder_inertial>
                </link>
        
        
            <joint name="${prefix}ee_fixed_joint" type="fixed">
              <parent link="${prefix}camera_link" />
              <child link = "${prefix}ee_link" />
              <origin xyz="0.0  0.0 0.01" rpy="0.0 0.0 0" />
           
            </joint>
        
        <!-- ee link -->
            <link name="${prefix}ee_link">
              <collision>
                <geometry>
                  <box size="0.01 0.01 0.01"/>
                </geometry>
                <origin rpy="0 0 0" xyz="0 0 0.005"/>
              </collision>
            </link>
        
            <xacro:arm_transmission prefix="${prefix}" />
            <xacro:arm_gazebo prefix="${prefix}" />
        
          </xacro:macro>
        
           <xacro:arm_robot prefix=""/>
        
        
        </robot>
        '''

        return data


    def urdf_xml(self,data, file_name):
        mydata = data
        myfile = open(file_name + " urdf.xacro", "w")
        myfile.write(mydata)
        myfile.close()
