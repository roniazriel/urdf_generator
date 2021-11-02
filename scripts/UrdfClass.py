from logging import warning
from datetime import datetime

class UrdfClass(object):
    """ this class create URDF files """

    def __init__(self, links=None, joints=None, joints_axis=None, rpy=None):
        """
        :param joints: array of joints types- can be 'revolute' or 'prismatic'
        :param links: array of the links lengths in meters [must be positive float]
        the first link will be the base link, who is always the same(connected to the world and link1  - his joint is limited to 0)
        """
        if rpy is None:
            rpy = []
        if joints_axis is None:
            joints_axis = ['z', 'y', 'y', 'y', 'z', 'y']
        if joints is None:
            joints = ['revolute', 'prismatic', 'revolute', 'revolute', 'revolute', 'revolute']
        if links is None:
            links = [1, 1, 1, 1, 1, 1]
        self.links = links
        self.joint_data = joints
        self.axis = self.init_calc(joints, joints_axis)
        self.links_number = len(self.links)
        self.rpy = rpy
        self.weights = self.calc_weight()

    def calc_weight(self):
        """
        this function calculate the weight of the links according to accumulated weight and length of arm
        :return: weights- the weight [kg] of each link - list of strings  (from the 2nd link)
        """
        coeffs = [8.79055, 4.2928]  # the coeffs of the linear eauation (found according UR5 and motoman)
        weights = [0]  # the wieght of each link
        acc_length = 0  # accumelated length
        acc_weight = 0  # accumelated weight
        for link in self.links[1:]:
            acc_length = acc_length + float(link)
            weights.append(round(acc_length * coeffs[0] + coeffs[1] - acc_weight, 2))
            acc_weight = acc_weight + weights[-1]
        while len(weights) < 7:
            weights.append(1)
        return [str(weight) for weight in weights]

    def urdf_data(self):
        head = '''<?xml version="1.0"?>
        <robot xmlns:xacro="http://wiki.ros.org/xacro"  name="arm">
        <xacro:include filename="$(find man_gazebo)/urdf/common.gazebo.xacro" />
        <xacro:include filename="$(find man_gazebo)/urdf/''' + str(self.links_number) + '''dof/transmission_''' + str(
            self.links_number) + '''dof.xacro" />
        <xacro:include filename="$(find man_gazebo)/urdf/gazebo.xacro" />
        <xacro:macro name="cylinder_inertial" params="radius length mass *origin">

        <link name="world" />
          <joint name="world_joint" type="fixed">
            <parent link="world" />
            <child link = "base_link" />
            <origin xyz="0 0 0" rpy="0.0 0.0 0.0" />
          </joint>


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
                <xacro:property name="joint_lower_limit" value="${0}" />
            </xacro:unless>
        <limit lower="${joint_lower_limit}" upper="${joint_upper_limit}" effort="150.0" velocity="3.15"/>
        </xacro:macro>

        <xacro:macro name="arm_robot" params="prefix ">'''

        inertia_parameters = '''
        <xacro:property name="base_radius" value="0.060" />
        <xacro:property name="link0_radius" value="0.060" /> 
        <xacro:property name="base_length"  value="11" /> 
        <xacro:property name="base_height"  value="0.5" />

            <!-- Inertia parameters -->
        <xacro:property name="base_mass" value="1.0" />
        <xacro:property name="link0_mass" value="40.7" />
        <xacro:property name="link1_mass" value="3.7" />
        <xacro:property name="link2_mass" value="''' + self.weights[1] + '''" />
        <xacro:property name="link3_mass" value="''' + self.weights[2] + '''" />
        <xacro:property name="link4_mass" value="''' + self.weights[3] + '''" />
        <xacro:property name="link5_mass" value="''' + self.weights[4] + '''" />
        <xacro:property name="link6_mass" value="''' + self.weights[5] + '''" />

        <xacro:property name="link1_radius" value="0.049" />
        <xacro:property name="link2_radius" value="0.045" />
        <xacro:property name="link3_radius" value="0.040" />
        <xacro:property name="link4_radius" value="0.035" />
        <xacro:property name="link5_radius" value="0.030" />
        <xacro:property name="link6_radius" value="0.025" /> '''

        base_link = '''
        <!-- Base Link -->
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

        <xacro:property name="joint0_type" value="prismatic" /> 
        <xacro:property name="joint0_axe" value="0 1 0" /> 
        <xacro:property name="link0_length" value="0.25" />

        <!--  joint 0   -->
        <joint name="${prefix}joint0" type="${joint0_type}">
          <parent link="${prefix}base_link" />
          <child link = "${prefix}link0" />
          <origin xyz="0.0 0 ${base_height + link0_radius+0.01}" rpy="0 0.0 0" />
          <axis xyz="${joint0_axe}" />
          <xacro:joint_limit joint_type="${joint0_type}" link_length="${base_length/2}"/>
          <dynamics damping="0.0" friction="0.0"/>
        </joint>

         <!--  link 0  -->
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


    '''
        data = ''

        for i in range(self.links_number):
            data = data + self.joint_create(i + 1) + self.link_create(i + 1)

        tail = '''
        <!-- camera joint - fictive joint -->
        <joint name="camera_joint" type="revolute">
            <parent link="${prefix}link''' + str(self.links_number) + '''" />
            <child link = "camera_link" />
            <origin xyz="0.0  0.0 ${link''' + str(self.links_number) + '''_length}" rpy="0.0 0.0 0" />
            <axis xyz="0 0 1"/>
            <xacro:joint_limit joint_type="revolute" link_length="0.1"/>
            <dynamics damping="0.0" friction="0.0"/>
        </joint>
        

        <!-- Camera -->
        <link name="camera_link">
          <collision>
            <origin rpy="0 0 0" xyz="0 0 0.005"/>
            <geometry>
            <box size="0.01 0.01 0.01"/>
            </geometry>
          </collision>
        <visual>
            <geometry>
            <box size="0.01 0.01 0.01"/>
            </geometry>
        </visual>
          <xacro:cylinder_inertial radius="0.01" length="0.01" mass="0.01">
            <origin xyz="0.0 0.0 0.005" rpy="0 0 0" />
          </xacro:cylinder_inertial>
        </link>
            
        <!-- ee link -->
        <link name="${prefix}ee_link">
          <collision>
            <geometry>
              <box size="0.01 0.01 0.01"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.005"/>
          </collision>
        </link>

            <!-- ee joint -->
        <joint name="${prefix}ee_fixed_joint" type="fixed">
          <parent link="camera_link" />
          <child link = "${prefix}ee_link" />
          <origin xyz="0.0  0.0 0.01" rpy="0.0 0.0 0" />
        </joint>
        
            <xacro:arm_transmission prefix="${prefix}" />
            <xacro:arm_gazebo prefix="${prefix}" />
            </xacro:macro>
            <xacro:arm_robot prefix=""/>
        </robot>  '''

        txt = head + inertia_parameters + base_link + data + tail
        return txt

    @staticmethod
    def link_create(n):
        """link- data about specific link. it buit from :
                *inertia - inertail data of the link -Mass, moment of inertia, pose in the frame
                *collision - the collision properties of a link.
                *visual - > the visual properties of the link. This element specifies the shape of the object (box, cylinder, etc.) for visualization purposes
                *velocity_decay - exponential damping of the link's velocity"""
        linkname = 'link' + str(n)
        link = ''
        if n == 1:
            link = link + '''<!--  link 1  -->
        <link name="${prefix}link1">
          <visual>
            <origin xyz="0 0 ${link1_length / 2} " rpy="0 0 0" />
            <geometry>
                <cylinder radius="${link1_radius}" length="${link1_length}"/>
            </geometry>
          </visual>
          <collision>
             <origin xyz="0 0 ${link1_length / 2}" rpy="0 0 0" />
            <geometry>
                <cylinder radius="${link1_radius}" length="${link1_length}"/>
            </geometry>
          </collision>
          <xacro:cylinder_inertial radius="${link1_radius}" length="${link1_length}" mass="${link1_mass}">
            <origin xyz="0.0 0.0 ${link1_length / 2}" rpy="0 0 0" />
          </xacro:cylinder_inertial>
        </link>'''
        else:
            link = link + '''<!-- link ''' + str(n) + '''   -->
        <link name="${prefix}''' + linkname + '''">
          <visual>
            <origin xyz="0 0 ${''' + linkname + '''_length / 2}" rpy="0 0 0" />
            <geometry>
                <cylinder radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}"/>
            </geometry>
          </visual>
          <collision>
            <origin xyz="0 0 ${''' + linkname + '''_length / 2 }" rpy="0 0 0" />
            <geometry>
                <cylinder radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}"/>
            </geometry>
          </collision>
          <xacro:cylinder_inertial radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}" mass="${''' + linkname + '''_mass}">
            <origin xyz="0.0 0.0 ${''' + linkname + '''_length / 2 }" rpy="0 0 0" />
          </xacro:cylinder_inertial>
        </link>'''
        return link

    def calc_origin(self, n):
        # calc the origin of the link according to the previuos joint
        if self.joint_data[n - 1] == "revolute":
            if self.axis[n - 1] == '0 0 1':  # roll
                if self.rpy[n - 1] == ['0 ', '0 ', '0 ']:  # links in the same directoin
                    return "0 0 ${link" + str(n - 1) + "_length}"
                elif self.rpy[n - 1] == ['${1/2*pi} ', '0 ', '0 ']:  # links in the same directoin
                    return "0 -${link" + str(n - 1) + "_radius} ${link" + str(n - 1) + "_length}"
                elif self.rpy[n - 1] == ['0 ', '${pi/2} ', '0 ']:
                    return "0 0 ${link" + str(n) + "_radius + link" + str(n - 1) + "_length}"
                else:  # the links are perpendiculars
                    return "0 ${link" + str(n - 1) + "_radius} ${link" + str(n - 1) + "_length}"
            else:  # pitch
                if self.rpy[n - 1] == ['0 ', '0 ', '0 ']:  # around y: links are in the same directoin
                    return "0 ${link" + str(n - 1) + "_radius+link" + str(n) + "_radius} ${link" + str(
                        n - 1) + "_length}"
                elif self.rpy[n - 1] == ['0 ', '0 ', '${-pi/2} ']:  # around x: links are not in the same directoin
                    return " ${link" + str(n - 1) + "_radius+link" + str(n) + "_radius} 0 ${link" + str(
                        n - 1) + "_length}"
                else:  # round x:  the links are perpendiculars
                    return "0 0 ${link" + str(n - 1) + "_length + link" + str(n) + "_radius}"
        else:  # prismatic
            if self.rpy[n - 1] == ['0 ', '0 ', '0 ']:  # links in the same directoin
                return "0 0 ${link" + str(n - 1) + "_length}"
            else:  # the links are perpendiculars
                return "0 0 ${link" + str(n - 1) + "_length + link" + str(n) + "_radius}"

    def joint_create(self, n):
        jointname = 'joint' + str(n)

        joint = '\n<xacro:property name="' + jointname + '_type" value="' + self.joint_data[n - 1] + '"/>\n' \
                                                                                                     '<xacro:property name="' + jointname + '_axe" value="' + \
                self.axis[n - 1] + '"/>\n' \
                                   '<xacro:property name="link' + str(n) + '_length" value="' + str(
            self.links[n - 1]) + '"/>\n'

        if n == 1:
            joint = joint + '''<!--  joint 1    -->
        <joint name="${prefix}joint1" type="${joint1_type}">
          <parent link="${prefix}link0" />
          <child link="${prefix}link1" />
          <origin xyz="0.0 0.0 ${link0_length}" rpy="${pi/2} 0.0 0.0" />
          <axis xyz="${joint1_axe}"/>
          <xacro:joint_limit joint_type="${joint1_type}" link_length="${link1_length}"/>
          <dynamics damping="0.0" friction="0.0"/>
        </joint>
    '''
        else:
            orgin = self.calc_origin(n)
            rpy = self.rpy[n - 1][0] + self.rpy[n - 1][1] + self.rpy[n - 1][2]
            joint = joint + '''<!--  joint ''' + str(n) + '''   -->
        <joint name="${prefix}''' + jointname + '''" type="${''' + jointname + '''_type}">
          <parent link="${prefix}link''' + str(n - 1) + '''"/>
          <child link="${prefix}link''' + str(n) + '''" />
          <origin xyz="''' + orgin + '''" rpy="''' + rpy + '''"/>
          <axis xyz="${''' + jointname + '''_axe}"/>
          <xacro:joint_limit joint_type="${''' + jointname + '''_type}" link_length="${link''' + str(n) + '''_length}"/>
          <dynamics damping="0.0" friction="0.0"/>
        </joint>
    '''
        return joint

    @staticmethod
    def urdf_write(data, filename=str(datetime.now().minute)):
        fil = open(filename + '.urdf.xacro', 'w')
        fil.write(data)
        fil.close()

    def init_calc(self, joints, joints_axis):
        axis = []
        a = 0
        for j in joints:  # make calculations for all the joints
            axis.append(self.axis_calc(joints_axis[a]))
            a = a + 1
        return axis

    @staticmethod
    def axis_calc(axe):
        if axe == 'x':
            return '1 0 0'
        elif axe == 'y':
            return '0 1 0'
        elif axe == 'z':
            return '0 0 1'
        else:
            warning('wrong axe input.' + axe + ' entered. returning [0 0 0] ' + str(
                datetime.datetime.now()))  # will print a message to the console
            return '0 0 0'
