## KUKA ARM K210 - PICK AND PLACE PROJECT
![image1](https://s-media-cache-ak0.pinimg.com/236x/62/44/2d/62442d955224718da89238b244578f43--industrial-robots-the-machine.jpg)

The goal of this project is to develop a kinematics model (both forward and inverse) that helps the 6 degree of 
freedom robot KUKA KR210 to calculate its joints angle to approach target object, pick up the object, and then place the
object in the target destination.
(Credit: This project is the second project in Robotics Nanodegree by Udacity.)

Video of the robot arm perfoms pick and place objects within ROS with the kinematics analysis in this project:
https://youtu.be/Ct7kLFxq5xo

### Getting Starter
1. Set up your ROS Workspace.
2. Download or clone this repo into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in demo mode by changing demo value to __True__ in `/kuka_arm/launch/inverse_kinematics.launch`
5. `/kuka_arm/scripts/IK_server.py` contains codes for kinematic analysis in this README.
Leave demo value to `False` to launch the kinematic analysis.

### Kinematics Analysis
#### Homogenous Transformation Between Links

The reference frame of the robot in its original position.
![image2](https://github.com/ancabilloni/Robot-Arm-Kinematics/blob/master/misc_images/original_pos.png)

Here is the diagram with reference frames being define to follow DH method to find the parameters for the 
transformation matrices:
![image3](https://github.com/ancabilloni/Robot-Arm-Kinematics/blob/master/misc_images/Reference_frame.png)

To define the homogeneous transformation between joints in this robot arm, I applied the Denavit–Hartenberg method
with the convention as follow:
![image7](https://github.com/ancabilloni/Robot-Arm-Kinematics/blob/master/misc_images/Selection_066.png)

This table is the result of all the parameters being define according to the DH convention above:
![image4](https://github.com/ancabilloni/Robot-Arm-Kinematics/blob/master/misc_images/DH_table.png)

The parameters in the table will be substitue in to this transformation matrix to find the transformation between joints:
```
def H_transform(alpha, a, d, theta):
    A = Matrix([[            cos(theta),           -sin(theta),           0,             a],
                [ sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                 0,                 0,           0,             1]])
    return A
```

The reason that joint_4, joint_5 and joint_6 being referenced at the same location as joint_5 is because the rotation axis
of these three joints are intersect at joint_5 location. For this reason, the model satisfies the condition to solve "closed-formed" method. In this method, we can consider the intersection of the last three joints as a __wrist center__ and divide the problem
into half to solve for the kinematically decouple problems.

### Solving for joint_1, joint_2, joint_3 angles
As mentioned earlier, joint_5 is the wrist center. To solve for the first half of the problem, which are the first three joint angles,
I can apply some trigonometry calculation to find these angles if I know the coordinate of the wrist-center. The diagrams of the problem are:

![image3](https://github.com/ancabilloni/Robot-Arm-Kinematics/blob/master/misc_images/3D.png)
![image4](https://github.com/ancabilloni/Robot-Arm-Kinematics/blob/master/misc_images/diagram_23.png)

The position of the end-effector is known as this information can be achieved by in `IK_server.py`:
```
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
```
Because the base_link is in fixed position where it can only allow rotation on z axis, the relationship between the end-effector and the base_link is extrinsic rotation. That means, I can achieve the rotation matrix for the end-effector by knowing the roll, pitch, yaw angles between end-effector and base_link then apply the following the Euler matrices calculation:
```
def RotationMatrix(roll, pitch, yaw):
    R_x = Matrix([[1,         0,          0],
                  [0, cos(roll), -sin(roll)],
                  [0, sin(roll),  cos(roll)]])

    R_y = Matrix([[ cos(pitch), 0, sin(pitch)],
                  [          0, 1,          0],
                  [-sin(pitch), 0, cos(pitch)]])

    R_z = Matrix([[cos(yaw),-sin(yaw), 0],
                  [sin(yaw), cos(yaw), 0],
                  [       0,        0, 1]])

    return R_z*R_y*R_x
```
```
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            R0_EE = RotationMatrix(roll, pitch, yaw)
```
Because the coordinate between URDF in ROS and DH convention I used here are different, some correction should be applied to bring the R0_EE calculated by applying Euler matrices to DH orientation:

```
def R_correction(roll, pitch, yaw):
    R_x = Matrix([[1,         0,          0],
                  [0, cos(roll), -sin(roll)],
                  [0, sin(roll),  cos(roll)]])

    R_y = Matrix([[ cos(pitch), 0, sin(pitch)],
                  [          0, 1,          0],
                  [-sin(pitch), 0, cos(pitch)]])

    R_z = Matrix([[cos(yaw),-sin(yaw), 0],
                  [sin(yaw), cos(yaw), 0],
                  [       0,        0, 1]])

    return R_y*R_z
    
 R_corr = R_correction(0, pi/2, pi)

 R0_EE = R0_EE*R_corr
```
With known postion and orientation of end-effector, I can achieve the position of the wrist center by:
```
            wx = px - (l6 + l_ee)*R0_EE[0,2]
            wy = py - (l6 + l_ee)*R0_EE[1,2]
            wz = pz - (l6 + l_ee)*R0_EE[2,2]
```
Subsequently, I can solve for joint_1,2,3 angles:
```
            r = sqrt(wx**2 + wy**2)
            phi = atan2(wz - 0.75, r - 0.35)
            l25 = sqrt((wz -0.75)**2 + (r-0.35)**2)
            # l3 = 1.25
            l45 = sqrt(1.5**2 + 0.054**2)

            angle3_2_5 = acos((l25**2 + l3**2 - l45**2)/(2*l25*l3))
            
            angle2_3_5 = acos((l3**2 + l45**2 - l25**2)/(2*l3*l45))

            theta1 = atan2(wy, wx)
            theta1 = theta1.evalf()
            theta2 = pi/2 - phi - angle3_2_5
            theta2 = theta2.evalf()
            theta3 = pi/2 - angle2_3_5 - atan2(0.054, 1.5)
            theta3 = theta3.evalf()
```
### Solving for joint_4, joint_5 and joint_6
In this second part of the decouple problem, I can define the rotation between joint_3 and the end effector as following:
```
   R0_EE = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6*R6_EE = R0_3*R3_4*R4_5*R5_6*R6_EE
   
   R0_3.inverse()*R0_EE = R0_3.inverse()*R0_3*R3_4*R4_5*R5_6*R6_EE
   
   R0_3.inverse()*R0_EE = I*R3_4*R4_5*R5_6*R6_EE = R3_4*R4_5*R5_6*R6_EE
   
   And:
   R3_EE = R3_4*R4_5*R5_6*R6_EE
   Hence:
   R3_EE = R0_3.inverse()*R0_EE
```
With R0_EE as known by using the exinstrict rotation from roll, pitch, yaw and R0_3 can be solved by substitute joint1,2,3
into its transformation matrix. I can solve for R3_EE
```
            T0_1 = H_transform(alpha0, a0, d1, theta1)
            T1_2 = H_transform(alpha1, a1, d2, theta2)
            T2_3 = H_transform(alpha2, a2, d3, theta3)
            T3_4 = H_transform(alpha3, a3, d4, theta4)
            T4_5 = H_transform(alpha4, a4, d5, theta5)
            T5_6 = H_transform(alpha5, a5, d6, theta6)
            T6_EE = H_transform(alpha6, a6, d7, theta7)
            
            T0_3 = simplify(T0_1*T1_2*T2_3)
            R0_3 = T0_3[:3, :3]
            
            R3_EE = R0_3.inverse()*R0_EE
 ```
 I can achieve the symbolic form of R3_EE by using its transformation matrices:
 ![image5](https://github.com/ancabilloni/Robot-Arm-Kinematics/blob/master/symbolic.png)
 
 And the last three joints can be solved as following:
 
 ```
            x4 = -R3_6[0,2]
            y6 = -R3_6[1,1]
            y5 = sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2])

            theta4 = atan2(R3_6[2,2], x4)
            theta4 = theta4.evalf()

            # theta5 = acos(R3_6[1,2])
            theta5 = atan2(y5,R3_6[1,2])
            theta5 = theta5.evalf()

            theta6 = atan2(y6, R3_6[1,0])
            theta6 = theta6.evalf()
```
## Conclusion
The arm successfully calculate and pick up the object closely the the reference trajectory in Rviz. There are occassions
that the gripper would drop othe object during the movement. There could be some precion errors. My further plan for improvement
is to look into what could cause the errors and enhance the accuracy for the object pick-up.
