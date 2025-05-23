# Drake quick start
This README.md will give you a brief introduction about how to use drake to do robot simulation.

## 1. URDF?
This section covers how to make the URDF of **DexNex** compatible with **Drake**.  
If you're not working on a DexNex-related project, feel free to skip it!

### 1.1 What is URDF?
The **Unified Robot Description Format (URDF)** is an XML-based format used to describe a robot's physical and visual properties. At its core, a robot is a collection of **links** (rigid bodies) and **joints** (connections between links), and URDF specifies how these elements are connected.

URDF files define:
- The **type of joints** (e.g., fixed, revolute, prismatic)
- The **shape and appearance** of links (geometry, materials, colors)
- The **structure** of the robot (parent-child relationships)

For **visualization tools** like RViz, a URDF that includes shapes and colors is usually sufficient.

However, for **physics simulation** (e.g., in Drake, MuJoCo or Isaac Sim), you'll also need:
- **Mass**
- **Moments of Inertia (MOI)**
- **Collision geometry**
- **Transmission and actuator info**

These additional properties ensure that the robot behaves realistically in simulated environments.

### 1.2 Changes Needed for the Current URDF

After cloning `avatar_master` into `your_workspace/src` and building it, **do not generate the URDF yet**.  
Please stop and read this section first.

DexNex is composed of five robots: two arms, two hands, and one neck.
As mentioned earlier, simulators require more detailed information than visualizers.  

Currently, the URDF is missing:
- The **mass** for the neck (`xarm6`)
- Proper **actuator definitions** for the entire robot

We'll address these issues first to ensure the URDF is ready for simulation with Drake.  

#### 1.2.1 restore neck mass
The current urdf doesn't have the mass of neck (xarm6), we need to make the following change to get it back.  
Open this file `/home/user/your_workspace/src/avatar_master/avatar_driver/robots/avatar.urdf.xacro` and find this line: 
```xml
<xacro:include filename="$(find avatar_driver)/robots/xarm6_minimal.urdf.xacro" />
```
replace it with this line

```xml 
<xacro:include filename="$(find xarm_description)/urdf/xarm6/xarm6.urdf.xacro" />
```
#### 1.2.2 restore the transmission (actuators) of neck
Go to the folder `/home/user/your_workspace/src/xarm_ros2/xarm_description/urdf/xarm6` and you will see a file called `xarm6.transmission.xacro`. This means that the transmission has already been written by someone (the vendor I guess). Therefore, we only need to include this file  and call the macor in `xarm6.urdf.xacro`.  
Open `xarm6.urdf.xacro` and add the following two lines at line 266 (after the definition of joint 6):
```xml
<xacro:include filename="$(find xarm_description)/urdf/xarm6/xarm6.transmission.xacro"/>
<xacro:xarm6_transmission prefix="xarm6_" reduction="1"/>
```

#### 1.2.3 restore the transmission (actuators) of arm
Open the file `/home/li-wen/avatar_ws_backup_2/src/abb_gofa_ros2/abb_gofa_support/urdf/gofa_macro.xacro`
and add the following xml code at line 246 (after definition of joint 6 and before base link):
```xml
<!-- transmission list list -->
    
    <transmission name="${prefix}joint_1_transmission">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="${prefix}J1">
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
        <joint name="${prefix}joint_1">
            <hardwareInterface>EffortJointInterface</hardwareInterface>
            <hardwareInterface>PositionJointInterface</hardwareInterface>
        </joint>
    </transmission>

    <transmission name="${prefix}joint_2_transmission">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="${prefix}J2">
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
        <joint name="${prefix}joint_2">
            <hardwareInterface>EffortJointInterface</hardwareInterface>
            <hardwareInterface>PositionJointInterface</hardwareInterface>
        </joint>
    </transmission>

    <transmission name="${prefix}joint_3_transmission">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="${prefix}J3">
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
        <joint name="${prefix}joint_3">
            <hardwareInterface>EffortJointInterface</hardwareInterface>
            <hardwareInterface>PositionJointInterface</hardwareInterface>
        </joint>
    </transmission>

    <transmission name="${prefix}joint_4_transmission">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="${prefix}J4">
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
        <joint name="${prefix}joint_4">
            <hardwareInterface>EffortJointInterface</hardwareInterface>
            <hardwareInterface>PositionJointInterface</hardwareInterface>
        </joint>
    </transmission>

    <transmission name="${prefix}joint_5_transmission">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="${prefix}J5">
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
        <joint name="${prefix}joint_5">
            <hardwareInterface>EffortJointInterface</hardwareInterface>
            <hardwareInterface>PositionJointInterface</hardwareInterface>
        </joint>
    </transmission>
    
    <transmission name="${prefix}joint_6_transmission">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="${prefix}J6">
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
        <joint name="${prefix}joint_6">
            <hardwareInterface>EffortJointInterface</hardwareInterface>
            <hardwareInterface>PositionJointInterface</hardwareInterface>
        </joint>
    </transmission>
    
    <!-- end of transmission list list -->
```

## 2. Drake

### 2.1 Parsing robots
1. **Activate your Drake environment**:
   Make sure the virtual environment where Drake is installed is active.

2. **Run the script**:
   ````bash
   python3 iiwa_parsing.py
   ````
3. **Access the simulation**:  
   After running the script, you should see a message like:
   ```` rust
   INFO:drake:Meshcat listening for connections at http://localhost:7000
   ````
   
   Open the provided link in your browser to view Drake's simulation environment. You should see a iiwa robot arm falling due to the gravity.
#### Code Explanation
In Drake, we use different systems to do different things. The following line adds two important systems, `MultibodyPlant` and `SceneGraph` into the diagrm. The former handles all the physics and the later handles rendering.

```phtyon
python plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
```

Following lines parse a iiwa robot to `MultibodyPlant` and fix the base link to world frame, without this the whole robot will free fall.
```phtyon

iiwa_url = "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf"

# Load the iiwa model
(iiwa,) = parser.AddModels(url=iiwa_url)

plant.WeldFrames(
    frame_on_parent_F=plant.world_frame(),
    frame_on_child_M=plant.GetFrameByName("iiwa_link_0", iiwa),
    X_FM=RigidTransform(RollPitchYaw(np.array([0.0, 0.0, 0.0]) * np.pi / 180), [0.5, 0.5, 0.0])
)
```

Here, I use the iiwa robot as an example. If you have your own robot's URDF, you can parse it by replacing the URL with the path to your URDF file and updating the argument of `GetFrameByName()` to match the base link of your robot.



After running the code, you will see a new file `Parsing_Diagram.png`. This is what the following lines do.

```phtyon
png_data = pydot.graph_from_dot_data(diagram.GetGraphvizString(max_depth=2))[0].create_png()

#Save the PNG to a file
with open("Parsing_Diagram.png", "wb") as f:
    f.write(png_data)

print("Parsing_Diagram.png") 
```
`Parsing_Diagram.png` gives you a clear picture of what's happening inside the simulator.
You might wonder why there are so many systems, even though we only added `MultibodyPlant` and `SceneGraph`.
That's because the following line automatically adds several default visualization systems for you:

```python
AddDefaultVisualization(builder=builder, meshcat=meshcat)
```
If you comment out this line, the diagram will become much simpler.
![iiwa_parsing diagram](iiwa_Parsing_Diagram.png)


### 2.2 Controlling Robots (Motion Control)

This section is more interesting because the robot can finally move, and we get to see how Drake is actually used in practice—at least the way *I* use it. One of Drake's core features is its **System Framework**. Instead of writing imperative Python-style logic to control a robot, you build systems (either using Drake's built-in systems or your own custom ones) and wire them together.  


In this section, we introduce three new systems:

- `InverseDynamicsController`: for computing joint torques to achieve a desired motion
- `ConstantVectorSource`: for supplying the desired joint positions and velocities to the controller
- `LogVectorOutput`: for logging data, which is useful for debugging

Only the first two are required for robot control. The third one is optional, but it's a useful system to debug.

#### Why System-Based Design is Preferable in Drake Simulations

When using **Drake** as a simulator, each **System** in the **Diagram** is updated in lockstep with the simulator’s progression. In the script, for example, the simulator steps forward by 1 ms, and during each step:

- The `ConstantVectorSource` outputs a constant vector representing the **desired joint positions**.
- This vector is sent to the `InverseDynamicsController`, which computes the corresponding **joint torques**.
- These torques are applied to the robot in the `MultibodyPlant`.
- Then, the `MultibodyPlant` advances the physics simulation by 1 ms.

This tightly integrated update mechanism is one reason why using **Drake Systems** for data processing is often preferable. If you instead extract data from one system, manually process it in a custom function, and then re-insert it into another system, you would break the continuous flow of the simulation. You’d also need to call `simulator.AdvanceTo(finish_time)` in small chunks, which complicates the simulation loop and defeats the purpose of using Drake’s declarative, system-based design.

But this is just my point of view and this is only true if you use Drake as a simulator. If you simply use Drake as a Inverse Dynamics solver, for example, you don't need to follow this fashion then.



#### Code Explanation

The following code declares two systems: `InverseDynamicsController` and `ConstantVectorSource`, and adds them to the diagram. In Drake terminology, a *diagram* is a meta-system composed of multiple interconnected subsystems.

```python
idc = InverseDynamicsController(plant, 
                                np.ones((U, 1)) * Kp,
                                np.ones((U, 1)) * Ki,
                                np.ones((U, 1)) * Kd,
                                False)
IDC = builder.AddSystem(idc)

joint_command_source = ConstantVectorSource(desired_state)
desired_state_source = builder.AddSystem(joint_command_source)
```
After we add these systems to our diagram, we need to wire them up. Remember that the first argument of `builder.Connect()` is always an **output port** and the second argument is **input port**. Because the set up of `LogVectorOutput` is simpler, the declaration and wiring just need one line of code. 

```phtyon
# set up a logging system, using a system called LogVectorOutput
# and wire it up to the data we want to save, in this case it's the joint torque
torque_logger = LogVectorOutput(IDC.get_output_port_generalized_force(), builder)

# feed joint command to controller
builder.Connect(desired_state_source.get_output_port(), IDC.get_input_port_desired_state())

# send joint torque computed by the controller to robot
builder.Connect(IDC.get_output_port_generalized_force(), plant.get_applied_generalized_force_input_port())

# get feedback from robots
builder.Connect(plant.get_state_output_port(iiwa), IDC.get_input_port_estimated_state())
```
That’s all it takes to set up a basic motion controller! Again, I recommend commenting out the `AddDefaultVisualization` line to generate a cleaner diagram and ensure that what you're doing matches what you think is happening. In next section, I will discuss on how to customize your system.
![iiwa_controller diagram](iiwa_Controller_Diagram.png)

> **Note:**  
> If your scene includes more than just the robot (e.g., a box that the robot interacts with), you'll need to set up a separate plant—usually called the *controller plant*. In that case, you'll parse only the robot into the controller plant and pass it as the first argument to the `InverseDynamicsController`.  
> For more on why this is necessary, see this helpful Stack Overflow post:  
> [URDF file parsing error in Drake – actuators not being instantiated](https://stackoverflow.com/questions/75917723/urdf-file-parsing-error-in-drake-actuators-not-being-instantiated)  
> Russ Tedrake’s response (he’s the creator of Drake) explains it in detail.


### 2.3 Advanced control
![Inverse Kinematics diagram](iiwa_IK_Diagram.png)
