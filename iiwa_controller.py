import copy
import os
import time

import pydot
from IPython.display import SVG, display

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt

from pydrake.geometry import (
    ClippingRange,
    ColorRenderCamera,
    DepthRange,
    DepthRenderCamera,
    MakeRenderEngineVtk,
    RenderCameraCore,
    RenderEngineVtkParams,
    RenderLabel,
    Role,
    StartMeshcat,
)
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import BodyIndex
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import (
    CameraInfo,
    RgbdSensor,
)
from pydrake.visualization import (
    AddDefaultVisualization,
    ColorizeDepthImage,
    ColorizeLabelImage,
)
from pydrake.systems.controllers import InverseDynamicsController
from pydrake.systems.primitives import ConstantVectorSource, LogVectorOutput



meshcat = StartMeshcat()

builder = DiagramBuilder()

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step = 0.001)

# Create a single Parser instance for the plant
parser = Parser(plant)



iiwa_url = "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf"

# Load the iiwa model
(iiwa,) = parser.AddModels(url=iiwa_url)

plant.WeldFrames(
    frame_on_parent_F=plant.world_frame(),
    frame_on_child_M=plant.GetFrameByName("iiwa_link_0", iiwa),
    X_FM=RigidTransform(RollPitchYaw(np.array([0.0, 0.0, 0.0]) * np.pi / 180), [0.5, 0.5, 0.0])
)

plant.Finalize()
#--------------------------------------------------------------------------------------------
# set up a position controller, using a system called InverseDynamicsController
U = plant.num_actuators() # U = 7 for iiwa because it has 7 joints
Kp = 0.6
Ki = 0
Kd = 0.8
idc = InverseDynamicsController(plant, 
                                np.ones((U, 1))*Kp,
                                np.ones((U, 1))*Ki,
                                np.ones((U, 1))*Kd, False)
IDC = builder.AddSystem(idc)

# set up a desired joint positions, using a system called ConstantVectorSource
# In drake, state often means position and velocity,
# So I use U*2 because 7 joints have 7 joint positions and 7 velocities
desired_state = np.zeros(U*2)
desired_state[0] = np.pi/2
desired_state[1] = np.pi/2
desired_state[2] = np.pi/2
desired_state[3] = np.pi/2
desired_state[4] = np.pi/2
desired_state[5] = np.pi/2
desired_state[6] = np.pi/2

joint_command_source = ConstantVectorSource(desired_state)
desired_state_source = builder.AddSystem(joint_command_source)

# set up a logging system, using a system called LogVectorOutput
# and wire it up to the data we want to save, let's say I want to save the joint torque
torque_logger = LogVectorOutput(IDC.get_output_port_generalized_force(), builder)

# wire things up:
# feed joint command to controller
builder.Connect(desired_state_source.get_output_port(), IDC.get_input_port_desired_state())

# send joint torque computed by the controller to robot
builder.Connect(IDC.get_output_port_generalized_force(), plant.get_applied_generalized_force_input_port())

# get feedback from robots
builder.Connect(plant.get_state_output_port(iiwa), IDC.get_input_port_estimated_state())

#--------------------------------------------------------------------------------------------
AddDefaultVisualization(builder=builder, meshcat=meshcat)
diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
diagram.SetDefaultContext(diagram_context)
#--------------------------------------------------------------------------------------------
# this part is not that important
# just a demonstration of how to set robot's initial positions and velocities
# by default, they are all zeros
plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)

qv = plant.GetPositionsAndVelocities(plant_context)
qv[1] = -np.pi/2
qv[3] = -np.pi/2
qv[5] = -np.pi/2
plant.SetPositionsAndVelocities(plant_context, qv)
#--------------------------------------------------------------------------------------------
png_data = pydot.graph_from_dot_data(diagram.GetGraphvizString(max_depth=2))[0].create_png()

#Save the PNG to a file
with open("Controller_Diagram.png", "wb") as f:
    f.write(png_data)

print("Controller_Diagram.png")
#-----------------------------------------------------------------------------------------------
simulator = Simulator(diagram, diagram_context)
#simulator.Initialize()
simulator.set_target_realtime_rate(1.0)


time.sleep(5)

finish_time = 10.0
simulator.AdvanceTo(finish_time)


# plot the result we save in the logger
torque_data = torque_logger.FindLog(simulator.get_context())
t = torque_data.sample_times()
print("torque logger's data type:", type(torque_data.data()), type(t))
print("state logger shape:", torque_data.data().shape, t.shape)
plt.plot(t, torque_data.data()[0, :], label='torque joint 0')
plt.plot(t, torque_data.data()[1, :], label='torque joint 1')
plt.plot(t, torque_data.data()[2, :], label='torque joint 2')
plt.plot(t, torque_data.data()[3, :], label='torque joint 3')
plt.plot(t, torque_data.data()[4, :], label='torque joint 4')
plt.plot(t, torque_data.data()[5, :], label='torque joint 5')
plt.plot(t, torque_data.data()[6, :], label='torque joint 6')

plt.xlabel('time')
plt.ylabel('torque')
plt.legend()
plt.show()

