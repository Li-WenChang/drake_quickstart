import copy
import os
import time

import pydot


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
from pydrake.systems.primitives import ConstantVectorSource, LogVectorOutput, StateInterpolatorWithDiscreteDerivative
from pydrake.multibody.inverse_kinematics import (
    DifferentialInverseKinematicsIntegrator,
    DifferentialInverseKinematicsParameters)
from pydrake.systems.framework import LeafSystem
from pydrake.common.value import AbstractValue



class RigidTransformSource(LeafSystem):
    def __init__(self):
        super().__init__()

        # Declare a fixed abstract output port with a hardcoded pose
        self.DeclareAbstractOutputPort(
            "X_AE",
            lambda: AbstractValue.Make(RigidTransform()),
            self.CalcOutput
        )

    def CalcOutput(self, context, output):
        time = simulator.get_context().get_time()
        r = 0.3 # radius = 0.3 m
        omega = np.pi*2/5 # period = 5 sec
        x = r*np.cos(omega*time)
        z = np.abs(r*np.sin(omega*time))
        
        pose = RigidTransform(RollPitchYaw(0.0, np.pi/2, 0.0), [x, 0.0, 1.0])
        output.set_value(pose)

meshcat = StartMeshcat()

builder = DiagramBuilder()

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step = 0.005)

# Create a single Parser instance for the plant
parser = Parser(plant)



iiwa_url = "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf"

# Load the iiwa model
(iiwa,) = parser.AddModels(url=iiwa_url)

plant.WeldFrames(
    frame_on_parent_F=plant.world_frame(),
    frame_on_child_M=plant.GetFrameByName("iiwa_link_0", iiwa),
    X_FM=RigidTransform(RollPitchYaw(np.array([0.0, 0.0, 0.0]) * np.pi / 180), [0.0, 0.0, 0.0])
)

plant.Finalize()
#--------------------------------------------------------------------------------------------
U = plant.num_actuators() # U = 7 for iiwa because it has 7 joints

pose_source = builder.AddSystem(RigidTransformSource())

params = DifferentialInverseKinematicsParameters(
    num_positions=plant.num_positions(),
    num_velocities=plant.num_velocities()
)

params.set_time_step(0.01)

params.set_joint_position_limits(
    (plant.GetPositionLowerLimits(),
    plant.GetPositionUpperLimits())
)
params.set_joint_velocity_limits((-1.0 * np.ones(plant.num_velocities()),
                                 1.0 * np.ones(plant.num_velocities())))


# Choose end-effector frame
end_effector_frame = plant.GetFrameByName("iiwa_link_7", iiwa)

world_frame = plant.GetFrameByName("iiwa_link_0", iiwa)


# Add the Diff IK Integrator system
ik_solver = builder.AddSystem(
    DifferentialInverseKinematicsIntegrator(
        plant,
        world_frame,
        end_effector_frame,
        0.01,
        params)
    )

builder.Connect(pose_source.get_output_port(), ik_solver.get_input_port(0))
joint_command_logger = LogVectorOutput(ik_solver.get_output_port(), builder)

Interpolator = builder.AddSystem(
                StateInterpolatorWithDiscreteDerivative(
                    U,
                    0.005,
                    suppress_initial_transient=True,
                )
            )


builder.Connect(ik_solver.get_output_port(), Interpolator.get_input_port(),)




# set up a position controller, using a system called InverseDynamicsController

Kp = 9.0
Ki = 0
Kd = 0.8
idc = InverseDynamicsController(plant, 
                                np.ones((U, 1))*Kp,
                                np.ones((U, 1))*Ki,
                                np.ones((U, 1))*Kd, False)
IDC = builder.AddSystem(idc)


# send joint torque computed by the controller to robot
builder.Connect(IDC.get_output_port_generalized_force(), plant.get_applied_generalized_force_input_port())

# get feedback from robots
builder.Connect(plant.get_state_output_port(iiwa), IDC.get_input_port_estimated_state())


builder.Connect(Interpolator.get_output_port(), IDC.get_input_port_desired_state())





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
# qv[1] = -np.pi/2
# qv[3] = -np.pi/2
# qv[5] = -np.pi/2
plant.SetPositionsAndVelocities(plant_context, qv)

ik_context = diagram.GetMutableSubsystemContext(ik_solver, diagram_context)
ik_solver.SetPositions(ik_context, qv[:7])
#--------------------------------------------------------------------------------------------
png_data = pydot.graph_from_dot_data(diagram.GetGraphvizString(max_depth=2))[0].create_png()

#Save the PNG to a file
with open("Inverse Kinematics_Diagram.png", "wb") as f:
    f.write(png_data)

print("Inverse Kinematics_Diagram.png")
#-----------------------------------------------------------------------------------------------
simulator = Simulator(diagram, diagram_context)
#simulator.Initialize()
simulator.set_target_realtime_rate(1.0)


time.sleep(5)

finish_time = 20.0
simulator.AdvanceTo(finish_time)


# plot the result we save in the logger
command_data = joint_command_logger.FindLog(simulator.get_context())
t = command_data.sample_times()
print("command logger's data type:", type(command_data.data()), type(t))
print("state logger shape:", command_data.data().shape, t.shape)
plt.plot(t, command_data.data()[0, :], label='command joint 0')
plt.plot(t, command_data.data()[1, :], label='command joint 1')
plt.plot(t, command_data.data()[2, :], label='command joint 2')
plt.plot(t, command_data.data()[3, :], label='command joint 3')
plt.plot(t, command_data.data()[4, :], label='command joint 4')
plt.plot(t, command_data.data()[5, :], label='command joint 5')
plt.plot(t, command_data.data()[6, :], label='command joint 6')

plt.xlabel('time')
plt.ylabel('command')
plt.legend()
plt.show()

