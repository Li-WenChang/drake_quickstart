import copy
import os
import time

import pydot
from IPython.display import SVG, display

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

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

from pydrake.systems.primitives import ConstantVectorSource
from pydrake.systems.controllers import InverseDynamicsController
from pydrake.systems.controllers import InverseDynamics
from pydrake.geometry import SceneGraph
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.all import  ModelInstanceIndex
from pydrake.systems.primitives import SparseMatrixGain

meshcat = StartMeshcat()

table_top_sdf = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="table_top">
    <link name="table_top_link">
      <visual name="visual">
        <pose>0 0 0.445 0 0 0</pose>
        <geometry>
          <box>
            <size>0.55 1.1 0.05</size>
          </box>
        </geometry>
        <material>
         <diffuse>0.9 0.8 0.7 1.0</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <pose>0 0 0.445  0 0 0</pose>
        <geometry>
          <box>
            <size>0.55 1.1 0.05</size>
          </box>
        </geometry>
      </collision>
    </link>
    <frame name="table_top_center">
      <pose relative_to="table_top_link">0 0 0.47 0 0 0</pose>
    </frame>
  </model>
</sdf>

"""
def xyz_rpy_deg(xyz, rpy_deg):
    """Shorthand for defining a pose."""
    rpy_deg = np.asarray(rpy_deg)
    return RigidTransform(RollPitchYaw(rpy_deg * np.pi / 180), xyz)

builder = DiagramBuilder()
simulation_plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
simulation_plant.set_name("plant")



# Parse the avatar into the simulation plant
dexnex_path1 = "/home/li-wen/avatar_ws_backup/learning_drake/avatar_new.urdf"
parser_sim = Parser(simulation_plant, "sim")
parser_sim.package_map().PopulateFromEnvironment('AMENT_PREFIX_PATH')
(dexnex_sim,) = parser_sim.AddModels(dexnex_path1)

# Parse the objects into the simulation plant
(table,) = parser_sim.AddModelsFromString(table_top_sdf, "sdf")


# Finalize the simulation plant
simulation_plant.Finalize()

# Create the controller plant (only the robot)
controller_plant = builder.AddSystem(MultibodyPlant(time_step=0.001))
#controller_plant.RegisterAsSourceForSceneGraph(SceneGraph())  # Optional, for visualization
controller_plant.set_name("controller plant")

# Parse the robot into the controller plant
parser_control = Parser(controller_plant, "control")
parser_control.package_map().PopulateFromEnvironment('AMENT_PREFIX_PATH')
(dexnex_control,) = parser_control.AddModels(dexnex_path1)


# Finalize the controller plant
controller_plant.Finalize()

for i in range(simulation_plant.num_model_instances()):
        model_instance = ModelInstanceIndex(i)
        model_instance_name = simulation_plant.GetModelInstanceName(model_instance)
        print(model_instance_name, "is", model_instance)
avatar_model = ModelInstanceIndex(2)
table = ModelInstanceIndex(3)
#----------------------------------------------------------------------------------------------------
# set the configuration of table
table = simulation_plant.GetBodyByName("table_top_link")
X_WorldTable = RigidTransform(RollPitchYaw(np.asarray([0, 0, 90]) * np.pi / 180), p=[1,1,2])
simulation_plant.SetDefaultFreeBodyPose(table, X_WorldTable)
#----------------------------------------------------------------------------------------------------
#setting desired acceleration
desired_state = np.zeros(controller_plant.num_positions())
#a = np.pi/sim_time**2
a = 0

desired_state[61] = -a
desired_state[62] = a


desired_state[30+1] = -a
desired_state[30+2] = a

desired_state[1] = -a
desired_state[2] = a

vector_source = ConstantVectorSource(desired_state)
desired_state_source = builder.AddSystem(vector_source)
desired_state_source.set_name("constant command state")
#----------------------------------------------------------------------------------------------------
# setting ID control
context = controller_plant.CreateDefaultContext()
#id_system = InverseDynamics(plant, mode = InverseDynamics.kGravityCompensation, context)
id_system = InverseDynamics(controller_plant, InverseDynamics.kInverseDynamics, context)
ID = builder.AddSystem(id_system)
ID.set_name("Inverse Dynamics")

builder.Connect(simulation_plant.get_state_output_port(avatar_model),
                ID.get_input_port_estimated_state())

builder.Connect(desired_state_source.get_output_port(),
                ID.get_input_port_desired_acceleration())
               
#----------------------------------------------------------------------------------------------------
# convert generalized force to actuation
# D = controller_plant.MakeActuationMatrixPseudoinverse()

# matrix_gain = builder.AddSystem(SparseMatrixGain(D))
# matrix_gain.set_name("PseudoInverse Actuation matrix")



# builder.Connect(ID.get_output_port_generalized_force(),
#                 matrix_gain.get_input_port())

# builder.Connect(matrix_gain.get_output_port(),
#                 simulation_plant.get_actuation_input_port(avatar_model))

#----------------------------------------------------------------------------------------------------
# convert generalized force to actuation
# cheating
cheating_matrix = np.zeros((72, 66))
cheating_matrix[:66, :66] = np.eye(66)

matrix_gain = builder.AddSystem(SparseMatrixGain(cheating_matrix))
matrix_gain.set_name("PseudoInverse Actuation matrix")

builder.Connect(ID.get_output_port_generalized_force(),
                matrix_gain.get_input_port())

builder.Connect(matrix_gain.get_output_port(),
                simulation_plant.get_applied_generalized_force_input_port())


#----------------------------------------------------------------------------------------------------
AddDefaultVisualization(builder=builder, meshcat=meshcat)
diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
diagram.SetDefaultContext(diagram_context)
plant_context = diagram.GetMutableSubsystemContext(simulation_plant, diagram_context)

# # following two lines make you see all state name
# states = simulation_plant.GetPositionNames()+simulation_plant.GetVelocityNames()
# print(len(states), states)
# print("Number of states: ", plant.num_multibody_states())
# print(plant.GetStateNames(True))
qv = simulation_plant.GetPositionsAndVelocities(plant_context)


qv[0] = np.pi/2
qv[30+0] = -np.pi/2

qv[1] = np.pi/2
qv[2] = -np.pi/2
qv[30+1] = np.pi/2
qv[30+2] = -np.pi/2

qv[61] = np.pi/2
qv[62] = -np.pi

simulation_plant.SetPositionsAndVelocities(plant_context, qv)
#--------------------------------------------------------------------------------------------
svg_data = pydot.graph_from_dot_data(diagram.GetGraphvizString(max_depth=2))[0].create_svg()

#Save the SVG to a file
with open("dexnex_2plant.svg", "wb") as f:
    f.write(svg_data)

print("SVG saved as output.svg")
#-----------------------------------------------------------------------------------------------
simulator = Simulator(diagram, diagram_context) # remember to add diagram_context to the argument
#simulator.Initialize()
simulator.set_target_realtime_rate(1.0)
time.sleep(5)
simulator.AdvanceTo(10.0)
print("finished")
time.sleep(50)