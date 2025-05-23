import time


import matplotlib.pyplot as plt
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


meshcat = StartMeshcat()

def xyz_rpy_deg(xyz, rpy_deg):
    """Shorthand for defining a pose."""
    rpy_deg = np.asarray(rpy_deg)
    return RigidTransform(RollPitchYaw(rpy_deg * np.pi / 180), xyz)

builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.005)



# Create a single Parser instance for the plant
parser = Parser(plant)

dexnex_path = "//home/li-wen/Desktop/drake_quickstart/dexnex_description/avatar_drake.urdf"

parser.package_map().PopulateFromEnvironment('AMENT_PREFIX_PATH')
(dexnex,) = parser.AddModels(dexnex_path)



plant.Finalize()

AddDefaultVisualization(builder=builder, meshcat=meshcat)

diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
diagram.SetDefaultContext(diagram_context)

plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)


qv = plant.GetPositionsAndVelocities(plant_context)

qv[0] = np.pi/2
qv[30+0] = -np.pi/2

#qv[61] = np.pi/2
qv[62] = -np.pi/2

plant.SetPositionsAndVelocities(plant_context, qv)

print("######################################################")
act_names = plant.GetActuatorNames()
print("Number of actuators:", len(act_names))
print("Actuation input:", act_names)



simulator = Simulator(diagram, diagram_context) # remember to add diagram_context to the argument
simulator.Initialize()
simulator.set_target_realtime_rate(1.0)
time.sleep(8)
sim_time = 8
simulator.AdvanceTo(sim_time)
print("finished")
