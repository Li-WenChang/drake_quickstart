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

AddDefaultVisualization(builder=builder, meshcat=meshcat)
diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
diagram.SetDefaultContext(diagram_context)
#--------------------------------------------------------------------------------------------
png_data = pydot.graph_from_dot_data(diagram.GetGraphvizString(max_depth=2))[0].create_png()

#Save the PNG to a file
with open("Parsing_Diagram.png", "wb") as f:
    f.write(png_data)

print("Parsing_Diagram.png")
#-----------------------------------------------------------------------------------------------
simulator = Simulator(diagram, diagram_context)
#simulator.Initialize()
simulator.set_target_realtime_rate(1.0)


time.sleep(5)

finish_time = 10.0
simulator.AdvanceTo(finish_time)

