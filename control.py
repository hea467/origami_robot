import numpy as np
import matplotlib.pyplot as plt
import mujoco
import glfw
import mujoco_viewer
import time

model = mujoco.MjModel.from_xml_path('./test.xml')
data = mujoco.MjData(model)

width, height = 500, 500

glfw.init()
glfw.window_hint(glfw.VISIBLE, 1)
window = glfw.create_window(width, height, "window", None, None)
if not window:
    print("Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n",file=sys.stderr)
    glfw.terminate()
glfw.make_context_current(window)
glfw.swap_interval(1)
framebuffer_width, framebuffer_height = glfw.get_framebuffer_size(window)

opt = mujoco.MjvOption()
cam = mujoco.MjvCamera()
cam.lookat = np.array((0.13125, 0.1407285, 1.5))
# cam.fovy = 42.1875
cam.distance = 0.85
cam.azimuth = 0
cam.elevation = 90
scene = mujoco.MjvScene(model, maxgeom=10000)
pert = mujoco.MjvPerturb()


context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)
viewport = mujoco.MjrRect(0, 0, framebuffer_width, framebuffer_height)
rgb_pixels = np.zeros((height, width, 3), dtype=np.uint8)
mujoco.mjr_render(viewport, scene, context)

for i in range(1000):
    mujoco.mjv_updateScene(model, data, opt, pert, cam, mujoco.mjtCatBit.mjCAT_ALL.value, scene)
    mujoco.mj_step(model, data)