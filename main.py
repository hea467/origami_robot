import numpy as np
import matplotlib.pyplot as plt

import mujoco
import glfw
from threading import Lock
# import mujoco_viewer

model = mujoco.MjModel.from_xml_path('./test.xml')
data = mujoco.MjData(model)

width, height = 960, 540

glfw.init()
glfw.window_hint(glfw.VISIBLE, 1)
window = glfw.create_window(width, height, "Window", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)
framebuffer_width, framebuffer_height = glfw.get_framebuffer_size(window)

opt = mujoco.MjvOption()
cam = mujoco.MjvCamera()
scene = mujoco.MjvScene(model, maxgeom=10000)
pert = mujoco.MjvPerturb()
context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)

cam.lookat = np.array((0, 0, 0))
# cam.fovy = 42.1875
cam.distance = 0.85
cam.azimuth = -180
cam.elevation = -90

# rgb_pixels = np.zeros((height, width, 3), dtype=np.uint8)
viewport = mujoco.MjrRect(0, 0, framebuffer_width, framebuffer_height)
width, height = glfw.get_framebuffer_size(window)
viewport.width, viewport.height = width, height
gui_lock = Lock()

for i in range(10000):
    data.qpos[0] = np.pi/6
    if i==100:
        model.body_pos[4, 0] = 0.05
        print(data.qpos[0])
        print(data.qpos[1])
        print(data.qpos[3])
    
    mujoco.mj_step(model, data)

    with gui_lock:
        mujoco.mjv_updateScene(model, data, opt, pert, cam, mujoco.mjtCatBit.mjCAT_ALL.value, scene)
        mujoco.mjr_render(viewport, scene, context)
        glfw.swap_buffers(window)
    glfw.poll_events()