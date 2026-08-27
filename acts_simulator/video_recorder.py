"""
`VideoRecorder`: captures MuJoCo simulation frames and saves them to an MP4 via imageio.
"""

import imageio
import mujoco

class VideoRecorder:
    def __init__(self, model, fps=15, width=640, height=480, distance=6.0, elevation=-20, azimuth=45):
        self.fps = fps
        self.framerate_dt = 1.0 / fps
        self.next_frame_time = 0.0
        self.frames = []
        
        self.renderer = mujoco.Renderer(model, height=height, width=width)
        
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.camera.trackbodyid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "payload")
        
        self.camera.distance = distance
        self.camera.elevation = elevation
        self.camera.azimuth = azimuth

    def capture_frame(self, data):
        if data.time >= self.next_frame_time:
            # Pass custom camera into update_scene
            self.renderer.update_scene(data, camera=self.camera)
            pixels = self.renderer.render()
            self.frames.append(pixels)
            self.next_frame_time += self.framerate_dt

    def save(self, output_path="mujoco_simulation.mp4"):
        if not self.frames:
            return
        output_path = f"collected_data/videos/{output_path}"
        imageio.mimsave(output_path, self.frames, fps=self.fps)
        print(f"Video saved successfully ({len(self.frames)} frames) to {output_path}")