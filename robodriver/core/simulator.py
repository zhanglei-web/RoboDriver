import mujoco
import mujoco.viewer
import logging_mp
import numpy as np

from typing import Any
from dataclasses import dataclass


logger = logging_mp.get_logger(__name__)


@dataclass
class SimulatorConfig():
    xml_path: str | None = None
    from_unit: str = "deg"
    render_height: int = 1200
    render_width: int = 1600
    timestep: float = 0.01
    show_viewer: bool = False
    log_data: bool = False

    def __post_init__(self):
        if self.from_unit != "deg" and self.from_unit != "rad":
            raise ValueError(
                f"from_unit only support \'deg\' or \'rad\' in sim"
            )
    
class Simulator:
    def __init__(self, config: SimulatorConfig):
        self.config = config

        self.model = mujoco.MjModel.from_xml_path(self.config.xml_path)
        self.data = mujoco.MjData(self.model)
        self.cam = mujoco.MjvCamera()
        self.renderer = mujoco.Renderer(self.model, height=self.config.render_height, width=self.config.render_width)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data) if self.config.show_viewer == True else None

        self.model.opt.timestep = config.timestep
        mujoco.mjv_defaultCamera(self.cam)

    def update(self, action: dict[str, Any], prefix: str, suffix: str):
        goal_joint = [ val for _key, val in action.items()]

        actuators_idx = [self.model.actuator(name.removeprefix(f"{prefix}").removesuffix(f"{suffix}")).id for name in action]
        
        goal_joint_numpy = np.array(goal_joint, dtype=np.float32)
        
        if self.config.from_unit == "deg":
            goal_joint_degrees = np.array(goal_joint, dtype=np.float32)  # 角度值
            print("原始角度值 (deg):", goal_joint_degrees)

            goal_joint_radians = goal_joint_degrees * (np.pi / 180.0)
            print("转换为弧度 (rad):", goal_joint_radians)

        elif self.config.from_unit == "rad":
            goal_joint_radians = np.array(goal_joint, dtype=np.float32)  # 角度值
            print("原始弧度值 (rad):", goal_joint_radians)

        for j, dof_id in enumerate(actuators_idx):
            if dof_id >= 0 and j < len(goal_joint_radians):
                self.data.ctrl[dof_id] = goal_joint_radians[j]

        mujoco.mj_step(self.model, self.data)

        self.viewer.sync()

        self.renderer.update_scene(self.data, self.cam)
        rgb = self.renderer.render()

        if self.config.log_data == True:
            logger.info(f"action: {action}"), 
            logger.info(f"actuators_idx: {actuators_idx}")
            logger.info(f"goal_joint_numpy: {goal_joint_numpy}")

        return rgb
    
    def stop(self):
        self.renderer.close()
        if self.viewer is not None:
            self.viewer.close()