import abc
from typing import Any, Optional
import numpy as np

# import genesis as gs
import mujoco
import mujoco.viewer

import cv2
from pathlib import Path

from dataclasses import dataclass

@dataclass
class SimulatorArmConfig():
    path: str | None = None
    type: str = "mjcf"
    unit: str = "rad"
    pos: Optional[tuple] = None
    
height = 1200  # 渲染高度（像素）
width = 1600   # 渲染宽度（像素）

@dataclass
class SimulatorConfig():
    # arm_config: dict[str, SimulatorArmConfig] | None = None
    # urdf_path: str | None = None
    # mjcf_path: str | None = None
    xml_path: str | None = None
    # backend: str = "cpu"
    from_unit: str = "deg"
    show_viewer: bool = False

    # def __post_init__(self):
    #     if self.arm_config is not None:
    #         for _, config in self.arm_config.items():
    #             for attr in ["path", "type", "unit", "pos"]:
    #                 if getattr(config, attr) is None:
    #                     raise ValueError(
    #                         f"Specifying '{attr}' is required for the arm to be used in sim"
    #                     )
    

class Simulator:
    def __init__(
        self,
        # backend: str, 
        show_viewer: bool,
        # arm_config: dict[str, SimulatorArmConfig] | None,
        # urdf_path: dict[str, str] | str | None,
        # mjcf_path: dict[str, str] | str | None
        xml_path: str | None,
        from_unit: str,
    ):
        self.arm = None
        self.arms = None
        self.units: dict[str, str] | None = None
        self.from_unit = from_unit

        # backend_mapping = {
        #     "cpu": gs.cpu,
        #     "gpu": gs.gpu,
        #     "cuda": gs.cuda,
        #     "vulkan": gs.vulkan,
        #     "metal": gs.metal,
        #     "opengl": gs.opengl,
        # }

        # if backend not in backend_mapping:
        #     valid_backends = ", ".join(backend_mapping.keys())
        #     raise ValueError(f"Invalid backend '{backend}'. Valid options are: {valid_backends}")
    
        # gs.init(backend=backend_mapping[backend], logging_level="warn")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # self.scene = gs.Scene(
        #     vis_options = gs.options.VisOptions(
        #         shadow = False,
        #         lights = [
        #             {"type": "directional", "dir": (-1, -1, -1), "color": (1.0, 1.0, 1.0), "intensity": 1.5},
        #             {"type": "directional", "dir": (-1, 1, -1), "color": (1.0, 1.0, 1.0), "intensity": 3.0},
        #             {"type": "directional", "dir": (1, 1, -1), "color": (1.0, 1.0, 1.0), "intensity": 1.5}
        #         ]
        #     ),
        #     show_viewer=show_viewer,
        # )

        # _plane = self.scene.add_entity(
        #     gs.morphs.Plane(),
        # )

        # if arm_config is not None:
        #     for name, config in arm_config.items():
        #         if config.type == "mjcf":
        #             self.arms[name] = self.scene.add_entity(
        #                 gs.morphs.MJCF(
        #                     file = config.path,
        #                     pos = config.pos,
        #                 ),
        #             )
        #         elif config.type == "urdf":
        #             self.arms[name] = self.scene.add_entity(
        #                 gs.morphs.URDF(
        #                     file = config.path,
        #                     pos = config.pos,
        #                 ),
        #             )
        #         self.units[name] = config.unit

        # else:
        #     if urdf_path is not None:
        #         self.arm = self.scene.add_entity(
        #             gs.morphs.URDF(
        #                 file = urdf_path,
        #                 pos = (0, 0, 0),
        #                 fixed = True,
        #             ),
        #         )
        #     elif mjcf_path is not None:
        #         self.arm = self.scene.add_entity(
        #             gs.morphs.MJCF(
        #                 file = mjcf_path,
        #             ),
        #         )

        # self.cam = self.scene.add_camera(
        #     res    = (1600, 1200),
        #     pos    = (-0.5, -0.5, 0.5),
        #     lookat = (0, 0, 0.1),
        #     fov    = 60,
        #     GUI    = False,
        # )

        # self.scene.build()

        # # self.arm.control_dofs_position(
        # #     np.zeros(self.arm.n_dofs),
        # # )

        # 创建相机对象
        self.cam = mujoco.MjvCamera()
        # 设置相机默认参数
        mujoco.mjv_defaultCamera(self.cam)

        self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

    def update(self, action: dict[str, Any], prefix: str, suffix: str):
        print("action:", action)
        
        if self.arms is None:
            goal_joint = [ val for _key, val in action.items()]

            actuators_idx = [self.model.actuator(name.removeprefix(f"{prefix}").removesuffix(f"{suffix}")).id for name in action]
            # for _, actuator_name in enumerate(actuator_names):
            #     actuators_idx.append({
            #         "id": ,
            #         "name": actuator_name
            #     })

            # dofs_idx = [self.arm.get_joint(name.removeprefix(f"{prefix}").removesuffix(f"{suffix}")).dof_idx_local for name in action]
            print("actuators_idx:", actuators_idx)

            goal_joint_numpy = np.array(goal_joint, dtype=np.float32)
            print("goal_joint_numpy:", goal_joint_numpy)

            if self.from_unit == "deg":
                # 假设 goal_joint 是角度值，需要转换为弧度
                goal_joint_degrees = np.array(goal_joint, dtype=np.float32)  # 角度值
                print("原始角度值 (deg):", goal_joint_degrees)

                # TODO (Xiang Yang): rad deg switch
                # if 需要从角度转化弧度
                # 转换为弧度
                goal_joint_radians = goal_joint_degrees * (np.pi / 180.0)
                print("转换为弧度 (rad):", goal_joint_radians)
            elif self.from_unit == "rad":
                goal_joint_radians = np.array(goal_joint, dtype=np.float32)  # 角度值
                print("原始弧度值 (rad):", goal_joint_radians)

            for j, dof_id in enumerate(actuators_idx):
                if dof_id >= 0 and j < len(goal_joint_radians):
                    self.data.ctrl[dof_id] = goal_joint_radians[j]


            # self.arm.control_dofs_position(
            #     goal_joint_radians,
            #     dofs_idx,
            # )
        
        # else:
        #     for name, arm in self.arms.items():
        #         goal_joint = [ val for _key, val in action.items()]

        #         dofs_idx = [self.arm.get_joint(name.removeprefix(f"{prefix}").removeprefix(f"{name}_").removesuffix(f"{suffix}")).dof_idx_local for name in action]
        #         print("dofs_idx:", dofs_idx)

        #         goal_joint_numpy = np.array(goal_joint, dtype=np.float32)
        #         print("goal_joint_numpy:", goal_joint_numpy)

        #         if self.units[name] == "rad":
        #             goal_joint_radians = np.array(goal_joint, dtype=np.float32)
        #             print("弧度值 (deg):", goal_joint_radians)

        #             self.arm.control_dofs_position(
        #                 goal_joint_radians,
        #                 dofs_idx,
        #             )

        #         elif self.units[name] == "deg":
        #             goal_joint_degrees = np.array(goal_joint, dtype=np.float32)
        #             print("角度值 (deg):", goal_joint_degrees)
        #             goal_joint_radians = goal_joint_degrees * (np.pi / 180.0)
        #             print("弧度值 (rad):", goal_joint_radians)

        #             self.arm.control_dofs_position(
        #                 goal_joint_radians,
        #                 dofs_idx,
        #             )

        # self.scene.step()
        # rgb, _, _, _ = self.cam.render()

        mujoco.mj_step(self.model, self.data)

        self.viewer.sync()

        self.renderer.update_scene(self.data, self.cam)
        rgb = self.renderer.render()

        return rgb
    
    def stop(self):
        self.renderer.close()
        self.viewer.close()