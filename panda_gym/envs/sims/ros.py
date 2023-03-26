from panda_gym.envs.core import Sim
from contextlib import contextmanager
from typing import Any, Dict, Iterator, Optional

import numpy as np

from panda_ros.moveit.move_group_interface import MoveGroupInterface
from panda_ros.utils import logger
from panda_ros.utils import geometry_tools

from geometry_msgs.msg import Quaternion


class Ros(Sim):
    """Class to send robot commands to ROS.

    Note: Must be initialized inside of a ROS node. geometry_tools must be initialized
    Note: Not used means that this is not used by panda.py, core.py, or reach.py

    Args:
        render (bool, optional): Enable rendering. Defaults to False.
        n_substeps (int, optional): Number of sim substep when step() is called. Defaults to 20.
        background_color (np.ndarray, optional): The background color as (red, green, blue).
            Defaults to np.array([223, 54, 45]).
    """

    def __init__(self, execution_speed=1, sim=True):
        self.panda = MoveGroupInterface(execution_speed=execution_speed,
                                        sim=sim)
        self.ready_joints = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
        self.ready_position = [0.30702, 0, 0.59028]
        self.ready_orientation = [0.92394, -0.38254, 0, 0]
        self.upper_joint_limits = [2.86, 1.73, 2.86, -0.1, 2.86, 3.72, 2.86, 0, 0, 0.035, 0.035]
        self.lower_joint_limits = [-2.86, -1.73, -2.86, -3.04, -2.86, 0.02, -2.86, 0, 0, -0.035, -0.035]
        self.type = "ros"

    def dt(self):
        """Timestep."""
        # NOT USED
        raise NotImplementedError

    def step(self) -> None:
        """Step the simulation."""
        # Movement is already done stepwise through other methods
        pass

    def close(self) -> None:
        """Close the simulation."""
        logger.log_text("ROS sim closed")

    def render(
        self,
        mode: str = "human",
        width: int = 720,
        height: int = 480,
        target_position: np.ndarray = np.zeros(3),
        distance: float = 1.4,
        yaw: float = 45,
        pitch: float = -30,
        roll: float = 0,
    ) -> Optional[np.ndarray]:
        # Rendering is done through RViz
        pass

    def get_base_position(self, body: str) -> np.ndarray:
        """Get the position of the body.

        Note: Position is relative to ready position

        Args:
            body (str): Body unique name.

        Returns:
            np.ndarray: The position, as (x, y, z).
        """
        # NOT USED
        raise NotImplementedError

    def get_base_orientation(self, body: str) -> np.ndarray:
        """Get the orientation of the body.

        Args:
            body (str): Body unique name.

        Returns:
            np.ndarray: The orientation, as quaternion (x, y, z, w).
        """
        # NOT USED
        raise NotImplementedError

    def get_base_rotation(self, body: str, type: str = "euler") -> np.ndarray:
        """Get the rotation of the body.

        Args:
            body (str): Body unique name.
            type (str): Type of angle, either "euler" or "quaternion"

        Returns:
            np.ndarray: The rotation.
        """
        # NOT USED
        raise NotImplementedError

    def get_base_velocity(self, body: str) -> np.ndarray:
        """Get the velocity of the body.

        Args:
            body (str): Body unique name.

        Returns:
            np.ndarray: The velocity, as (vx, vy, vz).
        """
        # NOT USED
        raise NotImplementedError

    def get_base_angular_velocity(self, body: str) -> np.ndarray:
        """Get the angular velocity of the body.

        Args:
            body (str): Body unique name.

        Returns:
            np.ndarray: The angular velocity, as (wx, wy, wz).
        """
        # NOT USED
        raise NotImplementedError

    def get_link_position(self, body: str, link: int) -> np.ndarray:
        """Get the position of the link of the body.

        Args:
            body (str): Body unique name.
            link (int): Link index in the body.

        Returns:
            np.ndarray: The position, as (x, y, z).
        """
        # NOTE: This is relative to the ready position. Not sure if this is correct

        # NOTE: tf2 transforms seem to be broken rn
        # pose_stamped = geometry_tools.find_pose_stamped_from_link(link_num=link)
        # x = pose_stamped.pose.position.x - self.ready_position[0]
        # y = pose_stamped.pose.position.y - self.ready_position[1]
        # z = pose_stamped.pose.position.z - self.ready_position[2]

        if link != 11:
            raise NotImplementedError

        pose = self.panda.get_pose()

        return (pose.position.x, pose.position.y, pose.position.z)

    def get_link_orientation(self, body: str, link: int) -> np.ndarray:
        """Get the orientation of the link of the body.

        Args:
            body (str): Body unique name.
            link (int): Link index in the body.

        Returns:
            np.ndarray: The rotation, as (rx, ry, rz).
        """
        # NOT USED
        raise NotImplementedError

    def get_link_velocity(self, body: str, link: int) -> np.ndarray:
        """Get the velocity of the link of the body.

        Args:
            body (str): Body unique name.
            link (int): Link index in the body.

        Returns:
            np.ndarray: The velocity, as (vx, vy, vz).
        """
        logger.log_text("Get link velocity returns 0")
        return (0, 0, 0)

    def get_link_angular_velocity(self, body: str, link: int) -> np.ndarray:
        """Get the angular velocity of the link of the body.

        Args:
            body (str): Body unique name.
            link (int): Link index in the body.

        Returns:
            np.ndarray: The angular velocity, as (wx, wy, wz).
        """
        # NOT USED
        raise NotImplementedError

    def get_joint_angle(self, body: str, joint: int) -> float:
        """Get the angle of the joint of the body.

        Args:
            body (str): Body unique name.
            joint (int): Joint index in the body

        Returns:
            float: The angle.
        """
        # TODO: add info for joints 9 and 10 (fingers)
        joints = self.panda.get_arm_joint_position()
        return joints[joint]

    def get_joint_velocity(self, body: str, joint: int) -> float:
        """Get the velocity of the joint of the body.

        Args:
            body (str): Body unique name.
            joint (int): Joint index in the body

        Returns:
            float: The velocity.
        """
        logger.log_text(text="get_joint_velocity returns zero", log_level=2)
        return 0.0

    def set_base_pose(self, body: str, position: np.ndarray,
                      orientation: np.ndarray) -> None:
        """Set the position of the body.

        Args:
            body (str): Body unique name.
            position (np.ndarray): The position, as (x, y, z).
            orientation (np.ndarray): The target orientation as quaternion (x, y, z, w).
        """
        # TODO: Not sure if this means move?
        x = position[0] + self.ready_position[0]
        y = position[1] + self.ready_position[1]
        z = position[2] + self.ready_position[2]
        roll, pitch, yaw = 0, 0, 0
        if len(orientation) == 3:
            roll = orientation[0]
            pitch = orientation[1]
            yaw = orientation[2]
        else:
            orientation1 = Quaternion()
            orientation1.x = orientation[3]
            orientation1.y = orientation[2]
            orientation1.z = orientation[1]
            orientation1.w = orientation[0]
            roll, pitch, yaw = geometry_tools.quaternion_to_rpy(
                orientation=orientation1)
        self.panda.go_to_pose(x, y, z, roll, pitch, yaw)

    def set_joint_angles(self, body: str, joints: np.ndarray,
                         angles: np.ndarray) -> None:
        """Set the angles of the joints of the body.

        Args:
            body (str): Body unique name.
            joints (np.ndarray): List of joint indices, as a list of ints.
            angles (np.ndarray): List of target angles, as a list of floats.
        """
        # TODO: Same Q
        self.control_joints(body=body,
                            joints=joints,
                            target_angles=angles,
                            forces=None)

    def set_joint_angle(self, body: str, joint: int, angle: float) -> None:
        """Set the angle of the joint of the body.

        Args:
            body (str): Body unique name.
            joint (int): Joint index in the body.
            angle (float): Target angle.
        """
        # NOT USED
        raise NotImplementedError

    def control_joints(self, body: str, joints: np.ndarray,
                       target_angles: np.ndarray, forces: np.ndarray) -> None:
        """Control the joints motor.

        Args:
            body (str): Body unique name.
            joints (np.ndarray): List of joint indices, as a list of ints.
            target_angles (np.ndarray): List of target angles, as a list of floats.
            forces (np.ndarray): Forces to apply, as a list of floats.
        """
        # TODO: test
        joint_goal = [None for _ in range(7)]
        hand_joint_goal = [None for _ in range(2)]
        for (index, joint) in enumerate(joints):
            joint_angle = target_angles[index]
            if joint_angle > self.upper_joint_limits[joint]:
                joint_angle = self.upper_joint_limits[joint]
            elif joint_angle < self.lower_joint_limits[joint]:
                joint_angle = self.lower_joint_limits[joint]

            if (joint < 7):
                joint_goal[joint] = joint_angle
            if (joint > 8 and joint < 11):
                hand_joint_goal[joint - 9] = joint_angle
        self.panda.go_to_arm_joint_goal(joint_goal=joint_goal, override_confirm=True)
        self.panda.go_to_hand_joint_goal(joint_goal=hand_joint_goal)

    def inverse_kinematics(self, body: str, link: int, position: np.ndarray,
                           orientation: np.ndarray) -> np.ndarray:
        """Compute the inverse kinematics and return the new joint state.

        Args:
            body (str): Body unique name.
            link (int): Link index in the body.
            position (np.ndarray): Desired position of the end-effector, as (x, y, z).
            orientation (np.ndarray): Desired orientation of the end-effector as quaternion (x, y, z, w).

        Returns:
            np.ndarray: The new joint state.
        """
        # TODO: ig we have to plan movement and then query the goal position
        # alternatively just implement the basic jacobian inverse ik
        # hopefully this is not called because we are using joints instead of ee
        raise NotImplementedError

    def place_visualizer(self, target_position: np.ndarray, distance: float,
                         yaw: float, pitch: float) -> None:
        """Orient the camera used for rendering.

        Args:
            target (np.ndarray): Target position, as (x, y, z).
            distance (float): Distance from the target position.
            yaw (float): Yaw.
            pitch (float): Pitch.
        """
        logger.log_text("Placing visualizer does nothing")

    @contextmanager
    def no_rendering(self) -> Iterator[None]:
        """Disable rendering within this context."""
        logger.log_text("Disabling rendering does nothing")

    def loadURDF(self, body_name: str, **kwargs: Any) -> None:
        """Load URDF file.

        Args:
            body_name (str): The name of the body. Must be unique in the sim.
        """
        logger.log_text("Loading URDF does nothing")

    def create_box(
        self,
        body_name: str,
        half_extents: np.ndarray,
        mass: float,
        position: np.ndarray,
        rgba_color: Optional[np.ndarray] = np.ones(4),
        specular_color: np.ndarray = np.zeros(3),
        ghost: bool = False,
        lateral_friction: Optional[float] = None,
        spinning_friction: Optional[float] = None,
        texture: Optional[str] = None,
    ) -> None:
        """Create a box.

        Args:
            body_name (str): The name of the body. Must be unique in the sim.
            half_extents (np.ndarray): Half size of the box in meters, as (x, y, z).
            mass (float): The mass in kg.
            position (np.ndarray): The position, as (x, y, z).
            rgba_color (np.ndarray, optional): Body color, as (r, g, b, a). Defaults as [0, 0, 0, 0]
            specular_color (np.ndarray, optional): Specular color, as (r, g, b). Defaults to [0, 0, 0].
            ghost (bool, optional): Whether the body can collide. Defaults to False.
            lateral_friction (float or None, optional): Lateral friction. If None, use the default pybullet
                value. Defaults to None.
            spinning_friction (float or None, optional): Spinning friction. If None, use the default pybullet
                value. Defaults to None.
            texture (str or None, optional): Texture file name. Defaults to None.
        """
        # NOT USED
        raise NotImplementedError

    def create_cylinder(
        self,
        body_name: str,
        radius: float,
        height: float,
        mass: float,
        position: np.ndarray,
        rgba_color: Optional[np.ndarray] = np.zeros(4),
        specular_color: np.ndarray = np.zeros(3),
        ghost: bool = False,
        lateral_friction: Optional[float] = None,
        spinning_friction: Optional[float] = None,
    ) -> None:
        """Create a cylinder.

        Args:
            body_name (str): The name of the body. Must be unique in the sim.
            radius (float): The radius in meter.
            height (float): The height in meter.
            mass (float): The mass in kg.
            position (np.ndarray): The position, as (x, y, z).
            rgba_color (np.ndarray, optional): Body color, as (r, g, b, a). Defaults as [0, 0, 0, 0]
            specular_color (np.ndarray, optional): Specular color, as (r, g, b). Defaults to [0, 0, 0].
            ghost (bool, optional): Whether the body can collide. Defaults to False.
            lateral_friction (float or None, optional): Lateral friction. If None, use the default pybullet
                value. Defaults to None.
            spinning_friction (float or None, optional): Spinning friction. If None, use the default pybullet
                value. Defaults to None.
        """
        # NOT USED
        raise NotImplementedError

    def create_sphere(
        self,
        body_name: str,
        radius: float,
        mass: float,
        position: np.ndarray,
        rgba_color: Optional[np.ndarray] = np.zeros(4),
        specular_color: np.ndarray = np.zeros(3),
        ghost: bool = False,
        lateral_friction: Optional[float] = None,
        spinning_friction: Optional[float] = None,
    ) -> None:
        """Create a sphere.

        Args:
            body_name (str): The name of the body. Must be unique in the sim.
            radius (float): The radius in meter.
            mass (float): The mass in kg.
            position (np.ndarray): The position, as (x, y, z).
            rgba_color (np.ndarray, optional): Body color, as (r, g, b, a). Defaults as [0, 0, 0, 0]
            specular_color (np.ndarray, optional): Specular color, as (r, g, b). Defaults to [0, 0, 0].
            ghost (bool, optional): Whether the body can collide. Defaults to False.
            lateral_friction (float or None, optional): Lateral friction. If None, use the default pybullet
                value. Defaults to None.
            spinning_friction (float or None, optional): Spinning friction. If None, use the default pybullet
                value. Defaults to None.
        """
        logger.log_text("Create sphere does nothing")

    def _create_geometry(
        self,
        body_name: str,
        geom_type: int,
        mass: float = 0.0,
        position: np.ndarray = np.zeros(3),
        ghost: bool = False,
        lateral_friction: Optional[float] = None,
        spinning_friction: Optional[float] = None,
        visual_kwargs: Dict[str, Any] = {},
        collision_kwargs: Dict[str, Any] = {},
    ) -> None:
        """Create a geometry.

        Args:
            body_name (str): The name of the body. Must be unique in the sim.
            geom_type (int): The geometry type. See self.physics_client.GEOM_<shape>.
            mass (float, optional): The mass in kg. Defaults to 0.
            position (np.ndarray, optional): The position, as (x, y, z). Defaults to [0, 0, 0].
            ghost (bool, optional): Whether the body can collide. Defaults to False.
            lateral_friction (float or None, optional): Lateral friction. If None, use the default pybullet
                value. Defaults to None.
            spinning_friction (float or None, optional): Spinning friction. If None, use the default pybullet
                value. Defaults to None.
            visual_kwargs (dict, optional): Visual kwargs. Defaults to {}.
            collision_kwargs (dict, optional): Collision kwargs. Defaults to {}.
        """
        logger.log_text("Create geometry does nothing")

    def create_plane(self, z_offset: float) -> None:
        """Create a plane. (Actually, it is a thin box.)

        Args:
            z_offset (float): Offset of the plane.
        """
        logger.log_text("Create plane does nothing")

    def create_table(
        self,
        length: float,
        width: float,
        height: float,
        x_offset: float = 0.0,
        lateral_friction: Optional[float] = None,
        spinning_friction: Optional[float] = None,
    ) -> None:
        """Create a fixed table. Top is z=0, centered in y.

        Args:
            length (float): The length of the table (x direction).
            width (float): The width of the table (y direction)
            height (float): The height of the table.
            x_offset (float, optional): The offet in the x direction.
            lateral_friction (float or None, optional): Lateral friction. If None, use the default pybullet
                value. Defaults to None.
            spinning_friction (float or None, optional): Spinning friction. If None, use the default pybullet
                value. Defaults to None.
        """
        logger.log_text("Create table does nothing")

    def set_lateral_friction(self, body: str, link: int,
                             lateral_friction: float) -> None:
        """Set the lateral friction of a link.

        Args:
            body (str): Body unique name.
            link (int): Link index in the body.
            lateral_friction (float): Lateral friction.
        """
        logger.log_text("Set lateral friction does nothing")

    def set_spinning_friction(self, body: str, link: int,
                              spinning_friction: float) -> None:
        """Set the spinning friction of a link.

        Args:
            body (str): Body unique name.
            link (int): Link index in the body.
            spinning_friction (float): Spinning friction.
        """
        logger.log_text("Set spinning friction does nothing")