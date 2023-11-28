import numpy as np
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.all import (
    AbstractValue,
    LeafSystem,
)
import time
from collections import namedtuple
from functools import partial

import numpy as np
from pydrake.geometry import (
    Cylinder,
    Rgba,
    Sphere,
)
from pydrake.solvers import BoundingBoxConstraint
from pydrake.systems.framework import EventStatus


class PrintPose(LeafSystem):
    def __init__(self, body_index):
        LeafSystem.__init__(self)
        self._body_index = body_index
        self.DeclareAbstractInputPort(
            "body_poses", AbstractValue.Make([RigidTransform()])
        )
        self.DeclareForcedPublishEvent(self.Publish)

    def Publish(self, context):
        pose = self.get_input_port().Eval(context)[self._body_index]
        print(pose)
        print(
            "gripper position (m): "
            + np.array2string(
                pose.translation(),
                formatter={"float": lambda x: "{:3.2f}".format(x)},
            )
        )
        print(
            "gripper roll-pitch-yaw (rad):"
            + np.array2string(
                RollPitchYaw(pose.rotation()).vector(),
                formatter={"float": lambda x: "{:3.2f}".format(x)},
            )
        )


class MeshcatPoseSliders(LeafSystem):
    """
    Provides a set of ipywidget sliders (to be used in a Jupyter notebook) with
    one slider for each of roll, pitch, yaw, x, y, and z.  This can be used,
    for instance, as an interface to teleoperate the end-effector of a robot.

    .. pydrake_system::

        name: PoseSliders
        input_ports:
        - pose (optional)
        output_ports:
        - pose

    The optional `pose` input port is used ONLY at initialization; it can be
    used to set the initial pose e.g. from the current pose of a MultibodyPlant
    frame.
    """

    # TODO(russt): Use namedtuple defaults parameter once we are Python >= 3.7.
    Visible = namedtuple("Visible", ("roll", "pitch", "yaw", "x", "y", "z"))
    Visible.__new__.__defaults__ = (True, True, True, True, True, True)
    MinRange = namedtuple("MinRange", ("roll", "pitch", "yaw", "x", "y", "z"))
    MinRange.__new__.__defaults__ = (-np.pi, -np.pi, -np.pi, -1.0, -1.0, -1.0)
    MaxRange = namedtuple("MaxRange", ("roll", "pitch", "yaw", "x", "y", "z"))
    MaxRange.__new__.__defaults__ = (np.pi, np.pi, np.pi, 1.0, 1.0, 1.0)
    Value = namedtuple("Value", ("roll", "pitch", "yaw", "x", "y", "z"))
    Value.__new__.__defaults__ = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    DecrementKey = namedtuple("DecrementKey", ("roll", "pitch", "yaw", "x", "y", "z"))
    DecrementKey.__new__.__defaults__ = (
        "KeyQ",
        "KeyW",
        "KeyA",
        "KeyJ",
        "KeyI",
        "KeyO",
    )
    IncrementKey = namedtuple("IncrementKey", ("roll", "pitch", "yaw", "x", "y", "z"))
    IncrementKey.__new__.__defaults__ = (
        "KeyE",
        "KeyS",
        "KeyD",
        "KeyL",
        "KeyK",
        "KeyU",
    )

    def __init__(
        self,
        meshcat,
        visible=Visible(),
        min_range=MinRange(),
        max_range=MaxRange(),
        value=Value(),
        decrement_keycode=DecrementKey(),
        increment_keycode=IncrementKey(),
        body_index=None,
    ):
        """
        Args:
            meshcat: A Meshcat instance.
            visible: An object with boolean elements for 'roll', 'pitch',
                     'yaw', 'x', 'y', 'z'; the intention is for this to be the
                     PoseSliders.Visible() namedtuple.  Defaults to all true.
            min_range, max_range, value: Objects with float values for 'roll',
                      'pitch', 'yaw', 'x', 'y', 'z'; the intention is for the
                      caller to use the PoseSliders.MinRange, MaxRange, and
                      Value namedtuples.  See those tuples for default values.
            body_index: if the body_poses input port is connected, then this
                        index determine which pose is used to set the initial
                        slider positions during the Initialization event.
        """
        LeafSystem.__init__(self)
        port = self.DeclareAbstractOutputPort(
            "pose",
            lambda: AbstractValue.Make(RigidTransform()),
            self.DoCalcOutput,
        )

        self.DeclareAbstractInputPort(
            "body_poses", AbstractValue.Make([RigidTransform()])
        )
        self.DeclareInitializationDiscreteUpdateEvent(self.Initialize)

        # The widgets themselves have undeclared state.  For now, we accept it,
        # and simply disable caching on the output port.
        # TODO(russt): consider implementing the more elaborate methods seen
        # in, e.g., LcmMessageSubscriber.
        port.disable_caching_by_default()

        self._meshcat = meshcat
        self._visible = visible
        self._value = list(value)
        self._body_index = body_index

        print("Keyboard Controls:")
        for i in range(6):
            if visible[i]:
                meshcat.AddSlider(
                    min=min_range[i],
                    max=max_range[i],
                    value=value[i],
                    step=0.02,
                    name=value._fields[i],
                    decrement_keycode=decrement_keycode[i],
                    increment_keycode=increment_keycode[i],
                )
                print(
                    f"{value._fields[i]} : {decrement_keycode[i]} / {increment_keycode[i]}"  # noqa
                )

    def __del__(self):
        for s in ["roll", "pitch", "yaw", "x", "y", "z"]:
            if visible[s]:
                self._meshcat.DeleteSlider(s)

    def SetPose(self, pose):
        """
        Sets the current value of the sliders.

        Args:
            pose: Any viable argument for the RigidTransform
                  constructor.
        """
        tf = RigidTransform(pose)
        self.SetRpy(RollPitchYaw(tf.rotation()))
        self.SetXyz(tf.translation())

    def SetRpy(self, rpy):
        """
        Sets the current value of the sliders for roll, pitch, and yaw.

        Args:
            rpy: An instance of drake.math.RollPitchYaw
        """
        self._value[0] = rpy.roll_angle()
        self._value[1] = rpy.pitch_angle()
        self._value[2] = rpy.yaw_angle()
        for i in range(3):
            if self._visible[i]:
                self._meshcat.SetSliderValue(self._visible._fields[i], self._value[i])

    def SetXyz(self, xyz):
        """
        Sets the current value of the sliders for x, y, and z.

        Args:
            xyz: A 3 element iterable object with x, y, z.
        """
        self._value[3:] = xyz
        for i in range(3, 6):
            if self._visible[i]:
                self._meshcat.SetSliderValue(self._visible._fields[i], self._value[i])

    def _update_values(self):
        changed = False
        for i in range(6):
            if self._visible[i]:
                old_value = self._value[i]
                self._value[i] = self._meshcat.GetSliderValue(self._visible._fields[i])
                changed = changed or self._value[i] != old_value
        return changed

    def _get_transform(self):
        return RigidTransform(
            RollPitchYaw(self._value[0], self._value[1], self._value[2]),
            self._value[3:],
        )

    def DoCalcOutput(self, context, output):
        """Constructs the output values from the sliders."""
        self._update_values()
        output.set_value(self._get_transform())

    def Initialize(self, context, discrete_state):
        if self.get_input_port().HasValue(context):
            if self._body_index is None:
                raise RuntimeError(
                    "If the `body_poses` input port is connected, then you "
                    "must also pass a `body_index` to the constructor."
                )
            self.SetPose(self.get_input_port().Eval(context)[self._body_index])
            return EventStatus.Succeeded()
        return EventStatus.DidNothing()

    def Run(self, publishing_system, root_context, callback):
        # Calls callback(root_context, pose), then
        # publishing_system.ForcedPublish() each time the sliders change value.

        publishing_context = publishing_system.GetMyContextFromRoot(root_context)

        print("Press the 'Stop PoseSliders' button in Meshcat to continue.")
        self._meshcat.AddButton("Stop PoseSliders", "Escape")
        while self._meshcat.GetButtonClicks("Stop PoseSliders") < 1:
            if self._update_values():
                callback(root_context, self._get_transform())
                publishing_system.ForcedPublish(publishing_context)
            time.sleep(0.01)

        self._meshcat.DeleteButton("Stop PoseSliders")


def PublishPositionTrajectory(
    trajectory, root_context, plant, visualizer, time_step=1.0 / 33.0
):
    """
    Args:
        trajectory: A Trajectory instance.
    """
    plant_context = plant.GetMyContextFromRoot(root_context)
    visualizer_context = visualizer.GetMyContextFromRoot(root_context)

    visualizer.StartRecording(False)

    for t in np.append(
        np.arange(trajectory.start_time(), trajectory.end_time(), time_step),
        trajectory.end_time(),
    ):
        root_context.SetTime(t)
        plant.SetPositions(plant_context, trajectory.value(t))
        visualizer.ForcedPublish(visualizer_context)

    visualizer.StopRecording()
    visualizer.PublishRecording()

def get_trajectory(trajectory, time_step=1.0 / 33.):
    t = np.arange(trajectory.start_time(), trajectory.end_time(), step=time_step)
    values = np.zeros((t.shape[0],6,1))

    for i,tt in enumerate(t):
        values[i] = trajectory.value(tt)[:6]
    return values, time_step