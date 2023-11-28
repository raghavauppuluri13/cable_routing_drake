# Import some basic libraries and functions for this tutorial.
from pydrake.geometry import (
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    StartMeshcat,
)
import numpy as np

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    RollPitchYaw,
    AngleAxis,
    Role,
    JointSliders,
    Parser,
    Simulator,
    InverseKinematics,
    RotationMatrix,
    Solve,
    PositionConstraint,
    OrientationConstraint,
    OrientationCost,
    BsplineTrajectory,
    KinematicTrajectoryOptimization,
    MinimumDistanceConstraint,
    Parser,
    PositionConstraint,
    RigidTransform,
    Role,
    Rgba,
    Sphere,
)


from IPython.display import clear_output

from pathlib import Path

from models import robot_directives, environment_directives
from triad import AddFrameTriadIllustration

from systems import PrintPose, MeshcatPoseSliders, PublishPositionTrajectory, get_trajectory


def initialize_simulation(diagram):
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    return simulator


def load_assembly_task():
    meshcat = StartMeshcat()
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.01)
    parser = Parser(plant, scene_graph)
    parser.SetAutoRenaming(True)
    package_map = parser.package_map()

    # get module path
    model_path = Path(__file__).absolute().parent.parent / "models"

    package_map.Add("ur5e_description", str(model_path / "ur5e_description"))
    package_map.Add("wedge_gripper", str(model_path / "wedge_gripper"))
    package_map.Add("nist_board", str(model_path / "nist_board"))

    parser.AddModelsFromString(robot_directives, ".dmd.yaml")
    parser.AddModelsFromString(environment_directives, ".dmd.yaml")

    plant.Finalize()

    wedge = plant.GetModelInstanceByName("wedge")
    eef = plant.GetBodyByName("tcp", wedge)

    print_pose = builder.AddSystem(PrintPose(eef.index()))
    builder.Connect(plant.get_body_poses_output_port(), print_pose.get_input_port())

    # Visualize frames
    AddFrameTriadIllustration(
        scene_graph=scene_graph, body=plant.GetBodyByName("waypoint_1")
    )
    AddFrameTriadIllustration(
        scene_graph=scene_graph, body=plant.GetBodyByName("waypoint_2")
    )
    AddFrameTriadIllustration(
        scene_graph=scene_graph, body=plant.GetBodyByName("wedge_root")
    )
    AddFrameTriadIllustration(scene_graph=scene_graph, body=plant.GetBodyByName("end"))
    AddFrameTriadIllustration(
        scene_graph=scene_graph, body=plant.GetBodyByName("start")
    )
    AddFrameTriadIllustration(scene_graph=scene_graph, body=plant.GetBodyByName("tcp"))
    visualizer = MeshcatVisualizer.AddToBuilder(
        builder,
        scene_graph,
        meshcat,
        params=MeshcatVisualizerParams(show_hydroelastic=True),
    )

    collision_visualizer = MeshcatVisualizer.AddToBuilder(
        builder,
        scene_graph,
        meshcat,
        MeshcatVisualizerParams(
            prefix="collision", role=Role.kProximity, visible_by_default=False
        ),
    )

    sliders = builder.AddSystem(JointSliders(meshcat, plant))

    diagram = builder.Build()
    diagram.set_name("nist_board_assembly")
    # sliders.Run(diagram, None)
    meshcat.DeleteAddedControls()

    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    q0 = plant.GetPositions(plant_context)
    gripper_frame = plant.GetFrameByName("tcp")

    wp_1 = plant.GetBodyByName("waypoint_1")
    wp_1_pose = plant.EvalBodyPoseInWorld(plant_context, wp_1)
    wp_2 = plant.GetBodyByName("waypoint_2")
    wp_2_pose = plant.EvalBodyPoseInWorld(plant_context, wp_2)

    start = plant.GetBodyByName("start")
    start_pose = plant.EvalBodyPoseInWorld(plant_context, start)
    end = plant.GetBodyByName("end")
    end_pose = plant.EvalBodyPoseInWorld(plant_context, end)

    X_WStart = RigidTransform([0.8, 0, 0.65])
    meshcat.SetObject("start", Sphere(0.02), rgba=Rgba(0.9, 0.1, 0.1, 1))
    meshcat.SetTransform("start", X_WStart)
    X_WGoal = RigidTransform([0.8, 0, 0.4])
    meshcat.SetObject("goal", Sphere(0.02), rgba=Rgba(0.1, 0.9, 0.1, 1))
    meshcat.SetTransform("goal", X_WGoal)

    def ik():
        def my_callback(context, pose):
            ik = InverseKinematics(plant, plant_context)
            ik.AddPositionConstraint(
                gripper_frame,
                [0, 0, 0],
                plant.world_frame(),
                pose.translation(),
                pose.translation(),
            )
            ik.AddOrientationConstraint(
                gripper_frame,
                RotationMatrix(),
                plant.world_frame(),
                pose.rotation(),
                0.0,
            )
            prog = ik.get_mutable_prog()
            q = ik.q()

            ik.AddMinimumDistanceConstraint(0.001, 0.005)
            prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
            prog.SetInitialGuess(q, q0)
            result = Solve(ik.prog())
            if result.is_success():
                print("IK success")
            else:
                print("IK failure")
            clear_output(wait=True)

        sliders = MeshcatPoseSliders(meshcat)
        sliders.SetPose(plant.EvalBodyPoseInWorld(plant_context, eef))
        sliders.Run(visualizer, context, my_callback)

    def traj_opt():
        num_q = plant.num_positions()
        q0 = plant.GetPositions(plant_context)

        trajopt = KinematicTrajectoryOptimization(num_q, 10)

        prog = trajopt.get_mutable_prog()

        q_guess = np.tile(q0.reshape((num_q, 1)), (1, trajopt.num_control_points()))
        q_guess[0, :] = np.linspace(0, np.pi / 2, trajopt.num_control_points())
        print(trajopt.num_control_points())
        q_guess[0, :] = np.array([0,-1.54,1.2,-1.27,-1.54,0,0,0,0,0])
        
        path_guess = BsplineTrajectory(trajopt.basis(), q_guess)
        #trajopt.SetInitialGuess(path_guess)

        trajopt.AddDurationCost(1.0)
        trajopt.AddPathLengthCost(10.0)
        trajopt.AddPositionBounds(
            plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits()
        )
        lower_vel_limits = plant.GetVelocityLowerLimits()
        upper_vel_limits = plant.GetVelocityUpperLimits()
        lower_vel_limits[-2:] = lower_vel_limits[0]
        upper_vel_limits[-2:] = upper_vel_limits[0]
        trajopt.AddVelocityBounds(lower_vel_limits, upper_vel_limits)

        trajopt.AddDurationConstraint(1, 5)

        waypoints = [start_pose, wp_2_pose, wp_1_pose, end_pose]
        path_inds = np.linspace(0,1,num=4)
        for i, wp in enumerate(waypoints):
            # start constraint
            c = PositionConstraint(
                plant,
                plant.world_frame(),
                wp.translation(),
                wp.translation() + np.array([0,0,0.02]),
                gripper_frame,
                [0, 0.0, 0.0],
                plant_context,
            )
            trajopt.AddPathPositionConstraint(c, path_inds[i])
            o = OrientationConstraint(
                plant,
                gripper_frame,
                RotationMatrix(),
                plant.world_frame(),
                RollPitchYaw(-np.pi/2,0,-np.pi/2).ToRotationMatrix(),
                0.0,
                plant_context
            )
            trajopt.AddPathPositionConstraint(o, path_inds[i])
        
        prog.AddQuadraticErrorCost(np.eye(num_q), q0, trajopt.control_points()[:, 0])
        prog.AddQuadraticErrorCost(np.eye(num_q), q0, trajopt.control_points()[:, 1])
        prog.AddQuadraticErrorCost(np.eye(num_q), q0, trajopt.control_points()[:, 2])
        prog.AddQuadraticErrorCost(np.eye(num_q), q0, trajopt.control_points()[:, 3])
        prog.AddQuadraticErrorCost(np.eye(num_q), q0, trajopt.control_points()[:, 4])
        prog.AddQuadraticErrorCost(np.eye(num_q), q0, trajopt.control_points()[:, 5])


        # start and end with zero velocity
        trajopt.AddPathVelocityConstraint(np.zeros((num_q, 1)), np.zeros((num_q, 1)), 0)
        trajopt.AddPathVelocityConstraint(np.zeros((num_q, 1)), np.zeros((num_q, 1)), 0.33)
        trajopt.AddPathVelocityConstraint(np.zeros((num_q, 1)), np.zeros((num_q, 1)), 0.66)
        trajopt.AddPathVelocityConstraint(np.zeros((num_q, 1)), np.zeros((num_q, 1)), 1)

        # collision constraints
        collision_constraint = MinimumDistanceConstraint(
            plant, 0.001, plant_context, None, 0.01
        )
        evaluate_at_s = np.linspace(0, 1, 100)
        for s in evaluate_at_s:
            trajopt.AddPathPositionConstraint(collision_constraint, s)

        result = Solve(prog)
        if not result.is_success():
            print("Trajectory optimization failed")
            print(result.get_solver_id().name())
            print(result.GetInfeasibleConstraintNames(prog))
            print(result.get_solver_details())
            print(result.get_solution_result())
        print(get_trajectory(trajopt.ReconstructTrajectory(result))) 

        PublishPositionTrajectory(
            trajopt.ReconstructTrajectory(result), context, plant, visualizer
        )

        def my_callback(context, pose):
            pass

        sliders = MeshcatPoseSliders(meshcat)
        sliders.SetPose(plant.EvalBodyPoseInWorld(plant_context, eef))
        sliders.Run(visualizer, context, my_callback)

    traj_opt()
    # ik()


if __name__ == "__main__":
    load_assembly_task()
