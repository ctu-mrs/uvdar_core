from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_default(value):
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, (int, float)):
        return str(value)
    return value


def argument(name, default, description):
    return DeclareLaunchArgument(
        name,
        default_value=launch_default(default),
        description=description,
    )


def typed_parameter(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    arguments = [
        argument(
            "namespace",
            EnvironmentVariable("UAV_NAME", default_value="uav1"),
            "ROS namespace",
        ),
        argument("use_sim_time", False, "Use ROS simulation time"),
        argument(
            "image_topic",
            "/camera/image_raw",
            "Input sensor_msgs/Image topic",
        ),
        argument(
            "output_calibration_file",
            PathJoinSubstitution([
                EnvironmentVariable("HOME", default_value="/tmp"),
                ".ros",
                "uvdar",
                "camera_calibration.yaml",
            ]),
            "Final YAML path consumed by the uvdar_core calib_file setting",
        ),
        argument(
            "calibration_model",
            "ocamcalib",
            "ocamcalib, pinhole, fisheye_equidistant, "
            "fisheye_equisolid, fisheye_stereographic, or "
            "fisheye_orthographic",
        ),
        argument(
            "pattern_type",
            "checkerboard",
            "checkerboard or led_grid using FIMD candidates",
        ),
        argument("pattern_rows", 6, "Number of ordered pattern rows"),
        argument("pattern_columns", 8, "Number of ordered pattern columns"),
        argument(
            "pattern_spacing",
            0.04,
            "Physical point spacing in metres or another consistent unit",
        ),
        argument(
            "required_pattern_frames",
            20,
            "Diverse complete views collected before calibration",
        ),
        argument(
            "minimum_valid_views",
            10,
            "Minimum views retained after outlier rejection",
        ),
        argument(
            "minimum_frame_interval_sec",
            0.35,
            "Minimum time between accepted views",
        ),
        argument(
            "minimum_frame_diversity",
            0.07,
            "Minimum normalized pose and coverage descriptor distance",
        ),
        argument(
            "maximum_detection_candidates",
            200,
            "Reject LED frames whose candidate count exceeds this limit",
        ),
        argument("fimd_threshold", 120, "FIMD center intensity threshold"),
        argument(
            "fimd_threshold_diff",
            60,
            "FIMD center-to-boundary contrast",
        ),
        argument(
            "fimd_max_markers",
            300,
            "FIMD candidate storage limit",
        ),
        argument("fimd_radius_small", 3, "First FIMD isolation radius"),
        argument("fimd_radius_large", 5, "Second FIMD isolation radius"),
        argument(
            "hull_maximum_concave_angle",
            0.7853981633974483,
            "Maximum inward turn accepted by the candidate hull",
        ),
        argument(
            "hull_similar_angle",
            0.3490658503988659,
            "Angular tie tolerance used by the candidate hull walk",
        ),
        argument(
            "maximum_optimization_iterations",
            100,
            "Initial analytic LM trial limit",
        ),
        argument(
            "outlier_refinement_iterations",
            50,
            "Analytic LM trial limit after view rejection",
        ),
        argument(
            "initial_lm_damping",
            1.0e-4,
            "Initial diagonal-scaled LM damping",
        ),
        argument(
            "huber_delta_px",
            3.0,
            "Exact robust-residual transition in pixels",
        ),
        argument(
            "view_outlier_factor",
            2.5,
            "Per-view rejection threshold relative to median RMS",
        ),
        argument(
            "maximum_final_rms_px",
            3.0,
            "Maximum RMS accepted for YAML output",
        ),
        argument(
            "ocam_inverse_polynomial_order",
            9,
            "OCam world-to-camera polynomial order",
        ),
        argument(
            "ocam_direct_polynomial_order",
            4,
            "OCam camera-to-world polynomial order",
        ),
        argument(
            "optimization_step_tolerance",
            1.0e-9,
            "LM local-step convergence tolerance",
        ),
        argument(
            "optimization_gradient_tolerance",
            1.0e-8,
            "LM infinity-norm gradient tolerance",
        ),
        argument(
            "optimization_relative_cost_tolerance",
            1.0e-10,
            "LM relative-cost convergence tolerance",
        ),
        argument(
            "visualization_topic",
            "calibrator/visualization",
            "Fixed-rate informational image topic",
        ),
        argument(
            "status_topic",
            "calibrator/status",
            "Latched textual progress topic",
        ),
        argument(
            "visualization_fps",
            5.0,
            "Informational visualization publication rate",
        ),
        argument(
            "completion_display_sec",
            2.0,
            "Final visualization duration before shutdown",
        ),
        argument(
            "terminate_on_failure",
            True,
            "Terminate automatically after a terminal failure",
        ),
    ]

    parameters = {
        "use_sim_time": typed_parameter("use_sim_time", bool),
        "image_topic": LaunchConfiguration("image_topic"),
        "output_calibration_file": LaunchConfiguration(
            "output_calibration_file"
        ),
        "calibration_model": LaunchConfiguration("calibration_model"),
        "pattern_type": LaunchConfiguration("pattern_type"),
        "pattern_rows": typed_parameter("pattern_rows", int),
        "pattern_columns": typed_parameter("pattern_columns", int),
        "pattern_spacing": typed_parameter("pattern_spacing", float),
        "required_pattern_frames": typed_parameter(
            "required_pattern_frames", int
        ),
        "minimum_valid_views": typed_parameter("minimum_valid_views", int),
        "minimum_frame_interval_sec": typed_parameter(
            "minimum_frame_interval_sec", float
        ),
        "minimum_frame_diversity": typed_parameter(
            "minimum_frame_diversity", float
        ),
        "maximum_detection_candidates": typed_parameter(
            "maximum_detection_candidates", int
        ),
        "fimd_threshold": typed_parameter("fimd_threshold", int),
        "fimd_threshold_diff": typed_parameter("fimd_threshold_diff", int),
        "fimd_max_markers": typed_parameter("fimd_max_markers", int),
        "fimd_radii": [
            [LaunchConfiguration("fimd_radius_small")],
            [LaunchConfiguration("fimd_radius_large")],
        ],
        "hull_maximum_concave_angle": typed_parameter(
            "hull_maximum_concave_angle", float
        ),
        "hull_similar_angle": typed_parameter("hull_similar_angle", float),
        "maximum_optimization_iterations": typed_parameter(
            "maximum_optimization_iterations", int
        ),
        "outlier_refinement_iterations": typed_parameter(
            "outlier_refinement_iterations", int
        ),
        "initial_lm_damping": typed_parameter("initial_lm_damping", float),
        "huber_delta_px": typed_parameter("huber_delta_px", float),
        "view_outlier_factor": typed_parameter("view_outlier_factor", float),
        "maximum_final_rms_px": typed_parameter(
            "maximum_final_rms_px", float
        ),
        "ocam_inverse_polynomial_order": typed_parameter(
            "ocam_inverse_polynomial_order", int
        ),
        "ocam_direct_polynomial_order": typed_parameter(
            "ocam_direct_polynomial_order", int
        ),
        "optimization_step_tolerance": typed_parameter(
            "optimization_step_tolerance", float
        ),
        "optimization_gradient_tolerance": typed_parameter(
            "optimization_gradient_tolerance", float
        ),
        "optimization_relative_cost_tolerance": typed_parameter(
            "optimization_relative_cost_tolerance", float
        ),
        "visualization_topic": LaunchConfiguration("visualization_topic"),
        "status_topic": LaunchConfiguration("status_topic"),
        "visualization_fps": typed_parameter("visualization_fps", float),
        "completion_display_sec": typed_parameter(
            "completion_display_sec", float
        ),
        "terminate_on_failure": typed_parameter("terminate_on_failure", bool),
    }

    return LaunchDescription(arguments + [
        Node(
            package="uvdar_core",
            executable="calibrator_node",
            name="calibrator",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            emulate_tty=True,
            parameters=[parameters],
        )
    ])
