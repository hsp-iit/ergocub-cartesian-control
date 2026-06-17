#!/usr/bin/env python3

"""Send test vectors for Module::checkAndReadNewInputs().

The module reads these packets from ctrl_local_port_name, which is
/mc-ergocub-cartesian-bimanual/cmd:i in the default configuration.
"""

import argparse
import math
import sys
import time


DEFAULT_SERVER_PORT = "/mc-ergocub-cartesian-bimanual/cmd:i"
DEFAULT_CLIENT_PORT = "/test-check-and-read-inputs/cmd:o"
IDENTITY_ROTATION = [1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0]
IDENTITY_QUATERNION_WXYZ = [1.0, 0.0, 0.0, 0.0]


def import_yarp():
    try:
        import yarp
    except ImportError:
        print("Cannot import yarp. Source your YARP/robot environment first.", file=sys.stderr)
        sys.exit(1)
    return yarp


def parse_csv_floats(text, expected_sizes, label):
    try:
        values = [float(item.strip()) for item in text.split(",") if item.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{label} must contain only numbers") from exc

    if len(values) not in expected_sizes:
        sizes = " or ".join(str(size) for size in expected_sizes)
        raise argparse.ArgumentTypeError(f"{label} must contain {sizes} comma-separated values")

    return values


def parse_pose(text):
    return parse_csv_floats(text, (3, 7, 12), "pose")


def parse_spatial_vector(text):
    return parse_csv_floats(text, (6,), "spatial vector")


def parse_arms(text):
    requested = {item.strip().lower() for item in text.split(",") if item.strip()}
    valid = {"right", "left"}
    unknown = requested - valid
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown arm(s): {', '.join(sorted(unknown))}")
    if not requested:
        raise argparse.ArgumentTypeError("at least one arm is required")

    # Keep the same ordering used by checkAndReadNewInputs().
    return [arm for arm in ("right", "left") if arm in requested]


def set_yarp_vector_item(vector, index, value):
    if hasattr(vector, "set"):
        vector.set(index, float(value))
    else:
        vector[index] = float(value)


def fill_yarp_vector(vector, values):
    if hasattr(vector, "clear"):
        vector.clear()
    vector.resize(len(values))
    for index, value in enumerate(values):
        set_yarp_vector_item(vector, index, value)


def write_vector(port, values):
    vector = port.prepare()
    fill_yarp_vector(vector, values)
    if hasattr(port, "writeStrict"):
        port.writeStrict()
    else:
        port.write()


def normalize_quaternion(quaternion_wxyz):
    norm = math.sqrt(sum(value * value for value in quaternion_wxyz))
    if norm == 0.0:
        raise ValueError("quaternion norm must be greater than zero")
    return [value / norm for value in quaternion_wxyz]


def quaternion_to_rotation_matrix(quaternion_wxyz):
    w, x, y, z = normalize_quaternion(quaternion_wxyz)

    return [
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y - z * w),
        2.0 * (x * z + y * w),
        2.0 * (x * y + z * w),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z - x * w),
        2.0 * (x * z - y * w),
        2.0 * (y * z + x * w),
        1.0 - 2.0 * (x * x + y * y),
    ]


def rotation_matrix_to_quaternion(matrix):
    r11, r12, r13, r21, r22, r23, r31, r32, r33 = matrix
    trace = r11 + r22 + r33

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (r32 - r23) / scale
        y = (r13 - r31) / scale
        z = (r21 - r12) / scale
    elif r11 > r22 and r11 > r33:
        scale = math.sqrt(1.0 + r11 - r22 - r33) * 2.0
        w = (r32 - r23) / scale
        x = 0.25 * scale
        y = (r12 + r21) / scale
        z = (r13 + r31) / scale
    elif r22 > r33:
        scale = math.sqrt(1.0 + r22 - r11 - r33) * 2.0
        w = (r13 - r31) / scale
        x = (r12 + r21) / scale
        y = 0.25 * scale
        z = (r23 + r32) / scale
    else:
        scale = math.sqrt(1.0 + r33 - r11 - r22) * 2.0
        w = (r21 - r12) / scale
        x = (r13 + r31) / scale
        y = (r23 + r32) / scale
        z = 0.25 * scale

    return normalize_quaternion([w, x, y, z])


def pose_quaternion_wxyz(pose, quaternion_order):
    if len(pose) == 3:
        return IDENTITY_QUATERNION_WXYZ[:]
    if len(pose) != 7:
        raise ValueError("quaternion pose conversion requires x,y,z plus four quaternion values")

    quaternion = pose[3:7]
    if quaternion_order == "xyzw":
        x, y, z, w = quaternion
        return [w, x, y, z]
    return quaternion


def pose_as_matrix(pose, quaternion_order):
    if len(pose) == 3:
        return pose + IDENTITY_ROTATION
    if len(pose) == 12:
        return pose
    return pose[:3] + quaternion_to_rotation_matrix(pose_quaternion_wxyz(pose, quaternion_order))


def pose_as_quaternion(pose, quaternion_order):
    if len(pose) == 3:
        return pose + IDENTITY_QUATERNION_WXYZ
    if len(pose) == 7:
        return pose[:3] + pose_quaternion_wxyz(pose, quaternion_order)
    return pose[:3] + rotation_matrix_to_quaternion(pose[3:12])


def choose_pose_representation(args, poses):
    if args.pose_representation != "auto":
        return args.pose_representation

    selected_sizes = {len(poses[arm]) for arm in args.arms}
    if 7 in selected_sizes and 12 not in selected_sizes:
        return "quaternion"

    return "matrix"


def build_packet(args):
    poses = {
        "right": args.right_pose,
        "left": args.left_pose,
    }
    velocities = {
        "right": args.right_velocity,
        "left": args.left_velocity,
    }
    accelerations = {
        "right": args.right_acceleration,
        "left": args.left_acceleration,
    }
    pose_representation = choose_pose_representation(args, poses)
    pose_size = 7 if pose_representation == "quaternion" else 12

    if args.mode == "invalid":
        return pose_representation, [float(index) for index in range(max(1, pose_size * len(args.arms) - 1))]

    packet = []
    for arm in args.arms:
        if pose_representation == "quaternion":
            packet.extend(pose_as_quaternion(poses[arm], args.quaternion_order))
        else:
            packet.extend(pose_as_matrix(poses[arm], args.quaternion_order))

    if args.mode in ("pose-velocity", "pose-velocity-acceleration"):
        for arm in args.arms:
            packet.extend(velocities[arm])

    if args.mode == "pose-velocity-acceleration":
        for arm in args.arms:
            packet.extend(accelerations[arm])

    return pose_representation, packet


def make_parser():
    parser = argparse.ArgumentParser(
        description="Send a yarp.sig.Vector packet to exercise checkAndReadNewInputs()."
    )
    parser.add_argument("--server-port", default=DEFAULT_SERVER_PORT)
    parser.add_argument("--client-port", default=DEFAULT_CLIENT_PORT)
    parser.add_argument(
        "--arms",
        type=parse_arms,
        default=parse_arms("right,left"),
        help="Enabled end effectors as right,left, right, or left. Packet order is always right then left.",
    )
    parser.add_argument(
        "--mode",
        choices=("pose", "pose-velocity", "pose-velocity-acceleration", "invalid"),
        default="pose",
        help="Vector format to send. invalid sends a wrong-sized packet.",
    )
    parser.add_argument(
        "--pose-representation",
        choices=("auto", "matrix", "quaternion"),
        default="auto",
        help="Pose wire format. auto uses quaternion when a selected pose is x,y,z,q* and none is x,y,z,R.",
    )
    parser.add_argument(
        "--quaternion-order",
        choices=("wxyz", "xyzw"),
        default="wxyz",
        help="Order used by 7-value pose arguments. The sent quaternion packet is always x,y,z,qw,qx,qy,qz.",
    )
    parser.add_argument(
        "--right-pose",
        type=parse_pose,
        default=parse_pose("0.35,-0.30,0.25"),
        help="Right pose as x,y,z, x,y,z,qw,qx,qy,qz, or x,y,z,r11,r12,...,r33.",
    )
    parser.add_argument(
        "--left-pose",
        type=parse_pose,
        default=parse_pose("0.35,0.30,0.25"),
        help="Left pose as x,y,z, x,y,z,qw,qx,qy,qz, or x,y,z,r11,r12,...,r33.",
    )
    parser.add_argument("--right-velocity", type=parse_spatial_vector, default=parse_spatial_vector("0,0,0,0,0,0"))
    parser.add_argument("--left-velocity", type=parse_spatial_vector, default=parse_spatial_vector("0,0,0,0,0,0"))
    parser.add_argument("--right-acceleration", type=parse_spatial_vector, default=parse_spatial_vector("0,0,0,0,0,0"))
    parser.add_argument("--left-acceleration", type=parse_spatial_vector, default=parse_spatial_vector("0,0,0,0,0,0"))
    parser.add_argument("--repeat", type=int, default=1, help="Number of packets to send.")
    parser.add_argument("--period", type=float, default=0.5, help="Seconds between repeated packets.")
    parser.add_argument("--dry-run", action="store_true", help="Print the packet without opening YARP ports.")
    return parser


def main():
    args = make_parser().parse_args()
    try:
        pose_representation, packet = build_packet(args)
    except ValueError as exc:
        print(f"Invalid input: {exc}", file=sys.stderr)
        return 2

    print(f"Mode: {args.mode}")
    print(f"Pose representation: {pose_representation}")
    print(f"Arms: {', '.join(args.arms)}")
    print(f"Vector size: {len(packet)}")
    print("Packet:")
    print(" ".join(f"{value:.9g}" for value in packet))

    if args.dry_run:
        return 0

    if args.repeat < 1:
        print("--repeat must be >= 1", file=sys.stderr)
        return 2

    yarp = import_yarp()
    yarp.Network.init()
    port = yarp.BufferedPortVector()

    try:
        if not yarp.Network.checkNetwork():
            print("YARP network is not available. Is yarpserver running?", file=sys.stderr)
            return 1

        if not port.open(args.client_port):
            print(f"Failed to open local port {args.client_port}", file=sys.stderr)
            return 1

        if not yarp.Network.connect(args.client_port, args.server_port):
            print(f"Warning: could not connect {args.client_port} -> {args.server_port}", file=sys.stderr)

        time.sleep(0.2)

        for index in range(args.repeat):
            write_vector(port, packet)
            print(f"Sent packet {index + 1}/{args.repeat} to {args.server_port}")
            if index + 1 < args.repeat:
                time.sleep(args.period)

        return 0
    finally:
        port.close()
        yarp.Network.fini()


if __name__ == "__main__":
    sys.exit(main())
