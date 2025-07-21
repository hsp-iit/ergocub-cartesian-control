// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

struct YARPMatrix{}
(
    yarp.name = "yarp::sig::Matrix"
    yarp.includefile="yarp/sig/Matrix.h"
)

struct ReachabilityEvaluationState
{
    1: string status;

    2: YARPMatrix reached_pose;
}

service ergoCubCartesianService
{
    bool go_to_pose(1: double x, 2: double y, 3: double z, 4: double q_x, 5: double q_y, 6: double q_z, 7: double q_w, 8: double traj_duration);

    bool go_to_position(1: double x, 2: double y, 3: double z, 4: double traj_duration);

    bool rotate_rad(1: double angle, 2: double x, 3: double y, 4: double z, 5: double traj_duration);

    bool rotate_deg(1: double angle, 2: double x, 3: double y, 4: double z, 5: double traj_duration);

    YARPMatrix get_pose();

    bool go_home();

    bool is_motion_done();

    bool ask_reachability_evaluation(1: YARPMatrix pose);

    bool is_pose_reachable(1: double x, 2: double y, 3: double z, 4: double q_x, 5: double q_y, 6: double q_z, 7: double q_w);

    YARPMatrix retrieve_reachable_pose();

    bool stop();

    bool setJointsMode(1: string mode);
}
