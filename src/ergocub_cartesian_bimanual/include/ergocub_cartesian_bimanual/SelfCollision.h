/*
 * Copyright (C) 2026 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * MIT license. See the accompanying LICENSE file for details.
 */

#ifndef SELF_COLLISION_H
#define SELF_COLLISION_H

#include <Eigen/Dense>

#include <iDynTree/KinDynComputations.h>

#include <string>
#include <vector>

/**
 * Self-collision avoidance constraints for the differential IK QP (unknowns: joint accelerations).
 *
 * The robot parts come from a model file written by scripts/make_self_collision_model.py: arm parts as sets of
 * spheres, body and arm parts as convex polytopes (planes), each in a frame of the kinematic model made of the
 * QP joints. The distance of a sphere from a polytope is the largest signed distance of its center from the
 * planes minus its radius: never more than the true distance.
 *
 * For every listed (spheres, polytope) pair closer than `activation`, the closest spheres give a row that limits
 * the approach velocity after the step so that the part can still stop at `margin` with deceleration `max_acc`:
 *     d_dot = J_d (dq + ddq dt) >= -sqrt(2 max_acc (d - margin))
 * Below the margin the parts must separate: d_dot >= repulsion_gain (margin - d), at most max_repulsion_vel.
 */
class SelfCollision
{
public:
    struct Params
    {
        double margin = 0.02;            // [m]
        double activation = 0.25;        // [m]
        double max_acc = 2.5;            // [m/s^2]
        double repulsion_gain = 1.0;     // [1/s]
        double max_repulsion_vel = 0.05; // [m/s]
        int rows_per_pair = 3;
    };

    /**
     * @param urdf_path robot URDF.
     * @param joints_list QP joints, in the QP order.
     * @param root_frame_name frame of the QP Cartesian quantities.
     * @param model_path self-collision model file.
     */
    SelfCollision(const std::string &urdf_path, const std::vector<std::string> &joints_list,
                  const std::string &root_frame_name, const std::string &model_path, const Params &params);

    /** Update the kinematics at the state the QP starts from. */
    void update(const Eigen::Ref<const Eigen::VectorXd> &joints, const Eigen::Ref<const Eigen::VectorXd> &joints_vel);

    /** Rows G ddq <= h for the given sampling time. */
    void constraints(const double dt, Eigen::MatrixXd &G, Eigen::VectorXd &h) const;

    /** Smallest distance (model, without margin) found by the last update and its pair. */
    double min_distance() const { return min_distance_; }
    const std::string &min_pair() const { return min_pair_; }

    /** Distance of each constraint row of the last update. */
    std::vector<double> row_distances() const { std::vector<double> d; for (const auto &r : rows_) d.push_back(r.distance); return d; }

private:
    struct Polytope
    {
        std::string name;
        int frame;
        Eigen::Matrix<double, Eigen::Dynamic, 3> normals;
        Eigen::VectorXd offsets; // inside: n.x + off <= 0
        Eigen::Vector3d center;
        double radius;           // bounding sphere of the polytope vertices (approx.: of its planes' support)
    };

    struct Spheres
    {
        std::string name;
        int frame;
        Eigen::Matrix<double, Eigen::Dynamic, 3> centers;
        Eigen::VectorXd radii;
        Eigen::Vector3d center;
        double radius;           // bounding sphere of the set
    };

    struct Row
    {
        double distance;
        Eigen::RowVectorXd jacobian; // d_dot = jacobian * dq
    };

    struct FrameState
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d p;
        Eigen::MatrixXd J; // 6 x n, mixed: origin linear velocity and angular velocity in the root frame
    };

    void load_model(const std::string &path);

    int frame_slot(const std::string &frame_name);

    iDynTree::KinDynComputations kindyn_;
    iDynTree::FrameIndex root_idx_;
    std::vector<iDynTree::FrameIndex> frames_;
    std::vector<FrameState> state_;
    std::vector<Polytope> polytopes_;
    std::vector<Spheres> spheres_;
    std::vector<std::pair<int, int>> pairs_;
    Params params_;
    int n_;
    std::vector<Row> rows_;
    Eigen::VectorXd joints_vel_;
    double min_distance_;
    std::string min_pair_;
};

#endif /* SELF_COLLISION_H */
