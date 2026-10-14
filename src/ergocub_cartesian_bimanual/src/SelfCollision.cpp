/*
 * Copyright (C) 2026 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * MIT license. See the accompanying LICENSE file for details.
 */

#include <ergocub_cartesian_bimanual/SelfCollision.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/ModelLoader.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

using namespace iDynTree;

namespace
{
Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d S;
    S << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return S;
}
}


SelfCollision::SelfCollision(const std::string &urdf_path, const std::vector<std::string> &joints_list,
                             const std::string &root_frame_name, const std::string &model_path, const Params &params) :
    params_(params),
    min_distance_(std::numeric_limits<double>::infinity())
{
    ModelLoader loader;
    if (!loader.loadReducedModelFromFile(urdf_path, joints_list))
        throw std::runtime_error("SelfCollision: cannot load the model from " + urdf_path);
    kindyn_.loadRobotModel(loader.model());
    kindyn_.setFrameVelocityRepresentation(FrameVelocityRepresentation::MIXED_REPRESENTATION);
    n_ = kindyn_.getNrOfDegreesOfFreedom();
    if (n_ != static_cast<int>(joints_list.size()))
        throw std::runtime_error("SelfCollision: the reduced model has " + std::to_string(n_) + " joints instead of " + std::to_string(joints_list.size()));
    root_idx_ = kindyn_.getFrameIndex(root_frame_name);
    if (root_idx_ < 0)
        throw std::runtime_error("SelfCollision: cannot find the root frame " + root_frame_name);
    joints_vel_ = Eigen::VectorXd::Zero(n_);

    load_model(model_path);
}


int SelfCollision::frame_slot(const std::string &frame_name)
{
    const FrameIndex idx = kindyn_.getFrameIndex(frame_name);
    if (idx < 0)
        throw std::runtime_error("SelfCollision: cannot find frame " + frame_name + " in the reduced model");
    const auto it = std::find(frames_.begin(), frames_.end(), idx);
    if (it != frames_.end())
        return static_cast<int>(it - frames_.begin());
    frames_.push_back(idx);
    return static_cast<int>(frames_.size()) - 1;
}


void SelfCollision::load_model(const std::string &path)
{
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("SelfCollision: cannot open " + path);

    std::vector<std::pair<std::string, std::string>> pair_names;
    std::string line;
    while (std::getline(in, line))
    {
        std::istringstream ls(line);
        std::string kind;
        if (!(ls >> kind) || kind[0] == '#')
            continue;
        if (kind == "pair")
        {
            std::string a, b;
            ls >> a >> b;
            pair_names.emplace_back(a, b);
            continue;
        }
        std::string name, frame;
        int count = 0;
        if (!(ls >> name >> frame >> count) || count <= 0 || (kind != "polytope" && kind != "spheres"))
            throw std::runtime_error("SelfCollision: bad line in " + path + ": " + line);

        Eigen::MatrixXd values(count, 4);
        for (int i = 0; i < count; i++)
            for (int j = 0; j < 4; j++)
                if (!(in >> values(i, j)))
                    throw std::runtime_error("SelfCollision: truncated " + kind + " " + name + " in " + path);

        if (kind == "polytope")
        {
            Polytope p;
            p.name = name;
            p.frame = frame_slot(frame);
            p.normals = values.leftCols<3>();
            p.offsets = values.col(3);
            // The support points of the k-DOP planes, -off * n, bound the polytope well enough for the broad phase.
            Eigen::MatrixXd support = (-p.offsets).asDiagonal() * p.normals;
            p.center = 0.5 * (support.colwise().minCoeff() + support.colwise().maxCoeff()).transpose();
            p.radius = 0.0;
            for (int i = 0; i < count; i++)
                p.radius = std::max(p.radius, -p.offsets(i) - p.normals.row(i).dot(p.center));
            // a k-DOP vertex can be farther than its planes from the center: be generous
            p.radius *= 1.3;
            polytopes_.push_back(p);
        }
        else
        {
            Spheres s;
            s.name = name;
            s.frame = frame_slot(frame);
            s.centers = values.leftCols<3>();
            s.radii = values.col(3);
            s.center = 0.5 * (s.centers.colwise().minCoeff() + s.centers.colwise().maxCoeff()).transpose();
            s.radius = 0.0;
            for (int i = 0; i < count; i++)
                s.radius = std::max(s.radius, (s.centers.row(i).transpose() - s.center).norm() + s.radii(i));
            spheres_.push_back(s);
        }
    }

    auto find = [&](const auto &v, const std::string &name) {
        for (std::size_t i = 0; i < v.size(); i++)
            if (v[i].name == name)
                return static_cast<int>(i);
        throw std::runtime_error("SelfCollision: pair with unknown part " + name + " in " + path);
    };
    for (const auto &[a, b] : pair_names)
        pairs_.emplace_back(find(spheres_, a), find(polytopes_, b));
    if (pairs_.empty())
        throw std::runtime_error("SelfCollision: no pairs in " + path);

    state_.resize(frames_.size());
}


void SelfCollision::update(const Eigen::Ref<const Eigen::VectorXd> &joints, const Eigen::Ref<const Eigen::VectorXd> &joints_vel)
{
    const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    kindyn_.setRobotState(joints, joints_vel, gravity);
    joints_vel_ = joints_vel;

    for (std::size_t f = 0; f < frames_.size(); f++)
    {
        Eigen::Matrix4d T;
        kindyn_.getRelativeTransform(root_idx_, frames_[f], T);
        state_[f].R = T.topLeftCorner<3, 3>();
        state_[f].p = T.topRightCorner<3, 1>();
        state_[f].J.resize(6, n_);
        kindyn_.getRelativeJacobian(root_idx_, frames_[f], state_[f].J);
    }

    rows_.clear();
    min_distance_ = std::numeric_limits<double>::infinity();
    min_pair_.clear();

    // Velocity, in the root frame, of the point p (root coordinates) rigidly attached to frame f.
    auto point_jacobian = [&](const FrameState &s, const Eigen::Vector3d &p) {
        return Eigen::MatrixXd(s.J.topRows<3>() - skew(p - s.p) * s.J.bottomRows<3>());
    };

    struct Candidate { double d; int sphere; int plane; };
    std::vector<Candidate> cand;

    for (const auto &[si, pi] : pairs_)
    {
        const Spheres &S = spheres_[si];
        const Polytope &P = polytopes_[pi];
        const FrameState &fa = state_[S.frame];
        const FrameState &fb = state_[P.frame];

        // Spheres frame -> polytope frame.
        const Eigen::Matrix3d R_ba = fb.R.transpose() * fa.R;
        const Eigen::Vector3d p_ba = fb.R.transpose() * (fa.p - fb.p);

        const double broad = (R_ba * S.center + p_ba - P.center).norm() - S.radius - P.radius;
        if (broad > params_.activation)
            continue;

        cand.clear();
        for (int i = 0; i < S.centers.rows(); i++)
        {
            const Eigen::Vector3d c = R_ba * S.centers.row(i).transpose() + p_ba;
            Eigen::Index k;
            const double plane_d = (P.normals * c + P.offsets).maxCoeff(&k);
            const double d = plane_d - S.radii(i);
            if (d < params_.activation)
                cand.push_back({d, i, static_cast<int>(k)});
            if (d < min_distance_)
            {
                min_distance_ = d;
                min_pair_ = S.name + "-" + P.name;
            }
        }
        if (cand.empty())
            continue;

        const int keep = std::min<int>(params_.rows_per_pair, cand.size());
        std::partial_sort(cand.begin(), cand.begin() + keep, cand.end(), [](const Candidate &x, const Candidate &y) { return x.d < y.d; });

        for (int r = 0; r < keep; r++)
        {
            const Candidate &c = cand[r];
            const Eigen::Vector3d center = fa.R * S.centers.row(c.sphere).transpose() + fa.p;   // root coordinates
            const Eigen::Vector3d normal = fb.R * P.normals.row(c.plane).transpose();          // from the polytope out
            Row row;
            row.distance = c.d;
            row.jacobian = normal.transpose() * (point_jacobian(fa, center) - point_jacobian(fb, center));
            rows_.push_back(row);
        }
    }
}


void SelfCollision::constraints(const double dt, Eigen::MatrixXd &G, Eigen::VectorXd &h) const
{
    G.resize(rows_.size(), n_);
    h.resize(rows_.size());
    for (std::size_t r = 0; r < rows_.size(); r++)
    {
        const Row &row = rows_[r];
        const double e = row.distance - params_.margin;
        const double min_vel = e >= 0.0 ? -std::sqrt(2.0 * params_.max_acc * e)
                                        : std::min(params_.repulsion_gain * -e, params_.max_repulsion_vel);
        // row.jacobian (dq + dt ddq) >= min_vel  ->  -dt row.jacobian ddq <= row.jacobian dq - min_vel
        G.row(r) = -dt * row.jacobian;
        h(r) = row.jacobian.dot(joints_vel_) - min_vel;
    }
}
