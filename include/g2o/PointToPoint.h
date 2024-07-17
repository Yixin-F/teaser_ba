#ifndef _POINT_TO_POINT_G2O_H
#define _POINT_TO_POINT_G2O_H

#include "../Utility.h"

class VertexPose : public g2o::BaseVertex<6, Eigen::Isometry3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexPose() {}

    virtual void setToOriginImpl() override {
        _estimate = Eigen::Isometry3d::Identity();
    }

    virtual void oplusImpl(const double* update) override {
        Eigen::Map<const Eigen::VectorXd> v(update, 6);
        Eigen::Matrix<double, 6, 1> update_se3 = v;
        Eigen::Isometry3d increment;
        increment = (Eigen::Translation3d(update_se3.head<3>()) *
                     Eigen::AngleAxisd(update_se3[3], Eigen::Vector3d::UnitX()) *
                     Eigen::AngleAxisd(update_se3[4], Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(update_se3[5], Eigen::Vector3d::UnitZ()));
        _estimate = _estimate * increment;
    }

    virtual bool read(std::istream& in) override {
        double data[7];
        for (int i = 0; i < 7; ++i) in >> data[i];
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        Eigen::Isometry3d t(q);
        t.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
        setEstimate(t);
        return true;
    }

    virtual bool write(std::ostream& out) const override {
        Eigen::Quaterniond q(estimate().rotation());
        out << estimate().translation().transpose() << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
        return out.good();
    }
};

class VertexPoint : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexPoint() {}

    virtual void setToOriginImpl() override {
        _estimate = Eigen::Vector3d::Zero();
    }

    virtual void oplusImpl(const double* update) override {
        Eigen::Vector3d::ConstMapType v(update);
        _estimate += v;
    }

    virtual bool read(std::istream& in) override {
        Eigen::Vector3d v;
        in >> v[0] >> v[1] >> v[2];
        setEstimate(v);
        return true;
    }

    virtual bool write(std::ostream& out) const override {
        out << estimate().transpose();
        return out.good();
    }
};

class EdgeSE3XYZ : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexPose, VertexPoint> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3XYZ() : information_(Eigen::Matrix3d::Identity()) {}

    virtual void computeError() override {
        const VertexPose* v1 = static_cast<const VertexPose*>(_vertices[0]);
        const VertexPoint* v2 = static_cast<const VertexPoint*>(_vertices[1]);
        _error = v1->estimate().inverse() * v2->estimate() - _measurement;
    }

    virtual void setMeasurement(const Eigen::Vector3d& m) override {
        _measurement = m;
    }

    virtual bool read(std::istream& in) override {
        Eigen::Vector3d m;
        in >> m[0] >> m[1] >> m[2];
        setMeasurement(m);
        for (int i = 0; i < 3; ++i)
            for (int j = i; j < 3; ++j) {
                double information;
                in >> information;
                if (i == j)
                    information_(i, j) = information;
                else {
                    information_(i, j) = information;
                    information_(j, i) = information;
                }
            }
        return true;
    }

    virtual bool write(std::ostream& out) const override {
        out << measurement().transpose() << " ";
        for (int i = 0; i < 3; ++i)
            for (int j = i; j < 3; ++j) {
                out << " " << information_(i, j);
            }
        return out.good();
    }

    virtual const Eigen::Matrix3d& information() const { return information_; }
    virtual Eigen::Matrix3d& information() { return information_; }

private:
    Eigen::Matrix3d information_;
};

G2O_REGISTER_TYPE(VERTEX_SE3:QUAT, VertexPose)
G2O_REGISTER_TYPE(VERTEX_XYZ, VertexPoint)
G2O_REGISTER_TYPE(EDGE_SE3_XYZ, EdgeSE3XYZ)



class PointBAusingG2O {
public:
    ~PointBAusingG2O() {}
    static void optimize(const std::map<int, std::map<int, Eigen::Vector3d>>& covisibility,
                         const std::map<int, Eigen::Matrix4d>& bf_poses,
                         std::map<int, Eigen::Vector3d>& opt_landmarks, 
                         std::map<int, Eigen::Matrix4d>& opt_poses) 
    {
        // initializa g2o optimizer
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(true);
        auto linearSolver = std::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();
        auto blockSolver = std::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
        auto solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));
        optimizer.setAlgorithm(solver);

        int vertex_id = 0;

        // add camera pose
        int pose_size = 0;
        for (const auto& pose : bf_poses) {
            int pose_id = pose.first;
            Eigen::Matrix4d pose_T(pose.second);

            VertexPose* vPose = new VertexPose();
            vPose->setId(vertex_id);

            Eigen::Isometry3d iso;
            iso = pose_T.block<3, 3>(0, 0);
            iso.translation() = pose_T.block<3, 1>(0, 3);
            vPose->setEstimate(iso);

            if (pose_id == 0) {
                vPose->setFixed(true);
            }

            optimizer.addVertex(vPose);
            pose_size ++;
            vertex_id ++;
        }
        LOG(INFO) << "[add vertex] pose size : " << pose_size;

        // add landmarks
        int lm_size = 0;
        for (const auto& iterm : covisibility) {
            int lm_id = iterm.first;
            auto obs = iterm.second;
            int anchor_id = obs.begin()->first;

            // TODO: transfer into world coordinate
            Eigen::Vector3d anchor_pos(obs.begin()->second);
            Eigen::Matrix4d anchor_pose(bf_poses.at(anchor_id));
            Eigen::Matrix3d anchor_rotation = anchor_pose.block(0, 0, 3, 3);
            Eigen::Vector3d anchor_translation = anchor_pose.block(0, 3, 3, 1);
            Eigen::Vector3d lm_assumption = anchor_rotation * anchor_pos + anchor_translation;

            VertexPoint* vPoint = new VertexPoint();
            vPoint->setId(vertex_id);
            vPoint->setEstimate(lm_assumption);

            // std::cout << lm_assumption << std::endl;

            optimizer.addVertex(vPoint);
            lm_size ++;
            vertex_id ++;
        }
        LOG(INFO) << "[add vertex] landmarks size : " << lm_size;

        // add observation
        int obs_size = 0;
        for (const auto& iterm : covisibility) {
            // if (obs_size > 1000) {
            //     break;
            // }
            for (const auto& cov : iterm.second) {
                if (cov.first == iterm.second.begin()->first) {
                    continue;
                }
                EdgeSE3XYZ* edge = new EdgeSE3XYZ();
                edge->setId(obs_size);
                edge->setParameterId(0, 0);
                edge->setVertex(0, (optimizer.vertex(cov.first)));
                edge->setVertex(1, (optimizer.vertex(iterm.first + bf_poses.size())));
                edge->setMeasurement(cov.second);

                // std::cout << cov.second << std::endl;

                edge->setInformation(Eigen::Matrix3d::Identity());
                optimizer.addEdge((edge));

                obs_size ++;
                vertex_id ++;
            }
        }
        LOG(INFO) << "[add edge] observation size : " << obs_size;

        // optimize g2o graph
        optimizer.initializeOptimization();
        LOG(INFO) << "begin to optimize ...";
        optimizer.optimize(10);

        // get optimized poses
        for (const auto& pose : bf_poses) {
            int pose_id = pose.first;
            VertexPose* vPose = dynamic_cast<VertexPose*>(optimizer.vertex(pose_id));
            auto opt_pose = vPose->estimate().matrix();
            opt_poses.insert(std::make_pair(pose_id, opt_pose));
        }
        LOG(INFO) << "[optimization results] optimized pose size: " << opt_poses.size();

        // get optimized landmarks
        for (const auto& iterm : covisibility) {
            int lm_id = iterm.first;
            VertexPoint* vPoint = dynamic_cast<VertexPoint*>(optimizer.vertex(lm_id + bf_poses.size()));
            Eigen::Vector3d opt_lm = vPoint->estimate().transpose();
            opt_landmarks.insert(std::make_pair(lm_id, opt_lm));
        }
        LOG(INFO) << "[optimization results] optimized landmark size: " << opt_landmarks.size();

        // output results
        LOG(INFO) << "[optimization results] done ... ";
    }

    void writeG2OTxt(const std::map<int, std::map<int, Eigen::Vector3d>>& covisibility,
                     const std::map<int, Eigen::Matrix4d>& bf_poses,
                     const std::string& filename)
    {
        // file open
        std::ofstream outFile(filename);
        if (!outFile.is_open()) {
            std::cerr << "Cannot open file: " << filename << std::endl;
            return;
        }

        // write poses
        int pose_size = 0;
        for (const auto& pose : bf_poses) {
            int pose_id = pose.first;
            Eigen::Matrix4d pose_T(pose.second);

            Eigen::Isometry3d iso;
            iso = pose_T.block<3, 3>(0, 0);
            iso.translation() = pose_T.block<3, 1>(0, 3);

            Eigen::Quaterniond q(iso.rotation());
            outFile << "VERTEX_SE3:QUAT " << pose_id << " "
                    << iso.translation().transpose() << " "
                    << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;

            pose_size ++;
        }
        LOG(INFO) << "[add vertex] pose size : " << pose_size;

        // write landmark
        int lm_size = 0;
        for (const auto& iterm : covisibility) {
            int lm_id = iterm.first;
            auto obs = iterm.second;
            int anchor_id = obs.begin()->first;

            // TODO: transfer into world coordinate
            Eigen::Vector3d anchor_pos(obs.begin()->second);
            Eigen::Matrix4d anchor_pose(bf_poses.at(anchor_id));
            Eigen::Matrix3d anchor_rotation = anchor_pose.block(0, 0, 3, 3);
            Eigen::Vector3d anchor_translation = anchor_pose.block(0, 3, 3, 1);
            Eigen::Vector3d lm_assumption = anchor_rotation * anchor_pos + anchor_translation;

            outFile << "VERTEX_XYZ " << (lm_size + bf_poses.size()) << " "
                    << lm_assumption.transpose() << std::endl;

            lm_size ++;
  
        }
        LOG(INFO) << "[add vertex] landmarks size : " << lm_size;

        // write SE3 edge
        int obs_size = 0;
        for (const auto& iterm : covisibility) {

            for (const auto& cov : iterm.second) {
                outFile << "EDGE_SE3_XYZ " << cov.first << " " << (iterm.first + bf_poses.size()) << " "
                        << cov.second.transpose() << " ";
                
                Eigen::Matrix<double, 6, 6> information;
                information.setIdentity();
                for (int i = 0; i < 3; ++i) {
                    for (int j = i; j < 3; ++j) {
                        outFile << information << " ";
                    }
                }
                outFile << std::endl;

                obs_size ++;

            }
        }
        LOG(INFO) << "[add edge] observation size : " << obs_size;

        outFile.close();
    }

    void optimizeTxt(const std::string& filename,
                     std::map<int, Eigen::Vector3d>& opt_landmarks, 
                     std::map<int, Eigen::Matrix4d>& opt_poses) 
    {
        if (filename.empty()) {
            std::cerr << "No input filename specified" << std::endl;
            return ;
        }

        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(true);
        auto linearSolver = std::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();
        auto blockSolver = std::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
        auto solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));
        optimizer.setAlgorithm(solver);

        if (!optimizer.load(filename.c_str())) {
            std::cerr << "Error loading graph" << std::endl;
            return ;
        }

        optimizer.initializeOptimization();
        std::cout << "Begin to optimize ..." << std::endl;
        optimizer.optimize(10);

        for (const auto& v : optimizer.vertices()) {
            auto* vertex = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v.second);
            if (vertex) {
                if (auto* pose = dynamic_cast<VertexPose*>(vertex)) {
                    Eigen::Matrix4d mat = pose->estimate().matrix();
                    opt_poses[pose->id()] = mat;
                } else if (auto* point = dynamic_cast<VertexPoint*>(vertex)) {
                opt_landmarks[point->id()] = point->estimate();
                }
            }
        }
    }

};

#endif _POINT_TO_POINT_G2O_H