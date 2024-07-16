#include "PointToPoint.h"

// int main() {
//     // Step 1: Create the optimizer
//     g2o::SparseOptimizer optimizer;
//     optimizer.setVerbose(true);
//     BlockSolverType::LinearSolverType* linearSolver = new LinearSolverType();
//     BlockSolverType* blockSolver = new BlockSolverType(linearSolver);
//     g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
//     optimizer.setAlgorithm(algorithm);

//     // Define initial camera poses and 3D points (example data)
//     std::vector<Eigen::Isometry3d> initial_camera_poses;
//     std::vector<Eigen::Vector3d> initial_3d_points;
//     // Define observations (example data)
//     struct Observation {
//         int camera_id;
//         int point_id;
//         Eigen::Vector2d measurement;
//     };
//     std::vector<Observation> observations;

//     // Add vertices (camera poses and 3D points)
//     for (int i = 0; i < initial_camera_poses.size(); ++i) {
//         g2o::VertexSE3Expmap* v_se3 = new g2o::VertexSE3Expmap();
//         v_se3->setId(i);
//         if (i == 0) {
//             v_se3->setFixed(true);  // Fix the first camera pose
//         }
//         v_se3->setEstimate(g2o::SE3Quat(initial_camera_poses[i].rotation(), initial_camera_poses[i].translation()));
//         optimizer.addVertex(v_se3);
//     }

//     for (int i = 0; i < initial_3d_points.size(); ++i) {
//         g2o::VertexSBAPointXYZ* v_p = new g2o::VertexSBAPointXYZ();
//         v_p->setId(i + initial_camera_poses.size());
//         v_p->setEstimate(initial_3d_points[i]);
//         optimizer.addVertex(v_p);
//     }

//     // Add edges (observations)
//     for (const auto& obs : observations) {
//         g2o::EdgeSE3ProjectXYZ* edge = new g2o::EdgeSE3ProjectXYZ();
//         edge->setId(obs.camera_id);
//         edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(obs.point_id + initial_camera_poses.size())));
//         edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(obs.camera_id)));
//         edge->setMeasurement(obs.measurement);
//         edge->setInformation(Eigen::Matrix2d::Identity());
//         optimizer.addEdge(edge);
//     }

//     // Optimize the graph
//     optimizer.initializeOptimization();
//     optimizer.optimize(10);

//     // Retrieve optimized results
//     std::vector<Eigen::Isometry3d> optimized_camera_poses(initial_camera_poses.size());
//     std::vector<Eigen::Vector3d> optimized_3d_points(initial_3d_points.size());

//     for (int i = 0; i < initial_camera_poses.size(); ++i) {
//         g2o::VertexSE3Expmap* v_se3 = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i));
//         optimized_camera_poses[i] = v_se3->estimate().to_homogeneous_matrix();
//     }

//     for (int i = 0; i < initial_3d_points.size(); ++i) {
//         g2o::VertexSBAPointXYZ* v_p = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i + initial_camera_poses.size()));
//         optimized_3d_points[i] = v_p->estimate();
//     }

//     // Output results
//     std::cout << "Optimized Camera Poses:" << std::endl;
//     for (const auto& pose : optimized_camera_poses) {
//         std::cout << pose.matrix() << std::endl;
//     }

//     std::cout << "Optimized 3D Points:" << std::endl;
//     for (const auto& point : optimized_3d_points) {
//         std::cout << point.transpose() << std::endl;
//     }

//     return 0;
// }