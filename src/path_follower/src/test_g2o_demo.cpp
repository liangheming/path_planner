#include "teb_follower/commons.h"
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include "teb_follower/g2o_edges/edge_velocity.hpp"

using TebBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
using TebLinearSolver = g2o::LinearSolverCSparse<TebBlockSolver::PoseMatrixType>;

void loadData(std::vector<Pose2E> &trajectory, std::vector<double> &time_diffs)
{
    trajectory.emplace_back(-0.358683, 1.14302, -0.636107);
    trajectory.emplace_back(-0.266473, 0.99442, -0.714154);
    trajectory.emplace_back(-0.149165, 0.892741, -0.714154);
    trajectory.emplace_back(-0.0318581, 0.791061, -0.714154);
    trajectory.emplace_back(0.0854492, 0.689382, -0.714154);
    trajectory.emplace_back(0.202756, 0.587702, -0.714154);
    trajectory.emplace_back(0.31349, 0.479166, -0.864592);
    trajectory.emplace_back(0.413954, 0.361382, -0.947507);
    trajectory.emplace_back(0.514164, 0.221986, -0.878059);
    trajectory.emplace_back(0.614259, 0.101382, -0.857191);
    trajectory.emplace_back(0.714303, -0.0141668, -0.879454);
    trajectory.emplace_back(0.814325, -0.135025, -0.950727);
    trajectory.emplace_back(0.91434, -0.2751, -0.873255);
    trajectory.emplace_back(1.01436, -0.394439, -0.839547);
    trajectory.emplace_back(1.21446, -0.617259, -0.869919);
    trajectory.emplace_back(1.31461, -0.735953, -0.943939);
    trajectory.emplace_back(1.41495, -0.874479, -0.861505);
    trajectory.emplace_back(1.51569, -0.991858, -0.814817);
    trajectory.emplace_back(1.721, -1.20314, -0.7535);
    trajectory.emplace_back(1.94706, -1.40487, -0.618343);
    trajectory.emplace_back(2.08683, -1.5043, -0.67986);
    trajectory.emplace_back(2.20826, -1.60247, -0.683912);
    trajectory.emplace_back(2.32563, -1.69815, -0.626015);
    trajectory.emplace_back(2.45053, -1.78846, -0.484683);
    time_diffs.emplace_back(0.116591);
    time_diffs.emplace_back(0.103494);
    time_diffs.emplace_back(0.103494);
    time_diffs.emplace_back(0.103494);
    time_diffs.emplace_back(0.103494);
    time_diffs.emplace_back(0.150437);
    time_diffs.emplace_back(0.103207);
    time_diffs.emplace_back(0.114452);
    time_diffs.emplace_back(0.104487);
    time_diffs.emplace_back(0.101894);
    time_diffs.emplace_back(0.104586);
    time_diffs.emplace_back(0.114744);
    time_diffs.emplace_back(0.103807);
    time_diffs.emplace_back(0.199655);
    time_diffs.emplace_back(0.103534);
    time_diffs.emplace_back(0.114031);
    time_diffs.emplace_back(0.103123);
    time_diffs.emplace_back(0.196424);
    time_diffs.emplace_back(0.202049);
    time_diffs.emplace_back(0.114352);
    time_diffs.emplace_back(0.104097);
    time_diffs.emplace_back(0.100954);
    time_diffs.emplace_back(0.141332);
}
int main(int argc, char **argv)
{
    std::vector<Pose2E> trajectory;
    std::vector<double> dts;
    loadData(trajectory, dts);
    std::unique_ptr<TebLinearSolver> linear_solver = std::make_unique<TebLinearSolver>();
    linear_solver->setBlockOrdering(true);
    std::unique_ptr<TebBlockSolver> block_solver = std::make_unique<TebBlockSolver>(std::move(linear_solver));
    std::shared_ptr<g2o::SparseOptimizer> optimizer = std::make_shared<g2o::SparseOptimizer>();
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
    optimizer->setAlgorithm(solver);
    optimizer->initMultiThreading();

    // std::vector<g2o::VertexSE2 *> trajectory_vertices;
    // std::vector<VertexDouble *> time_diff_vertices;

    std::vector<g2o::VertexSE2> trajectory_vertices;
    std::vector<VertexDouble> time_diff_vertices;

    unsigned int vertex_id = 0;
    for (int i = 0; i < trajectory.size(); i++)
    {
        // g2o::VertexSE2 *vertex = new g2o::VertexSE2();
        // vertex->setId(vertex_id++);
        // vertex->setEstimate(g2o::SE2(trajectory[i].x, trajectory[i].y, trajectory[i].theta));
        // vertex->setFixed(false);
        // if (i == 0 || i == trajectory.size() - 1)
        //     vertex->setFixed(true);
        // optimizer->addVertex(vertex);
        // trajectory_vertices.emplace_back(vertex);
        trajectory_vertices.emplace_back(g2o::VertexSE2());
        g2o::VertexSE2 &vertex = trajectory_vertices.back();
        vertex.setId(vertex_id++);
        vertex.setEstimate(g2o::SE2(trajectory[i].x, trajectory[i].y, trajectory[i].theta));
        vertex.setFixed(false);
        if (i == 0 || i == trajectory.size() - 1)
            vertex.setFixed(true);
        optimizer->addVertex(&vertex);
    }
    for (int i = 0; i < dts.size(); i++)
    {
        // VertexDouble *vertex = new VertexDouble();
        // vertex->setId(vertex_id++);
        // vertex->setEstimate(dts[i]);
        // optimizer->addVertex(vertex);
        // time_diff_vertices.emplace_back(vertex);
        time_diff_vertices.emplace_back(VertexDouble());
        time_diff_vertices.back().setId(vertex_id++);
        time_diff_vertices.back().setEstimate(dts[i]);
        optimizer->addVertex(&time_diff_vertices.back());
    }

    Eigen::Matrix2d info = Eigen::Matrix2d::Identity() * 0.1;
    for (int i = 0; i < dts.size(); i++)
    {
        // EdgeVelocity *edge = new EdgeVelocity();
        // edge->setVertex(0, trajectory_vertices[i]);
        // edge->setVertex(1, trajectory_vertices[i + 1]);
        // edge->setVertex(2, time_diff_vertices[i]);
        // edge->setInformation(info);
        // edge->setMeasurement(VelocityMessurement(-0.3, 0.5, -0.5, 0.5, 0.1));
        // optimizer->addEdge(edge);

        EdgeVelocity *edge = new EdgeVelocity();
        edge->setVertex(0, &trajectory_vertices[i]);
        edge->setVertex(1, &trajectory_vertices[i + 1]);
        edge->setVertex(2, &time_diff_vertices[i]);
        edge->setInformation(info);
        edge->setMeasurement(VelocityMessurement(-0.3, 0.5, -0.5, 0.5, 0.1));
        optimizer->addEdge(edge);
    }
    std::cout << "edges: " << optimizer->edges().size() << std::endl;
    std::cout << "vertices: " << optimizer->vertices().size() << std::endl;
    // optimizer->initializeOptimization();
    // optimizer->optimize(10);
    std::cout << "optimize" << std::endl;
    // optimizer->clear();
    std::cout << "after" << std::endl;
}