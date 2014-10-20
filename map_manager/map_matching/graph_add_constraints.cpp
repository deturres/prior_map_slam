/*
 * graph_add_constraints.cpp
 *
 *  Created on: Sept 14, 2014
 *      Author: Martina
 */

#include "utility.h"

#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"

#include <g2o/core/hyper_graph.h>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/data/robot_laser.h"
//#include "g2o_frontend/sensor_data/laser_robot_data.h"
//#include "g2o_frontend/sensor_data/rgbd_data.h"
#include "bm_se2.h"

using namespace std;
using namespace g2o;


//typedef std::pair<g2o::VertexSE3*, g2o::VertexSE2*> Vertex3to2;
//typedef std::vector<Vertex3to2> v3Mapv2;
//typedef std::pair< int,Eigen::Vector2d > NewPoseSeq;
//typedef std::vector<NewPoseSeq, Eigen::aligned_allocator<Eigen::Vector2d> > NewPoses;


g2o::VertexSE2* fake = 0;
EdgeSE2* fake2 = 0;
//LaserRobotData* data = 0;
RobotLaser* data_raw = 0;

int main(int argc, char**argv){

    if( argc != 3 ) {
        cout << " Usage: ./graph_add_constraints <graph> <new Poses filename> optional:<graph_enanched filename>" << endl;
        return -1;
    }
  string filename;
  string outfilename;
  string newposes;
  g2o::CommandArgs arg;
  arg.param("o", outfilename, "graph_gps.g2o", "output file name");
  arg.paramLeftOver("gi", filename , "", "graph file which will be processed", true);
  arg.paramLeftOver("f", newposes, "" , "file_gpsFakePose to be read", true);
  arg.parseArgs(argc, argv);
  ofstream ofG2O_gps(outfilename.c_str());
  ifstream fakeGPS(newposes.c_str());

// graph construction
  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  OptimizationAlgorithmGaussNewton* solverGauss   = new OptimizationAlgorithmGaussNewton(blockSolver);
  SparseOptimizer * graph = new SparseOptimizer();
  graph->setAlgorithm(solverGauss);
  graph->load(filename.c_str());

  // sort the vertices based on the id
  std::vector<int> vertexIds(graph->vertices().size());
  int k=0;
  for (OptimizableGraph::VertexIDMap::iterator it=graph->vertices().begin(); it!= graph->vertices().end(); it ++){
    vertexIds[k++] = (it->first);
    }
  std::sort(vertexIds.begin(), vertexIds.end());

//    OptimizableGraph::Data* d = 0;

  //read  from file the new GPS fake prior
  utility::NewPosesGPS gps;
  utility::read_FakeGPS(fakeGPS, gps);

  cout << "Graph edges before: " << graph->edges().size() << endl;
  cout << "dimension file " <<  gps.size() << endl;

  //adding edges_prior
  for (int i = 0; i < (int)gps.size(); i++) {

      int idv = gps[i].first;
      // ATTENTION: if the graph is a 2D graph, the node type are different
      // to be changed in 3D on the go!!!
      VertexSE3* v;

      //searching for the correct associated vertex
      for (size_t i = 0; i<vertexIds.size(); i++)
      {
          OptimizableGraph::Vertex* _v = graph->vertex(vertexIds[i]);

          v = dynamic_cast<VertexSE3*>(_v);
          if (!v)
              continue;
          if(idv != v->id())
              continue;

          //adding the edge prior correspondent to the vertex
          Vector6d np = gps[i].second;
          const Eigen::Isometry3d meas = utility::v2t(np);
          EdgeSE3Prior* esp = new EdgeSE3Prior();
          esp->setVertex(0,v);
          esp->setMeasurement(meas);
          Eigen::Matrix<double, 6 , 6> info;
          info.setIdentity();
          info.block<3,3>(0,0)*0; //fake gps 3d just traslation??
          info.block<3,3>(3,3)*1000; //fake gps 3d just traslation??
//          Eigen::Matrix3d info;
//          info.block<2,2>(1,1)*1000;
          esp->setInformation(info);
          graph->addEdge(esp);
          cout << "dimension file " << endl;
          cout << "Graph edges: " << graph->edges().size() << endl;
      }
  }

  cout << "Graph edges: " << graph->edges().size() << endl;

  cout << "...saving graph in " << outfilename.c_str() << endl;
  graph->save(ofG2O_gps);
  ofG2O_gps.close();
  return (0);
}
