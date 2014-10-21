/*
 * graph_add_constraints.cpp
 *
 *  Created on: Sept 14, 2014
 *      Author: Martina
 */

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


typedef std::pair< int,Isometry3d > PoseSeq;
typedef std::vector<PoseSeq, Eigen::aligned_allocator<Isometry3d> > PoseVector;
//debug
ofstream os("odometry2d.dat");
ofstream osangle("angles2d.dat");

g2o::VertexSE2* fake = 0;
EdgeSE2* fake2 = 0;
//LaserRobotData* data = 0;
RobotLaser* data_raw = 0;

int main(int argc, char**argv){

    if( argc != 3 ) {
        cout << " Usage: ./graph2d_getthings <graph> <new Poses filename> optional:<graph_enanched filename>" << endl;
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

  graph->vertex(0)->setFixed(true);
  graph->initializeOptimization();
  graph->setVerbose(false);

  Isometry2d odom0to1 = Isometry2d::Identity();
  VertexSE2* v_current = 0;
  EdgeSE2* eSE2 = 0;
  OptimizableGraph::EdgeSet es;

  PoseVector odom;

  for (size_t j = 0; j<vertexIds.size(); j++)
  {
      OptimizableGraph::Vertex* _v = graph->vertex(vertexIds[j]);

      VertexSE2* v = dynamic_cast<VertexSE2*>(_v);
      if (!v)
          continue;
      v_current = v;
      es = v_current->edges();

      for (OptimizableGraph::EdgeSet::iterator itv = es.begin(); itv != es.end(); itv++) {
          eSE2 = dynamic_cast<EdgeSE2*>(*itv);

          if (!eSE2)
              continue;
          VertexSE2* tmp0 = dynamic_cast<VertexSE2*>(eSE2->vertices()[0]);
          VertexSE2* tmp1 = dynamic_cast<VertexSE2*>(eSE2->vertices()[1]);
          cout << "- Odom edge from vertex  " << tmp0->id() << " to " << tmp1->id() << endl;
          if(tmp0->id() == v_current->id())
          {
              odom0to1 = eSE2->measurement().toIsometry();
              cout << "Odometry transformation between the current vertex " << v_current->id() << " and the next one " << tmp1->id() << ":\n" << odom0to1.matrix() << endl;
          } else {
              odom0to1 = Eigen::Isometry2d::Identity();
              cout << "###Skipping this edge (forward evaluation of the odometry)###" << endl;
          }
          Vector3d currentTransform = t2v_2d(odom0to1);
          PoseSeq current;
          current.first = v_current->id();
          current.second = odom0to1;
          odom.push_back(current);
          cerr << "CurrentRobotReferenceFrame >> " <<  currentTransform.transpose() << endl;
          cerr << "number of poses >> " <<  odom.size() << endl;
    #if 1
          os << v_current->id() << " " << currentTransform.transpose() << endl;
          os.flush();
    #endif
      }
  }


  cout << "...saving graph in " << outfilename.c_str() << endl;
  graph->save(ofG2O_gps);
  ofG2O_gps.close();
  return (0);
}
