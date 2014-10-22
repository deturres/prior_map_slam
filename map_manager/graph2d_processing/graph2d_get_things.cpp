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
#include "bm_se2.h"

using namespace std;
using namespace g2o;


typedef std::pair< int,Isometry2d > PoseSeq;
typedef std::vector<PoseSeq, Eigen::aligned_allocator<Isometry2d> > PoseVector;
//debug
ofstream os("odometry2d.dat");
ofstream osangle("angles2d.dat");

g2o::VertexSE2* fake = 0;
EdgeSE2* fake2 = 0;
RobotLaser* data_raw = 0;

int main(int argc, char**argv){

    if( argc != 2 ) {
        cout << " Usage: ./graph2d_get_things <graphSE2_test_mapper.g2o>" << endl;
        return -1;
    }
  string filename;
  g2o::CommandArgs arg;
  arg.paramLeftOver("gi", filename , "", "graph file which will be processed", true);
  arg.parseArgs(argc, argv);

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
//  VertexSE2* v_current = 0;
//  EdgeSE2* eSE2 = 0;
//  OptimizableGraph::EdgeSet es;

  PoseVector odom;

  for (size_t j = 0; j<vertexIds.size(); j++)
  {
      OptimizableGraph::Vertex* _v = graph->vertex(vertexIds[j]);

      VertexSE2* v = dynamic_cast<VertexSE2*>(_v);
      if (!v)
          continue;
//      odom0to1 = se2t(v->estimate());
      Vector3d ev = v->estimate().toVector();
      odom0to1.linear() = Rotation2Dd(ev.z()).matrix();
      odom0to1.translation() = ev.head<2>();
//      cout << "current estimate in iso2" << odom0to1.matrix() << endl;

//      es = v->edges();
//      for (OptimizableGraph::EdgeSet::iterator itv = es.begin(); itv != es.end(); itv++) {
//          eSE2 = dynamic_cast<EdgeSE2*>(*itv);

//          if (!eSE2)
//              continue;
//          VertexSE2* tmp0 = dynamic_cast<VertexSE2*>(eSE2->vertices()[0]);
//          VertexSE2* tmp1 = dynamic_cast<VertexSE2*>(eSE2->vertices()[1]);
//          cout << "- Odom edge from vertex  " << tmp0->id() << " to " << tmp1->id() << endl;
//          if(tmp0->id() == v->id())
//          {
//              odom0to1 = eSE2->measurement().toIsometry();
//              cout << "Odometry transformation between the current vertex " << v->id() << " and the next one " << tmp1->id() << ":\n" << odom0to1.matrix() << endl;
//          } else {
//              odom0to1 = Eigen::Isometry2d::Identity();
//              cout << "###Skipping this edge (forward evaluation of the odometry)###" << endl;
//          }
          Vector3d currentTransform = t2v_2d(odom0to1);
          PoseSeq current;
          current.first = v->id();
          current.second = odom0to1;
          odom.push_back(current);
          cerr << "CurrentRobotReferenceFrame >> " <<  currentTransform.transpose() << endl;
          cerr << "number of poses >> " <<  odom.size() << endl;
    #if 1
          os << v->id() << " " << currentTransform.transpose() << endl;
          os.flush();
    #endif
//      }
  }

  cerr << "number of poses Finale >> " <<  odom.size() << endl;

  //extracting suddend changes of direction from odom
  //ugly parameters
  int windowsize = 5;
  int step = 2;
  double angleth = 50*M_PI/180;
  double current_angle, last_angle = -1;
  PoseVector feature;
  for(int i = 0; i+windowsize <= (int)odom.size(); i+=step) {

      current_angle = mat2angle_2d(odom[i].second.linear());
      last_angle = mat2angle_2d(odom[i+windowsize].second.linear());
      double diff = fabs(last_angle - current_angle);

      if(diff > angleth) {
          cout << "angle feature " << i << " - " << diff << endl;
          Vector3d pose = t2v_2d(odom[i+windowsize].second);
          feature.push_back(make_pair(odom[i+windowsize].first,odom[i+windowsize].second));
          i=i+windowsize-step;
#if 1
          osangle << odom[i+windowsize].first << " " << pose.transpose() << endl;
          osangle.flush();
#endif
      }
  }
  cout << "features size" << feature.size() << endl;

  return (0);
}
