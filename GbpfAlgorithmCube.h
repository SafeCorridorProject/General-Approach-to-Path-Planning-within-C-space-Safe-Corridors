#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkMotorAll.h"
#include "chrono/physics/ChLinkMotorLinear.h"
#include "chrono/physics/ChLinkMotionImposed.h"
#include "chrono/functions/ChFunctionPositionLine.h"
#include "chrono/functions/ChFunctionRotationBSpline.h"
#include "chrono/functions/ChFunctionRotationBSpline.h"
#include "chrono/geometry/ChLine.h"
#include "chrono/geometry/ChBasisToolsBSpline.h"
#include "chrono/geometry/ChLineBSpline.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChLine.h"
#include "chrono/geometry/ChLineSegment.h"
#include "chrono/functions/ChFunctionPositionLine.h"
#include "chrono/functions/ChFunctionPositionXYZFunctions.h"
#include "chrono/functions/ChFunctionPositionSetpoint.h"
#include "chrono/functions/ChFunctionRotationBSpline.h"
#include "chrono/functions/ChFunctionRotationABCFunctions.h"
#include "chrono/functions/ChFunctionRotationSetpoint.h"
#include "chrono/functions/ChFunctionRotationAxis.h"
#include "chrono/functions/ChFunctionRotationSQUAD.h"
#include <random>

#include "chrono/physics/ChLinkMotionImposed.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "osqp/osqp.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include "osqp.h"
#include <stdlib.h>
#include <stdio.h>
#include <osqp/osqp_api_types.h>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include "chrono/core/ChGlobal.h"
#include "chrono/functions/ChFunctionInterp.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_thirdparty/filesystem/path.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <random>
#include <algorithm>
#include <cmath>
#include <limits>

#include "chrono/core/ChClassFactory.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/serialization/ChArchive.h"
#include "chrono/serialization/ChOutputASCII.h"

#include <chrono>
#include <vector>
#include <tuple>
#include <array>
#include <random>
#include <algorithm>
#include <iostream>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "ChPlanModel.h"
#include <utility>
#include <fstream>
#include "chrono/serialization/ChArchiveJSON.h"


#ifndef GBPF_ALGORITHM_CUBE_H
#define GBPF_ALGORITHM_CUBE_H

using namespace chrono;

/// Struttura dati per un cubo
///   center     : central cfg
///   half       : half
///   parent     : index of parent node
struct CubeNode {
    chrono::ChVectorDynamic<> center;
    double half;
    int parent;

    CubeNode(const chrono::ChVectorDynamic<>& c, double h, int p)
        : center(c), half(h), parent(p) {}
};


bool insideCube(const ChVectorDynamic<>& q, const ChVectorDynamic<>& center, double half, ChPlanModel* model);

bool interiorCube(const ChVectorDynamic<>& q, const std::vector<CubeNode>& foam, ChPlanModel* model);

std::tuple<int, ChVectorDynamic<>, double> nearestCube(const ChVectorDynamic<>& qaux, const std::vector<CubeNode>& foam, ChPlanModel* model);

ChVectorDynamic<> ConcatenateCube(const ChVectorDynamic<>& centerP, double halfP, double halfC, const ChVectorDynamic<>& qaux);

double expandCube2(const ChVectorDynamic<>& center, ChPlanModel* model, double step, double maxHalf);

std::vector<CubeNode> BuildChannelCubes(const std::vector<ChVectorDynamic<>>& path, ChPlanModel* model, ChSystemNSC& sys, double stepExpand, double maxHalf, chrono::ChVectorDynamic<>& qinit, chrono::ChVectorDynamic<>& qgoal);

std::vector<ChVectorDynamic<>> GoalBiasedProbabilisticFoamCubeConnect(const chrono::ChVectorDynamic<>& qinit, const chrono::ChVectorDynamic<>& qgoal, chrono::ChPlanModel* model, chrono::ChSystemNSC& sys, double minHalf, int maxIterations);

std::vector<chrono::ChVectorDynamic<>> PartialShortcut(std::vector<chrono::ChVectorDynamic<>>& path, chrono::ChPlanModel* model, int maxIterations);

void SaveFoamToJson(const std::vector<CubeNode>& foam, const std::string& filename);

//////////////////////// TRY SHORTCUT //////////////////////
bool segmentCollisionFree(ChPlanModel* model, const ChVectorDynamic<>& a, const ChVectorDynamic<>& b, int checks);

void appendInterpolated(std::vector<ChVectorDynamic<>>& out, const ChVectorDynamic<>& from, const ChVectorDynamic<>& to, ChPlanModel* model, double step);

std::vector<ChVectorDynamic<>> shortcutOptimize(const std::vector<ChVectorDynamic<>>& path, ChPlanModel* model, int   maxIter, int   checksPerSeg, double step);

#endif // GBPF_ALGORITHM_CUBE_H
