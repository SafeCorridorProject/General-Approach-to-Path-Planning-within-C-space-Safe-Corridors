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


#ifndef GBPF_ALGORITHM_CUBOID_H
#define GBPF_ALGORITHM_CUBOID_H

using namespace chrono;




struct CuboidNode {
    ChVectorDynamic<> center;
    ChVectorDynamic<> half_neg;
    ChVectorDynamic<> half_pos;
    int parent;
};

struct CuboidSize {
    ChVectorDynamic<> neg;
    ChVectorDynamic<> pos;
};



CuboidSize expandCuboid2(const ChVectorDynamic<>& center, ChPlanModel* model, double step, double maxHalf);

CuboidSize expandCuboid(const ChVectorDynamic<>& center, ChPlanModel* model, double step, double maxHalf);

bool insideCuboid(const ChVectorDynamic<>& q, const ChVectorDynamic<>& center, const ChVectorDynamic<>& half_neg, const ChVectorDynamic<>& half_pos, ChPlanModel* model);

std::tuple<int, ChVectorDynamic<>, ChVectorDynamic<>, ChVectorDynamic<>> nearestCuboid(const ChVectorDynamic<>& q, const std::vector<CuboidNode>& foam, ChPlanModel* model);

ChVectorDynamic<> ConcatenateCuboid(const ChVectorDynamic<>& centerP, const ChVectorDynamic<>& halfP_neg, const ChVectorDynamic<>& halfP_pos, const ChVectorDynamic<>& halfC_neg, const ChVectorDynamic<>& halfC_pos, const ChVectorDynamic<>& qaux);

bool interiorCuboid(const ChVectorDynamic<>& q, const std::vector<CuboidNode>& foam, ChPlanModel* model);

int extendFoamCuboid(std::vector<CuboidNode>& foam, const ChVectorDynamic<>& qrand, ChPlanModel* model, double minSide, double sideTry, double expandStep, double expandMaxSide);

inline bool cuboidIntersect(const CuboidNode& A, const CuboidNode& B, double margin);

CuboidSize expandCuboidBinary(const ChVectorDynamic<>& center, ChPlanModel* model, double resolution, double maxHalf);

int checkConnectFoamCuboid(const CuboidNode& newCube, const std::vector<CuboidNode>& foamB);

std::vector<ChVectorDynamic<>> buildDoublePathCuboid(const std::vector<CuboidNode>& foamA, int idxA, const std::vector<CuboidNode>& foamB, int idxB, const ChVectorDynamic<>& qinit, const ChVectorDynamic<>& qgoal);

std::vector<CuboidNode> BuildChannelCuboids(const std::vector<ChVectorDynamic<>>& path, ChPlanModel* model, ChSystemNSC& sys, double stepExpand, double maxHalf);

std::vector<ChVectorDynamic<>> GoalBiasedProbabilisticFoamCuboidConnect(const chrono::ChVectorDynamic<>& qinit, const chrono::ChVectorDynamic<>& qgoal, chrono::ChPlanModel* model, chrono::ChSystemNSC& sys, double minHalf, int maxIterations);

void SaveCuboidsFoamToJson(const std::vector<CuboidNode> & foam, const std::string & filename); 


std::vector<chrono::ChVectorDynamic<>> PartialShortcut2(std::vector<chrono::ChVectorDynamic<>>& path, chrono::ChPlanModel* model, int maxIterations);

#endif // GBPF_ALGORITHM_CUBOID_H