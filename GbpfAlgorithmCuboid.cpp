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
#include "ChPlanner.h"  
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
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/functions/ChFunctionSine.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include <cmath>
#include <queue>
#include <unordered_map>
//#include "chrono/core/ChLog.h"
//#include "chrono/core/ChTransform.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChFrameMoving.h"
#include "chrono/core/ChTimer.h"
#include <chrono>
#include <vector>
#include <tuple>
#include <array>
#include <random>
#include <algorithm>
#include <iostream>
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/functions/ChFunctionSine.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include <cmath>
#include <physics/ChLinkMotorLinear.h>
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono/core/ChFrame.h"
#include "chrono/core/ChFrameMoving.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "ChPlanModel.h"

#include <algorithm>
#include <set>
#include "GbpfAlgorithmCube.h"

#include "GbpfAlgorithmCuboid.h"

inline ChVectorDynamic<> sampleGaussianState(const ChPlanModel* model, const ChVectorDynamic<>& mean, double stdDev)
{
    const std::size_t ndofs = model->getDof();


    const ChVectorDynamic<> qMin = model->getMinimum();
    const ChVectorDynamic<> qMax = model->getMaximum();


    static thread_local std::mt19937_64 rng{ std::random_device{}() };
    std::normal_distribution<double>  gauss{ 0.0, stdDev };

    ChVectorDynamic<> sample(ndofs);

    for (std::size_t i = 0; i < ndofs; ++i) {
        double r = mean[i] + gauss(rng);

        if (r < qMin[i])
            r = qMin[i];
        else if (r > qMax[i])
            r = qMax[i];

        sample[i] = r;
    }
    return sample;
}


///////////////////////////////////////////// EXPAND CUBOID WITH DIFFERENT HALF IN EACH DIMENSION //////////////////////////////////

CuboidSize expandCuboidBinary(const ChVectorDynamic<>& center, ChPlanModel* model, double resolution, double maxHalf)
{
    const int n = center.size();

    CuboidSize s;
    s.neg = ChVectorDynamic<>(n);  
    s.pos = ChVectorDynamic<>(n);  

    s.neg.setZero();
    s.pos.setZero();

    ChVectorDynamic<> probe(center);                 

    auto find_extent = [&](int axis, int sign) -> double
        {
            double lo = 0.0;
            double hi = maxHalf;

            // If in the maximum bounds there is no collision, return the maximum edge
            probe[axis] = center[axis] + sign * hi;
            if (!model->isColliding(probe)) {
                probe[axis] = center[axis];              
                return hi;
            }

            // Binary search in [lo, hi]
            while (hi - lo > resolution) {
                double mid = 0.5 * (lo + hi);
                probe[axis] = center[axis] + sign * mid;

                if (model->isColliding(probe))
                    hi = mid;                            
                else
                    lo = mid;                            
            }

            probe[axis] = center[axis];                  // reset
            return lo;                                   // maximum extent found
    };

    //loop on aaxis
    for (int d = 0; d < n; ++d) {
        s.pos[d] = find_extent(d, +1);   // edge +d
        s.neg[d] = find_extent(d, -1);   // edge −d
    }
    //////////////////////////////////////////////////////////////////// more samples reduce the speed of the algorithm and increse the number of iterations, but increase the robustness
    //static thread_local std::mt19937 rng(std::random_device{}());
    //std::uniform_real_distribution<double> dist01(0.0, 1.0);
    //const int maxSamples = 1;
    //for (int k = 0; k < maxSamples; ++k) {
    //    ChVectorDynamic<> sample(n);
    //    for (int d = 0; d < n; ++d) {
    //        double low = center[d] - s.neg[d];
    //        double high = center[d] + s.pos[d];
    //        sample[d] = low + dist01(rng) * (high - low);
    //    }
    //    if (model->isColliding(sample)) {
    //        // Reject whole cuboid: return zeros so caller can skip it.
    //        s.neg.setZero();
    //        s.pos.setZero();
    //        return s;
    //    }
    //}
    //////////////////////////////////////
    return s;
}
CuboidSize expandCuboid(const ChVectorDynamic<>& center, ChPlanModel* model, double step, double maxHalf)
{
    int n = center.size();
    CuboidSize size;
    size.neg.resize(n);   size.neg.setZero();
    size.pos.resize(n);   size.pos.setZero();

    ChVectorDynamic<> probe(center.size());

    // for each dimension d, try to expand the cuboid
    for (int d = 0; d < n; ++d) {

        // + edge
        double pos = 0.0;
        while (pos + step <= maxHalf) {
            pos += step;
            probe = center;
            probe[d] += pos;                // incrase along +d

            if (model->isColliding(probe)) {       // cllision detected go back on last step
                pos -= step;
                break;
            }
        }
        size.pos[d] = pos;

        // - edge
        double neg = 0.0;
        while (neg + step <= maxHalf) {
            neg += step;
            probe = center;
            probe[d] -= neg;                // incrase along +d

            if (model->isColliding(probe)) {
                neg -= step;
                break;
            }
        }
        size.neg[d] = neg;
    }
    return size;
}


CuboidSize expandCuboid2(const ChVectorDynamic<>& center, ChPlanModel* model, double step, double maxHalf) {
    int n = center.size();
    CuboidSize size;
    size.neg = ChVectorDynamic<>(n);
    size.pos = ChVectorDynamic<>(n);
    size.pos.setZero();
    size.neg.setZero();


    ChVectorDynamic<> vector(center.size());

    for (int i = 0; i < center.size(); i++) {
        while (size.neg[i] + step <= maxHalf) {
            size.neg[i] += step;
            vector = center;
            vector[i] -= size.neg[i];  // increase along -i
            if (model->isColliding(vector)) {
                size.neg[i] -= step;  // if collision was found, go back on last step
                break;
            }
            size.neg[i];
        }

        while (size.pos[i] + step <= maxHalf) {
            size.pos[i] += step;
            vector = center;
            vector[i] += size.pos[i];  // increase along +i
            if (model->isColliding(vector)) {
                size.pos[i] -= step;  // if collision was found, go back on last step
                break;
            }
            size.pos[i];
        }
    }
    return size;
}



bool insideCuboid(const ChVectorDynamic<>& q, const ChVectorDynamic<>& center, const ChVectorDynamic<>& half_neg, const ChVectorDynamic<>& half_pos, ChPlanModel* model)
{
    for (int i = 0; i < q.size(); i++) {
        if (q[i] < center[i] - half_neg[i] || q[i] > center[i] + half_pos[i]) {
            return false;
        }
    }
    return true;
}

std::tuple<int, ChVectorDynamic<>, ChVectorDynamic<>, ChVectorDynamic<>> nearestCuboid(const ChVectorDynamic<>& q, const std::vector<CuboidNode>& foam, ChPlanModel* model)
{
    int    nearest = -1;
    double best_d = std::numeric_limits<double>::max();

    for (int i = 0; i < (int)foam.size(); ++i) {
        double d = model->distance(q, foam[i].center);
        if (d < best_d) {
            best_d = d;
            nearest = i;
        }
    }
    if (nearest < 0)
        throw std::runtime_error("[nearestCube] empty");

    const auto& node = foam[nearest];
    return { nearest, node.center, node.half_neg, node.half_pos };
}


ChVectorDynamic<> ConcatenateCuboid(const ChVectorDynamic<>& centerP, const ChVectorDynamic<>& halfP_neg, const ChVectorDynamic<>& halfP_pos, const ChVectorDynamic<>& halfC_neg, const ChVectorDynamic<>& halfC_pos, const ChVectorDynamic<>& qaux)
{
    // 1 direction vector from centerP to qaux
    ChVectorDynamic<> dir = qaux - centerP;
    double norm = dir.norm();
    if (norm < 1e-9)
        return centerP;                       // qaux = centerP

    dir /= norm;

    // 2
    auto choose_half = [](const ChVectorDynamic<>& neg,
        const ChVectorDynamic<>& pos,
        const ChVectorDynamic<>& d) -> double
        {
            double h = 0.0;
            for (int i = 0; i < d.size(); ++i)
                h = std::max(h, (d[i] >= 0.0) ? pos[i] : neg[i]);
            return h;
        };

    const double halfP = choose_half(halfP_neg, halfP_pos, dir);
    const double halfC = choose_half(halfC_neg, halfC_pos, dir);

    // 3 update position
    return centerP + dir * (halfP + halfC);
}

//bool cuboidIntersect(const CuboidNode& A, const CuboidNode& B, double margin)
//{
//    // 1)
//    if (insideCuboid(A.center, B.center, B.half_neg + margin, B.half_pos + margin, nullptr) ||
//        insideCuboid(B.center, A.center, A.half_neg + margin, A.half_pos + margin, nullptr))
//        return true;
//
//    // 2) 
//    for (int d = 0; d < A.center.size(); ++d) {
//        double a_min = A.center[d] - A.half_neg[d] - margin;
//        double a_max = A.center[d] + A.half_pos[d] + margin;
//        double b_min = B.center[d] - B.half_neg[d] - margin;
//        double b_max = B.center[d] + B.half_pos[d] + margin;
//
//        if (a_max < b_min || b_max < a_min)
//            return false;               
//    }
//    return true;                        
//}

int extendFoamCuboid(std::vector<CuboidNode>& foam, const ChVectorDynamic<>& qrand, ChPlanModel* model, double minSide, double sideTry, double expandStep, double expandMaxSide)
{
    if (foam.empty()) {
        std::cerr << "[extendFoam] foam is empty: no parent.\n";
        return -1;
    }

    // 1) nearest parent cuboid
    auto [idxParent, centerP, halfP_neg, halfP_pos] = nearestCuboid(qrand, foam, model);

    // 2) temporary center
    ChVectorDynamic<> centerC_tmp = ConcatenateCuboid(centerP, halfP_neg, halfP_pos,/*halfC_neg*/ChVectorDynamic<>(centerP.size()).setConstant(sideTry),/*halfC_pos*/ChVectorDynamic<>(centerP.size()).setConstant(sideTry),
        qrand);

    if (model->isColliding(centerC_tmp))
        return -1;

    // 3) expand cuboid with variable half in each dimension
    CuboidSize halfC = expandCuboidBinary(centerC_tmp, model, expandStep, expandMaxSide);

    // min half check
    if (halfC.neg.minCoeff() < minSide && halfC.pos.minCoeff() < minSide)
        return -1;

    // 4) real center
    ChVectorDynamic<> centerC =
        ConcatenateCuboid(centerP, halfP_neg, halfP_pos,
            halfC.neg, halfC.pos,
            qrand);

    if (model->isColliding(centerC))
        return -1;

    // 5) rejct if inside another cuboid
    if (interiorCuboid(centerC, foam, model))
        return -1;

    // 6) Add new node to foam
    CuboidNode newNode;
    newNode.center = centerC;
    newNode.half_neg = halfC.neg;
    newNode.half_pos = halfC.pos;
    newNode.parent = idxParent;

    foam.push_back(std::move(newNode));
    return static_cast<int>(foam.size()) - 1;
}


bool interiorCuboid(const ChVectorDynamic<>& q, const std::vector<CuboidNode>& foam, ChPlanModel* model)
{
    for (const auto& c : foam) {
        if (insideCuboid(q, c.center, c.half_neg, c.half_pos, model)) {
            return true;
        }
    }
    return false;
}


int checkConnectFoamCuboid(const CuboidNode& newCube, const std::vector<CuboidNode>& foamB)
{
    for (int i = 0; i < static_cast<int>(foamB.size()); ++i)
        if (insideCuboid(newCube.center, foamB[i].center, foamB[i].half_neg, foamB[i].half_pos, nullptr))
            return i;
    return -1;
}


std::vector<ChVectorDynamic<>> buildDoublePathCuboid(const std::vector<CuboidNode>& foamA, int idxA, const std::vector<CuboidNode>& foamB, int idxB, const ChVectorDynamic<>& qinit, const ChVectorDynamic<>& qgoal)
{
    {
        // rebuild pathA
        std::vector<chrono::ChVectorDynamic<>> pathA;
        {
            int c = idxA;
            while (c != -1) {
                pathA.push_back(foamA[c].center);
                c = foamA[c].parent;
            }
            // invert pathA
            std::reverse(pathA.begin(), pathA.end());
            // in the beginning, add qinit
            if (!pathA.empty()) {

                pathA.insert(pathA.begin(), qinit);
            }
        }

        // rebuild B
        std::vector<chrono::ChVectorDynamic<>> pathB;
        {
            int c = idxB;
            while (c != -1) {
                pathB.push_back(foamB[c].center);
                c = foamB[c].parent;
            }
            // reverse
            // If foamB is built starting from qgoal, then qgoal is the root => we get
            // pathB = [center, center, ..., qgoal].
            // If foamB starts at qgoal and parents go “backward,”
            // the tail is the root => i.e., c = -1
            // then the last element in pathB would be qgoal
            // So we might *not* reverse if the root of B is qgoal.
            // Decide based on how you created foamB.
            // We assume the same conventions as foamA.
            std::reverse(pathB.begin(), pathB.end());
            // Add qgoal
            if (!pathB.empty()) {
                pathB.push_back(qgoal);
            }
        }

        // concat pathA + pathB
        std::vector<chrono::ChVectorDynamic<>> finalPath = pathA;
        if (!pathB.empty()) {
            if (!finalPath.empty()) {
                if (finalPath.back().size() == pathB.front().size()) {
                    // check if are the same
                    bool same = true;
                    for (int i = 0; i < finalPath.back().size(); i++) {
                        double diff = finalPath.back()[i] - pathB.front()[i];
                        if (std::fabs(diff) > 1e-7) {
                            same = false;
                            break;
                        }
                    }
                    if (same) {
                        // remove duplicate
                        pathB.erase(pathB.begin());
                    }
                }
            }
            finalPath.insert(finalPath.end(), pathB.begin(), pathB.end());
        }

        return finalPath;
    }

}


std::vector<ChVectorDynamic<>>GoalBiasedProbabilisticFoamCuboidConnect(const ChVectorDynamic<>& qinit, const ChVectorDynamic<>& qgoal, ChPlanModel* model, ChSystemNSC& sys, double minSide, int maxIterations)
{

    std::vector<CuboidNode> foamA, foamB;
    foamA.reserve(2000);   foamB.reserve(2000);

    auto initA = expandCuboidBinary(qinit, model, /*step*/0.01, /*maxHalf*/0.4);
    auto initB = expandCuboidBinary(qgoal, model, /*step*/0.01, /*maxHalf*/0.4);

    foamA.push_back({ qinit, initA.neg.cwiseMax(minSide), initA.pos.cwiseMax(minSide), -1 });
    foamB.push_back({ qgoal, initB.neg.cwiseMax(minSide), initB.pos.cwiseMax(minSide), -1 });


    const double halfC_try = /*0.01*/0.1;
    const double expandStep = 0.01/*0.001*/;
    const double expandMax = 0.3;  //0.5
    const double beta0 = 0.3/*0.1*/;
    const double beta_max = 1.0;
    const double sigma0 = 0.4; // 0.1

    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<>  urand(0.0, 1.0);

    //main loop
    for (int iter = 0; iter < maxIterations; ++iter) {
        std::cout << "[TWO-FOAM-CONNECT] iter " << iter << std::endl;

        double beta = beta0 + (beta_max - beta0) * iter / double(maxIterations);
        double sigma = sigma0 * (1.0 - double(iter) / maxIterations);
        if (sigma < 0.01) {
            sigma = 0.05;  
        }
        ChVectorDynamic<> qrand(model->getDof());
        if (urand(gen) < beta) {
            qrand = qgoal;                           // bias
        }
        else {
            ChVectorDynamic<> qmin = model->getMinimum();
            ChVectorDynamic<> qmax = model->getMaximum();

            const int maxAttempts = 10;
            int attempts = 0;
            bool collisionFree = false;
            while (attempts < maxAttempts && !collisionFree) {
                //// Generate a random configuration in [qmin, qmax]
                //for (int d = 0; d < qrand.size(); ++d) {
                //    double r = urand(gen);
                //    qrand[d] = qmin[d] + r * (qmax[d] - qmin[d]);
                //}
                //// Check if the configuration is collision-free
                //collisionFree = !model->isColliding(qrand);
                //attempts++;
                while (!collisionFree && attempts < maxAttempts) {
                    qrand = sampleGaussianState(model, qgoal, sigma);
                    collisionFree = !model->isColliding(qrand);
                    ++attempts;
                }
            }
            if (!collisionFree) {
                continue;
            }
            //for (int d = 0; d < qrand.size(); ++d) {
            //    double r = urand(gen);
            //    qrand[d] = qmin[d] + r * (qmax[d] - qmin[d]);
            //    if (model->isColliding(qrand)) {
            //        break;
            //    }
            //}
        }   



        int newA = extendFoamCuboid(foamA, qrand, model,
            minSide, halfC_try,
            expandStep, expandMax);
        if (newA < 0) continue;


        int newB = extendFoamCuboid(foamB, foamA[newA].center, model,
            minSide, halfC_try,
            expandStep, expandMax);
        if (newB < 0) continue;


        int idxConnect = checkConnectFoamCuboid(foamB[newB], foamA);
        if (idxConnect >= 0) {
            std::cout << "[TWO-FOAM-CONNECT] goal found at iteration " << iter << std::endl;

            return buildDoublePathCuboid(foamA, idxConnect,
                foamB, newB,
                qinit, qgoal);
        }
    }

    // fail ⇒ empty path
    return {};
}


std::vector<CuboidNode> BuildChannelCuboids(const std::vector<ChVectorDynamic<>>& path, ChPlanModel* model, ChSystemNSC& sys, double stepExpand, double maxHalf)
{
    std::vector<CuboidNode> foam;
    if (path.empty()) return foam;

    //for(auto &cfg : path){

    for (size_t seg = 0; seg + 1 < path.size(); ++seg) {
        const auto& cfgA = path[seg];
        const auto& cfgB = path[seg + 1];

        for (int s = 0; s <= 4; ++s) {
            double  alpha = double(s) / 5;
            auto    cfg = model->interpolate(cfgA, cfgB, alpha);

            // expansion with variable half in each dimension
            auto half = expandCuboidBinary(cfg, model, stepExpand, maxHalf);
            if (half.neg.minCoeff() < 1e-3 && half.pos.minCoeff() < 1e-3)
                continue;

            if (interiorCuboid(cfg, foam, model))
                continue;

            int parent = foam.empty() ? -1 : int(foam.size()) - 1;
            foam.push_back({ cfg, half.neg, half.pos, parent });


            double diag = (half.neg.cwiseMax(half.pos)).maxCoeff();
            auto box = chrono_types::make_shared<ChBodyEasyBox>(2 * diag, 2 * diag, 2 * diag,
                1000, true, false);
            box->SetPos({ cfg[0], cfg[1], cfg[2] });
            box->SetFixed(true);
            box->GetVisualShape(0)->SetColor({ 0.f, 1.f, 0.f });
            box->GetVisualShape(0)->SetOpacity(0.4f);
            sys.Add(box);
       }
    }
    return foam;
}

void SaveCuboidsFoamToJson(const std::vector<CuboidNode>& foam, const std::string& filename)
{
    std::vector<ChVectorDynamic<double>> centers;
    std::vector<ChVectorDynamic<double>> half_neg;
    std::vector<ChVectorDynamic<double>> half_pos;
    std::vector<ChVectorDynamic<double>> mins;
    std::vector<ChVectorDynamic<double>> maxs;
    std::vector<int> parents;

    centers.reserve(foam.size());
    half_neg.reserve(foam.size());
    half_pos.reserve(foam.size());
    mins.reserve(foam.size());
    maxs.reserve(foam.size());
    parents.reserve(foam.size());

    for (const auto& c : foam) {

        centers.push_back(c.center);
        half_neg.push_back(c.half_neg);
        half_pos.push_back(c.half_pos);
        parents.push_back(c.parent);

        // bounding box 
        ChVectorDynamic<double> cmin(c.center.size());
        ChVectorDynamic<double> cmax(c.center.size());
        for (int d = 0; d < c.center.size(); ++d) {
            cmin[d] = c.center[d] - c.half_neg[d];
            cmax[d] = c.center[d] + c.half_pos[d];
        }
        mins.push_back(std::move(cmin));
        maxs.push_back(std::move(cmax));
    }

    std::ofstream file(filename);
    chrono::ChArchiveOutJSON marchive(file);

    marchive << CHNVP(centers, "channel_centers");
    //marchive << CHNVP(half_neg, "cuboid_half_neg");
    //marchive << CHNVP(half_pos, "cuboid_half_pos");
    marchive << CHNVP(mins, "channel_mins");    // bounding-box min
    marchive << CHNVP(maxs, "channel_maxs");    // bounding-box max
    //marchive << CHNVP(parents, "cuboid_parents"); 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


static bool segmentFreeBinary(const chrono::ChVectorDynamic<>& qa, const chrono::ChVectorDynamic<>& qb, chrono::ChPlanModel* model, int maxDepth = 5, double minStep = 1e-2)    
{
    // 0) early exit on extremes
    if (model->isColliding(qa) || model->isColliding(qb))
        return false;

    // 1) termination criteria: depth exhausted OR resolution reached
    if (maxDepth == 0 || model->distance(qa, qb) < minStep)
        return true;       // nothing found so far => accept

    // 2) mid‑point
    const auto mid = model->interpolate(qa, qb, 0.5);
    if (model->isColliding(mid))
        return false;      // obstacle on the segment

    // 3) recurse on the two halves
    return segmentFreeBinary(qa, mid, model, maxDepth - 1, minStep) &&
        segmentFreeBinary(mid, qb, model, maxDepth - 1, minStep);
}



std::vector<chrono::ChVectorDynamic<>> PartialShortcut2(std::vector<chrono::ChVectorDynamic<>>& path, chrono::ChPlanModel* model, int maxIterations)
{
    if (path.size() < 2)
        return path;        // nothing to do

    const int nDof = static_cast<int>(model->getDof());

    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist_index(0, static_cast<int>(path.size()) - 1);
    std::uniform_int_distribution<int> dist_dof(0, nDof - 1);

    for (int iter = 0; iter < maxIterations && path.size() >= 3; ++iter)
    {
        std::cout << "[PartialShortcut2] iter " << iter << std::endl;
        // 1) choose two distinct indices with at least one point in between
        int a = dist_index(rng);
        int b = dist_index(rng);
        if (a > b) std::swap(a, b);
        if (b - a < 2) continue;

        // 2) select one DOF to straighten
        const int dofSel = dist_dof(rng);

        // 3) build the candidate shortcut segment (copy + linear interp on dofSel)
        std::vector<chrono::ChVectorDynamic<>> newSegment;
        newSegment.reserve(b - a + 1);
        for (int i = a; i <= b; ++i)
            newSegment.push_back(path[i]);

        const double v0 = newSegment.front()(dofSel);
        const double v1 = newSegment.back()(dofSel);
        const int    N = static_cast<int>(newSegment.size()) - 1;
        for (int i = 1; i < N; ++i)
            newSegment[i](dofSel) = v0 + (v1 - v0) * (static_cast<double>(i) / static_cast<double>(N));

        // 4)  collision check on the whole segment
        if (!segmentFreeBinary(newSegment.front(), newSegment.back(), model))
            continue;       

        // 5) accept: overwrite original sub‑path [a+1 … b‑1]
        path.erase(path.begin() + a + 1, path.begin() + b);
        path.insert(path.begin() + a + 1,
            newSegment.begin() + 1,
            newSegment.end() - 1);
    }

    return path;
}