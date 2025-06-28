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



/// Returns true if the configuration q is inside an axis-aligned cube
/// defined by "center ± half" (i.e., if in each dimension
/// abs(q[i] - center[i]) <= half).

bool insideCube(const ChVectorDynamic<>& q, const ChVectorDynamic<>& center, double half, ChPlanModel* model)
{
    for (int i = 0; i < q.size(); i++) {
        if (std::fabs(q[i] - center[i]) > half) {
            return false;
        }
    }
    return true;
}

/// Checks if q is "inside" any of the cubes in foam.

bool interiorCube(const ChVectorDynamic<>& q, const std::vector<CubeNode>& foam, ChPlanModel* model)
{
    for (const auto& c : foam) {
        if (insideCube(q, c.center, c.half, model)) {
            return true;
        }
    }
    return false;
}

/// ExpandCube: given a center configuration, it searches for the maximum half-length
/// (axis-aligned) for which there is no collision.

double expandCube2(const ChVectorDynamic<>& center, ChPlanModel* model, double step, double maxHalf)
{
    double half = 0.0;
    bool collision_found = false;

        ChVectorDynamic<> candidate_min(center.size());
        ChVectorDynamic<> candidate_max(center.size());
        static thread_local std::mt19937 rng(std::random_device{}());
        static thread_local std::uniform_real_distribution<double> dist01(0, 1);
        ChVectorDynamic<> candidate_rand(center.size());

        for (double i = 0; i <= maxHalf; i += step) {
            //double step = i;
            double new_half = half + step;
            for (int j = 0; j < center.size(); j++) {
                candidate_min(j) = center[j] - new_half;
                candidate_max(j) = center[j] + new_half;

            }
            if (model->isColliding(candidate_min)) {
                collision_found = true;
                break;
            }

            if (model->isColliding(candidate_max)) {
                collision_found = true;
                break;
            }
            int n_samples = 2;
            for(int i = 0; i < n_samples; i++) {
                // gnerate random point inside the cube ± try_half
				for (int d = 0; d < center.size(); d++) {
					double low = center[d] - new_half;
					double high = center[d] + new_half;
                    double rnd = dist01(rng);
					candidate_rand[d] = low + rnd * (high - low);
				}
                if (model->isColliding(candidate_rand)) {
                    collision_found = true;
                    break;
                }
			}

        
            if (collision_found) {
                break;
            }
        //if there is no collision, increase the half
            half = new_half;
        }

    return half;
}


///////////////////////////////// END EXPAND CUBE 2 ///////////////////////////




/// nearestCube: given a configuration qaux, it finds the closest cube in foam
std::tuple<int, ChVectorDynamic<>, double> nearestCube(const ChVectorDynamic<>& qaux, const std::vector<CubeNode>& foam, ChPlanModel* model)
{
    int nearest_idx = -1;
    double min_dist = std::numeric_limits<double>::max();

    for (int i = 0; i < (int)foam.size(); i++) {
        double d = model->distance(qaux, foam[i].center);
        if (d < min_dist) {
            min_dist = d;
            nearest_idx = i;
        }
    }

    return { nearest_idx, foam[nearest_idx].center, foam[nearest_idx].half };
}

/// creates a new center for the new cube, given the center of the parent cube

ChVectorDynamic<> ConcatenateCube(const ChVectorDynamic<>& centerP, double halfP, double halfC, const ChVectorDynamic<>& qaux)
{
    // direction = (qaux - centerP). normalized
    ChVectorDynamic<> dir = qaux - centerP;
    double norm = 0.0;
    for (int i = 0; i < dir.size(); i++) {
        norm += dir[i] * dir[i];
    }
    norm = std::sqrt(norm);
    if (norm < 1e-9) {
        return centerP;
    }
    for (int i = 0; i < dir.size(); i++) {
        dir[i] /= norm;
    }

    // the distance between the centers = halfP + halfC
    double distCenters = halfP + halfC;
    auto centerC = centerP + distCenters * dir;
    return centerC;
}

//////////////////////////////////////////////////////  CONNECT /////////////////////////////////////////

namespace {
    
     // Expands the foam (similar to “extend” in RRT) toward qrand:
     //       1) finds the nearest cube
     //       2) builds centerC_tmp
     //       3) calls expandCube2
     //       4) creates final_center
     //       5) checks if it is inside and if halfC >= minHalf
     //       6) creates a new cube and adds it
     //
     //return the index of the new cube, or -1 if it fails
     
    int extendFoam(std::vector<CubeNode>& foam, const chrono::ChVectorDynamic<>& qrand, chrono::ChPlanModel* model, double minHalf, double halfC_try, double expandStep, double expandMaxHalf)
    {
        if (foam.empty()) {
            std::cerr << "[extendFoam] foam is empty: no parent.\n";
            return -1;
        }

        // nearest
        auto [idxParent, centerP, halfP] = nearestCube(qrand, foam, model);

        // Temporary center 
        chrono::ChVectorDynamic<> centerC_tmp = ConcatenateCube(centerP, halfP, halfC_try, qrand);
        
        if (model->isColliding(centerC_tmp))
            return -1;
        
        // expandCube2
        double halfC = expandCube2(centerC_tmp, model, expandStep, expandMaxHalf);
        if (halfC < minHalf) {
            return -1; // fail
        }

        //  final_center
        chrono::ChVectorDynamic<> centerC = ConcatenateCube(centerP, halfP, halfC, qrand);

        if (model->isColliding(centerC))
            return -1;

        // if insideCube( centerC ) in foam => skip
        if (interiorCube(centerC, foam, model)) {
            return -1;
        }

        //  Create new cube
        CubeNode newcube(centerC, halfC, idxParent);
        foam.push_back(newcube);
        int newIndex = (int)foam.size() - 1;
        return newIndex;
    }

    
     //       Checks if the last cube created in foamA “intersects” any cube in foamB.
     //       consider the simplified intersection as:
     //       insideCube(centerC, centerB, halfB)
     //       If it finds a colliding cube, it returns the index of foamB; otherwise -1.
    

     
    int checkConnectFoam(const CubeNode& newCube, const std::vector<CubeNode>& foamB, chrono::ChPlanModel* model)
    {
        for (int i = 0; i < (int)foamB.size(); i++) {
            const auto& cB = foamB[i];
            // if centerB ± halfB 
            if (insideCube(newCube.center, cB.center, cB.half, model)) {
                // collision found
                return i;
            }

        }
        return -1;
    }

    //
    // Reconstructs the path starting from a newIndex in foamA and a connectIndex in foamB,
    // moving up their parent references (then reversing). It then appends the ascent from foamB.
    // Finally, it adds the goal configuration.
    // 
    std::vector<chrono::ChVectorDynamic<>> buildDoublePath(const std::vector<CubeNode>& foamA, int idxA, const std::vector<CubeNode>& foamB, int idxB, const chrono::ChVectorDynamic<>& qinit, const chrono::ChVectorDynamic<>& qgoal)
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
            std::reverse(pathB.begin(), pathB.end());
            // Add qgoal
            if (!pathB.empty()) {
                pathB.push_back(qgoal);
            }
        }

        // concatenate pathA + pathB
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




std::vector<CubeNode> BuildChannelCubes(const std::vector<ChVectorDynamic<>>& path, ChPlanModel* model, ChSystemNSC& sys, double stepExpand, double maxHalf, chrono::ChVectorDynamic<>& qinit, chrono::ChVectorDynamic<>& qgoal)
{
    std::vector<CubeNode> foam;
    if (path.empty()) return foam;

    for (int i = 0; i < path.size() - 1; i++) {
        auto cfg1 = path[i];
        auto cfg2 = path[i + 1];
        ///*for (const auto& cfg : path)*/ 
        //    // expand cube on cfg
        std::cout << "iter: " << i << std::endl;
        for (int step = 0; step <= 5; step++) {
            std::cout << "step: " << step << std::endl;
            double alpha = (double)step / (double)6;
            auto cfgMid = model->interpolate(cfg1, cfg2, alpha);
            double half = expandCube2(/*cfg1*/cfgMid, model, stepExpand, maxHalf);
            if (half < 0.001)
                continue;

            if (interiorCube(cfgMid, foam, model))
                continue;
            //if (cubeContainedSampling(cfgMid, half, foam, 40, 1e-4, model))
            //    continue;

            int parent = foam.empty() ? -1 : static_cast<int>(foam.size()) - 1;
            foam.emplace_back(cfgMid, half, parent);



            // visual cube (option)
            auto box = chrono_types::make_shared<ChBodyEasyBox>(2 * half, 2 * half, 2 * half, 1000, true, false);
            box->SetPos({ cfgMid(0), cfgMid(1), cfgMid(2) });
            box->SetFixed(true);
            box->GetVisualShape(0)->SetColor(ChColor(0.f, 1.f, 0.f));
            box->GetVisualShape(0)->SetOpacity(0.4f);
            sys.Add(box);
        }


    }

    return foam;
}





std::vector<ChVectorDynamic<>> GoalBiasedProbabilisticFoamCubeConnect(const chrono::ChVectorDynamic<>& qinit, const chrono::ChVectorDynamic<>& qgoal, chrono::ChPlanModel* model, chrono::ChSystemNSC& sys, double minHalf, int maxIterations) {
    // Inizialize foamA e foamB
    std::vector<CubeNode> foamA;
    std::vector<CubeNode> foamB;
    std::vector<ChVectorDynamic<>> fullPath;
    foamA.reserve(2000);
    foamB.reserve(2000);
    auto start_time = std::chrono::high_resolution_clock::now();

    // expandCube2 on qinit e qgoal
    double halfInitA = expandCube2(qinit, model, 0.01, 0.4);
    if (halfInitA < minHalf) halfInitA = minHalf;
    foamA.push_back(CubeNode(qinit, halfInitA, -1)); // parent=-1

    double halfInitB = expandCube2(qgoal, model, 0.01, 0.4);
    if (halfInitB < minHalf) halfInitB = minHalf;
    foamB.push_back(CubeNode(qgoal, halfInitB, -1)); // parent=-1

    // random generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> urand(0.0, 1.0);

    // expansion parameters
    double halfC_try =0.01; // try new half  
    double expandStep = 0.01;// step on expandCube2  
    double expandMaxHalf = 0.5; // maxHalf on expandCube2

    //bias parameters
    const double beta0 = 0.1;   
    const double beta_max = 1.0;
    
    //  main loop
    for (int iter = 0; iter < maxIterations; iter++) {


        std::cout << "[TWO-FOAM-CONNECT] iter " << iter << std::endl;
        ////////
        double beta = beta0 + (beta_max - beta0) * iter / static_cast<double>(maxIterations);     
        //double beta = 0.3;

        double rv = urand(gen);


        ChVectorDynamic<> qrand(model->getDof());
        if (rv < beta) {
            qrand = qgoal;
        }
        else {

            ChVectorDynamic<> qmin = model->getMinimum();
            ChVectorDynamic<> qmax = model->getMaximum();

            const int maxAttempts = 10;
            int attempts = 0;
            bool collisionFree = false;
            while (attempts < maxAttempts && !collisionFree) {
				// Generate a random configuration in [qmin, qmax]
                for (int d = 0; d < qrand.size(); ++d) {
					double r = urand(gen);
					qrand[d] = qmin[d] + r * (qmax[d] - qmin[d]);
				}
				// Check if the configuration is collision-free
				collisionFree = !model->isColliding(qrand);
				attempts++;
			}
            if (!collisionFree) {
                continue;
            }

        }
        //////////

        //Expand foamA e foamB
        int newA = extendFoam(foamA, qrand, model, minHalf, halfC_try, expandStep, expandMaxHalf);
        if (newA < 0) {
            continue;
        }

        //Try to connect foamA e foamB
        int newB = extendFoam(foamB, foamA[newA].center, model, minHalf, halfC_try, expandStep, expandMaxHalf);
        if (newB < 0) {

            continue;
        }


        // Check if foamA[newA] and foamB[newB] are in collisions
        int connectA = checkConnectFoam(foamB[newB], foamA, model);
        if (connectA >= 0) {
            std::cout << "[TWO-FOAM-CONNECT] goal found at iteration " << iter << std::endl;
            // build path
            fullPath = buildDoublePath(foamA, connectA, foamB, newB, qinit, qgoal);
            std::cout << "[TWO-FOAM-CONNECT] path size: " << fullPath.size() << std::endl;



            return fullPath;
        }


    }
    
    std::cout << "[TWO-FOAM-CONNECT] Not found path after " << maxIterations << " iterations.\n";
    // if it fail, return empty path
    return {};
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << "[TWO-FOAM-CONNECT] Iteration " <<  " took " << elapsed.count() << " seconds.\n";
}




  //PARTIAL SHORTCUT
 
  // Takes a discrete path (list of configurations).
  // In each iteration, picks two random points (indices a,b).
  // Only modifies one degree of freedom (DOF) at a time,
  // replacing the intermediate segment with the “line” on that DOF
  // (while the other DOFs remain fixed).
 
  //Parameters:
  //  path: the initial discrete trajectory (input and output).
  //  model: pointer to your ChPlanModel (for collisions, interpolation, etc.).
  //  maxIterations: how many times to attempt partial shortcuts.
 

std::vector<chrono::ChVectorDynamic<>> PartialShortcut(std::vector<chrono::ChVectorDynamic<>>& path, chrono::ChPlanModel* model, int maxIterations) {
    if (path.size() < 2) {
        return path;  
    }

    // Number of DOFs
    const int nDof = (int)model->getDof();
    // random generator
    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist_index(0, (int)path.size() - 1);
    std::uniform_int_distribution<int> dist_dof(0, nDof - 1);

    for (int iter = 0; iter < maxIterations; ++iter) {
        if (path.size() < 3)
            break;
        std::cout << "[PartialShortcut] iter " << iter << std::endl;
        // two random indices a, b
        int a = dist_index(rng);
        int b = dist_index(rng);
        if (a > b) std::swap(a, b);
        if (b - a < 2) {
            continue;
        }

        // One random DOF to modify
        int dofSelected = dist_dof(rng);

        // Extract the segment path [a..b] and try to "straighten" it with the values of dofSelected with a line, if is it collision-free
        std::vector<chrono::ChVectorDynamic<>> newSegment;
        newSegment.reserve(b - a + 1);

        // Copy the segment path[a..b] in newSegment
        for (int i = a; i <= b; i++) {
            newSegment.push_back(path[i]);
        }

        double startVal = newSegment.front()(dofSelected);
        double endVal = newSegment.back()(dofSelected);

        for (int i = 1; i < (int)newSegment.size() - 1; i++) {
            double alpha = (double)i / (double)(newSegment.size() - 1);
            double val = (1.0 - alpha) * startVal + alpha * endVal;
            newSegment[i](dofSelected) = val;
        }

        // Check collision on the new segment
        bool collisionFound = false;
        for (size_t i = 0; i < newSegment.size() && !collisionFound; ++i) {

            const auto& cfg = newSegment[i];

            // check on cfg
            if (model->isColliding(cfg)) { collisionFound = true;
            break;
            }

            //2) check on the expanded cube
            //double half = expandCube2(cfg, model, 0.05, 0.1);
            //if (half < 0.01) { collisionFound = true;
            //break;
            //}

            // dample between cfg[i‑1] and cfg[i]
            if (i > 0) {
                int nSteps = 20;
                for (int s = 1; s < nSteps; ++s) {
                    double al = double(s) / nSteps;
                    auto mid = model->interpolate(newSegment[i - 1], cfg, al);

                    if (model->isColliding(mid) /*||
                        expandCube2(mid, model, 0.05, 0.1) < 0.01*/)
                    {
                        collisionFound = true;
                        break;
                    }
                }
            }


        }
        // If there is no collision , replace the path segment [a+1..b-1] with newSegment[1..size-2]
        if (!collisionFound) {

            path.erase(path.begin() + a + 1, path.begin() + b);
            path.insert(path.begin() + a + 1,
                newSegment.begin() + 1,
                newSegment.end() - 1);
        }
    }

    return path;
}


void SaveFoamToJson(const std::vector<CubeNode>& foam, const std::string& filename)
{
    std::vector<ChVectorDynamic<double>> centers, mins, maxs;

    for (const auto& c : foam) {
        centers.push_back(c.center);

        ChVectorDynamic<double> cmin(c.center.size()), cmax(c.center.size());
        for (int d = 0; d < c.center.size(); ++d) {
            cmin[d] = c.center[d] - c.half;
            cmax[d] = c.center[d] + c.half;
        }
        mins.push_back(cmin);
        maxs.push_back(cmax);
    }

    std::ofstream file(filename);
    chrono::ChArchiveOutJSON marchive(file);
    marchive << CHNVP(centers, "channel_centers");
    marchive << CHNVP(mins, "channel_mins");
    marchive << CHNVP(maxs, "channel_maxs");
}





//////////////////////// TRY SHORTCUT //////////////////////
bool segmentCollisionFree(ChPlanModel* model, const ChVectorDynamic<>& a, const ChVectorDynamic<>& b, int checks) {
    if (model->isColliding(a) || model->isColliding(b))
        return false;
    for (int i = 1; i <= checks; ++i) {
        double alpha = static_cast<double>(i) / static_cast<double>(checks + 1);
        ChVectorDynamic<> mid = model->interpolate(a, b, alpha);
        if (model->isColliding(mid))
            return false;
    }
    return true;
}


void appendInterpolated(std::vector<ChVectorDynamic<>>& out, const ChVectorDynamic<>& from, const ChVectorDynamic<>& to, ChPlanModel* model, double step) {
    double dist = model->distance(from, to);
    int   pieces = std::max(1, static_cast<int>(std::ceil(dist / step)));
    for (int i = 1; i < pieces; ++i) {
        double alpha = static_cast<double>(i) / static_cast<double>(pieces);
        out.push_back(model->interpolate(from, to, alpha));
    }
    out.push_back(to);
}


std::vector<ChVectorDynamic<>> shortcutOptimize(const std::vector<ChVectorDynamic<>>& path, ChPlanModel* model, int   maxIter , int   checksPerSeg , double step ) {
    if (path.size() < 3)
        return path; // nothing to do

    std::vector<ChVectorDynamic<>> p = path; // working copy


    std::mt19937 rng(std::random_device{}());

    std::uniform_int_distribution<> distIdx; // will be reset each loop

    for (int iter = 0; iter < maxIter; ++iter) {
        if (p.size() < 3) break;
        distIdx = std::uniform_int_distribution<>(0, static_cast<int>(p.size()) - 3);
        int a = distIdx(rng);
        std::uniform_int_distribution<> distIdxB(a + 2, static_cast<int>(p.size()) - 1);
        int b = distIdxB(rng);

        if (!segmentCollisionFree(model, p[a], p[b], checksPerSeg))
            continue; // cannot shortcut

        // build a new path replacing the middle segment
        std::vector<ChVectorDynamic<>> newPath;
        newPath.reserve(p.size());
        newPath.insert(newPath.end(), p.begin(), p.begin() + a + 1);
        appendInterpolated(newPath, p[a], p[b], model, step);
        newPath.insert(newPath.end(), p.begin() + b + 1, p.end());
        p.swap(newPath);
    }
    return p;
}






