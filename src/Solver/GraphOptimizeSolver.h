/*--------------------------------------------------------------------------------
 * POSGO: a GNSS POSition system based on Graph Optimization
 *
 * Copyright (C) 2022 Satellite POD and Navigation Augmentation Group,
 *                    Wuhan University
 *
 *     Author : Zhen Li
 *    Contact : sdkjlizhen@foxmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * reference :
 *
 * Created by lizhen on 2022/9/1.
 *-------------------------------------------------------------------------------*/

#ifndef GRAPHOPTIMIZESOLVER_H
#define GRAPHOPTIMIZESOLVER_H

#include "ceres/ceres.h"
#include "Solver.h"
#include "../SensorErrorModel/GnssErrorModel.h"

#define MAX_WINDOW_INERVAL 3

class GraphOptimizeSolver: public Solver{
public:
    GraphOptimizeSolver() = default;
    ~GraphOptimizeSolver();

public:
    struct  GraphInfo{
        int ref_sat[NUMSYS][NFREQ] = {{0}};
    };

    virtual bool Solving(Solution& solution, const GnssObsEpoch& obs){
        Log.Trace(TERROR, "*Solving function in GraphOptimizeSolver class overload error, please check!");
        return false;
    }

    virtual bool Solving(Solution& solution, const GnssObsEpoch& obs, const GnssObsEpoch& obs_base, GraphInfo& FgInfo){
        Log.Trace(TERROR, "*Solving function in GraphOptimizeSolver class overload error, please check!");
        return false;
    }

protected:
    struct Factor{
        int Qc;
        STimes time;
        Vector3d vel;
        Vector3d Qvel;
        map<int, GnssObsData> obs;
        map<int, GnssObsData> obs_base;
        GraphInfo info;
    };

protected:
    int mSlideWindowSize;                       /* size of slide window for optimization */
    int* mNumSat;                               /* number of used satellite, L1          */
    bool mIsVelocity;
    Vector3d mPreviousPos;                      /* position of previous epoch            */
    VectorXd mPreviousClk;                      /* rover clock bias of previous epoch    */
    Vector3d mPrioriPos;                        /* rover clock bias of previous epoch    */
    GnssObsEpoch mObs;                          /* observation of GNSS for this epoch    */
    vector<Factor> mSlideWindow;                /* window of observation for graph       */
    vector<double*> mStateArray;                /* state array of factor graph           */
    Matrix<double, Dynamic, Dynamic> mStateVar; /* covariance of position                */

    std::shared_ptr <GnssErrorModel> pEm = nullptr;  /* pointer of GNSS error model  */
    ceres::LossFunction *mLossFunction = nullptr;    /* loss function                */

protected:
    /* initialize factor graph */
    bool SetupSolverOptions(ceres::Solver::Options& options);
    bool SetupLossFunction();

    /* Prepare graph model */
    bool CheckSlideWindow();    /* check time difference in slide window   */

    /* add cost function to solver */
    virtual bool AddParameterBlocksToGraph(ceres::Problem& problem) = 0;             /* add parameter blocks as graph node      */
    virtual bool AddPseudorangeFactors(ceres::Problem& problem){                     /* add pseudorange factors to factor graph */
        Log.Trace(TERROR, "*AddPseudorangeFactors function in GraphOptimizeSolver class overload error, please check!");
        return false;
    }
    virtual bool AddDopplerFactors(ceres::Problem& problem) = 0;                    /* add Doppler factors to factor graph     */

    /* solve the factor graph */
    bool SolveGraph(ceres::Problem& problem, ceres::Solver::Options& options, ceres::Solver::Summary summary);
    bool SolveCovariance(ceres::Problem& problem, int state_num);

    bool SaveResult(Solution& solution);            /* get solution form the last pane of slide window */
    bool SaveCovariance(Solution& solution);        /* get position ovariance form convar mat          */
    void AdjustSlideWindow();                       /* remove the data outside sliding window          */

protected:

    struct DopplerFactor{
        DopplerFactor(double vx, double vy, double vz, Eigen::Vector3d var_vector)
                :vx(vx),vy(vy), vz(vz), var_vector(var_vector){}

        template <typename T>
        bool operator()(const T* const state_i, const T* const state_j, T* residuals) const
        {
            T est_vx = (state_j[0] - state_i[0]);
            T est_vy = (state_j[1] - state_i[1]);
            T est_vz = (state_j[2] - state_i[2]);

            residuals[0] = (est_vx - T(vx)) / T(var_vector(0));
            residuals[1] = (est_vy - T(vy)) / T(var_vector(1));
            residuals[2] = (est_vz - T(vz)) / T(var_vector(2));

            return true;
        }

        double vx, vy, vz;
        Vector3d var_vector;
    };

};


#endif //GRAPHOPTIMIZESOLVER_H
