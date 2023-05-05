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
 * Created by lizhen on 2021/11/9.
 *-------------------------------------------------------------------------------*/

#include "SppGraphOptimize.h"


SppGraphOptimize::SppGraphOptimize(){
    mPreviousClk = VectorXd::Zero(5);
}

bool SppGraphOptimize::Solving(Solution& solution, const GnssObsEpoch& obs){
    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    /* init factor graph */
    this->InitSolving(obs, solution, options);
    this->CheckSlideWindow();
    this->InitNewlyAddedGraph(solution);

    /* add factor to graph */
    this->AddParameterBlocksToGraph(problem);
    this->AddDopplerFactors(problem);
    this->AddPseudorangeFactors(problem);

    /* solve and position and covariance */
    this->SolveGraph(problem, options, summary);
    this->SolveCovariance(problem, SppStateSize);

    /* save result and correspond covariance */
    this->SaveResult(solution);
    this->SaveCovariance(solution);
    this->AdjustSlideWindow();

    return true;
}


bool SppGraphOptimize::InitSolving(const GnssObsEpoch& obs, const Solution& solution, ceres::Solver::Options& options) {

    this->mObs = obs;
    this->mIsVelocity = solution.vstat > 0? true: false;

    if(!pEm) pEm = std::make_shared<SppErrorModel>();

    this->SetupSolverOptions(options);
    this->SetupLossFunction();

    return true;
}


bool SppGraphOptimize::InitNewlyAddedGraph(const Solution& solution){
    int index;
    Factor fg;

    mStateArray.push_back(new double[SppStateSize]());
    index = int(mStateArray.size()) - 1;

    /* init position and receiver clock */
    for(int i = 0; i < 3; i++)
    {
        mStateArray[index][i] = 0;  //solution.pos[i];
    }
    for(int i = 3; i < 3 + NUMSYS; i++)
    {
        mStateArray[index][i] = 0;  //solution.drck[i-3];
    }

    /* velocity */
    fg.vel = solution.vel;
    fg.Qvel << solution.Qvel(0, 0), solution.Qvel(1, 1), solution.Qvel(2, 2);

    if (index > 0)
    {   /* store Previous position to prevent optimization failure */
        for (int i = 0; i < 3; i++) mPreviousPos[i] = mStateArray[index-1][i];
        for (int i = 0; i < 5; i++) mPreviousClk[i] = mStateArray[index-1][3+i];
    }

    fg.time = mObs.time;
    fg.obs = mObs.obsp;
    mSlideWindow.push_back(fg);
    mSlideWindowSize = int(mSlideWindow.size());
    return true;
}


bool SppGraphOptimize::AddParameterBlocksToGraph(ceres::Problem& problem){
    for (int i = 0; i < mSlideWindowSize; i++)
    {
        problem.AddParameterBlock(mStateArray[i], SppStateSize);
    }
    return true;
}


bool SppGraphOptimize::AddPseudorangeFactors(ceres::Problem& problem){
    int index = 0;
    double P;
    ceres::CostFunction* ps_function;

    for (auto iter = mSlideWindow.begin(); iter != mSlideWindow.end(); iter++, index++)
    {
        memset(mSysMask, 0, sizeof(mSysMask));
        for (auto ob = iter->obs.begin(); ob != iter->obs.end(); ob++)
        {
            int sys = CString::satsys(ob->first);
            auto &obsp = ob->second;

            if (!obsp.sat.vaild || !obsp.isGo) continue;

            /* Ionospheric free combination */
            if (config.Ionospheric == IONOOPT_IFLC)
            {
                if ((P = pEm->ObsCombination(obsp)) == 0) continue;
            }
            else
            {
                P = obsp.P[CState::GetFrequencyIndex(obsp.prn, 0)];
                if (P == 0) continue;
            }

            P = P - obsp.ion - obsp.trp + CLIGHT * obsp.sat.dts[0];
            Vector3d sat_pos(obsp.sat.pos[0], obsp.sat.pos[1], obsp.sat.pos[2]);
            double var = pEm->ErrorVariance(sys, PSEUDORANGE1, obsp, 0.0, SNR_UNIT * obsp.SNR[0]) +
                                            obsp.ion_var + obsp.trp_var + obsp.sat.var;

            ps_function = new ceres::AutoDiffCostFunction<PseudorangeFactor, 1, SppStateSize>
                                (new PseudorangeFactor(obsp.prn, sat_pos, P, sqrt(var)));

            problem.AddResidualBlock(ps_function, mLossFunction, mStateArray[index]);
            this->RemarkVaildSystem(obsp.prn);
        }
        this->AddConstrainFactors(problem, mStateArray[index]);  /* constraint to avoid rank-deficient */
    }

    return true;
}


bool SppGraphOptimize::AddDopplerFactors(ceres::Problem& problem){
    double vx, vy, vz;
    ceres::CostFunction* doppler_function;

    if (mSlideWindowSize < 2 || !config.VelocityMode) return false;
    for (int i = 0; i < mSlideWindowSize - 1; i++)
    {
        double dt = CTimes::timediff(mSlideWindow[i + 1].time, mSlideWindow[i].time);
        if      (config.VelocityMode == VELQ_DOPPLER)
        {
            vx = (mSlideWindow[i + 1].vel[0] + mSlideWindow[i].vel[0]) * 0.5 * dt;
            vy = (mSlideWindow[i + 1].vel[1] + mSlideWindow[i].vel[1]) * 0.5 * dt;
            vz = (mSlideWindow[i + 1].vel[2] + mSlideWindow[i].vel[2]) * 0.5 * dt;
        }
        else if (config.VelocityMode == VELQ_TDCP)
        {
            vx = mSlideWindow[i + 1].vel[0] * dt;
            vy = mSlideWindow[i + 1].vel[1] * dt;
            vz = mSlideWindow[i + 1].vel[2] * dt;
        }

        Vector3d var_vec = mSlideWindow[i].Qvel;
        if (var_vec.norm() < 1e-16) continue;

        doppler_function = new ceres::AutoDiffCostFunction<DopplerFactor, 3, SppStateSize, SppStateSize>
                (new DopplerFactor(vx, vy, vz, var_vec));
        problem.AddResidualBlock(doppler_function, mLossFunction, mStateArray[i], mStateArray[i+1]);
    }
    return true;
}


bool SppGraphOptimize::AddConstrainFactors(ceres::Problem& problem, double* x0){
    ceres::CostFunction* ps_function;

    for (int i = 0; i < NUMSYS; i++)
    {
        if (!mSysMask[i])
        {
            ps_function = new ceres::AutoDiffCostFunction<ConstrainFactor, 1, SppStateSize>
                    (new ConstrainFactor(i, 0, 0.01));
            problem.AddResidualBlock(ps_function, mLossFunction, x0);
        }
    }
    return true;
}
