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

#include "RtkGraphOptimize.h"


bool RtkGraphOptimize::Solving(Solution& solution, const GnssObsEpoch& obs, const GnssObsEpoch& obs_base, GraphInfo& FgInfo){
    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    /* init factor graph */
    this->InitSolving(obs, obs_base, solution, options);
    this->CheckSlideWindow();
    this->InitNewlyAddedGraph(solution, FgInfo);

    /* add factor to graph */
    this->AddParameterBlocksToGraph(problem);
    this->AddDDPseudorangeFactors(problem);
    this->AddDopplerFactors(problem);

    /* solve and position and covariance */
    if (mPsrNum <= 3)
    {
        Log.Trace(TERROR, obs.time, "GO error, insufficient number of pseudorange:" + to_string(mPsrNum));
        return false;
    }

    this->SolveGraph(problem, options, summary);
    this->SolveCovariance(problem, RtkStateSize);

    /* save result and correspond covariance */
    this->SaveResult(solution);
    this->SaveCovariance(solution);
    this->AdjustSlideWindow();

    return true;
}


bool RtkGraphOptimize::InitSolving(const GnssObsEpoch& obs, const GnssObsEpoch& obs_base, const Solution& solution, ceres::Solver::Options& options) {

    mPsrNum = 0;
    mIsVelocity = solution.vstat > 0? true: false;

    this->mObs = obs;
    this->mObsBase = obs_base;
    if(!pEm) pEm = std::make_shared<RtkErrorModel>();

    this->SetupSolverOptions(options);
    this->SetupLossFunction();

    return true;
}


bool RtkGraphOptimize::InitNewlyAddedGraph(Solution& solution, GraphInfo& FgInfo){
    int index;
    Factor fg;

    mStateArray.emplace_back(new double[RtkStateSize]());
    index = int(mStateArray.size()) - 1;

    /* store Previous position to prevent optimization failure */
    for(int i = 0; i < 3; i++)
    {
        if (index == 0) break;
        mPreviousPos[i] = mStateArray[index-1][i];
    }

    /* velocity */
    fg.vel = solution.vel;
    fg.Qvel << solution.Qvel(0, 0), solution.Qvel(1, 1), solution.Qvel(2, 2);

    /* init using previous position and velocity */
    for(int i = 0; i < 3; i++)
    {
        mStateArray[index][i] = mPreviousPos[i] + solution.vel[i];
        mPrioriPos[i] = mStateArray[index][i];
    }

    memset(solution.nsat, 0, sizeof(solution.nsat));
    mNumSat = solution.nsat;

    fg.time = mObs.time;
    fg.obs = mObs.obsp;
    fg.obs_base = mObsBase.obsp;
    fg.info = FgInfo;
    mSlideWindow.push_back(fg);
    mSlideWindowSize = int(mSlideWindow.size());
    return true;
}


bool RtkGraphOptimize::AddParameterBlocksToGraph(ceres::Problem& problem){
    for (int i = 0; i < mSlideWindowSize; i++)
    {   /* add parameter block for position state (ECEF_x, ECEF_y, ECEF_z) */
        problem.AddParameterBlock(mStateArray[i], RtkStateSize);
    }
    return true;
}


bool RtkGraphOptimize::AddDDPseudorangeFactors(ceres::Problem& problem){
    int index = 0, sys, freqid, ref_sat;
    ceres::CostFunction* ps_function;

    for (auto it = mSlideWindow.begin(); it != mSlideWindow.end(); it++, index++)
    {
        auto &obsp_base = mSlideWindow[index].obs_base;
        Vector3d pos(mStateArray[index][0], mStateArray[index][1], mStateArray[index][2]);
        double baseline = CMaths::distance(pos, config.BasePos);
        for (int system = 0; system < NUMSYS; system++)
        {
            for (int f = 0; f < config.NumFreq; f++)
            {
                ref_sat = it->info.ref_sat[system][f];
                for (auto ob = it->obs.begin(); ob != it->obs.end(); ob++)
                {
                    auto &obsp = ob->second;
                    sys = CString::satsys(obsp.prn);
                    if (sys == SYS_BDS && config.BdsOption == BDSOPT_BD2_3) sys = CString::checkbd23(obsp.prn);
                    freqid = CState::GetFrequencyIndex(obsp.sat.prn, f);

                    if (!CString::testsystem(sys, system)) continue;                                    /* check system       */
                    if (!obsp.sat.vaild || obsp.r <= 0 || obsp.elevation < config.Elevation) continue;  /* check elevation    */
                    if (pEm->CheckSnrMask(obsp.elevation, obsp.SNR[freqid] * SNR_UNIT)) continue;       /* check snr          */
                    if (ob->first == ref_sat) continue;                                                 /* skip ref sat       */

                    if (obsp.P[freqid]                 <= 0) continue;                                  /* check obs of rover */
                    if (obsp_base[ob->first].P[freqid] <= 0) continue;                                  /* check obs of base  */

                    double var1 = pEm->ErrorVariance(sys, f + 1, obsp, baseline, SNR_UNIT*obsp.SNR[f]);                           // R
                    double var2 = pEm->ErrorVariance(sys, f + 1, it->obs[ref_sat], baseline, SNR_UNIT * it->obs[ref_sat].SNR[f]); //Rb

                    DDObservation Pseudorange;
                    Pseudorange.Sat_Pos      << obsp.sat.pos[0],                 obsp.sat.pos[1],                 obsp.sat.pos[2];
                    Pseudorange.Ref_Pos      << mObs.obsp[ref_sat].sat.pos[0],   mObs.obsp[ref_sat].sat.pos[1],   mObs.obsp[ref_sat].sat.pos[2];
                    Pseudorange.Sat_Base_Pos << obsp_base[ob->first].sat.pos[0], obsp_base[ob->first].sat.pos[1], obsp_base[ob->first].sat.pos[2];
                    Pseudorange.Ref_Base_Pos << obsp_base[ref_sat].sat.pos[0],   obsp_base[ref_sat].sat.pos[1],   obsp_base[ref_sat].sat.pos[2];

                    Pseudorange.DD_Sat      = obsp.P[freqid];
                    Pseudorange.DD_Ref      = mObs.obsp[ref_sat].P[freqid];
                    Pseudorange.DD_Sat_Base = obsp_base[ob->first].P[freqid];
                    Pseudorange.DD_Ref_Base = obsp_base[ref_sat].P[freqid];

                    Pseudorange.DDVar = sqrt(var1 + var2);

                  //  if (Pseudorange.DDVar < 1e-6) continue;
                    mPsrNum += 1;
                    if (f == 0 && index == mSlideWindowSize-1) mNumSat[0]++;    /* valid satellite count by L1 */

                    ps_function = new ceres::AutoDiffCostFunction<DDPseudorangeFactor, 1, RtkStateSize> (new DDPseudorangeFactor(Pseudorange, mBasePose));
                    problem.AddResidualBlock(ps_function, mLossFunction, mStateArray[index]);
                }
            }
        }
    }

    return true;
}


bool RtkGraphOptimize::AddDopplerFactors(ceres::Problem& problem){
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

        doppler_function = new ceres::AutoDiffCostFunction<DopplerFactor, 3, RtkStateSize, RtkStateSize>
                (new DopplerFactor(vx, vy, vz, var_vec));
        problem.AddResidualBlock(doppler_function, mLossFunction, mStateArray[i], mStateArray[i+1]);
    }
    return true;
}

