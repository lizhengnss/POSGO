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
 * Created by lizhen on 2021/9/22.
 *-------------------------------------------------------------------------------*/

#include "RtkPosition.h"


RtkPosition::RtkPosition(){

    pSpp         = std::make_shared<SppPosition>();
    pEm          = std::make_shared<RtkErrorModel>();
    pGnssObs     = std::make_shared<ReadRinexObs>();
    pGnssObsBase = std::make_shared<ReadRinexObs>();  // base
    pGnssNav     = std::make_shared<ReadRinexNav>();

    mXf = MatrixXd::Zero(CState::XnX(), 1);
    mPf = MatrixXd::Zero(CState::XnX(), CState::XnX());

    mXvf = MatrixXd::Zero(CState::XnGnssVel(), 1);
    mPvf = MatrixXd::Zero(CState::XnGnssVel(), CState::XnGnssVel());

    switch (config.SolverMode)
    {
        case SOLVER_LSQ: Log.Trace(TERROR, "RtkPosition: rtk process cann't use lsq mode");
                         exit(-1);
        case SOLVER_EKF: pEkf    = std::make_shared<KalmanFilterSolver>();
                         pEkfVel = std::make_shared<KalmanFilterSolver>();
                         break;
        case SOLVER_GO : pLsq = std::make_shared<LeastSquareSolver>();
                         pGo  = std::make_shared<RtkGraphOptimize>();
                         break;
        default: Log.Trace(TERROR, "RtkPosition: Solver mode set error"); break;
    }

    if (config.SolveType == MODE_SMOOTH)
    {   // output result to temp bin file
        pEx = std::make_shared<WriteTempResult>();
    }
    else
    {   // output result to file
        pEx = std::make_shared<WriteResultFile>();
    }

    if (config.DebugRobust) pRobLog    = std::make_shared<DebugRobust>(config.DebugFile);
    if (config.DebugResPos) pPosResLog = std::make_shared<DebugResidual>(config.DebugFile, 1);
    if (config.DebugResVel) pVelResLog = std::make_shared<DebugResidual>(config.DebugFile, 2);
}


RtkPosition::~RtkPosition(){

    if (config.SolveType == MODE_BACKWARD)
    {
        mObss.clear();
        mObssBase.clear();
    }

}


bool RtkPosition::Processing(){
    if (!this->Initialization())
    {
        Log.Trace(TERROR, "*Rtk initialization error!");
        return false;
    }

    while (true)
    {
        if (!this->DataPrepared()) break; // complete data processing of all epochs
        this->SaveLastSolution();

        if (!mIsInitialize)
        {
            if(!this->RtkInitialize()) continue;
        }

        /* compute satellite positions, velocities and clocks */
        this->GetAllSatPosVelClock(mObs);
        this->GetAllSatPosVelClock(mObsBase);

        if     (config.SolverMode == SOLVER_EKF)
        {
            if (!this->FilterPosition()) continue;
            this->VelocityProcess();
        }
        else if(config.SolverMode == SOLVER_GO)
        {
            if (!this->GraphPosition()) continue;
        }

        // write solution
        this->SaveSolutionState();
        pEx->Export(mSol);
    }

    pEx->Close();
    return true;
}


bool RtkPosition::GraphPosition(){

    if (!pSpp->SimplePosition(mObs, mNavs)) return false;
    pSpp->GetSolution(mSol);

    for (auto iter = mObs.obsp.begin(); iter != mObs.obsp.end() ; iter++)
    {   // calculate the elevation angle distance between satellite and rover
        auto &obsp = iter->second;
        obsp.r = this->GeoDistance(obsp.sat, mSol.pos, obsp.los);
        this->SatAzimuthElevation(mSol.blh, obsp.los, obsp);
    }

    for (int m = 0; m < NUMSYS; m++)
    {  /* find reference satellite with highest elevation, set to ref_sat[sys][frq] */
       this->SelectRefSatellite(m);
    }

    this->VelocityProcess();

    memcpy(mFgInfo.ref_sat, mRefSat, sizeof(mRefSat));

    pGo->Solving(mSol, mObs, mObsBase, mFgInfo);

    mSol.stat = SOLQ_FLOAT;
    return true;
}


bool RtkPosition::Initialization(){

    pEm->SetEphemerisSel();

    if(!pGnssObs->OpenFile(config.GnssObs_File)) return false;
    pGnssObs->ReadHead();
    mRoverInf = pGnssObs->GetRoverInf();

    if(!pGnssObsBase->OpenFile(config.GnssObs_Base_File)) return false;
    pGnssObsBase->ReadHead();
    mBaseInf = pGnssObsBase->GetRoverInf();

    if (!config.Nav_File.empty())
    {   /* read mixed navigation file if exist */
        if(!pGnssNav->OpenFile(config.Nav_File)) return false;
        mRoverInf.nav_head = pGnssNav->GetNavHead();
        pGnssNav->Read();
    }

    for (int i = 0; i < 4; i++)
    {   /* each system navigation file if exist */
        if (!config.Nav_Sys_File[i].empty())
        {
            pGnssNav->CloseFile();
            if(!pGnssNav->OpenFile(config.Nav_Sys_File[i])) return false;
            pGnssNav->ReadHead();  /* Read but not save */
            pGnssNav->Read();
        }
    }

    pGnssNav->UniqueNav();
    mNavs = pGnssNav->GetNavData();
    pOrbclk = std::make_shared<NavOrbClk>(mNavs);
    pSpp->SetRoverInformation(mRoverInf);

 //   if (!this->CheckBaseCoord()) return false;

    if (mSolveType == MODE_BACKWARD)
    {
        if (!this->ReadAllObservation(pGnssObs, mObss))         return false;
        if (!this->ReadAllObservation(pGnssObsBase, mObssBase)) return false;
        mRoverId = int(mObss.size()) - 1;
        mBaseId  = int(mObssBase.size()) - 1;
        mObsBase = mObssBase[mBaseId];
    }

    mSol.drcd = 1e-16;

    return true;
}


bool RtkPosition::DataPrepared(){
    if (!this->MatchBaseRoverData()) return false;  // if complete data processing of all epochs, break

    return true;
}


bool RtkPosition::FilterPosition(){
    Solution sol_t;  //

    /* calculate omc for base station (phase and code) */
    if(!this->CalZeroDiffResiduals(mObsBase, config.BasePos, mOmcBase, mBaseInf))
    {
        Log.Trace(TERROR, mObs.time, " initial base station position error");
        return false;
    }

    /* select common satellites between rover and base-station */
    if((mObsNum = this->SelectCommonSatellite()) <= 0)
    {
        Log.Trace(TINFO, mObs.time, " no common satellite");
        return false;
    }

    if (!this->UpdateStates()) return false;
    sol_t = mSol;
    mXt = mXf;
    mPt = mPf;
    mQc.clear();

    for (mIterQc = 0; mIterQc <= (config.RobustMode?config.RobThres[0]:1); mIterQc++)
    {
        mPrePos = mXf.block(CState::XiP(), 0, CState::XnP(), 1);

        /* calculate zero diff residuals [range - measured pseudorange] for rover (phase and code) */
        if (!this->CalZeroDiffResiduals(mObs, mPrePos, mOmc, mRoverInf))
        {
            Log.Trace(TINFO, mObs.time, " rover initial position error");
            mSol.stat = SOLQ_NONE;
            break;
        }

        /* calculate double-differenced residuals and create state matrix from sat angles  */
        if ( this->CalDoubleDiffResiduals() < 1)
        {
            Log.Trace(TINFO, mObs.time, " no DD residual to measurement update");
            mSol.stat = SOLQ_NONE;
            break;
        }

        if (config.DebugResPos && mIterQc == 0) mPosEqPri = mPosEq;
        for (auto it = mQc.begin(); it != mQc.end(); it++)  // for robust
        {
            mPosEq[it->index].R *= it->fact;
        }

        this->FormVarMatrix(mPosEq);

        if (!pEkf->Solving(mPosEq, mSol, mXf, mPf, mR))
        {
            Log.Trace(TINFO, mObs.time, " filter error");
            mSol.stat = SOLQ_NONE;
            break;
        }

        mPostPos = mXf.block(CState::XiP(), 0, CState::XnP(), 1);

        /* robust algorithm-----------------------------------------------*/
        this->CalZeroDiffResiduals(mObs, mPostPos, mOmc, mRoverInf);
        this->CalDoubleDiffResiduals();

        if (!this->Robust(mPosEq) || mIterQc == config.RobThres[0]) break;
        mSol = sol_t;
        mXf = mXt;
        mPf = mPt;
    }

    mPostPos = mXf.block(CState::XiP(), 0, CState::XnP(), 1);

    /* post residual test --------------------------------------------*/
    if (mSol.stat != SOLQ_NONE && this->CalZeroDiffResiduals(mObs, mPostPos, mOmc, mRoverInf))
    {
        /* calc double diff residuals again after kalman filter update for float solution */
        this->CalDoubleDiffResiduals();
        /* update valid satellite status for ambiguity control */
        memset(mSol.nsat, 0, sizeof(mSol.nsat));
        for (auto it = mObs.obsp.begin(); it != mObs.obsp.end(); it++)
        {
            for (int f = 0; f < config.NumFreq; f++)
            {
                if (!mVsat[it->first - 1][f]) continue;
                if (f==0) mSol.nsat[0]++;     /* valid satellite count by L1 */
            }
        }
        mSol.Qc = this->SolutionQualityCheck(mSol, mPosEq);
    }

    for (auto it = mQc.begin(); it != mQc.end(); it++)
    {   //todo check, for robust log export
        mPosEq[it->index].R *= it->fact;
    }
    if(config.DebugResPos) pPosResLog->TraceRes(mTime, mPosEqPri, mPosEq, mRefSat);

    return true;
}


bool RtkPosition::RtkInitialize(){

    if (!pSpp->SimplePosition(mObs, mNavs)) return false;
    pSpp->GetSolution(mSol);
    for(int i = 0; i < 3; i++) mSol.vel[i] = 1e-6;

    if (config.SolverMode == SOLVER_EKF)
    {
        pEkf->InitGnssFilter(mSol, mXf, mPf);
        if (config.VelocityMode > VELOPT_OFF) pEkfVel->InitVelFilter(mSol, mXvf, mPvf);
    }

    mSol.stat  = SOLQ_FLOAT;
    mSol.vstat = VELQ_NONE;
    mSol.dt = 0.0;

    mIsInitialize = true;
    return true;
}


bool RtkPosition::CheckBaseCoord(){
    Solution sol_base;

    auto spp_base = std::make_shared<SppPosition>();

    while (true)
    {
        if (!pGnssObsBase->Reading())
        {
            Log.Trace(TERROR, "*No base station observation, please check!");
            return false;
        }

        mObsBase = pGnssObsBase->GetObsData();

        this->GetAllSatPosVelClock(mObsBase);

        if (!spp_base->SimplePosition(mObsBase, mNavs))
        {
            Log.Trace(TERROR, "*Base station LSQ solution failed, please check!");
            continue;
        }
        break;  // lsq sucess
    }

    spp_base->GetSolution(sol_base);
    if ((sol_base.pos - config.BasePos).norm() > 500)
    {
        Log.Trace(TERROR, "*Coordinates of the base station may incorrectly, please check!");
        return false;
    }

    return true;
}


bool RtkPosition::MatchBaseRoverData() {
    double age;

    if      (mSolveType == MODE_FORWARD)
    {
        if (!pGnssObs->Reading()) return false;
        mObs = pGnssObs->GetObsData();

        while (true)    // match
        {
            age = CTimes::timediff(mObs.time, mObsBase.time);
            if      (age <= -DTAGE) // base is too later than rover, read rover
            {
                if (!pGnssObs->Reading()) break;
                mObs = pGnssObs->GetObsData();
                mObsBase = mObsBase_t;
                continue;
            }
            else if (age >=  DTAGE) // rover is too later than base, read base
            {
                if (!pGnssObsBase->Reading()) break;
                mObsBase = pGnssObsBase->GetObsData();
                mObsBase_t = mObsBase;
                continue;
            }
            else
            {
                mSol.age  = age;
                mObsBase = mObsBase_t;
                return true;
            }
        }
    }
    else if (mSolveType == MODE_BACKWARD)
    {
        if (mRoverId < 0) return false;
        mObs = mObss[mRoverId--];

        while (true)    // match
        {
            age = CTimes::timediff(mObs.time, mObsBase.time);
            if (age <= -DTAGE)  // base is too later than rover, backward base
            {
                if (mBaseId < 0) break;
                mObsBase = mObssBase[mBaseId--];
                continue;
            }
            else if (age >= DTAGE) // rover is too later than base, backward rover
            {
                if (mRoverId < 0) break;
                mObs = mObss[mRoverId--];
                continue;
            }
            else
            {
                mSol.age  = age;
                //obs_base = obs_bt;
                return true;
            }
        }
    }

    Log.Trace(TERROR, "MatchBaseRoverData, cann't match proper observation with base station! ");
    return false;
}


int RtkPosition::SelectCommonSatellite() {
    for (auto iter = mObsBase.obsp.begin(); iter != mObsBase.obsp.end();) // will check base elevation
    {
        if        (mObs.obsp.find(iter->first) == mObs.obsp.end())   // no match
        {
            Log.Trace(TEXPORT, "SelectCommonSatellite, Delete base station observation due to "
                                   "lock common satellite in rover, Sat=" + iter->second.prn);
            mObsBase.obsp.erase(iter++);
        }
        else if (mObs.obsp.find(iter->first) != mObs.obsp.end() && iter->second.elevation < config.Elevation)
        {
            Log.Trace(TEXPORT, "SelectCommonSatellite, Delete base station observation due to "
                                  "over elevation, Sat=" + iter->second.prn);
            mObsBase.obsp.erase(iter++);
        }
        else
        {
            iter++;
        }
    }

    for (auto iter = mObs.obsp.begin(); iter != mObs.obsp.end();)
    {
        if (mObsBase.obsp.find(iter->first) == mObsBase.obsp.end())  // no match
        {
            Log.Trace(TEXPORT, "SelectCommonSatellite, Delete rover station observation due to "
                                  "lock common satellite in base, Sat=" + iter->second.prn);
            mObs.obsp.erase(iter++);
        }
        else
        {
            iter++;
        }
    }
    return int(mObs.obsp.size());
}


bool RtkPosition::SelectRefSatellite(int sys){
    int sysi;
    double el[3] = {0.0};

    for (int f = 0; f < config.NumFreq; f++)
    {
        mRefSat[sys][f] = -1; //mObs.obsp.begin()->first;
        for (auto it = mObs.obsp.begin(); it != mObs.obsp.end(); it++)
        {
            auto &obsp = it->second;
            sysi = CString::satsys(obsp.prn);
            if (sysi == SYS_BDS && config.BdsOption == BDSOPT_BD2_3) sysi = CString::checkbd23(obsp.prn);
            if (!CString::testsystem(sysi, sys)) continue;
            if (!this->VaildSatellite(it->first, f)) continue;
            if (obsp.elevation >= el[f])
            {
                el[f] = obsp.elevation;
                mRefSat[sys][f] = it->first;
            }
        }
    }

    if (el[0] <= 0) return false;
    return true;
}


bool RtkPosition::VaildSatellite(int id, int frq){
    string prn = CString::satid2prn(id);
    int frqid = CState::GetFrequencyIndex(prn, frq);

    if (config.SolverMode == SOLVER_EKF)
    {
        return abs(mObs.obsp[id].L[frqid]) > 1e-3 && abs(mObsBase.obsp[id].L[frqid]) > 1e-3 &&
                    mOmc [id].L[frq] != 0 && mOmc [id].P[frq] != 0 &&
                    mOmcBase[id].L[frq] != 0 && mOmcBase[id].P[frq] != 0;
    }
    else
    {   // GO
        return abs(mObs.obsp[id].P[frqid]) > 1e-3 && abs(mObsBase.obsp[id].P[frqid]) > 1e-3;
    }
}


bool RtkPosition::CalZeroDiffResiduals(GnssObsEpoch &ob, const Vector3d& pos, map<int, omcinfo> &omc, const ReceiverInfo& head) {
    omc.clear();
    if (pos.norm() <= 0) return false; /* no receiver position */

    /* todo adjust rcvr pos for earth tide correction */
    this->CalGeometricRange(ob, pos, head);
    this->CalUndifferResidual(ob, omc);
    return true;
}


int RtkPosition::CalDoubleDiffResiduals(){
    Equation eq_t;

    mPosEq.clear();
    mGroup = 0;
    for (int f = 0; f < NUMSYS*NFREQ*2; f++) mPair[f] = 0;

    /* bl=distance from base to rover, dr=x,y,z components */
    mBaseline = CMaths::distance(mSol.pos, config.BasePos);

    /* step through sat systems: m=0:gps,1:bd2,2:glo,3:gal 4:bd3*/
    for (int m = 0; m < NUMSYS; m++)
    {
        /* find reference satellite with highest elevation, set to ref_sat[sys][frq] */
        if (!this->SelectRefSatellite(m)) continue;

        /* calculate double differences of residuals (code) for each sat */
        this->CalCodeResiduals(m);
    }

    return int(mPosEq.size());
}


void RtkPosition::CalGeometricRange(GnssObsEpoch &ob, const Vector3d& pos, const ReceiverInfo& head) {

    Vector3d blh = CEarth::ecef2blh(pos); /* translate pos from ecef to geodetic */

    for (auto iter = ob.obsp.begin(); iter != ob.obsp.end() ; iter++)
    {
        auto &obsp = iter->second;

        if (this->SatelliteExclude(obsp.sat)) continue;

         /* compute geometric-range and azimuth/elevation angle */
        if ((obsp.r = this->GeoDistance(obsp.sat, pos, obsp.los)) <= 0.0) continue;
        if (this->SatAzimuthElevation(blh, obsp.los, obsp)) continue;

        /* adjust range for satellite clock-bias */
        obsp.r -= CLIGHT * obsp.sat.dts[0];

        /* adjust range for troposphere delay model (hydrostatic) */
        if (!pEm->TroposphericCorrect(mTime, obsp, mSol.blh)) continue;
        obsp.r += obsp.trp;
        /* todo receiver antenna phase center correction */

    }
}


void RtkPosition:: CalUndifferResidual(GnssObsEpoch &ob, map<int, omcinfo> &omc){
    int freqid;
    double freq;
    omcinfo omc_t;

    for (auto iter = ob.obsp.begin(); iter != ob.obsp.end() ; iter++)
    {
        auto &obsp = iter->second;
        if (!obsp.sat.vaild) continue;
        for (int f = 0; f < config.NumFreq; f++)
        {
            freqid = CState::GetFrequencyIndex(obsp.sat.prn, f);
            freq =  CState::GetFrequency(obsp.sat.prn, f, obsp.sat.glofcn);  // mObs.sat.glofcn ??

            if (pEm->CheckSnrMask(obsp.elevation, obsp.SNR[freqid] * SNR_UNIT)) continue;  /* check snr */

            /* residuals = observable - pseudorange */
            if (obsp.L[freqid] <= 1e-3 || obsp.r <= 1e-3) omc_t.L[f] = 0.0;
            else omc_t.L[f] = obsp.L[freqid] * CLIGHT / freq - obsp.r;

            if (obsp.P[freqid] <= 1e-3 || obsp.r <= 1e-3) omc_t.P[f] = 0.0;
            else omc_t.P[f] = obsp.P[freqid] - obsp.r;
        }
        omc.insert(pair<int, omcinfo>(iter->first, omc_t));
    }
}


void RtkPosition::CalCodeResiduals(int sys){
    Vector3d e;
    Equation eq_t;

    for (int f = 0; f < config.NumFreq; f++)
    {
        for (auto it = mObs.obsp.begin(); it != mObs.obsp.end(); it++)
        {
            auto &obsp = it->second;
            if (it->first == mRefSat[sys][f]) continue;  /* skip ref sat */

            int sysj = CString::satsys(obsp.prn);
            if (sysj == SYS_BDS && config.BdsOption == BDSOPT_BD2_3) sysj = CString::checkbd23(obsp.prn);
            if (!CString::testsystem(sysj, sys)) continue;
            if (!this->VaildSatellite(it->first, f)) continue;
            eq_t.prn = obsp.prn;
            eq_t.system = sysj;

            /* partial derivatives by rover position, combine unit vectors from two sats */
            for (int i = 0; i < 3; i++) e[i] = mObs.obsp[mRefSat[sys][f]].los[i] - obsp.los[i];

            /* double-differenced measurements from 2 receivers and 2 sats in meters */
            eq_t.omc = (mOmc[mRefSat[sys][f]].P[f] - mOmcBase[mRefSat[sys][f]].P[f]) -
                       (mOmc[it->first].P[f]       - mOmcBase[it->first].P[f]);

            eq_t.Rb = pEm->ErrorVariance(eq_t.system, f + 1, mObs.obsp[mRefSat[sys][f]], mBaseline,
                                         SNR_UNIT * mObs.obsp[mRefSat[sys][f]].SNR[f]);
            eq_t.R  = pEm->ErrorVariance(eq_t.system, f + 1, obsp, mBaseline,
                                         SNR_UNIT*obsp.SNR[f]);
            this->FormPositionDesignMatrix(eq_t, e, -1, -1, -1, -1, -1);
            eq_t.freq = f;
            eq_t.el = obsp.elevation;
            eq_t.snr = obsp.SNR[CState::GetFrequencyIndex(obsp.sat.prn, f)];

            /* if residual too large, flag as outlier */
            if (this->CheckMaxResidual(it->first, f, eq_t.omc, true, mPf))
            {
                mPosEq.push_back(eq_t);
                mVsat[it->first - 1][f] = mVsat[mRefSat[sys][f] - 1][f] = true;
                mPair[mGroup]++;
            }

        }
        mGroup++;
    }
}


bool RtkPosition::FormPositionDesignMatrix(Equation& eq_t, const Vector3d& e, int ambR, int ambB,
                                                    int frq, double lami, double lamj){
    eq_t.H.clear();
    eq_t.H.resize(CState::XnGnss(), 0.0);

    for (int i = 0; i < CState::XnP(); i++)         /* position */
    {
        eq_t.H[CState::XiP() + i] = -e[i];
    }

    return true;
}


void RtkPosition::FormVarMatrix(const vector<Equation> &eq){
    int k = 0;
    int row = int(eq.size());

    mR = MatrixXd::Zero(row, row);

    for (int b = 0; b < mGroup; k += mPair[b++]) /* loop through each system */
    {
        for (int i = 0; i < mPair[b]; i++) for (int j = 0; j < mPair[b]; j++)
        {
            mR(k + i, k + j) = eq[k + i].Rb + (i == j ? eq[k + i].R : 0.0);  //todo
        }
    }
}


bool RtkPosition::UpdateStates(){

    if(!this->UpdatePosition()) // large position variance
    {
        if (!this->RtkInitialize())
        {
            mIsInitialize = false;
            return false;
        }
    }

    Log.Trace(TEXPORT, "Timeupdate completed, Pos: " + to_string(mXf(CState::XiP(), 0)) + " " +
                       to_string(mXf(CState::XiP() + 1, 0)) + " " + to_string(mXf(CState::XiP() + 2, 0)));

    return true;

}

