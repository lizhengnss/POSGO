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
 * Created by lizhen on 2021/8/20.
 *-------------------------------------------------------------------------------*/

#include "SppPosition.h"


SppPosition::SppPosition(){

    pEx      = std::make_shared<WriteResultFile>();
    pEm      = std::make_shared<SppErrorModel>();
    pGnssObs = std::make_shared<ReadRinexObs>();
    pGnssNav = std::make_shared<ReadRinexNav>();

    if (config.Ionospheric == IONOOPT_TEC) pGnssIon = std::make_shared<ReadIonTecGrid>();

    switch (config.SolverMode)
    {
        case SOLVER_LSQ: pLsq = std::make_shared<LeastSquareSolver>(); break;
        case SOLVER_EKF: if(config.ProcessMode == PMODE_SINGLE)
                         {
                            Log.Trace(TERROR, "SppPosition: spp process cann't use lsq mode");
                            exit(-1);
                         }
                         break;
        case SOLVER_GO : pLsq = std::make_shared<LeastSquareSolver>();
                         pGo  = std::make_shared<SppGraphOptimize>();
                         break;
        default: Log.Trace(TERROR, "SppPosition: Solver mode set error " + to_string(config.SolverMode)); break;
    }

    if (config.DebugRobust) pRobLog    = std::make_shared<DebugRobust>(config.DebugFile);
    if (config.DebugResPos) pPosResLog = std::make_shared<DebugResidual>(config.DebugFile, 1);
    if (config.DebugResVel) pVelResLog = std::make_shared<DebugResidual>(config.DebugFile, 2);
}


bool SppPosition::Processing(){
    if (!this->Initialization()){
        Log.Trace(TERROR, "*SPP initialization error");
        return false;
    }

    while (true)
    {
        if (!this->DataPrepared()) break; // complete data processing of all epochs
        this->SaveLastSolution();

        this->GetAllSatPosVelClock(mObs);

        // For GO, estimated position here is used for velocimetry, and correction for ion and trop etc.
        if (!this->SolvePosition()) continue;

        mIsUseVelocity = this->VelocityProcess();

        if (config.SolverMode == SOLVER_GO) pGo->Solving(mSol, mObs);

        // write solution
        mSol.time = mObs.time;
        pEx->Export(mSol);
        Log.ShowState(CTimes::time2str(mSol.time), mSol.stat, mSol.epoch);
    }

    pEx->Close();
    return true;
}


bool SppPosition::Initialization(){

    pEm->SetEphemerisSel();

    if(!pGnssObs->OpenFile(config.GnssObs_File)) return false;
    pGnssObs->ReadHead();
    mRoverInf = pGnssObs->GetRoverInf();

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
            if(!pGnssNav->OpenFile(config.Nav_Sys_File[i])) return false;
            pGnssNav->ReadHead();  /* Read but not save */
            pGnssNav->Read();
        }
    }

    pGnssNav->UniqueNav();
    mNavs = pGnssNav->GetNavData();
    if (mNavs.n <= 0 && mNavs.ng <= 0)
    {
        Log.Trace(TERROR, "*There is no valid broadcast ephemeris, please check!");
        return false;
    }

    if (config.Ionospheric == IONOOPT_TEC)
    {
       if(!pGnssIon->OpenFile(config.Ion_File)) return false;
       mRoverInf.IonTec = pGnssIon->GetIonTecData();
       mRoverInf.nIonTec = pGnssIon->GetIonTecDataNumber();
    }

    pOrbclk = std::make_shared<NavOrbClk>(mNavs);

    this->CheckFrequency();
    return true;
}


bool SppPosition::DataPrepared(){
    if (!pGnssObs->Reading()) return false;
    mObs = pGnssObs->GetObsData();
    return true;
}


bool SppPosition::SimplePosition(GnssObsEpoch ob, GnssNavData& nav){
    Vector3d los;
    Equation eq_t;

    mObs = ob;
    mQc.clear();

    pLsq = std::make_shared<LeastSquareSolver>();
    pOrbclk = std::make_shared<NavOrbClk>(nav);

    pEm->SetEphemerisSel();
    this->GetAllSatPosVelClock(mObs);

    for (mIter = 0; mIter <= MAX_ITR; mIter++)
    {
        mPosEq.clear();
        memset(mSol.nsat, 0, sizeof(mSol.nsat));
        memset(mSysMask, 0, sizeof(mSysMask));
        mSol.blh = CEarth::ecef2blh(mSol.pos);

        for (auto iter = mObs.obsp.begin(); iter != mObs.obsp.end(); iter++) 
        {
            auto &obsp = iter->second;
            eq_t.system = CString::satsys(obsp.prn);
            eq_t.prn = obsp.prn;

            if (this->SatelliteExclude(obsp.sat)) continue;
            if ((obsp.r = this->GeoDistance(obsp.sat, mSol.pos, los)) <= 0.0) continue;
            if (this->SatAzimuthElevation(mSol.blh, los, obsp)) continue;

            /* psudorange with code bias correction */
            if (pEm->CheckSnrMask(obsp.elevation, obsp.SNR[0] * SNR_UNIT)) continue;
            if (!pEm->IonosphericCorrect(mObs.time, obsp, mSol.blh, mRoverInf)) continue;
            if (!pEm->TroposphericCorrect(mObs.time, obsp, mSol.blh)) continue;
            if (mIter == 0) pEm->CodeBiasCorrect(obsp, obsp.sat);

            double P = obsp.P[CState::GetFrequencyIndex(obsp.prn, 0)];
            if (P == 0) continue;
            eq_t.omc = P - (obsp.r + mSol.drck[0] - CLIGHT * obsp.sat.dts[0] + obsp.ion + obsp.trp);

            this->SppPositionDesignMatrix(eq_t, los);
            this->RecClockBiasCorrect(eq_t);

            if (mIter == 0) { obsp.azimuth = 0; obsp.elevation = 0;}
            eq_t.R = pEm->ErrorVariance(eq_t.system, PSEUDORANGE1, obsp, 0.0, SNR_UNIT * obsp.SNR[0]) +
                     obsp.ion_var + obsp.trp_var + obsp.sat.var;
            mPosEq.push_back(eq_t);
        }
        this->FormConstrainDesignMatrix();

        if (!this->MinNumSatellite()) return false;

        if((mSol.pdop = pLsq->Solving(mSol.pdx, mSol.Qpos, mPosEq)) == 0) return false;
        pLsq->SolutionFeedback(mSol, LSQ_SOLVE_POS);

        if     (mSol.pdx.norm() < 1E-4 )
        {
            mSol.stat = SOLQ_SINGLE;
            mSol.time = mObs.time;
            return true;
        }
    }

    return false;
}


void SppPosition::SetRoverInformation(ReceiverInfo info){
    mRoverInf = info;
}


bool SppPosition::SolvePosition(){
    mQc.clear();

    for (mIter = 0; mIter <= MAX_ITR; mIter++)
    {
        this->FormEquation();
        if (!this->MinNumSatellite()) return false;
        if (mIsUseVelocity && mSol.dt <= 2.0) this->FormVelocityConstrain();

        if (config.DebugResPos && mIter == 0) mPosEqPri = mPosEq;
        if((mSol.pdop = pLsq->Solving(mSol.pdx, mSol.Qpos, mPosEq)) == 0) return false;
        pLsq->SolutionFeedback(mSol, LSQ_SOLVE_POS);

        if (mSol.pdx.norm() < 1E-4 || mIter == MAX_ITR)
        {
            if (config.RobustMode != ROBUST_OFF)
            {
                for (mSol.nqc = 0; mSol.nqc < config.RobThres[0]; mSol.nqc++)   // iter for robust
                {
                    this->FormEquation();
                    if(!this->Robust(mPosEq) && mSol.pdx.norm() < 1E-2) break;  // when don't need check, break
                    if((mSol.pdop = pLsq->Solving(mSol.pdx, mSol.Qpos, mPosEq)) == 0) return false;
                    pLsq->SolutionFeedback(mSol, LSQ_SOLVE_POS);
                }
            }
            if (isnan(mSol.pos[0]) || mSol.nqc == config.RobThres[0]) break;
            this->FormEquation();
            mSol.Qc = this->SolutionQualityCheck(mSol, mPosEq);
            mSol.stat = SOLQ_SINGLE;

            if(config.DebugResPos) pPosResLog->TraceRes(mTime, mPosEqPri, mPosEq);
            return true;
        }
    }

    this->LoadLastSolution();  // Number of cycles exceeded
    Log.Trace(TINFO, mObs.time, "SolvePosition, Number of iterations exceeds the threshold");
    return false;
}


bool SppPosition::FormEquation(){
    double P;
    Equation eq_t;
    Vector3d PrePos =  mSol.pos;
    mSol.blh = CEarth::ecef2blh(PrePos);

    mPosEq.clear();
    memset(mSol.nsat, 0, sizeof(mSol.nsat));
    memset(mSysMask, 0, sizeof(mSysMask));

    for (auto iter = mObs.obsp.begin(); iter != mObs.obsp.end() ; iter++)
    {
        auto &obsp = iter->second;
        eq_t.prn = obsp.prn;
        eq_t.system = CString::satsys(obsp.prn);

        if (this->SatelliteExclude(obsp.sat)) continue;
        if ((obsp.r = this->GeoDistance(obsp.sat, PrePos, obsp.los)) <= 0.0) continue;
        if (this->SatAzimuthElevation(mSol.blh, obsp.los, obsp)) continue;

        if (pEm->CheckSnrMask(obsp.elevation, obsp.SNR[0] * SNR_UNIT)) continue;
        if (!pEm->IonosphericCorrect(mObs.time, obsp, mSol.blh, mRoverInf)) continue;
        if (!pEm->TroposphericCorrect(mObs.time, obsp, mSol.blh))  continue;

        /* psudorange with code bias correction */
        if (mIter == 0) pEm->CodeBiasCorrect(obsp, obsp.sat);

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

        eq_t.omc = P - (obsp.r + mSol.drck[0] - CLIGHT * obsp.sat.dts[0] + obsp.ion + obsp.trp);

        this->SppPositionDesignMatrix(eq_t, obsp.los);

        /* time system offset and receiver bias correction */
        this->RecClockBiasCorrect(eq_t);

        if (mIter == 0){ obsp.azimuth = 0; obsp.elevation = 0;}
        eq_t.R = pEm->ErrorVariance(eq_t.system, PSEUDORANGE1, obsp, 0.0, SNR_UNIT * obsp.SNR[0]) +
                 obsp.ion_var + obsp.trp_var + obsp.sat.var;
        obsp.isGo = true;
        eq_t.freq = 0;
        eq_t.el = obsp.elevation;
        eq_t.snr = obsp.SNR[0];
        mPosEq.push_back(eq_t);
    }

    for (auto iter = mQc.begin(); iter != mQc.end(); iter++)  // for robust
    {
        mPosEq[iter->index].R *= iter->fact;
    }

    this->FormConstrainDesignMatrix();
    return true;
}


bool SppPosition::FormVelocityConstrain(){
    for (int i = 0; i < CState::XnV(); i++)
    {
        Equation eq_t;
        eq_t.system = -1;
        eq_t.prn = "velocity";
        eq_t.omc = mSol.lpos(i) + mSol.vel(i) * mSol.dt - mSol.pos(i);
        eq_t.R = mSol.Qvel(i, i);
        eq_t.H.clear();
        for (int j = 0; j < 3 + 5; j++) eq_t.H.push_back(j==i? 1.0: 0.0);
        mPosEq.push_back(eq_t);
    }

    return true;
}


bool SppPosition::SppPositionDesignMatrix(Equation& eq_t, const Vector3d& e){
    eq_t.H.clear();
    for (int i = 0; i < 3 + 5; i++)
    {
        eq_t.H.push_back(i < 3? -e[i]: (i==3? 1.0: 0.0));
    }

    /* time system offset and receiver bias correction */
    if (config.BdsOption == BDSOPT_BD2_3)
    {
        if(eq_t.system == SYS_BDS && CString::str2num(eq_t.prn, 1, 2) > 18) eq_t.system = SYS_BD3;
    }

    switch (eq_t.system)
    {
        case SYS_BDS: eq_t.H[4] = 1.0; break;
        case SYS_GLO: eq_t.H[5] = 1.0; break;
        case SYS_GAL: eq_t.H[6] = 1.0; break;
        case SYS_BD3: eq_t.H[7] = 1.0; break;
        default:                       break;
    }
    return true;
}


void SppPosition::FormVarMatrix(const vector<Equation> &eq){
    int counter = 0;
    int row = int(eq.size());

    mR = MatrixXd::Zero(row, row);
    for (auto iter = eq.begin(); iter != eq.end() ; iter++, counter++) 
    {
        mR(counter, counter) = iter->R;
    }
}


bool SppPosition::RecClockBiasCorrect(Equation& eq_t) {
    switch (eq_t.system)
    {
        case SYS_BDS: eq_t.omc -= mSol.drck[1]; mSysMask[1] = 1; mSol.nsat[SYS_BDS]++; break;
        case SYS_GLO: eq_t.omc -= mSol.drck[2]; mSysMask[2] = 1; mSol.nsat[SYS_GLO]++; break;
        case SYS_GAL: eq_t.omc -= mSol.drck[3]; mSysMask[3] = 1; mSol.nsat[SYS_GAL]++; break;
        case SYS_BD3: eq_t.omc -= mSol.drck[4]; mSysMask[4] = 1; mSol.nsat[SYS_BD3]++; break;
        default: mSysMask[0] = 1; mSol.nsat[SYS_GPS]++; break;
    }
    mSol.nsat[0] += 1;
    return true;
}


bool SppPosition::FormConstrainDesignMatrix() {
    Equation eq_t;
    for (int i = 0; i < 5; i++)   // five system GPS, BD2, GLO, GAL, BD3
    {
        if (mSysMask[i]) continue;
        eq_t.system = -1;
        eq_t.prn = "constraint";
        eq_t.omc = 0.0;
        eq_t.R = 0.01;
        eq_t.H.clear();
        for (int j = 0; j < 3 + 5; j++) eq_t.H.push_back(j==i+3? 1.0: 0.0);
        mPosEq.push_back(eq_t);
    }
    return true;
}


bool SppPosition::MinNumSatellite(){
    int num = 4;
    for (int i = 1; i < 5; i++)
    {
        if (mSysMask[i]) num += 1;
    }

    if(mSol.nsat[0] < num)
    {
        Log.Trace(TINFO, mObs.time, "SPP, insufficient number of satellites, num:" + to_string(mSol.nsat[0]));
        return false;
    }

    return true;
}


void SppPosition::CheckFrequency(){
    double frequency[4][MAXFREQ]={
            {FREQ1,     FREQ2,     FREQ5,      0.0,        0.0,        0.0,   0.0},       /*GPS*/
            {FREQ1_CMP, FREQ2_CMP, FREQ3_CMP,  FREQ1,      FREQ5,      FREQ8, FREQ2_CMP}, /*BDS*/
            {FREQ1_GLO, FREQ2_GLO, FREQ3_GLO,  FREQ1a_GLO, FREQ2a_GLO, 0.0,   0.0},       /*GLO*/
            {FREQ1,     FREQ7,     FREQ5,      FREQ6,      FREQ8,      0.0,   0.0},       /*GAL*/
    };

    if (config.NumFreq < 2) return;
    for (int f = 0; f < MIN(config.GPS_FreqID.size(), 2); f++)
    {
        if (f != config.GPS_FreqID[f])
        {
            config.GPS_FreqID[f] = f;
            config.GPS_Freq[f] = frequency[SYS_GPS-1][config.GPS_FreqID[f]];
            Log.Trace(TINFO, "GPS frequency set as default: L1+L2");
        }
    }

    for (int f = 0; f < MIN(config.BD3_FreqID.size(), 2); f++)
    {   // bd2 not required check, all frequency supported
        if (config.BD3_FreqID[f] != 0 && config.BD3_FreqID[f] != 2)
        {
            config.BD3_FreqID[0] = 0;
            config.BD3_Freq[0] = frequency[SYS_BDS-1][config.BD3_FreqID[0]];
            if (config.BD3_FreqID.size() >= 2)
            {
                config.BD3_FreqID[1] = 2;
                config.BD3_Freq[1] = frequency[SYS_BDS-1][config.BD3_FreqID[1]];
            }
            Log.Trace(TINFO, "BD3 frequency set as default: B1I+B3I");
        }
    }  // B1I and B3I for BD3

    for (int f = 0; f < MIN(config.GLO_FreqID.size(), 2); f++)
    {
        if (f != config.GLO_FreqID[f])
        {
            config.GLO_FreqID[f] = f;
            config.GLO_Freq[f] = frequency[SYS_GLO-1][config.GLO_FreqID[f]];
            Log.Trace(TINFO, "GLONASS frequency set as default: G1+G2");
        }
    }

    for (int f = 0; f < MIN(config.GAL_FreqID.size(), 2); f++)
    {
        if (f != config.GAL_FreqID[f])
        {
            if (f == 1 && config.GAL_FreqID[f] == 2) continue;  // E5a
            config.GAL_FreqID[f] = f;
            config.GAL_Freq[f] = frequency[SYS_GAL-1][config.GAL_FreqID[f]];
            Log.Trace(TINFO, "GALILEO frequency set as default: E1+E5b");
        }
    }
}
