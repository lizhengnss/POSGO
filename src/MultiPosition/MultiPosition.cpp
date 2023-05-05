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
 * Created by lizhen on 2021/7/21.
 *-------------------------------------------------------------------------------*/

#include "MultiPosition.h"


bool MultiPosition::UpdatePosition(){
    int mPartCol = 0;
    double var = 0;
    vector<int> mValueId;   /* list of non-zero states  */

    /* check variance of estimated position */
    for (int i = 0; i < 3; i++)
    {
        var += mPf(CState::XiP()+i, CState::XiP()+i);
    }
    var /= 3.0;

    if (config.ProcessMode == PMODE_RTK && var > VAR_POS)
    {   /* reset position with large variance */
        Log.Trace(TINFO, "reset rtk position due to large variance: var=" + to_string(var));
        return false;
    }

    if (config.VelocityMode > VELOPT_OFF)
    {
        for (int i = 0; i < CState::XnV(); i++)
        {
            mXf(CState::XiV()  + i) = mXvf(i);
            mXf(CState::XiAc() + i) = mXvf(3+i);
        }
        mPf.block<6, 6>(CState::XiV(), CState::XiV()) = mPvf.block<6, 6>(0, 0);
    }

    /* generate valid state index */
    for (int i = 0; i < CState::XnX(); i++)
    {
        if (i<9 || (mXf(i, 0)!=0.0 && mPf(i, i)>0.0))
        {
            mValueId.push_back(i);
            mPartCol++;
        }
    }

    MatrixXd Fk = MatrixXd::Identity(mPartCol, mPartCol);
    MatrixXd X = MatrixXd::Zero(mPartCol, 1);
    MatrixXd P = MatrixXd::Zero(mPartCol, mPartCol);

    /* compress array by removing zero elements to save computation time */
    for (int i = 0; i < mPartCol; i++)
    {
        X(i, 0) = mXf(mValueId[i], 0);
        for (int j = 0; j < mPartCol; j++)
        {
            P(i, j) = mPf(mValueId[i], mValueId[j]);
        }

    }

    /* state transition of position/velocity */
    for (int i = 0; i < CState::XnP(); i++)
    {
        Fk(i, i + CState::XiV())  = fabs(mSol.dt);
        Fk(i, i + CState::XiAc()) = 0.5 * SQR(mSol.dt);
        Fk(i + CState::XiV(), i + CState::XiAc()) = fabs(mSol.dt);
    }

    /* x=F*x, P=F*P*F+Q */
    X = Fk * X;
    P = Fk * P * Fk.transpose();

    /* copy values from compressed arrays back to full arrays */
    for (int i = 0; i < mPartCol; i++)
    {
        mXf(mValueId[i], 0) = X(i, 0);
        for (int j = 0; j < mPartCol; j++)
        {
            mPf(mValueId[i], mValueId[j]) = P(i, j);
        }
    }

    // process add to acceleration only
    Matrix3d Q = Matrix3d::Zero(3, 3);
    Matrix3d E = CEarth::xyz2enu(mSol.blh);
    Q << SQR(config.Process[3])*fabs(mSol.dt), 0.0,                                  0.0,
         0.0,                                  SQR(config.Process[3])*fabs(mSol.dt), 0.0,
         0.0,                                  0.0,                                  SQR(config.Process[4])*fabs(mSol.dt);
    Q = E.transpose() * Q * E;
    mPf.block<3, 3>(CState::XiAc(), CState::XiAc()) += Q;

    return true;
}


bool MultiPosition::UpdataReceiverClock(){
    for (int i = 0; i < CState::XnClk(); i++)  // recever clock
    {
        pEkf->initx(mSol.drck[i], VAR_CLK, CState::XiClk() + i, mXf, mPf);
    }

    for (int i = 0; i < CState::XnCdr(); i++)  // recever clock drift
    {
        pEkf->initx(mSol.drcd, VAR_CDR, CState::XiCdr() + i, mXf, mPf);
    }
    return true;
}


void MultiPosition::GetAllSatPosVelClock(GnssObsEpoch &ob) {
    for (auto iter = ob.obsp.begin(); iter != ob.obsp.end() ; iter++)
    {   /* get satellite information one by one */
        auto &obsp = iter->second;
        obsp.sat.prn = obsp.prn;
        if (!this->GetSatellitePosVelClock(ob.time, obsp.sat, obsp.P)) continue;
    }
}


bool MultiPosition::GetSatellitePosVelClock(STimes tTime, SatInfo &sat, const double *P){
    int f;
    double pr;
    STimes satclock; /* signal transform time in satellite clock       */

    for (f = 0; f < NFREQ; f++)
    {   /* search any pseudorange */
        if ((pr = P[f]) != 0.0) break;
    }

    if (f >= NFREQ)
    {
        Log.Trace(TDEBUG, tTime, "no pseudorange, sat= " +  sat.prn);
        sat.vaild = false;
        return false;
    }

    /* transmission time by satellite clock */
    satclock = CTimes::timeadd(tTime, -pr / CLIGHT);

    int system = CString::satsys(sat.prn);
    int satid = CString::satprn2id(sat.prn);
    /* satellite clock bias by broadcast ephemeris */
    double dts = pOrbclk->CalSatClock(satclock, system, satid);
    if(dts > 1e8)
    {
        Log.Trace(TDEBUG, tTime, "no broadcast clock, sat= " + sat.prn);
        sat.vaild = false;
        return false;
    }

    satclock = CTimes::timeadd(satclock, -dts);

    /* satellite position and clock at transmission time */
    if(!pOrbclk->CalSatPosVel(satclock, sat))
    {
        Log.Trace(TWARNING, tTime, "no ephemeris, sat= " + sat.prn);
        sat.vaild = false;
        return false;
    }
    return true;
}


/* test excluded satellite -----------------------------------------------------
*  return : vaild (true:excluded, false:not excluded)
*-----------------------------------------------------------------------------*/
bool MultiPosition::SatelliteExclude(SatInfo &sat){
    bool vaild;
    if (!sat.vaild) return true;

    if (sat.var > MAX_VAR_EPH)
    {   /* variance of ephemeris (m^2) */
        Log.Trace(TEXPORT, "invalid ura satellite: sat=" + sat.prn + " ura=" + to_string(sqrt(sat.var)));
        sat.vaild = false;
        return true;
    }

    if (sat.svh)
    {   /* ephemeris unavailable */
        Log.Trace(TEXPORT, "unhealthy satellite: sat=" + sat.prn + " svh=" + to_string(sat.svh));
        sat.vaild = false;
        return true;
    }

    vaild = config.ExcludeSatellite.find(sat.prn) != string::npos;  // false is vaild
    if (vaild) sat.vaild = false;
    return vaild;
}


/* geometric distance ----------------------------------------------------------
* compute geometric distance and receiver-to-satellite unit vector
* args   : Vector3d sat.pos       I   satellilte position (ecef at transmission) (m)
*          Vector3d      rr       I   receiver position (ecef at reception) (m)
*          Vector3d       e       O   line-of-sight vector (ecef)
* return : geometric distance (m) (0>:error/no satellite position)
* notes  : distance includes sagnac effect correction
*-----------------------------------------------------------------------------*/
double MultiPosition::GeoDistance(SatInfo &sat, const Vector3d& rr, Vector3d& e){
    double r;
    if (sat.pos.norm() < WGS84_RA) return -1.0;
    e = sat.pos - rr;
    r = e.norm();
    e = e / e.norm();
    if (r <= 0) sat.vaild = false;
    return r + WGS84_WIE * (sat.pos[0] * rr[1] - sat.pos[1] * rr[0]) / CLIGHT;
}


/* satellite azimuth/elevation angle -------------------------------------------
* compute satellite azimuth/elevation angle
* args   : Vector3d pos      I   geodetic position {lat,lon,h} (rad,m)
*          Vector3d  e       I   receiver-to-satellilte unit vevtor (ecef)
*          double            O   azimuth/elevation {az,el} (rad)
*                               (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
* return : elevation angle < config.Elevation?
*-----------------------------------------------------------------------------*/
bool MultiPosition::SatAzimuthElevation(const Vector3d& pos, const Vector3d& e, GnssObsData& obs){
    double az = 0.0, el = PI / 2.0;

    if (pos[2] > -WGS84_RA)
    {
        Vector3d enu = CEarth::ecef2enu(pos, e);
        az = enu.norm()-enu[2]*enu[2] < 1E-12? 0.0: atan2(enu[0],enu[1]);
        if (az < 0.0) az += 2 * PI;
        el = asin(enu[2]);
    }
    obs.azimuth = az;
    obs.elevation = el;

    if (el < config.Elevation)
    {
        obs.sat.vaild = false;
        return true;
    }

    return false;
}


bool MultiPosition::ReadAllObservation(std::shared_ptr <ReadRinexObs> p, vector<GnssObsEpoch>& obss){
    GnssObsEpoch obs_t;
    while (true)
    {
        if (!p->Reading()) break;
        obs_t = p->GetObsData();
        obss.push_back(obs_t);
    }

    if (obss.empty())
    {
        Log.Trace(TERROR, "*Error in read all observation value, please check!");
        return false;
    }
    return true;
}


void MultiPosition::GetSolution(Solution& solution){
    solution.pos = mSol.pos;
    solution.Qpos = mSol.Qpos;
    solution.blh = CEarth::ecef2blh(mSol.pos);
}


void MultiPosition::SaveLastSolution(){
    mEpoch += 1; mSol.epoch += 1;
    mTime = mObs.time;
    mSol.dt = CTimes::timediff(mObs.time, mSol.time);
    mSol.time = mObs.time;
    mSol.stat = SOLQ_FLOAT;
    mSol.vstat = VELQ_NONE;

    mSol.nqc = 0;
    mSol.lpos = mSol.pos;
    mSol.lvel = mSol.vel;

    for (int i = 0; i < 5; i++)
    {
        mSol.ldrck[i] = mSol.drck[i];
    }

    /* init satellite status arrays */
    if (config.ProcessMode == PMODE_RTK)
    {
        for (int i = 0; i < MAXSAT; i++)
        {
            for (int j = 0; j < config.NumFreq; j++)
            {
                mVsat[i][j] = false;       /* valid satellite */
            }
        }
    }

    if (config.DebugRobust) pRobLog->mExportTime  = true;
}


double MultiPosition::SingleDifferObs(double pi, double pj){
    return pi==0.0 || pj==0.0? 0.0: pi-pj;
}


bool MultiPosition::RobustVelocity(vector<Equation>& eq){
    int id;
    double vv = 0;
    sQuality mQc_t;    /* index and variance expansion factor in eq     */

    for (auto iter = mQc.begin(); iter != mQc.end(); iter++)
    {
        eq[iter->index].R *= iter->fact;
    }
    for (auto it = eq.begin(); it != eq.end() ; it++)
    {
        vv += it->omc / it->R *it->omc;
    }
    vv = sqrt(vv / (int(eq.size())-4));

    mQcMask = false;
    id = FindMaxNormResidual(eq);
    if (abs(eq[id].omc) > 0.08 || abs(eq[id].omc) > 4 * vv)
    {
        mQcMask = true;
        mQc_t.index = id;
        mQc_t.fact = 1e3;
        mQc.push_back(mQc_t);
        if(config.DebugRobust) pRobLog->TraceRobust(mObs.time, eq[id], ROBUST_REJECT, mV0, mQc_t.fact);
    }
    return mQcMask;
}


void MultiPosition::LoadLastSolution() {
    for (int i = 0; i < 3; i++)
    {
        mSol.pos[i] = mSol.lpos[i];
        mSol.vel[i] = mSol.lvel[i];
    }

    for (int i = 0; i < 5; i++)
    {
        mSol.drck[i] = mSol.ldrck[i];
    }
}


bool MultiPosition::isReduced(int id) {
    for (auto iter = mQc.begin(); iter != mQc.end(); iter++)
    {
        if(iter->index == id) return true;
    }
    return false;
}


int MultiPosition::FindMaxNormResidual(const vector<Equation>& eq){
    int i = 0, id = 0;
    mV0 = abs(eq.begin()->omc / sqrt(eq.begin()->R));
    for (auto iter = eq.begin(); iter != eq.end(); iter++, i++)
    {
        if (iter->system == -1) continue;     //  constraint equation
        if ((mV1 = abs(iter->omc / sqrt(iter->R))) > mV0)
        {   // find max
            id = i;
            mV0 = mV1;
        }
    }
    return id;
}


bool MultiPosition::CheckMaxResidual(int id, int frq, double res, bool code, const MatrixXd& Pf){
    /* adjust threshold by error stdev ratio unless one of the phase biases was just initialized*/
    double thres = code || (Pf(CState::XiAmb(id, frq), CState::XiAmb(id,frq)) == SQR(config.Std[0]))? config.ErrRatio[frq]: 1.0;

    if (config.MaxResidual > 0.0 && fabs(res) > config.MaxResidual * thres) 
    {
        mVsat[id - 1][frq] = false;
        Log.Trace(TINFO, mTime, "outlier rejected: " + CString::satid2prn(id) + "-" + " freq=" + to_string(frq) + " v=" + to_string(res));
        return false;
    }
    return true;
}


int MultiPosition::SolutionQualityCheck(const Solution& solution, const vector<Equation>& eq){
    int nv = 0, nx = 4 + NUMSYS;
    double vv = 0;

    /* Chi-square validation of residuals */
    for (auto iter = eq.begin(); iter != eq.end(); iter++)
    {
        if (iter->system < 0) continue;   // constrain
        nv += 1;
        vv += pow(iter->omc / sqrt(iter->R), 2);
    }

    if (vv > sChisqr[nv - nx - 1]) return QC_CHISQR;

    /* lack of valid satellites */
    if (solution.nsat[0] < 4) return QC_LACK;

    /* large PDOP check */
    if (solution.pdop <= 0.0 || solution.pdop > MAX_PDOP) return QC_PDOP;

    /* check whether robust sucess */
    if (solution.nqc >= config.RobThres[0]) return QC_ROBUST;

    return QC_GOOD;
}


int MultiPosition::VelocityQualityCheck(const vector<Equation>& eq){
    int nv = 0, nx = 4 + NUMSYS;
    double vv = 0;

    /* Chi-square validation of residuals */
    for (auto iter = eq.begin(); iter != eq.end(); iter++)
    {
        if (iter->system < 0) continue;   // constrain
        nv += 1;
        vv += pow(iter->omc / sqrt(iter->R), 2);
    }
    if (vv > sChisqr[nv - nx - 1]) return QC_BADVEL;
    return QC_GOOD;
}


void MultiPosition::SaveSolutionState(){

    if (config.SolverMode == SOLVER_EKF)
    {   // EKF mode
        for (int i = 0; i < CState::XnP();  i++) mSol.pos(i) = mXf(CState::XiP()  + i);   // pos 
        mSol.Qpos = mPf.block<3, 3>(CState::XiP(),  CState::XiP());
        if (config.VelocityMode == VELOPT_OFF)
        {
            for (int i = 0; i < CState::XnV();  i++) mSol.vel(i) = mXf(CState::XiV()  + i);   // vel
            for (int i = 0; i < CState::XnAc(); i++) mSol.acc(i) = mXf(CState::XiAc() + i);   // acc
            mSol.Qvel = mPf.block<3, 3>(CState::XiV(),  CState::XiV());
        }

    }

    mSol.blh = CEarth::ecef2blh(mSol.pos);

    Log.ShowState(CTimes::time2str(mSol.time), mSol.stat, mSol.epoch);

}


bool MultiPosition::Robust(vector<Equation>& eq){
    if       (config.RobustMode == ROBUST_IGG3)
    {
        return this->Robust_IGG3(eq);
    }
    else if (config.RobustMode == ROBUST_HUBER)
    {
        return this->Robust_Huber(eq);
    }
    else if (config.RobustMode == ROBUST_OFF)
    {
        return false;
    }
    else
    {
        Log.Trace(TERROR, "Robust mode set error, please check!");
        return false;
    }
}


bool MultiPosition::Robust_IGG3(vector<Equation>& eq){
    int id;
    sQuality mQc_t;    /* index and variance expansion factor in eq     */

    for (auto iter = mQc.begin(); iter != mQc.end(); iter++)
    {
        eq[iter->index].R *= iter->fact;
    }

    mQcMask = false;
    id = FindMaxNormResidual(eq);
    if (mV0 > mK1 && mV0 > 0.01)
    {
        mQcMask = true;
        mQc_t.index = id;
        mQc_t.fact = 1e3;
        mQc.push_back(mQc_t);
        if(config.DebugRobust) pRobLog->TraceRobust(mObs.time, eq[id], ROBUST_REJECT, mV0, mQc_t.fact);
    }
    else if (mV0 > mK0 && mV0 < mK1 && mV0 > 0.01)
    {
        mQcMask = true;
        mQc_t.index = id;
        if (this->isReduced(id))
        {
            mQc_t.fact = 2.0;
            if(config.DebugRobust) pRobLog->TraceRobust(mObs.time, eq[id], ROBUST_CONRED, mV0, mQc_t.fact);  //continuous redeced
        }
        else
        {
            mQc_t.fact = (mV0 / mK0) * SQR((mK1 - mK0) / (mK1 - mV0));
            if(config.DebugRobust) pRobLog->TraceRobust(mObs.time, eq[id], ROBUST_REDUCE, mV0, mQc_t.fact);
        }

        mQc.push_back(mQc_t);
    }

    return mQcMask;
}


bool MultiPosition::Robust_Huber(vector<Equation>& eq){
    int id;
    sQuality mQc_t;    /* index and variance expansion factor in eq     */

    for (auto iter = mQc.begin(); iter != mQc.end(); iter++)
    {
        eq[iter->index].R *= iter->fact;
    }

    mQcMask = false;
    id = FindMaxNormResidual(eq);
    if (mV0 > mK0)
    {
        mQcMask = true;
        mQc_t.index = id;
        if (this->isReduced(id))
        {
            mQc_t.fact = 3.0;
            if(config.DebugRobust) pRobLog->TraceRobust(mObs.time, eq[id], ROBUST_CONRED, mV0, mQc_t.fact);  //continuous redeced
        }
        else
        {
            mQc_t.fact = abs(mV0) / mK0;
            if(config.DebugRobust) pRobLog->TraceRobust(mObs.time, eq[id], ROBUST_REDUCE, mV0, mQc_t.fact);
        }
        mQc.push_back(mQc_t);
    }

    return mQcMask;
}


bool MultiPosition::VelocityProcess(){
    mSol.Qvel  = Matrix3d::Zero(3, 3);
    mSol.Qveli = Matrix3d::Zero(3, 3);

    if (config.VelocityMode == VELOPT_OFF) return false;

    switch (config.SolverMode)
    {
        case SOLVER_LSQ:
        case SOLVER_GO : return this->SolveVelocity();
        case SOLVER_EKF: return this->FilterVelocity();
        default: Log.Trace(TERROR, "Velocoty: Solver mode set error"); break;
    }
    return true;
}


bool MultiPosition::SolveVelocity(){
    mQc.clear();

    for (int iter = 0; iter < MAX_ITR; iter++)
    {
        mVelEq.clear();
        this->FormVelocity(mSol.vel, mSol.drcd);
        if (config.DebugResVel && iter == 0) mVelEqPri = mVelEq;
        if (mSol.nsat[6] < 4)
        {
            Log.Trace(TINFO, "SolveVelocity, Not enough observation for velocity estimate");
            return false;
        }

        if(!pLsq->Solving(mSol.vdx, mSol.Qvel, mVelEq)) return false;
        pLsq->SolutionFeedback(mSol, LSQ_SOLVE_VEL);
        if (mSol.vdx.norm() < 1E-4 || iter == MAX_ITR)
        {
            if (config.RobustMode != ROBUST_OFF)
            {
                for (mIterQc = 0; mIterQc < config.RobThres[0]; mIterQc++)
                {   // iter for robust
                    this->FormVelocity(mSol.vel, mSol.drcd);
                    if(!this->RobustVelocity(mVelEq) && mSol.vdx.norm() < 1E-3) break;  // when don't need check, break
                    if(!pLsq->Solving(mSol.vdx, mSol.Qvel, mVelEq)) return false;
                    pLsq->SolutionFeedback(mSol, LSQ_SOLVE_VEL);
                }
            }
            if (isnan(mSol.vel[0])) break;
            this->FormVelocity(mSol.vel, mSol.drcd);
            mSol.Qc = this->VelocityQualityCheck(mVelEq);
            mSol.vstat = (config.VelocityMode == VELOPT_DOPPLER) ? VELQ_DOPPLER : VELQ_TDCP;
            if (config.VelocityMode == VELOPT_DOPPLER) mSol.Qveli = mSol.Qvel;
            if(config.DebugResVel) pVelResLog->TraceRes(mTime, mVelEqPri, mVelEq);
            return true;
        }
    }
    Log.Trace(TINFO, mObs.time, "SolveVelocity, Number of iterations exceeds the threshold");
    return false;
}


bool MultiPosition::FilterVelocity(){
    Solution sol_t;

    if(!this->UpdateVelState())
    {
        pEkfVel->InitVelFilter(mSol, mXvf, mPvf);
    }

    sol_t = mSol;
    mXt = mXvf;
    mPt = mPvf;
    mQc.clear();

    for (mIterQc = 0; mIterQc < (config.RobustMode?config.RobThres[0]*config.NumFreq:1); mIterQc++)
    {
        mVelEq.clear();
        Vector3d PriorVel = mXvf.block(0, 0, CState::XnV(), 1);
        this->FormVelocity(PriorVel, mXvf(6));
        if (config.DebugResVel && mIterQc == 0) mVelEqPri = mVelEq;
        if (mVelEq.size() < 4) return false;

        for (auto it = mQc.begin(); it != mQc.end(); it++) // for robust
        {
            mVelEq[it->index].R *= it->fact;
        }

        this->FormVelVarMatrix(mVelEq);

        if (!pEkf->Solving(mVelEq, mSol, mXvf, mPvf, mR))
        {
            mSol.vstat = VELQ_NONE;
            Log.Trace(TINFO, mObs.time, " filter error");
        }

        Vector3d PostVel = mXvf.block(0, 0, CState::XnV(), 1);
        this->FormVelocity(PostVel, mXvf(6));  // vel res
        if (!this->RobustVelocity(mVelEq) || mIterQc == config.RobThres[0]) break;  // when don't need check, break
        mSol = sol_t;
        mXvf = mXt;
        mPvf = mPt;
    }

    for (int i = 0; i < CState::XnV(); i++)
    {
        if      (config.VelocityMode == VELOPT_DOPPLER)
        {   // instantaneous vel
            mSol.veli(i) = mXvf(i);
            mSol.Qveli   = mPvf.block<3, 3>(0, 0);
        }
        else if (config.VelocityMode == VELOPT_TDCP)
        {   // average vel
            mSol.vel(i) = mXvf(i);
            mSol.Qvel   = mPvf.block<3, 3>(0, 0);
        }
        mSol.acc(i) = mXvf(3+i);    // acceleration
    }

    mSol.drcd = mXvf(6);
    mSol.vstat = config.VelocityMode;

    if(config.DebugResVel) pVelResLog->TraceRes(mTime, mVelEqPri, mVelEq);
    return true;
}


bool MultiPosition::UpdateVelState(){
    double var = 0;
    Matrix3d E = MatrixXd::Zero(3, 3);
    Matrix3d Q = MatrixXd::Zero(3, 3);

    /* check variance of estimated velocity */
    for (int i = 0; i < 3; i++) var += mPvf(i, i);
    var /= 3.0;

    if (var > VAR_VEL)
    {   /* reset velocity with large variance */
        Log.Trace(TINFO, "reset rtk velocity due to large variance: var=" + to_string(var));
        return false;
    }

    MatrixXd Fk = MatrixXd::Identity(CState::XnGnssVel(), CState::XnGnssVel());

    /* state transition of velocity/acceleration */
    for (int i = 0; i < 3; i++) Fk(i, i + 3)  = fabs(mSol.dt);

    /* x=F*x, P=F*P*F+Q */
    mXvf = Fk * mXvf;
    mPvf = Fk * mPvf * Fk.transpose();

    // process add to acceleration only
    E = CEarth::xyz2enu(mSol.blh);
    Q << SQR(config.Process[3])*fabs(mSol.dt), 0.0,                                  0.0,
         0.0,                                  SQR(config.Process[3])*fabs(mSol.dt), 0.0,
         0.0,                                  0.0,                                  SQR(config.Process[4])*fabs(mSol.dt);
    Q = E.transpose() * Q * E;
    mPvf.block<3, 3>(3, 3) += Q;

    pEkfVel->initx(mSol.drcd, VAR_CDR, 6, mXvf, mPvf);  // recever clock drift estimate as white noise

    return true;
}


bool MultiPosition::FormVelocity(const Vector3d &vel,  const double drcd){
    int freqindex;
    double freq, cosel, rate;
    Equation eq_t;
    Matrix3d E = MatrixXd::Zero(3, 3);
    MatrixXd a = MatrixXd::Zero(3, 1);
    Vector3d e = Vector3d::Zero(3);
    Vector3d vs = Vector3d::Zero(3);

    mSol.nsat[6] = 0;
    mVelEq.clear();
    mSol.blh = CEarth::ecef2blh(mSol.pos);
    E = CEarth::xyz2enu(mSol.blh);

    for (auto iter = mObs.obsp.begin(); iter != mObs.obsp.end() ; iter++)
    {
        auto &obsp = iter->second;
        if (this->SatelliteExclude(obsp.sat)) continue;
        if (obsp.elevation < config.Elevation) continue;

        eq_t.system = CString::satsys(obsp.prn);
        eq_t.prn = obsp.prn;

        /* LOS (line-of-sight) vector in ECEF */
        cosel = cos(obsp.elevation);
        a << sin(obsp.azimuth) * cosel, cos(obsp.azimuth) * cosel, sin(obsp.elevation);
        e = E.transpose() * a;

        /* satellite velocity relative to receiver in ECEF */
        vs << obsp.sat.vel[0] - vel[0], obsp.sat.vel[1] - vel[1], obsp.sat.vel[2] - vel[2];
        /* range rate with earth rotation correction */
        rate = vs.dot(e) + WGS84_WIE / CLIGHT * (obsp.sat.vel[1] * mSol.pos[0] + obsp.sat.pos[1] * vel[0]
                                             - obsp.sat.vel[0] * mSol.pos[1] - obsp.sat.pos[0] * vel[1]);

        for (int f = 0; f < config.NumFreq; f++)
        {
            this->FormVelDesignMatrix(eq_t, e);

            freqindex = CState::GetFrequencyIndex(obsp.sat.prn, f);
            freq =  CState::GetFrequency(obsp.sat.prn, f, obsp.sat.glofcn);  // mObs.sat.glofcn ??

            if (config.VelocityMode == VELOPT_DOPPLER)
            {
                if (obsp.D[freqindex] == 0.0 || freq == 0 || obsp.sat.vel.norm() <= 0.0)
                {
                    continue;
                }
                /* range rate residual (m/s) */
                eq_t.omc = -obsp.D[freqindex] * CLIGHT / freq - (rate + drcd - CLIGHT * obsp.sat.dts[1]);
                eq_t.R = pEm->ErrorVariance(eq_t.system, DOPPLER, obsp, 0.0, SNR_UNIT * obsp.SNR[freqindex]) + obsp.sat.var * 0.00;
                eq_t.Rb = 0.0;

            }
            else if(config.VelocityMode == VELOPT_TDCP)
            {
                if (obsp.DL[freqindex] == 0.0 || freq == 0 ||  obsp.sat.vel.norm() <=0.0)
                {
                    continue;
                }
                eq_t.omc = -obsp.DL[freqindex] * CLIGHT / freq - (rate + drcd - CLIGHT * obsp.sat.dts[1]);
                eq_t.R = pEm->ErrorVariance(eq_t.system, TDCP, obsp, 0.0, SNR_UNIT * obsp.SNR[freqindex]) * CLIGHT / freq + obsp.sat.var * 0.0;
                eq_t.Rb = 0.0;
            }
            eq_t.freq = f;
            eq_t.el = obsp.elevation;
            eq_t.snr = obsp.SNR[freqindex];
            if (f == 0) mSol.nsat[6]++;
            mVelEq.push_back(eq_t);
        }

    }
    return true;
}


void MultiPosition::FormVelVarMatrix(const vector<Equation> &eq){
    int row = int(eq.size());

    mR = MatrixXd::Zero(row, row);

    for (int i = 0; i < row; i++)
    {
        mR(i, i) = eq[i].R;
    }
}


bool MultiPosition::FormVelDesignMatrix(Equation& eq_t, const Vector3d& e){
    eq_t.H.clear();
    eq_t.H.resize(CState::XnGnssVel(), 0.0);

    for (int i = 0; i < 3; i++)
    {
        eq_t.H[i] = -e[i];
    }

    eq_t.H[(config.SolverMode==SOLVER_LSQ || config.SolverMode==SOLVER_GO)? 3: 6] = 1.0;
    return true;
}


SmoothPosition::SmoothPosition(){
    pForward  = std::make_shared<ReadGnssResultBin>();
    pBackward = std::make_shared<ReadGnssResultBin>();
    pEx       = std::make_shared<WriteResultFile>();
}


SmoothPosition::~SmoothPosition(){
    pForward->CloseFile();
    pBackward->CloseFile();
}


bool SmoothPosition::Smooth(){

    if (!this->Initialization()){
        Log.Trace(TERROR, "Smooth initialization error");
        return false;
    }

    for (auto it = mForwardSol.begin(); it != mForwardSol.end(); it++)
    {
        mSol.time.time = it->first;
        if (mBackwardSol.find(it->first) != mForwardSol.end())
        {   // match forward and back ward
            this->Smoothing(it->second, mBackwardSol[it->first]);
        }
        else
        {   // only use forward solution
            mSol.pos = it->second.pos;
            mSol.vel = it->second.vel;
            mSol.nsat[0] = it->second.nsat;
            mSol.stat = SOLQ_FIXFLOAT;
            for (int i = 0; i < 3; i++) mSol.Qpos(i, i) = it->second.Qpos[i];
            if (config.VelocityMode > VELOPT_OFF)
            {   // instantaneous velocity
                for (int i = 0; i < 3; i++) mSol.Qveli(i, i) = it->second.Qvel[i];
            }
            else
            {   // average velocity
                for (int i = 0; i < 3; i++) mSol.Qvel(i, i) = it->second.Qvel[i];
            }

        }
        pEx->Export(mSol);
    }

    return true;
}


bool SmoothPosition::Initialization(){
    string file = "forward.bin";
    if(!pForward->OpenBinFile(file)) return false;
    mForwardSol = pForward->GetSolutionData();

    file = "backward.bin";
    if(!pBackward->OpenBinFile(file)) return false;
    mBackwardSol = pBackward->GetSolutionData();

    mSol.Qpos  = Matrix3d::Zero(3, 3);
    mSol.Qvel  = Matrix3d::Zero(3, 3);
    mSol.Qveli = Matrix3d::Zero(3, 3);

    return true;
}


bool SmoothPosition::Smoothing(const SmoothSolution& fs, const SmoothSolution& bs){

    for (int i = 0; i < CState::XnP(); i++)
    {
        this->SmoothCalculate(fs.pos[i], fs.Qpos[i], bs.pos[i], bs.Qpos[i], mSol.pos[i], mSol.Qpos(i, i));
    }

    for (int i = 0; i < CState::XnV(); i++)
    {
        double veladj = config.VelocityMode > VELOPT_OFF? bs.vel[i]: -bs.vel[i];
        auto &vel = config.VelocityMode > VELOPT_OFF? mSol.veli[i]: mSol.vel[i];
        auto &Qvel = config.VelocityMode > VELOPT_OFF? mSol.Qveli(i, i): mSol.Qvel(i, i);
        this->SmoothCalculate(fs.vel[i], fs.Qvel[i], veladj, bs.Qvel[i], vel, Qvel);
    }

    mSol.nsat[0] = (fs.nsat + bs.nsat) * 0.5;

    if      (fs.stat == SOLQ_FIX and bs.stat == SOLQ_FIX)
    {
        mSol.stat = SOLQ_FIX;
    }
    else if ( (fs.stat == SOLQ_FIX and bs.stat == SOLQ_FLOAT) || (fs.stat == SOLQ_FLOAT and bs.stat ==SOLQ_FIX))
    {
        mSol.stat = SOLQ_FIXFLOAT;
    }
    else if (fs.stat == SOLQ_FLOAT and bs.stat == SOLQ_FLOAT)
    {
        mSol.stat = SOLQ_FLOAT;
    }
    else
    {
        mSol.stat = SOLQ_NONE;
    }

    return true;
}


/* smoother --------------------------------------------------------------------
* combine forward and backward filters by fixed-interval smoother as follows:
*   xs=Qs*(Qf^-1*xf+Qb^-1*xb), Qs=(Qf^-1+Qb^-1)^-1)
*-----------------------------------------------------------------------------*/
void SmoothPosition::SmoothCalculate(const double xf, const double Qf, const double xb, const double Qb, double& xs, double& Qs){
    Qs = 1.0 / (1.0 / Qf + 1.0 / Qb);
    xs = Qs * (xf / Qf + xb / Qb);
}

