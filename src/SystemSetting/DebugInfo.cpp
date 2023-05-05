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
 * Created by lizhen on 2021/11/22.
 *-------------------------------------------------------------------------------*/

#include "DebugInfo.h"


DebugResidual::DebugResidual(const string filename, int type){
    int n;
    string file;

    if ( (n = (int)filename.find(".")) != string::npos)
    {
        file = filename.substr(0, n) + ".res" + to_string(type);
    }
    else
    {
        file = filename + ".res" + to_string(type);
    }

    mResLog.open(file, ios::out);
    if (!mResLog.is_open())
    {
        Log.Trace(TERROR, "Cann't open Residual debug file!");
    }
    else
    {
        mResLog << "% Residual debug information" << endl;
        mResLog << "% Sat    Freq" << setw(12) << "Prior" << setw(12) << "Post"
                << setw(12) << "PriorR" << setw(12) << "PostR"
                << setw(15) << "Elevation(°)"
                << setw( 8) << "SNR(dB)"
                << endl;
        mResLog << "% ---------------------------------------------------------------------------------" << endl;
    }
}


DebugResidual::~DebugResidual(){
    if (!mResLog.is_open()) mResLog.close();
}


void DebugResidual::TraceRes(const STimes tTime, const vector<Equation>& pri, const vector<Equation>& post, const int ref[][NFREQ]){
    int sys;
    string ref_prn;
    string stime = CTimes::time2str(tTime);
    mResLog << ">" << stime << "      " << pri.size() << endl;

    if (pri.empty() || post.empty()) return;

    for(int i = 0; i < pri.size(); i++)
    {
        if ((sys = CString::satsys(pri[i].prn) - 1) < 0) continue;
        if (sys == SYS_BDS-1 && config.BdsOption == BDSOPT_BD2_3) sys = CString::checkbd23(pri[i].prn) - 1;
        ref_prn = CString::satid2prn(ref[sys][pri[i].freq]);
        mResLog << "  " << ref_prn << "-" << pri[i].prn << " ";
        if (pri[i].R < 0.05) mResLog << "L";   // carrier-phase
        else                 mResLog << "P";   // pseudorange
        mResLog << pri[i].freq + 1 << ":";
        mResLog << fixed << setprecision(4) << setw(12) << pri[i].omc << setw(12) << post[i].omc
                << setw(12) << sqrt(pri[i].R+pri[i].Rb) << setw(12) << sqrt(post[i].R+post[i].Rb)
                << setprecision(2) << setw(14) <<post[i].el * R2D << setw(8) << post[i].snr * SNR_UNIT
                << endl;
    }

}


void DebugResidual::TraceRes(const STimes tTime, const vector<Equation>& pri, const vector<Equation>& post){
    string stime = CTimes::time2str(tTime);
    mResLog << ">" << stime << endl;

    if (pri.empty() || post.empty()) return;

    for(int i = 0; i < pri.size(); i++)
    {
        if ((CString::satsys(pri[i].prn) - 1) < 0) continue;
        mResLog << "  " << pri[i].prn << "     ";
        if (pri[i].R < 0.05) mResLog << "L";   // carrier-phase
        else                 mResLog << "P";   // pseudorange
        mResLog << pri[i].freq + 1 << ":";
        mResLog << fixed << setprecision(4) << setw(12) << pri[i].omc << setw(12) << post[i].omc << " "
                << setw(11) << sqrt(pri[i].R) << setw(11) << sqrt(post[i].R)
                << setprecision(2) << setw(14) << post[i].el * R2D << setw(8) << post[i].snr * SNR_UNIT
                << endl;
    }

}


DebugRobust::DebugRobust(const string filename){
    int n;
    string file;

    if ( (n = (int)filename.find(".")) != string::npos)
    {
        file = filename.substr(0, n) + ".rob";
    }
    else
    {
        file = filename + ".rob";
    }

    mRobLog.open(file, ios::out);
    if (!mRobLog.is_open())
    {
        Log.Trace(TERROR, "Cann't open Robust debug file!");
    }
    else
    {
        mRobLog << "% Robust debug information" << endl;
        mRobLog << "%" << setw(3) << "Sat" << setw(5) << "Freq" << setw(12) << "Type"
                << setw(15) << "Residual" << setw(15) << "NormResidual"
                << setw(10) << "Var" << setw(8) << "Factor" << endl;
        mRobLog << "% -------------------------------------------------------------------" << endl;
    }
}


DebugRobust::~DebugRobust(){
    if (!mRobLog.is_open()) mRobLog.close();
}


void DebugRobust::TraceTime(const STimes tTime){
    mRobLog << ">" << CTimes::time2str(tTime) << endl;
}


void DebugRobust::TraceRobust(const STimes tTime, const Equation& eq, int type, double v0, double fact){
    string freq;
    if (mExportTime) this->TraceTime(tTime);
    if (eq.R < 0.1) freq = "L";
    else            freq = "P";
    freq += to_string(eq.freq+1);

    mRobLog << setw(4) << eq.prn << setw(5) << freq;
    switch (type)
    {
        case ROBUST_REJECT:
            mRobLog << setw(12) << "Rejected"; break;
        case ROBUST_REDUCE:
            mRobLog << setw(12) << "Reduced"; break;
        case ROBUST_CONRED:
            mRobLog << setw(12) << "Continuity"; break;
        default:
            mRobLog << setw(12) << "Unknown"; break;
    }
    mRobLog << fixed << setprecision(3) << setw(15) << eq.omc << setw(15) << v0
            << setw(10) << sqrt(eq.R) << setw(8) << setprecision(1) << fact << endl;
    this->mExportTime = false;
}
