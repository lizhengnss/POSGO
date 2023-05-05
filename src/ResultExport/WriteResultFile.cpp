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
 * Created by lizhen on 2021/8/24.
 *-------------------------------------------------------------------------------*/

#include "WriteResultFile.h"

WriteResultFile::WriteResultFile(){
    mOutfile.open(config.Result_File, ios::out);
    if (!mOutfile.is_open())
    {
        Log.Trace(TERROR, "*ResultFile: " + config.Result_File + " cann't open, please check");
        exit(-1);
    }
    if (config.ExportHead) this->WriteHead();
    mSep = config.sep;
}


bool WriteResultFile::Export(Solution& sol){

    mSol = &sol;

    if (mSol->stat == SOLQ_NONE) return false;
    mOutfile << " ";
    if      (config.ExportTimeSystem >= TIMES_UTC) mSol->time = CTimes::gpst2utc(mSol->time);// default is gpst
    else if (config.ExportTimeSystem == TIMES_BST) mSol->time = CTimes::timeadd(mSol->time, 8 * 3600.0);

    /* export time information */
    if      (config.ExportTimeFormat == TIMEF_TOW) this->WriteTimeTOW();
    else if (config.ExportTimeFormat == TIMEF_HMS) this->WriteTimeHMS();

    /* export position information */
    if      (config.ExportCoordinate == SOLF_XYZ) this->WritePositionECEF();
    else if (config.ExportCoordinate == SOLF_BLH) this->WritePositionBLH();

    /* export solution state */
    mOutfile << mSep << setw(3) << mSol->stat;
    mOutfile << fixed << mSep << setw(3) << mSol->nsat[0];
    mOutfile << mSep << setw(3) << mSol->Qc;

    /* export velocity information */
    if     (config.ExportVelocity && config.ExportCoordinate == SOLF_XYZ) this->WriteVelocityECEF();
    else if(config.ExportVelocity && config.ExportCoordinate == SOLF_BLH) this->WriteVelocityBLH();

    /* export velocity information */
    if     (config.ExportAcceleration && config.ExportCoordinate == SOLF_XYZ) this->WriteAccelerationECEF();
    else if(config.ExportAcceleration && config.ExportCoordinate == SOLF_BLH) this->WriteAccelerationBLH();

    if (config.ProcessMode == PMODE_RTK) mOutfile << mSep << setprecision(1) << setw(6) << mSol->age;
    if (config.ExportDop) mOutfile << fixed << mSep << setprecision(2) << setw(5) << mSol->pdop;

    /* export clock information */
    if (config.ExportClock)      this->WriteClock();
    if (config.ExportClockDrift) this->WriteClockDrift();

    mOutfile << endl;

    return true;
}


void WriteResultFile::Close(){
    if (mOutfile.is_open())
    {
        mOutfile.close();
    }
}


void WriteResultFile::WriteHead(){
    mOutfile << "% Program         : POSGO V" << MAJOR_VERSION << "." << MINOR_VERSION << endl;
    mOutfile << "% Pos Mode        : " << opt_Process[config.ProcessMode] << endl;
    mOutfile << "% Solve Type      : " << opt_SloveMode[config.SolveType] << endl;
    mOutfile << "% Solver Mode     : " << opt_Slover[config.SolverMode] << endl;
    mOutfile << "% Robust Mode     : " << opt_Robust[config.RobustMode] << endl;
    mOutfile << "% Velocity        : " << opt_Velocity[config.VelocityMode] << endl;
    mOutfile << "% Elevation(deg)  : " << config.Elevation * R2D << endl;
    mOutfile << "% Ionospheric     : " << opt_Iono[config.Ionospheric] << endl;
    mOutfile << "% Tropospheric    : " << opt_Trop[config.Tropospheric] << endl;
    mOutfile << "% CodeBias        : " << opt_Code[config.CodeBias] << endl;
    mOutfile << "% Weight Mode     : " << opt_WeightMode[config.WeightMode] << endl;


    if (config.SolverMode == SOLVER_GO)
    {
        mOutfile << "% Loss Function   : " << opt_LossFun[config.LossFunction] << endl;
        mOutfile << "% Slide Window    : " << config.SlideWindowSize << endl;
    }

    this->WriteSystemHead();
    this->WriteFrequencyHead();
    this->WriteResultHead();
}


void WriteResultFile::WriteSystemHead(){
    mOutfile << "% GNSS System     : ";
    if (config.System[1]) mOutfile << "GPS ";
    if (config.System[2]) mOutfile << "BDS ";
    if (config.System[3]) mOutfile << "GLO ";
    if (config.System[4]) mOutfile << "GAL";
    mOutfile << endl;
}


void WriteResultFile::WriteFrequencyHead(){
    mOutfile << "% GNSS Frequency  : ";

    if (config.System[1])
    {
        mOutfile << "G(";
        for (int i = 0; i < config.NumFreq; i++)
        {
            mOutfile << sFreqStr[SYS_GPS][config.GPS_FreqID[i]];
            if (i < config.NumFreq -1) mOutfile << "+";
        }
        mOutfile << ") ";
    }

    if (config.System[2])
    {
        if (config.BdsOption == BDSOPT_BD2 || config.BdsOption == BDSOPT_BD23 || config.BdsOption == BDSOPT_BD2_3){
            mOutfile << "B2(";
            for (int i = 0; i < config.NumFreq; i++)
            {
                mOutfile << sFreqStr[SYS_BDS][config.BD2_FreqID[i]];
                if (i < config.NumFreq -1) mOutfile << "+";
            }
            mOutfile << ") ";
        }

        if (config.BdsOption == BDSOPT_BD3 || config.BdsOption == BDSOPT_BD23 || config.BdsOption == BDSOPT_BD2_3)
        {
            mOutfile << "B3(";
            for (int i = 0; i < config.NumFreq; i++)
            {
                mOutfile << sFreqStr[SYS_BDS][config.BD3_FreqID[i]];
                if (i < config.NumFreq - 1) mOutfile << "+";
            }
            mOutfile << ") ";
        }
    }

    if (config.System[3])
    {
        mOutfile << "R(";
        for (int i = 0; i < config.NumFreq; i++)
        {
            mOutfile << sFreqStr[SYS_GLO][config.GLO_FreqID[i]];
            if (i < config.NumFreq -1) mOutfile << "+";
        }
        mOutfile << ") ";
    }

    if (config.System[4])
    {
        mOutfile << "E(";
        for (int i = 0; i < config.NumFreq; i++)
        {
            mOutfile << sFreqStr[SYS_GAL][config.GAL_FreqID[i]];
            if (i < config.NumFreq -1) mOutfile << "+";
        }
        mOutfile << ")";
    }
    mOutfile << endl;
}


void WriteResultFile::WriteResultHead(){
    mOutfile << "%";

    if      (config.ExportTimeFormat == TIMEF_HMS) mOutfile << setw(23) << "Time";
    else if (config.ExportTimeFormat == TIMEF_TOW) mOutfile << setw(15) << "           Time";

    if      (config.ExportCoordinate == SOLF_XYZ)
    {
        mOutfile << setw(16) << "x-ecef(m)" << setw(16) << "y-ecef(m)" << setw(16) << "z-ecef(m)";
        mOutfile << setw(10) << "sdx(m)"    << setw(10) << "sdy(m)"    << setw(10) << "sdz(m)";
    }
    else if (config.ExportCoordinate == SOLF_BLH)
    {
        string unit;
        if      (config.ExportBLHFormat == BLHF_DEG) unit = "(deg)";
        else if (config.ExportBLHFormat == BLHF_DMS) unit = "(d'\") ";
        mOutfile << setw(12) << "latitude" << unit << setw(11) << "longitude" << unit << setw(10) << "height(m)";
        mOutfile << setw(10) << "sdn(m)" << setw(10) << "sde(m)" << setw(10) << "sdu(m)";
    }

    mOutfile << setw(4) << "Q";
    mOutfile << setw(4) << "ns";
    mOutfile << setw(4) << "Qc";

    if (config.ExportVelocity)
    {
        if      (config.ExportCoordinate == SOLF_XYZ)
        {
            mOutfile << setw(11) << "vx(m/s)"   << setw(11) << "vy(m/s)"   << setw(11) << "vz(m/s)";
            mOutfile << setw(10) << "sdvx(m/s)" << setw(10) << "sdvy(m/s)" << setw(10) << "sdvz(m/s)";
        }
        else if (config.ExportCoordinate == SOLF_BLH)
        {
            mOutfile << setw(11) << "vn(m/s)"   << setw(11) << "ve(m/s)"   << setw(11) << "vu(m/s)";
            mOutfile << setw(10) << "sdvn(m/s)" << setw(10) << "sdve(m/s)" << setw(10) << "sdvu(m/s)";
        }
    }

    if (config.ExportAcceleration)
    {
        if      (config.ExportCoordinate == SOLF_XYZ)
        {
            mOutfile << setw(11) << "ax(m/s2)"   << setw(11) << "ay(m/s2)"   << setw(11) << "az(m/s2)";
        }
        else if (config.ExportCoordinate == SOLF_BLH)
        {
            mOutfile << setw(11) << "an(m/s2)"   << setw(11) << "ae(m/s2)"   << setw(11) << "au(m/s2)";
        }
    }

    if (config.ProcessMode == PMODE_RTK) mOutfile << setw(7) << "age(s)";
    if (config.ExportDop) mOutfile << setw(6) << "  PDOP";

    if (config.ExportClock)
    {
        if (config.System[1]) mOutfile << setw(9) << "clkG";   //GPS
        if (config.System[2])
        {
            if (config.BdsOption == BDSOPT_BD2 || config.BdsOption == BDSOPT_BD23) mOutfile << setw(9) << "clkB2"; //BD2
            if (config.BdsOption == BDSOPT_BD3 || config.BdsOption == BDSOPT_BD23) mOutfile << setw(9) << "clkB3"; //BD3
        }
        if (config.System[3]) mOutfile << setw(9) << "clkR";  //GLO
        if (config.System[4]) mOutfile << setw(9) << "clkE";  //GAL
    }

    if (config.ExportClockDrift) mOutfile << setw(9) << "clkdrift";

    mOutfile << endl;
}


void WriteResultFile::WriteTimeHMS(){
    mTime = CTimes::time2str(mSol->time);
    mOutfile << mTime;
}


void WriteResultFile::WriteTimeTOW(){
    int week;
    double gpst = CTimes::time2gpst(mSol->time, &week);
    if (86400 * 7 - gpst < 0.5 / pow(10.0, 3))
    {
        week++;
        gpst = 0.0;
    }
    mOutfile << fixed << setw(4) << week << mSep << setprecision(3) << setw(10) << gpst;
}


void WriteResultFile::WritePositionECEF(){
    mOutfile << fixed << setprecision(4) << mSep
             << setw(15) << mSol->pos[0] << mSep
             << setw(15) << mSol->pos[1] << mSep
             << setw(15) << mSol->pos[2];

    mOutfile << fixed << setprecision(4) << mSep
             << setw(9) << sqrt(mSol->Qpos(0,0)) << mSep
             << setw(9) << sqrt(mSol->Qpos(1,1)) << mSep
             << setw(9) << sqrt(mSol->Qpos(2,2));
}


void WriteResultFile::WritePositionBLH(){
    double dh = 0;
    Matrix3d E  = mSol->Qpos.block<3, 3>(0,0);

    mSol->blh = CEarth::ecef2blh(mSol->pos);
    E = CEarth::xyz2enu(mSol->blh);
    E = E * mSol->Qpos.block<3, 3>(0,0) * E.transpose();

    if (config.ExportHeightDatum == HEIGHT_GEODETIC)  dh = CEarth::geoidh(mSol->blh);

    if        (config.ExportBLHFormat == BLHF_DEG)
    {
        mOutfile << fixed << setprecision(9) << mSep
                 << setw(15) << mSol->blh[0] * R2D << mSep
                 << setw(15) << mSol->blh[1] * R2D << mSep
                 << setw(10) << setprecision(4) << mSol->blh[2] + dh;
    }
    else if (config.ExportBLHFormat == BLHF_DMS)
    {
        vector<double> lat = CMaths::deg2dms(mSol->blh[0] * R2D, 5);
        vector<double> lon = CMaths::deg2dms(mSol->blh[1] * R2D, 5);
        mOutfile << fixed << setprecision(0) << mSep
                 << setw(4) << lat[0] << mSep << setw(2) << lat[1] << mSep
                 << setprecision(5) << setw(7) << lat[2];
        mOutfile << fixed << setprecision(0) << mSep
                 << setw(4) << lon[0] << mSep << setw(2) << lon[1] << mSep
                 << setprecision(5) << setw(7) << lon[2];
        mOutfile << fixed << setprecision(4) << mSep << setw(10) << mSol->blh[2] + dh;
    }

    mOutfile << fixed << setprecision(4) << mSep
             << setw(9) << sqrt(E(0,0)) << mSep
             << setw(9) << sqrt(E(1,1)) << mSep
             << setw(9) << sqrt(E(2,2));

}


void WriteResultFile::WriteVelocityECEF(){
    if ((!config.VelocityMode || mSol->vstat == VELQ_NONE))
    {
        mSol->Qvel = MatrixXd::Zero(3, 3);
    }

    Vector3d vel;
    Matrix3d Qvel;

    if (config.ProcessMode != PMODE_SINGLE && config.VelocityMode > VELOPT_OFF && config.SolverMode != SOLVER_GO)
    {   // instantaneous velocity
        vel  = mSol->veli;
        Qvel = mSol->Qveli.block<3, 3>(0,0);
    }
    else
    {   // average velocity
        vel  = mSol->vel;
        Qvel = mSol->Qvel.block<3, 3>(0,0);
    }

    mOutfile << fixed << setprecision(3) << mSep
             << setw(10) << vel[0] << mSep
             << setw(10) << vel[1] << mSep
             << setw(10) << vel[2];

    mOutfile << fixed << setprecision(3) << mSep
             << setw(9) << sqrt(Qvel(0,0)) << mSep
             << setw(9) << sqrt(Qvel(1,1)) << mSep
             << setw(9) << sqrt(Qvel(2,2));
}


void WriteResultFile::WriteVelocityBLH(){
    Matrix3d E  = MatrixXd::Zero(3, 3);

    if ((!config.VelocityMode || mSol->vstat == VELQ_NONE))
    {
        mSol->Qvel = MatrixXd::Zero(3, 3);
    }

    Vector3d vel;
    Matrix3d Qvel;

    if (config.VelocityMode == VELOPT_DOPPLER)
    {   // instantaneous velocity
        vel  = mSol->veli;
        Qvel = mSol->Qveli.block<3, 3>(0,0);
    }
    else
    {   // average velocity
        vel  = mSol->vel;
        Qvel = mSol->Qvel.block<3, 3>(0,0);
    }

    mSol->veln = CEarth::ecef2enu(mSol->blh, vel);
    E = CEarth::xyz2enu(mSol->blh);
    E = E * Qvel.block<3, 3>(0,0) * E.transpose();

    mOutfile << fixed << setprecision(3) << mSep
             << setw(10) << mSol->veln[1] << mSep
             << setw(10) << mSol->veln[0] << mSep
             << setw(10) << mSol->veln[2];

    mOutfile << fixed << setprecision(3) << mSep
             << setw(9) << sqrt(E(0,0))   << mSep
             << setw(9) << sqrt(E(1,1))   << mSep
             << setw(9) << sqrt(E(2,2));
}


void WriteResultFile::WriteAccelerationECEF(){
    mOutfile << fixed << setprecision(3) << mSep
             << setw(10) << mSol->acc[0] << mSep
             << setw(10) << mSol->acc[1] << mSep
             << setw(10) << mSol->acc[2];
}


void WriteResultFile::WriteAccelerationBLH(){
    Vector3d accn = CEarth::ecef2enu(mSol->blh, mSol->acc);

    mOutfile << fixed << setprecision(3) << mSep
             << setw(10) << accn[1] << mSep
             << setw(10) << accn[0] << mSep
             << setw(10) << accn[2];
}


void WriteResultFile::WriteClock(){
    if (config.System[1]) mOutfile << fixed << setprecision(3) << mSep << setw(8) << mSol->drck[0];   //GPS
    if (config.System[2])
    {
        if (config.BdsOption == BDSOPT_BD2 || config.BdsOption == BDSOPT_BD23)
        {
            mOutfile << fixed << setprecision(3) << mSep << setw(8) << mSol->drck[1];   //BD2
        }
        if (config.BdsOption == BDSOPT_BD3 || config.BdsOption == BDSOPT_BD23)
        {
            mOutfile << fixed << setprecision(3) << mSep << setw(8) << mSol->drck[4];   //BD3
        }
    }
    if (config.System[3]) mOutfile << fixed << setprecision(3) << mSep << setw(8) << mSol->drck[2];  //GLO
    if (config.System[4]) mOutfile << fixed << setprecision(3) << mSep << setw(8) << mSol->drck[3];  //GAL
}


void WriteResultFile::WriteClockDrift(){
    mOutfile << fixed << setprecision(3) << mSep << setw(8) << mSol->drcd;
}


WriteTempResult::WriteTempResult(){
    mOutfile.open(config.Temp_File, ios::binary);
    if (!mOutfile.is_open())
    {
        Log.Trace(TERROR, "*Temp result file: " + config.Temp_File + " cann't open, please check");
        exit(-1);
    }
}


bool WriteTempResult::Export(Solution& sol){
    mSol = &sol;

    if (mSol->stat == SOLQ_NONE) return false;

    double tt = double(mSol->time.time) + mSol->time.sec;
    mOutfile.write((char*)&tt, sizeof(double));   // time

    this->WriteStates();
    this->WriteVariance();

    mOutfile.write((char*)&mSol->nsat[0], sizeof(int)); // number of sat
    mOutfile.write((char*)&mSol->stat, sizeof(int));    // solution stat

    return true;
}


void WriteTempResult::Close(){
    if (mOutfile.is_open()) mOutfile.close();
}


void WriteTempResult::WriteStates(){
    mOutfile.write((char*)&mSol->pos, sizeof(mSol->pos));    // pos

    if (config.VelocityMode > VELOPT_OFF)  // instantaneous vel
    {
        mOutfile.write((char*)&mSol->veli, sizeof(mSol->veli));
    }
    else  // mean vel
    {
        mOutfile.write((char*)&mSol->vel, sizeof(mSol->vel));
    }

}


void WriteTempResult::WriteVariance(){
    Vector3d Qvel = Vector3d::Zero();
    Vector3d Qpos(mSol->Qpos(0, 0), mSol->Qpos(1, 1), mSol->Qpos(2, 2));   //pos

    if (config.VelocityMode > VELOPT_OFF)  // instantaneous vel
    {
        Qvel<<mSol->Qveli(0, 0), mSol->Qveli(1, 1), mSol->Qveli(2, 2);
    }
    else  // mean vel
    {
        Qvel<<mSol->Qvel(0, 0), mSol->Qvel(1, 1), mSol->Qvel(2, 2);
    }

    mOutfile.write((char*)&Qpos, sizeof(Qpos));
    mOutfile.write((char*)&Qvel, sizeof(Qvel));
}
