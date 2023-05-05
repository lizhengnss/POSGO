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
 * Created by lizhen on 2021/7/2.
 *-------------------------------------------------------------------------------*/

#include "Configuration.h"

vector<vector<double>> CTimes::gLeapsec;

bool Config::PosGoPasteCommand(char *argv[]){
    if (!strstr(argv[1],"-C") || !strstr(argv[3],"-S") || !strstr(argv[5],"-M") || !strstr(argv[7],"-L"))
    {
        this->PrintPosGoUsage();
        return false;
    }

    mFilename = argv[2];
    if (access(mFilename.c_str(), F_OK))
    {
        Log.Trace(TERROR, "*PasteCommand: Config file doesn't exit, Please check: " + mFilename);
        return false;
    }
    this->InitConfig();

    mBuff = argv[4];
    if(!this->GetVaildSystem())
    {
        Log.Trace(TERROR, "*PasteCommand: System set error: " + mBuff);
        return false;
    }

    mBuff = argv[6];
    if(!this->GetSloveMode())
    {
        Log.Trace(TERROR, "*PasteCommand: Slove mode set error: " + mBuff);
        return false;
    }

    mBuff = argv[8];
    config.DebugLevel = (int)strtod(mBuff.c_str(), nullptr);
    Log.TraceLevel(config.DebugLevel);
    return true;
}


void Config::PrintPosGoUsage(){
    Log.Trace(TERROR, "*Usage: POSGO -C [config] -S ['GREC'] -M [SolveMode] -L [TraceLevel]");
}



void Config::InitConfig(){
    config.ProcessMode    = PMODE_SINGLE; //SPP
    config.SolverMode     = SOLVER_LSQ;
    config.SolveType      = MODE_FORWARD;

    config.NumFreq         = 2;
    config.SampleRate      = 1;
    config.StartTime.time  = 0.0;
    config.EndTime.time    = 0.0;
    config.GPS_FreqID = {0, 1};
    config.BD2_FreqID = {0, 2};
    config.BD3_FreqID = {0, 2};
    config.GLO_FreqID = {0, 1};
    config.GAL_FreqID = {0, 1};

    config.GPS_Freq = {FREQ1,     FREQ2    };   // L1  + L2
    config.BD2_Freq = {FREQ1_CMP, FREQ3_CMP};   // B1I + B3I
    config.BD3_Freq = {FREQ1_CMP, FREQ3_CMP};   // B1I + B3I
    config.GLO_Freq = {FREQ1_GLO, FREQ2_GLO};   // G1  + G2
    config.GAL_Freq = {FREQ1,     FREQ7};       // E1  + E5b

    config.Ionospheric  = IONOOPT_OFF;
    config.Tropospheric = TROPOPT_OFF;
    config.TropMapFunction = TROPMAP_NMF;
    config.CodeBias     = CODEBIAS_TGD;
    config.WeightMode   = WEIGHTOPT_ELEVATION;
    config.VelocityMode = VELOPT_DOPPLER;
    config.BdsOption    = BDSOPT_BD23;
    config.MaxResidual  = 10;

    config.PseudorangeSmooth = SMOOTH_OFF;
    config.PseudorangeSmoothWindow = 50;
    config.PseudorangeSmoothThreshold = 30;

    config.Elevation          = 7.0 * D2R;
    config.AdjustObservation  = false;
    config.ExcludeSatellite   = "";

    config.LossFunction = FGLOSSFUN_HUBER;
    config.SlideWindowSize = 5;
    config.LossFunctionValue = 1.0;

    config.sep                  = " ";
    config.ExportHead           = SWITCH_ON;
    config.ExportVelocity       = SWITCH_OFF;
    config.ExportAcceleration   = SWITCH_OFF;
    config.ExportClock          = SWITCH_OFF;
    config.ExportClockDrift     = SWITCH_OFF;
    config.ExportDop            = SWITCH_OFF;
    config.ExportCoordinate     = SOLF_XYZ;
    config.ExportTimeFormat     = TIMEF_TOW;
    config.ExportTimeSystem     = TIMES_GPST;
    config.ExportHeightDatum    = HEIGHT_ELLIPS;
    config.ExportBLHFormat      = BLHF_DEG;

    config.Result_File = "result.txt";
    config.Temp_File = "forward.bin";
    config.Leapsec_File = "tables/leap.sec";

    config.DebugLevel = TERROR;
    config.DebugFile = "trace.log";
    config.DebugRobust = false;
    config.DebugResPos = false;
    config.DebugResVel = false;

}


void Config::DeleteComments(){
    if (mBuff.find("#") == string::npos) return; // no comments
    int id = mBuff.find("#");
    mBuff = mBuff.substr(0, id);
    mBuff.erase(mBuff.find_last_not_of(" ") + 1, mBuff.length());
}


bool Config::CheckConfigSetting(){

    if (config.Ionospheric == IONOOPT_IFLC && config.NumFreq < 2)
    {
        Log.Trace(TERROR, "*Ionospheric combination need 2 or more frequency" );
        return false;
    }

    if (config.ProcessMode == PMODE_RTK && config.GnssObs_Base_File.empty())
    {
        Log.Trace(TERROR, "*RTK requires base station GNSS observation!");
        return false;
    }

    if (fabs(config.BasePos[0]) < 1000 && fabs(config.BasePos[1]) < 1000)
    {   // blh -> xyz
        config.BasePos[0] *= D2R;
        config.BasePos[1] *= D2R;
        config.BasePos = CEarth::blh2ecef(config.BasePos);
    }
    config.BaseBLH = CEarth::ecef2blh(config.BasePos);

    return true;
}


bool Config::GetVaildSystem(){
    for (int i = 0; i < int(mBuff.length()); i++)
    {
        size_t sys = sSysCodes.find(mBuff.substr(i, 1));
        if(sys == string::npos) return false;
    }
    if (mBuff.find("G") != string::npos)
    {
        config.System[SYS_GPS] = true;
    }
    if (mBuff.find("C") != string::npos)
    {
        config.System[SYS_BDS] = true;
    }
    if (mBuff.find("R") != string::npos)
    {
        config.System[SYS_GLO] = true;
    }
    if (mBuff.find("E") != string::npos)
    {
        config.System[SYS_GAL] = true;
    }
    return true;
}


bool Config::GetSloveMode(){
    if      (mBuff.find("SPP") != string::npos || mBuff.find("spp") != string::npos)
    {
        config.ProcessMode = PMODE_SINGLE; return true;
    }
    else if (mBuff.find("RTK") != string::npos || mBuff.find("rtk") != string::npos)
    {
        config.ProcessMode = PMODE_RTK; return true;
    }
    else
    {
        return false;
    }
}


double Config::GetConfigInformation(vector<string> info){
    if (info.size() < 2)
    {
        Log.Trace(TERROR, "Config: need set parameter: " + CString::trim(info[0]));
        exit(-1);
    }
    return CString::str2num(info[1], 0, info[1].size());
}


int Config::GetConfigInformation(vector<string> info, const vector<string> opt){
    if (info.size() < 2)
    {
        Log.Trace(TERROR, "Config: need set parameter: " + CString::trim(info[0]));
        exit(-1);
    }
    return GetOptionID(info[1], opt);
}


void Config::GetConfigInformation(vector<string> info, double *cfg, int num){
    if (info.size() < 2)
    {
        Log.Trace(TERROR, "Config: need set parameter: " + CString::trim(info[0]));
        exit(-1);
    }
    vector<string> tp = CString::split(info[1], ",");
    if (tp.size() < num)
    {
        Log.Trace(TERROR, "Config: " + CString::trim(info[0]) + " parameter number isn't enough, now set " + to_string(tp.size())
                            + " and program requires " + to_string(num));
        exit(-1);
    }
    for (int i = 0; i < num; i++) cfg[i] = CString::str2num(tp[i], 0, tp[i].size());
}


void Config::GetConfigInformation(vector<string> info, Vector3d& cfg){
    if (info.size() < 2)
    {
        Log.Trace(TERROR, "Config: need set parameter: " + info[0]);
        exit(-1);
    }
    vector<string> tp = CString::split(info[1], ",");
    if (tp.size() < 3)
    {
        Log.Trace(TERROR, "Config: " + CString::trim(info[0]) + " parameter number isn't enough, now set " + to_string(tp.size())
                            + " and program requires 3");
        exit(-1);
    }
    for (int i = 0; i < 3; i++) cfg[i] = CString::str2num(tp[i], 0, tp[i].size());
}


void Config::GetConfigInformation(vector<string> info, VectorXd& cfg){
    if (info.size() < 2)
    {
        Log.Trace(TERROR, "Config: need set parameter: " + info[0]);
        exit(-1);
    }
    vector<string> tp = CString::split(info[1], ",");
    if (tp.size() < cfg.size())
    {
        Log.Trace(TERROR, "Config: " + CString::trim(info[0]) + " parameter number isn't enough, now set " + to_string(tp.size())
                            + " and program requires " + to_string(cfg.size()));
        exit(-1);
    }
    for (int i = 0; i < cfg.size(); i++) cfg[i] = CString::str2num(tp[i], 0, tp[i].size());
}


string Config::GetConfigFilename(vector<string> info){
    if (info.size() < 2)
    {
        Log.Trace(TERROR, "Config: need set parameter: " + info[0]);
        exit(-1);
    }
    return CString::trim(info[1]);
}


bool Config::SetFequencyIndex(int sys, const vector<string> freq){
    switch (sys)
    {
        case SYS_GPS:  config.GPS_FreqID.clear(); break;
        case SYS_BDS:  config.BD2_FreqID.clear(); break;
        case SYS_BD3:  config.BD3_FreqID.clear(); break;
        case SYS_GLO:  config.GLO_FreqID.clear(); break;
        case SYS_GAL:  config.GAL_FreqID.clear(); break;
        default:       Log.Trace(TERROR, "SetFequencyIndex: Check system!"); return false;
    }
    for (int i = 0; i < freq.size(); i++)
    {
        switch (sys)
        {
            case SYS_GPS:  config.GPS_FreqID.push_back(GetFequencyIndex(SYS_GPS - 1, freq[i])); break;
            case SYS_BDS:  config.BD2_FreqID.push_back(GetFequencyIndex(SYS_BDS - 1, freq[i])); break;
            case SYS_BD3:  config.BD3_FreqID.push_back(GetFequencyIndex(SYS_BDS - 1, freq[i])); break;
            case SYS_GLO:  config.GLO_FreqID.push_back(GetFequencyIndex(SYS_GLO - 1, freq[i])); break;
            case SYS_GAL:  config.GAL_FreqID.push_back(GetFequencyIndex(SYS_GAL - 1, freq[i])); break;
        }
    }
    return true;
}


int Config::GetFequencyIndex(int sys, string freq){
    CString::trim(freq);
 //   if (sys==SYS_BDS-1 && freq == "B2b") return 1;  // B2b is B2I
    for (int i = 0; i < MAXFREQ; i++)
    {
        if (freq == mFreqStr[sys][i]) return i;
    }
    Log.Trace(TERROR, "Config: Frequency set error  " + freq);
    exit(-1);
}


double Config::GetFequency(int sys, int freqID){
    double frquency[4][MAXFREQ]={
            {FREQ1,     FREQ2,     FREQ5,      0.0,        0.0,        0.0,   0.0},       /*GPS*/
            {FREQ1_CMP, FREQ2_CMP, FREQ3_CMP,  FREQ1,      FREQ5,      FREQ8, FREQ2_CMP}, /*BDS*/
            {FREQ1_GLO, FREQ2_GLO, FREQ3_GLO,  FREQ1a_GLO, FREQ2a_GLO, 0.0,   0.0},       /*GLO*/
            {FREQ1,     FREQ7,     FREQ5,      FREQ6,      FREQ8,      0.0,   0.0},       /*GAL*/
    };
    return frquency[sys][freqID];
}


bool Config::SetFequency(int sys){
    switch (sys)
    {
        case SYS_GPS:  config.GPS_Freq.clear(); break;
        case SYS_BDS:  config.BD2_Freq.clear(); break;
        case SYS_BD3:  config.BD3_Freq.clear(); break;
        case SYS_GLO:  config.GLO_Freq.clear(); break;
        case SYS_GAL:  config.GAL_Freq.clear(); break;
        default:       Log.Trace(TERROR, "SetFequency: Check system!"); return false;
    }
    for (int i = 0; i < config.NumFreq; i++)
    {
        switch (sys)
        {
            case SYS_GPS:  config.GPS_Freq.push_back(GetFequency(SYS_GPS - 1, config.GPS_FreqID[i])); break;
            case SYS_BDS:  config.BD2_Freq.push_back(GetFequency(SYS_BDS - 1, config.BD2_FreqID[i])); break;
            case SYS_BD3:  config.BD3_Freq.push_back(GetFequency(SYS_BDS - 1, config.BD3_FreqID[i])); break;
            case SYS_GLO:  config.GLO_Freq.push_back(GetFequency(SYS_GLO - 1, config.GLO_FreqID[i])); break;
            case SYS_GAL:  config.GAL_Freq.push_back(GetFequency(SYS_GAL - 1, config.GAL_FreqID[i])); break;
        }
    }
    return true;
}


int Config::GetOptionID(string opt, const vector<string> candidate){
    CString::trim(opt);
    for(int i = 0; i < candidate.size(); i++)
    {
        if (candidate[i].find(opt)!= string::npos)
        {
            return i;
        }
    }
    Log.Trace(TERROR, "Config: set parameter error, " + mBuff);
    exit(-1);
}


void Config::GetStartEndTime(vector<string> info){
    vector<double> ep;
    vector<string> tp, ttp, tp1, tp2;

    if (info.size() < 2)
    {
        Log.Trace(TERROR, "Config: need set parameter: " + info[0]);
        exit(-1);
    }
    tp = CString::split(info[1], "-");
    if (tp.size() < 2)
    {
        Log.Trace(TERROR, "Config: set time parameter error: " + info[0]);
        exit(-1);
    }

    ttp = CString::split(tp[0], " ");   // start time
    tp1 = CString::split(ttp[0], "/");
    tp2 = CString::split(ttp[1], ":");
    for (int i = 0; i < 3; i++) ep.push_back(strtod(tp1[i].c_str(), nullptr));
    for (int i = 0; i < 3; i++) ep.push_back(strtod(tp2[i].c_str(), nullptr));
    config.StartTime = CTimes::epoch2time(ep);

    ttp = CString::split(tp[1], " ");   // end time
    tp1 = CString::split(ttp[0], "/");
    tp2 = CString::split(ttp[1], ":");
    ep.clear();
    for (int i = 0; i < 3; i++) ep.push_back(strtod(tp1[i].c_str(), nullptr));
    for (int i = 0; i < 3; i++) ep.push_back(strtod(tp2[i].c_str(), nullptr));
    config.EndTime = CTimes::epoch2time(ep);
}


bool Config::ReadLeapSecond(){
    if (access(config.Leapsec_File.c_str(), F_OK))
    {
        Log.Trace(TERROR, "*ReadLeapSecond: Leap second file doesn't exit, Please check: " + config.Leapsec_File);
        return false;
    }

    mInfile.open(config.Leapsec_File, ios::in);
    if (!mInfile.is_open())
    {
        Log.Trace(TERROR, "*Leap second file: " + config.Leapsec_File + " cann't open, please check");
        return false;
    }

    while (!mInfile.eof())
    {
        getline(mInfile, mBuff);
        if (mBuff.substr(0, 1) == "#") continue;
        this->DeleteComments();
        mTempdata = CString::split(mBuff, ",");

        if (mTempdata.size() <= 1) continue;
        if (mTempdata.size() < 7)
        {
            Log.Trace(TERROR, "*ReadLeapSecond: Format error, please check " + mBuff);
            return false;
        }

        vector<double> epoch;
        for (int i = 0; i <= 6; i++)
        {
            epoch.push_back(CString::str2num(mTempdata[i], 0, mTempdata[i].size()));
        }
        CTimes::gLeapsec.push_back(epoch);
    }

    mInfile.close();
    return true;
}


bool Config::ReadConfigFile() {
    mInfile.open(mFilename, ios::in);
    if (!mInfile.is_open())
    {
        Log.Trace(TERROR, "Config file: " + mFilename + " cann't open, please check");
        return false;
    }

    while (!mInfile.eof())
    {
        getline(mInfile, mBuff);
        if (mBuff.substr(0, 1) == "#") continue;
        this->DeleteComments();
        mTempdata = CString::split(mBuff, "=");

        if      (mBuff.find("NumFreq") != string::npos)
        {
            config.NumFreq = (int)GetConfigInformation(mTempdata);
        }
        else if (mBuff.find("SolverMode") != string::npos)
        {
            config.SolverMode = GetConfigInformation(mTempdata, opt_Solver);
        }
        else if (mBuff.find("SolveType") != string::npos)
        {
            config.SolveType = GetConfigInformation(mTempdata, opt_SolveType);
        }
        else if (mBuff.find("GPS_Freq") != string::npos)
        {
            vector<string> freq = CString::split(mTempdata[1], "+");
            if (freq.size() != config.NumFreq)
            {
                Log.Trace(TERROR, "Frequency number of GPS set error: " + mTempdata[1] +
                                  ", The number of frequency set is: " + to_string(config.NumFreq));
                exit(-1);
            }
            SetFequencyIndex(SYS_GPS, freq);
            SetFequency(SYS_GPS);
        }
        else if (mBuff.find("BD2_Freq") != string::npos)
        {
            vector<string> freq = CString::split(mTempdata[1], "+");
            if (freq.size() != config.NumFreq)
            {
                Log.Trace(TERROR, "Frequency number of BD2 set error: " + mTempdata[1] +
                                  ", The number of frequency set is: " + to_string(config.NumFreq));
                exit(-1);
            }
            SetFequencyIndex(SYS_BDS, freq);
            SetFequency(SYS_BDS);
        }
        else if (mBuff.find("BD3_Freq") != string::npos)
        {
            vector<string> freq = CString::split(mTempdata[1], "+");
            if (freq.size() != config.NumFreq)
            {
                Log.Trace(TERROR, "Frequency number of BD3 set error: " + mTempdata[1] +
                                  ", The number of frequency set is: " + to_string(config.NumFreq));
                exit(-1);
            }
            SetFequencyIndex(SYS_BD3, freq);
            SetFequency(SYS_BD3);
        }
        else if (mBuff.find("GLO_Freq") != string::npos)
        {
            vector<string> freq = CString::split(mTempdata[1], "+");
            if (freq.size() != config.NumFreq)
            {
                Log.Trace(TERROR, "Frequency number of GLONASS set error: " + mTempdata[1] +
                                  ", The number of frequency set is: " + to_string(config.NumFreq));
                exit(-1);
            }
            SetFequencyIndex(SYS_GLO, freq);
            SetFequency(SYS_GLO);
        }
        else if (mBuff.find("GAL_Freq") != string::npos)
        {
            vector<string> freq = CString::split(mTempdata[1], "+");
            if (freq.size() != config.NumFreq)
            {
                Log.Trace(TERROR, "Frequency number of GAL set error: " + mTempdata[1] +
                                  ", The number of frequency set is: " + to_string(config.NumFreq));
                exit(-1);
            }
            SetFequencyIndex(SYS_GAL, freq);
            SetFequency(SYS_GAL);
        }
        else if (mBuff.find("Start_End_Time") != string::npos)
        {
            this->GetStartEndTime(mTempdata);
        }
        else if (mBuff.find("GnssSampleRate") != string::npos)
        {
            config.SampleRate = GetConfigInformation(mTempdata);
        }
        else if (mBuff.find("Ionospheric") != string::npos)
        {
            config.Ionospheric = GetConfigInformation(mTempdata, opt_Iono);
        }
        else if (mBuff.find("Tropospheric") != string::npos)
        {
            config.Tropospheric = GetConfigInformation(mTempdata, opt_Trop);
        }
        else if (mBuff.find("TropMapFunction") != string::npos)
        {
            config.TropMapFunction = GetConfigInformation(mTempdata, opt_TropMap);
        }
        else if (mBuff.find("CodeBias") != string::npos)
        {
            config.CodeBias = GetConfigInformation(mTempdata, opt_Code);
        }
        else if (mBuff.find("BdsOption") != string::npos)
        {
            config.BdsOption = GetConfigInformation(mTempdata, opt_BdsOption);
        }
        else if (mBuff.find("ExcludeSatellite") != string::npos)
        {
            if (mTempdata.size() >= 2)
            {
                config.ExcludeSatellite = mTempdata[1];
            }
        }
        else if (mBuff.find("ElevationAngle") != string::npos)
        {
            config.Elevation = GetConfigInformation(mTempdata) * D2R;
        }
        else if (mBuff.find("AdjustObservation") != string::npos)
        {
            config.AdjustObservation = GetConfigInformation(mTempdata, opt_Switch);
        }
        else if (mBuff.find("ErrorRatio") != string::npos)
        {
            this->GetConfigInformation(mTempdata, config.ErrRatio, 3);
        }
        else if (mBuff.find("MeasurementError") != string::npos)
        {
            this->GetConfigInformation(mTempdata, config.Error, 5);
        }
        else if (mBuff.find("InitialStateError") != string::npos)
        {
            this->GetConfigInformation(mTempdata, config.Std, 3);
        }
        else if (mBuff.find("ProcessError") != string::npos)
        {
            this->GetConfigInformation(mTempdata, config.Process, 5);
        }
        else if (mBuff.find("RobustThres") != string::npos)
        {
            this->GetConfigInformation(mTempdata, config.RobThres, 3);
        }
        else if (mBuff.find("SnrMask") != string::npos)
        {
            this->GetConfigInformation(mTempdata, config.SnrMask, 9);
        }
        else if (mBuff.find("MaxResidual") != string::npos)
        {
            config.MaxResidual = GetConfigInformation(mTempdata);
        }
        else if (mBuff.find("WeightMode") != string::npos)
        {
            config.WeightMode = GetConfigInformation(mTempdata, opt_WeightMode);
        }
        else if (mBuff.find("RobustMode") != string::npos)
        {
            config.RobustMode = GetConfigInformation(mTempdata, opt_Robust);
        }
        else if (mBuff.find("PseudorangeSmooth") != string::npos){
            mTempdata = CString::split(mTempdata[1], ",");
            config.PseudorangeSmooth          = GetOptionID(mTempdata[0], opt_PsrSmooth);
            if (config.PseudorangeSmooth == SMOOTH_OFF) continue;
            config.PseudorangeSmoothWindow    = int(CString::str2num(mTempdata[1], 0, mTempdata[2].size()));
            config.PseudorangeSmoothThreshold = CString::str2num(mTempdata[2], 0, mTempdata[2].size());
        }
        else if (mBuff.find("LossFunction") != string::npos)
        {
            config.LossFunction = GetConfigInformation(mTempdata, opt_LossFun);
        }
        else if (mBuff.find("LossFunValue") != string::npos)
        {
            config.LossFunctionValue = GetConfigInformation(mTempdata);
        }
        else if (mBuff.find("SlideWindowSize") != string::npos)
        {
            config.SlideWindowSize = (int)GetConfigInformation(mTempdata);
        }
        else if (mBuff.find("Velocimetry") != string::npos)
        {
            config.VelocityMode = GetConfigInformation(mTempdata, opt_Velocity);
        }
        else if (mBuff.find("BaseStationPos") != string::npos)
        {
            this->GetConfigInformation(mTempdata, config.BasePos);
        }
        else if (mBuff.find("ExportSeparator") != string::npos)
        {
            if (mTempdata.size() >= 2)
            {
                config.sep = CString::trim(mTempdata[1]);
            }
            if (config.sep.length() < 1 || mTempdata.size() < 2)
            {
                config.sep = " ";
            }
        }
        else if (mBuff.find("ExportHead") != string::npos)
        {
            config.ExportHead = GetConfigInformation(mTempdata, opt_Switch);
        }
        else if (mBuff.find("ExportTimeFormat") != string::npos)
        {
            config.ExportTimeFormat = GetConfigInformation(mTempdata, opt_TimeFormat);
        }
        else if (mBuff.find("ExportTimeSystem") != string::npos)
        {
            config.ExportTimeSystem = GetConfigInformation(mTempdata, opt_TimeSystem);
        }
        else if (mBuff.find("ExportCoordinate") != string::npos)
        {
            config.ExportCoordinate = GetConfigInformation(mTempdata, opt_Coord);
        }
        else if (mBuff.find("ExportBLHFormat") != string::npos)
        {
            config.ExportBLHFormat = GetConfigInformation(mTempdata, opt_BLHFormat);
        }
        else if (mBuff.find("ExportHeightDatum") != string::npos)
        {
            config.ExportHeightDatum = GetConfigInformation(mTempdata, opt_Heigth);
        }
        else if (mBuff.find("ExportVelocity") != string::npos)
        {
            config.ExportVelocity = GetConfigInformation(mTempdata, opt_Switch);
        }
        else if (mBuff.find("ExportAcceleration") != string::npos)
        {
            config.ExportAcceleration = GetConfigInformation(mTempdata, opt_Switch);
        }
        else if (mBuff.find("ExportClockBias") != string::npos)
        {
            config.ExportClock = GetConfigInformation(mTempdata, opt_Switch);
        }
        else if (mBuff.find("ExportClockDrift") != string::npos)
        {
            config.ExportClockDrift = GetConfigInformation(mTempdata, opt_Switch);
        }
        else if (mBuff.find("ExportDop") != string::npos)
        {
            config.ExportDop = GetConfigInformation(mTempdata, opt_Switch);
        }

        else if (mBuff.find("File-GnssObs") != string::npos)
        {
            config.GnssObs_File = GetConfigFilename(mTempdata);
        }
        else if (mBuff.find("File-GnssBase") != string::npos)
        {
            config.GnssObs_Base_File = GetConfigFilename(mTempdata);
        }
        else if (mBuff.find("File-Navigation") != string::npos)
        {
            config.Nav_File = GetConfigFilename(mTempdata);
        }
        else if (mBuff.find("File-GpsNavigation") != string::npos)
        {
            config.Nav_Sys_File[0] = GetConfigFilename(mTempdata);
        }
        else if (mBuff.find("File-BdsNavigation") != string::npos)
        {
            config.Nav_Sys_File[1] = GetConfigFilename(mTempdata);
        }
        else if (mBuff.find("File-GloNavigation") != string::npos)
        {
            config.Nav_Sys_File[2] = GetConfigFilename(mTempdata);
        }
        else if (mBuff.find("File-GalNavigation") != string::npos)
        {
            config.Nav_Sys_File[3] = GetConfigFilename(mTempdata);
        }
        else if (mBuff.find("File-Ion") != string::npos)
        {
            config.Ion_File = GetConfigFilename(mTempdata);
        }
        else if (mBuff.find("File-Result") != string::npos)
        {
            config.Result_File = GetConfigFilename(mTempdata);
        }
        else if (mBuff.find("File-Leapsec") != string::npos)
        {
            config.Leapsec_File = GetConfigFilename(mTempdata);
        }
        else if (mBuff.find("DebugFile") != string::npos)
        {
            config.DebugFile = GetConfigFilename(mTempdata);
        }
        else if (mBuff.find("DebugRobust") != string::npos)
        {
            config.DebugRobust = GetConfigInformation(mTempdata, opt_Switch);
        }
        else if (mBuff.find("DebugResidual") != string::npos)
        {
            if (mTempdata.size() >= 2)
            {
                vector<string> tp = CString::split(mTempdata[1], ",");
                config.DebugResPos = GetOptionID(tp[0], opt_Switch);  // debug pos res
                config.DebugResVel = GetOptionID(tp[1], opt_Switch);  // debut vel res
            }
        }
    }

    mInfile.close();

    if (!this->ReadLeapSecond()) return false;

    if (!this->CheckConfigSetting()) return false;
    return true;
}
