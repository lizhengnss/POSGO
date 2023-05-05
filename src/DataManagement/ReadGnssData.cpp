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
 * Created by lizhen on 2021/7/22.
 *-------------------------------------------------------------------------------*/

#include "ReadGnssData.h"


bool ReadRinexObs::ReadHead() {
    int i, j, k, nt;
    while (true)
    {
        getline(mInfile, mBuff);
        if (mBuff.find("END OF HEADER") != string::npos) return true;
        if (EndFile()) return false;
        mTempdata =  CString::split(mBuff, " ");

        if (mBuff.find("RINEX VERSION / TYPE") != string::npos)
        {
            mRover.version = CString::str2num(mBuff, 0, 9);
        }
        else if (mBuff.find("MARKER NAME") != string::npos)
        {
            mRover.name = mBuff.substr(0, 60);
        }
        else if (mBuff.find("MARKER NUMBER") != string::npos)
        {
            mRover.marker = mBuff.substr(0, 20);
        }
        else if (mBuff.find("REC # / TYPE / VERS" ) != string::npos)
        {
            mRover.recsno  = mBuff.substr(0, 20);
            mRover.rectype = mBuff.substr(20, 20);
            mRover.recver  = mBuff.substr(40, 20);
        }
        else if (mBuff.find("ANT # / TYPE"        ) != string::npos)
        {
            mRover.antsno = mBuff.substr(0, 20);
            mRover.antdes = mBuff.substr(20, 20);
        }
        else if (mBuff.find("APPROX POSITION XYZ" ) != string::npos)
        {
            for (i = 0; i < 3; i++) mRover.pos[i] = strtod(mTempdata[i].c_str(), nullptr);
        }
        else if (mBuff.find("ANTENNA: DELTA H/E/N") != string::npos)
        {
            for (i = 0; i < 3; i++) mRover.del[i] = strtod(mTempdata[i].c_str(), nullptr);
        }
        else if (mBuff.find("SYS / # / OBS TYPES") != string::npos)
        {
            size_t sys = sSysCodes.find(mBuff.substr(0, 1));
            int ntype = int(CString::str2num(mBuff, 3, 3));

            if(sys == string::npos)
            {
                Log.Trace(TWARNING, "Invalid system code: sys=" + mBuff.substr(0, 1));
                continue;
            }

            for (j = 0, k = 7, nt = 0; j < ntype; j++, k+=4)
            {
                if (k > 58)
                {
                    getline(mInfile, mBuff);
                    k = 7;
                }
                mPriIndex[sys][nt++] = mBuff.substr(k, 3);
            }

            mPriIndex[sys][nt]='\0';
            /* change BDS B1 code: 3.02 */
            if (sys + 1 == SYS_BDS && fabs(mRover.version - 3.02) < 1e-3)
            {
                for (j = 0; j < nt; j++) if (mPriIndex[sys][j][1] == '1') mPriIndex[sys][j].replace(1, 1, "2");
            }
        }

    }
}


bool ReadRinexObs::Reading() {
    while (true)
    {
        this->ReadInit();
        mEndOfFile = this->ReadEpochHead();
        if (!mEndOfFile) return false;
        this->ReadEpochData();

        if (config.StartTime.time > 0 && CTimes::timediff(mObs.time, config.StartTime) < 0 ) continue;      // obs earlier than setting
        if (config.EndTime.time   > 0 && CTimes::timediff(mObs.time, config.EndTime)   > 0 ) return false;  // obs later than setting

        return true;
    }
}


GnssObsEpoch ReadRinexObs:: GetObsData(){
    if (config.AdjustObservation) this->AdjustObservation();
    if (config.VelocityMode == VELOPT_TDCP)  this->FormTDCP();
    if (config.PseudorangeSmooth > SMOOTH_OFF) this->PseudorangeSmooth();

  //  this->BD2_Multipath();   // bds2 multipath correctiong
    return this->mObs;
}


ReceiverInfo ReadRinexObs::GetRoverInf(){
    return this->mRover;
}


bool ReadRinexObs::ReadInit() {
    mObs.satnum = 0;
    mObs.num = 0;

    this->SavePreviousData();
    mObs.obsp.clear();

    return true;
}


bool ReadRinexObs::ReadEpochHead() {
    while (true)
    {
        getline(mInfile, mBuff);
        if (EndFile()) return false;
        if (mRover.version > 2.99)
        {
            if (CString::str2num(mBuff, 31, 1))
            {   // no sat in this epoch
                Log.Trace(TINFO, "No observation data in: " + mBuff.substr(1, 18));
                continue;   // epoch flag
            }
            mObs.satnum = int(CString::str2num(mBuff, 32, 3));
            if (mObs.satnum <= 0) continue;
            mTempdata = CString::split(mBuff, " ");
            mObs.time = CTimes::str2time(mBuff.substr(1, 30));
        }
        else
        {
            /// todo 2.x
        }

        this->SetIndex();
        return true;
    }

}


bool ReadRinexObs::ReadEpochData() {
    int n, sys, sat_id = 0, stat = 1;
    int i, j, m, num;
    int k[16],l[16];
    vector<double> val;
    vector<uint8_t> lli;
    vector<uint8_t> sst;
    vector<int> p;

    for (num = 0; num < mObs.satnum; num++)
    {
        getline(mInfile, mBuff);
        if (EndFile()) return false;
        if (mBuff[1] == ' ') mBuff.replace(mBuff.find(" "), 1, "0");
        mTempdata =  CString::split(mBuff, " ");
        sys = CString::satsys(mTempdata[0]);
        if (!config.System[sys]) continue;

        if (sys == SYS_BDS)
        {
            if      (config.BdsOption == BDSOPT_BD2 && CString::str2num(mTempdata[0], 1, 2) > 18) continue;  // only bd2
            else if (config.BdsOption == BDSOPT_BD3 && CString::str2num(mTempdata[0], 1, 2) < 18) continue;  // only bd3
        }

        val.clear();
        lli.clear();
        p.clear();
        sst.clear();

        if(mRover.version > 2.99)
        {   /* ver.3 */
            sat_id = CString::satprn2id(mTempdata[0]);
        }

        if (!sat_id)
        {
            Log.Trace(TWARNING, "ReadEpochData: unsupported sat sat=" + mTempdata[0]);
            stat = 0;
        }

        /* read observation data fields */
        for (i = 0, j = mRover.version <= 2.99 ? 0 : 3; i < mTypeIndex[sys - 1].n; i++, j += 16)
        {
            if (mRover.version <= 2.99 && j >= 80)
            {   /* ver.2 */
                // todo 2.x
            }

            if (stat)
            {
                if (j >= mBuff.size()) break;
                val.push_back(strtod(mBuff.substr(j, 14).c_str(), nullptr));
                if (j+14 >= mBuff.size()) break;
                lli.push_back((uint8_t)strtod(mBuff.substr(j + 14, 1).c_str(), nullptr) & 3);
                sst.push_back((uint8_t)strtod(mBuff.substr(j + 15, 1).c_str(), nullptr));
            }
        }

        if (!stat) return false;

        GnssObsData obs_t;
        for (i = 0; i < NFREQ + NEXOBS; i++)
        {
            obs_t.P[i] = obs_t.L[i] = 0.0;
            obs_t.D[i] = obs_t.DL[i] = 0.0;
            obs_t.SNR[i] = obs_t.LLI[i] = obs_t.code[i] = 0;
        }

        /* assign position in observation data */
        for (i = n = m = 0; i < mTypeIndex[sys - 1].n; i++)
        {
            p.push_back((mRover.version <= 2.11) ? mTypeIndex[sys - 1].idx[i] : mTypeIndex[sys - 1].pos[i]);
            if (mTypeIndex[sys - 1].type[i] == 0 && p[i] == 0) k[n++]=i; /* C1? index */
            if (mTypeIndex[sys - 1].type[i] == 0 && p[i] == 1) l[m++]=i; /* C2? index */
        }

        if (mRover.version <= 2.11){}   // todo

        /* save observation data */
        for (i = 0; i < mTypeIndex[sys - 1].n; i++)
        {
            if ( p[i] < 0 || i >= val.size() || val[i] == 0.0) continue;
            switch (mTypeIndex[sys - 1].type[i])
            {
                case 0: obs_t.P[p[i]] = val[i]; obs_t.code[p[i]] = mTypeIndex[sys - 1].code[i]; break;
                case 1: obs_t.L[p[i]] = val[i]; obs_t.LLI [p[i]] = lli[i]; obs_t.sst[p[i]] = sst[i]; break;
                case 2: obs_t.D[p[i]] = val[i];                                 break;
                case 3: obs_t.SNR[p[i]] = (uint16_t)(val[i] / SNR_UNIT + 0.5);  break;
            }
        }

        mObs.num += 1;
        obs_t.prn = mTempdata[0];
        obs_t.sat.vaild = true;
        mObs.obsp.insert(pair<int, GnssObsData>(sat_id, obs_t));
    }

    return true;
}


bool ReadRinexObs::SavePreviousData(){
    mObs.lobsp = mObs.obsp;
    return true;
}


void ReadRinexObs::AdjustObservation(){
    int f;
    for (auto it = mObs.obsp.begin(); it != mObs.obsp.end() ; ++it)
    {
        auto &obsp = it->second;
        for (int freq = 0; freq < config.NumFreq; freq++)
        {
            f = CState::GetFrequencyIndex(obsp.prn, freq);
            if (obsp.L[f]==0.0 && obsp.L[f+NEXOBS]!=0.0)
            {
                obsp.P[f]    = obsp.P[f+NEXOBS];
                obsp.L[f]    = obsp.L[f+NEXOBS];
                obsp.D[f]    = obsp.D[f+NEXOBS];
                obsp.SNR[f]  = obsp.SNR[f+NEXOBS];
                obsp.LLI[f]  = obsp.LLI[f+NEXOBS];
                obsp.code[f] = obsp.code[f+NEXOBS];
            }

        }
    }
}


void ReadRinexObs::PseudorangeSmooth(){
    int f;
    if (mObs.lobsp.empty())
    {   // no obs data of last epoch
        for (auto it = mObs.obsp.begin(); it != mObs.obsp.end() ; ++it)
        {
            auto &obsp = it->second;
            for (int freq = 0; freq < config.NumFreq; freq++)
            {
                f = CState::GetFrequencyIndex(obsp.prn, freq);
                obsp.lock[f]++;
            }
        }
        return;
    }

    for (auto it = mObs.obsp.begin(); it != mObs.obsp.end() ; ++it)
    {
        auto &obsp = it->second;
        for (int freq = 0; freq < config.NumFreq; freq++)
        {
            f = CState::GetFrequencyIndex(obsp.prn, freq);
            if (obsp.P[f] < 1e-3) continue;
            if (mObs.lobsp.find(it->first) != mObs.lobsp.end())
            {   //check whether there are observation in the previous epoch
                obsp.lock[f] = mObs.lobsp[it->first].lock[f] + 1;
            }
            else
            {   // no observation of last epoch
                obsp.lock[f] = 1;
            }

            if      (config.PseudorangeSmooth == SMOOTH_DOPPLER)
            {
                if (mObs.lobsp[it->first].P[f] < 1e-3 || abs(mObs.lobsp[it->first].D[f]) < 1e-3) continue;
                if (abs(obsp.D[f]) < 1e-3) continue;

                double num = obsp.lock[f]>config.PseudorangeSmoothWindow? config.PseudorangeSmoothWindow: obsp.lock[f];
                double lam = CLIGHT / CState::GetFrequency(obsp.prn, freq, obsp.sat.glofcn);
                double dSn = -0.5 * config.SampleRate * lam * (mObs.lobsp[it->first].D[f] + obsp.D[f]);
                double PSmooth = (1.0/num) * obsp.P[f] + ((num-1.0)/num) * (mObs.lobsp[it->first].P[f] + dSn);
                if (abs(PSmooth-obsp.P[f]) < config.PseudorangeSmoothThreshold)
                {
                    obsp.P[f] = PSmooth;
                }
                else
                {

                }

            }
            else if (config.PseudorangeSmooth == SMOOTH_PHASE)
            {
              // todo
            }

        }
    }

}


bool ReadRinexObs::SetIndex() {
    int i, j, k, n;
    string optstr;
    for (int sys = 0; sys < NUMSYS; sys++)
    {
        for (i = n = 0; i < MAXOBSTYPE; i++, n++)
        {
            if (mPriIndex[sys][i].length() < 3)  break;
            mTypeIndex[sys].code[i] = obs2code(mPriIndex[sys][i]);
            mTypeIndex[sys].type[i] = sObsCodeType.find(mPriIndex[sys][i].substr(0, 1));
            mTypeIndex[sys].idx[i] = code2idx(sys, mTypeIndex[sys].code[i]);
            mTypeIndex[sys].pri[i] = getcodepri(sys, mTypeIndex[sys].code[i]); //
            mTypeIndex[sys].pos[i] = -1;
        }

        /* assign index for highest priority code */
        for (i = 0; i < NFREQ; i++)
        {
            for (j = 0, k = -1; j < n; j++)
            {
                if (mTypeIndex[sys].idx[j] == i && mTypeIndex[sys].pri[j] && (k < 0 || mTypeIndex[sys].pri[j] > mTypeIndex[sys].pri[k])) 
                {
                    k = j;
                }
            }

            if (k < 0) continue;

            for (j = 0; j < n; j++)
            {
                if (mTypeIndex[sys].code[j] == mTypeIndex[sys].code[k]) mTypeIndex[sys].pos[j]=i;
            }
        }

        /* assign mTypeIndex of extended observation data */
        for (i = 0; i < NEXOBS; i++)
        {
            for (j = 0; j < n; j++)
            {
                if (mTypeIndex[sys].code[j] && mTypeIndex[sys].pri[j] && mTypeIndex[sys].pos[j] < 0) break;
            }

            if (j >= n) break;

            for (k = 0; k < n; k++)
            {
                if (mTypeIndex[sys].code[k] == mTypeIndex[sys].code[j]) mTypeIndex[sys].pos[k] = NFREQ + mTypeIndex[sys].idx[k];
            }
        }

        for (i = 0; i < n; i++)
        {
            if (!mTypeIndex[sys].code[i] || !mTypeIndex[sys].pri[i] || mTypeIndex[sys].pos[i] >= 0) continue;
            Log.Trace(TWARNING, "reject obs type: sys=" + to_string(sys) + ", obs=" + mPriIndex[sys][i]);
        }
        mTypeIndex[sys].n = n;
    }
    return true;
}


void ReadRinexObs::FormTDCP() {
    if (mObs.obsp.empty() || mObs.lobsp.empty()) return;

    for (auto iter = mObs.obsp.begin(); iter != mObs.obsp.end() ; iter++)
    {
        if (mObs.lobsp.find(iter->first) == mObs.lobsp.end()) continue;   // no continuous observation

        for (int f = 0; f < NFREQ+NEXOBS; f++)
        {
            if (iter->second.L[f] == 0 || mObs.lobsp[iter->first].L[f] == 0) continue;
            iter->second.DL[f] = (mObs.lobsp[iter->first].L[f] - iter->second.L[f]) / config.SampleRate;
        }

    }
}


void ReadRinexObs::BD2Multipath() {
    int b, j;
    int prn;
    double a, elev;
    double dmp[3];

    for (auto iter = mObs.obsp.begin(); iter != mObs.obsp.end() ; iter++)
    {
        auto &obsp = iter->second;
        if (CString::satsys(obsp.prn) != SYS_BDS) continue;
        prn = int(CString::str2num(obsp.prn, 1, 2));
        if (prn > 18 || prn <= 5) continue;
        elev = obsp.elevation * R2D;
        if (elev <= config.Elevation * R2D) continue;

        a = elev * 0.1;
        b = (int)a;
        for (j = 0; j < 3; j++) dmp[j]=0.0;

        if ((prn >= 6 && prn <= 10) || (prn == 13 || prn == 16))
        {   // IGSO(C06, C07, C08, C09, C10, C13, C16)
            if (b<0)
            {
                for (j = 0; j < 3; j++) dmp[j] = sIGSOCoef[j][0];
            }
            else if (b>=9)
            {
                for (j = 0; j < 3; j++) dmp[j] = sIGSOCoef[j][9];
            }
            else
            {
                for (j = 0; j < 3; j++) dmp[j] = sIGSOCoef[j][b] * (1.0 - a + b) + sIGSOCoef[j][b + 1] * (a - b);
            }
        }
        else if (prn == 11 || prn == 12 || prn == 14)
        {   // MEO(C11, C12, C14)
            if (b<0)
            {
                for (j = 0; j < 3; j++) dmp[j] = sMEOCoef[j][0];
            }
            else if (b >= 9)
            {
                for (j = 0; j < 3; j++) dmp[j] = sMEOCoef[j][9];
            }
            else
            {
                for (j = 0; j < 3; j++) dmp[j] = sMEOCoef[j][b] * (1.0 - a + b) + sMEOCoef[j][b + 1] * (a - b);
            }
        }

        if(obsp.P[0] != 0.0) obsp.P[0] += dmp[0];
        if(obsp.P[1] != 0.0) obsp.P[1] += dmp[1];
        if(obsp.P[2] != 0.0) obsp.P[2] += dmp[2];

    }
}


/* convert mObs code type string to mObs code
* args   : string    obstype      I   mObs code string ("1C","1P","1Y",...)
* return : mObs code (CODE_???)
* notes  : mObs codes are based on RINEX 3.04
*-----------------------------------------------------------------------------*/
int ReadRinexObs::obs2code(string& obstype){
    int i;
    for (i = 1; i < MAXCODESTRING; i++)
    {
        if (obstype.find(sObsCodes[i]) == string::npos) continue;
        return i;
    }
    return 0;
}


int ReadRinexObs:: code2freq_GPS(uint8_t code){
    string code_string = sObsCodes[code];
    switch (code_string[0])
    {
        case '1': return 0; /* L1 */
        case '2': return 1; /* L2 */
        case '5': return 2; /* L5 */
    }
    return -1;
}


int ReadRinexObs:: code2freq_GLO(uint8_t code){
    string code_string = sObsCodes[code];

    switch (code_string[0])
    {
        case '1': return 0; /* G1 */
        case '2': return 1; /* G2 */
        case '3': return 2; /* G3 */
        case '4': return 0; /* G1a */
        case '6': return 1; /* G2a */
    }

    return -1;
}


int ReadRinexObs:: code2freq_GAL(uint8_t code){
    string code_string = sObsCodes[code];
    switch (code_string[0]) {
        case '1': return 0; /* E1 */
        case '7': return 1; /* E5b */
        case '5': return 2; /* E5a */
        case '6': return 3; /* E6 */
        case '8': return 4; /* E5ab */
    }
    return -1;
}


int ReadRinexObs::code2freq_BDS(uint8_t code){
    string code_string = sObsCodes[code];
    switch (code_string[0])
    {
        case '2': return 0; /* B1I */
        case '7':
                  switch (code_string[1])
                  {
                      case 'I':
                      case 'Q':
                      case 'X': return 1; /* B2I */
                      case 'D':
                      case 'P':
                      case 'Z':return 6;  /* B2b */
                  }
        case '6': return 2; /* B3 */
        case '1': return 3; /* B1C */
        case '5': return 4; /* B2a */
        case '8': return 5; /* B2ab */
    }
    return -1;
}


/* system and mObs code to frequency mTypeIndex --------------------------------------
* convert system and mObs code to frequency mTypeIndex
* args   : int    sys       I   satellite system (SYS_???)
*          uint8_t code     I   mObs code (CODE_???)
* return : frequency mTypeIndex (-1: error)
*                       0     1     2     3     4
*           --------------------------------------
*            GPS       L1    L2    L5     -     -
*            BDS       B1I   B2I   B3    B1C  B2a  B2ab  B2b
*            GLONASS   G1    G2    G3     -     -  (G1=G1,G1a,G2=G2,G2a)
*            Galileo   E1    E5b   E5a   E6   E5ab
*            QZSS      L1    L2    L5    L6     -
*            SBAS      L1     -    L5     -     -
*            NavIC     L5     S     -     -     -
*-----------------------------------------------------------------------------*/
int ReadRinexObs::code2idx(int sys, int code)
{
    switch (sys + 1)
    {
        case SYS_GPS: return code2freq_GPS(code);
        case SYS_BDS: return code2freq_BDS(code);
        case SYS_GLO: return code2freq_GLO(code);
        case SYS_GAL: return code2freq_GAL(code);
            //   case SYS_QZS: return code2freq_QZS(code,&freq);
            //   case SYS_SBS: return code2freq_SBS(code,&freq);
            //   case SYS_IRN: return code2freq_IRN(code,&freq);
    }
    return -1;
}


/* get code priority -----------------------------------------------------------
* get code priority for multiple codes in a frequency
* args   : int    sys     I     system (SYS_???)
*          unsigned char code I mObs code (CODE_???)
* return : priority (15:highest-1:lowest,0:error)
*-----------------------------------------------------------------------------*/
int ReadRinexObs::getcodepri(int sys, uint8_t code)
{
    int i, j;
    string optstr;
    switch (sys + 1)
    {
        case SYS_GPS: i=0; optstr="-GL%2s"; break;
        case SYS_GLO: i=1; optstr="-RL%2s"; break;
        case SYS_GAL: i=2; optstr="-EL%2s"; break;
        case SYS_BDS: i=3; optstr="-CL%2s"; break;
        default: return 0;
    }
    if ((j = code2idx(sys, code)) < 0) return 0;
    string code_string = sObsCodes[code];

    /* parse code options, note no use? by li*/

    /* search code priority */
    int priority = sCodePris[i][j].find(code_string[1]);
    return priority != -1? 14 - priority : 0;
}


bool ReadGnssResultBin::Read(){
    double time;
    double data[3];

    while (true)
    {
        mInfile.read((char*)&time, sizeof(double));      // time

        mInfile.read((char*)&data, sizeof(double) * 3);        // pos
        mSol_t.pos << data[0], data[1], data[2];
        mInfile.read((char*)&data, sizeof(double) * 3);        // vel
        mSol_t.vel << data[0], data[1], data[2];

        mInfile.read((char*)&data, sizeof(double) * 3);        // Qpos
        mSol_t.Qpos << data[0], data[1], data[2];
        mInfile.read((char*)&data, sizeof(double) * 3);        // Qvel
        mSol_t.Qvel << data[0], data[1], data[2];

        mInfile.read((char*)&mSol_t.nsat, sizeof(int));         // number of sat
        mInfile.read((char*)&mSol_t.stat, sizeof(int));         // solution stat

        mSols.insert(pair<double, SmoothSolution>(time, mSol_t));
        if (mInfile.eof()) break;
    }

    return true;
}


map<double, SmoothSolution> ReadGnssResultBin::GetSolutionData(){
    this->Read();
    return mSols;
}

