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
 * Created by lizhen on 2021/9/17.
 *-------------------------------------------------------------------------------*/

#ifndef CSTATE_H
#define CSTATE_H

#include "../Global.h"


class CState {
public:
    static int XnP() { return 3; }

    static int XnV() { return 3; }

    static int XnAc() { // only for gnss
        if (config.SolverMode == SOLVER_LSQ) return 0;
        if (config.SolverMode == SOLVER_GO) return 0;
        return 3;
    }

    static int XnClk() {
        int n = 0;
        if (config.ProcessMode == PMODE_SINGLE)    // for spp and ppp(todo)
        {
            for (int i = 1; i < 5; i++)
            {
                if (config.System[i]) n += 1;
            }
            if (config.BdsOption == BDSOPT_BD23) n += 1;
            return n;
        }
        return 0;
    }

    static int XnCdr(){
        if (config.VelocityMode) return 1;
        else                     return 0;
    }

    static int XnAmb(){
        if (config.ProcessMode > PMODE_SINGLE) return config.NumFreq * MAXSAT;
        return 0; // no amb
    }

    static int XnState(){
        return XnP() + XnV() + XnAc() + XnClk() + XnCdr();
    }

    static int XnGnss() {
        return XnP() + XnV() + XnAc() + XnClk() + XnCdr() + XnAmb();
    }

    static int XnGnssVel(){
        return XnV() + XnAc() + XnCdr();
    }

    static int XnX(){   // all state
        return XnP() + XnV() + XnAc() + XnClk() + XnCdr() + XnAmb();
    }

    static int XiP() { return 0; }

    static int XiV() { return XnP(); }

    static int XiAc() {
        return XnP() + XnV();
    }

    static int XiClk() { return XnP() + XnV() + XnAc(); }

    static int XiCdr() { return XnP() + XnV() + XnAc() + XnClk(); }

    static int XiAmb(int sat, int frq){
        return XnP() + XnV() + XnAc() + XnClk() + XnCdr() +
                MAXSAT * frq + sat - 1;
    }

    /* get satellite carrier frequency from prn ------------------------------------
    * args   : string  prn      I   satellite prn
    *          int     freq     I   freq index
    *          int     glofcn   I   channel number of GLONASS
    * return : carrier frequency (Hz) (0.0: error)
    *-----------------------------------------------------------------------------*/
    static double GetFrequency(string prn, int freq, int glofcn){
        int bdprn;
        int sys = CString::satsys(prn);
        switch (sys)
        {
            case SYS_GPS: return config.GPS_Freq[freq];
            case SYS_BDS:
                bdprn = CString::str2num(prn, 1, 2);
                if (bdprn < 18) return config.BD2_Freq[freq];
                else            return config.BD3_Freq[freq];
            case SYS_GLO:  // todo G3,G1a,G2a?
                switch (freq)
                {
                    case 0 : return config.GLO_Freq[freq] + glofcn * DFRQ1_GLO;
                    case 1 : return config.GLO_Freq[freq] + glofcn * DFRQ2_GLO;
                    default: return config.GLO_Freq[freq];
                }
            case SYS_GAL: return config.GAL_Freq[freq];
            default: return 0;
        }
    }

    /* get satellite frequency index from prn -------------------------------------
    * args   : string  prn      I   satellite prn
    *          int     freq     I   freq order, usually 0,1,2
    * return : carrier frequency (Hz) (0.0: error)
    *-----------------------------------------------------------------------------*/
    static int GetFrequencyIndex(string prn, int freq){
        int bdprn;
        int sys = CString::satsys(prn);
        switch (sys)
        {
            case SYS_GPS: return config.GPS_FreqID[freq];
            case SYS_BDS:
                bdprn = CString::str2num(prn, 1, 2);
                if (bdprn < 18) return config.BD2_FreqID[freq];
                else            return config.BD3_FreqID[freq];
            case SYS_GLO:  return config.GLO_FreqID[freq];
            case SYS_GAL:  return config.GAL_FreqID[freq];
            default: return 0;
        }
    }

};



#endif //CSTATE_H
