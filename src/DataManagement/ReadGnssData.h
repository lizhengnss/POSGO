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

#ifndef READGNSSDATA_H
#define READGNSSDATA_H

#include "ReadFile.h"


class ReadRinexObs: public ReadFile{
public:
    ReadRinexObs()  = default;
    ~ReadRinexObs() = default;
    bool ReadHead() override;                /* read Obs file head              */
    bool Reading() override;                 /* read one epoch data             */
    GnssObsEpoch GetObsData();               /* get one epoch of observation    */
    ReceiverInfo GetRoverInf();              /* get Rover information in Obs    */

private:
    struct GNSSObsIndex{
        int n = 0;                           /* number of index                  */
        int idx[MAXOBSTYPE] = {0};           /* signal freq-index                */
        int frq[MAXOBSTYPE] = {0};           /* signal frequency (1:L1,2:L2,...) */
        int pos[MAXOBSTYPE] = {0};           /* signal index in Obs data (-1:no) */
        uint8_t pri [MAXOBSTYPE] = {0};      /* signal priority (15-0)           */
        uint8_t type[MAXOBSTYPE] = {0};      /* type (0:C,1:L,2:D,3:S)           */
        uint8_t code[MAXOBSTYPE] = {0};      /* obs code (CODE_L??)              */
        double shift[MAXOBSTYPE] = {0};      /* phase shift (cycle)              */
    };                                       /* signal index type                */

private:
    bool mEndOfFile = false;                 /* whether read to the end of the file   */
    ReceiverInfo mRover;                     /* informaton of rover                   */
    GnssObsEpoch mObs;                       /* observation of on epoch               */
    GNSSObsIndex mTypeIndex[NUMSYS];         /* index of observation type             */
    string mPriIndex[NUMSYS][MAXOBSTYPE] = {""};  /* index of observation priority    */

    bool ReadInit();                         /* save last obs and init for next one     */
    bool ReadEpochHead();                    /* read one epoch obs data of head         */
    bool ReadEpochData();                    /* read one epoch obs data of body         */
    bool SavePreviousData();                 /* storing data of the previous epoch      */
    void AdjustObservation();                /* use candidate for prior code miss obs   */
    void PseudorangeSmooth();                /* smooth code using doppler and phase     */

    bool SetIndex();                         /* set index of obs type and priority      */
    void FormTDCP();                         /* form time difference of carrier-phase   */
    void BD2Multipath();                     /* correct BDS2 multipath in pseudo range  */

    int obs2code(string& obstype);           /* obs type string to obs code             */
    int code2idx(int sys, int code);         /* system and obs code to frequency index  */
    int code2freq_GPS(uint8_t code);         /* GPS obs code to frequency               */
    int code2freq_BDS(uint8_t code);         /* BDS obs code to frequency               */
    int code2freq_GLO(uint8_t code);         /* GLONASS obs code to frequency           */
    int code2freq_GAL(uint8_t code);         /* Galileo obs code to frequency           */
    int getcodepri(int sys, uint8_t code);   /* get code priority                       */


private:
    const string sSysCodes = "GCREJ";        /* satellite system codes                */
    const string sObsCodeType ="CLDS";       /* observation type codes                */
    const string sCodePris[7][MAXFREQ]={     /* code priority for each freq-index     */
            /*    0         1          2          3        4         5        6 */
            {"CPYWMNSL","CPYWMNDLSX","IQX"     ,""       ,""       ,""      ,""},    /* GPS */
            {"CPABX"   ,"CPABX"     ,"IQX"     ,""       ,""       ,""      ,""},    /* GLO */
            {"CABXZ"   ,"IQX"       ,"IQX"     ,"ABCXZ"  ,"IQX"    ,""      ,""},    /* GAL */
            {"IQXDPAN" ,"IQX"       ,"IQXA"    ,"DPX"    ,"DPX"    ,"DPX"   ,"DPZ"}, /* BDS */
            {"CLSXZ"   ,"LSX"       ,"IQXDPZ"  ,"LSXEZ"  ,""       ,""      ,""},    /* QZS */
    };
    const string sObsCodes[MAXCODESTRING]={         /* observation code strings  */
            ""  ,"1C","1P","1W","1Y", "1M","1N","1S","1L","1E", /*  0- 9 */
            "1A","1B","1X","1Z","2C", "2D","2S","2L","2X","2P", /* 10-19 */
            "2W","2Y","2M","2N","5I", "5Q","5X","7I","7Q","7X", /* 20-29 */
            "6A","6B","6C","6X","6Z", "6S","6L","8L","8Q","8X", /* 30-39 */
            "2I","2Q","6I","6Q","3I", "3Q","3X","1I","1Q","5A", /* 40-49 */
            "5B","5C","9A","9B","9C", "9X","1D","5D","5P","5Z", /* 50-59 */
            "6E","7D","7P","7Z","8D", "8P","4A","4B","4X",""    /* 60-69 */
    };
    const double sIGSOCoef[3][10]={		/* m */
            {-0.55,-0.40,-0.34,-0.23,-0.15,-0.04,0.09,0.19,0.27,0.35},	//B1
            {-0.71,-0.36,-0.33,-0.19,-0.14,-0.03,0.08,0.17,0.24,0.33},	//B2
            {-0.27,-0.23,-0.21,-0.15,-0.11,-0.04,0.05,0.14,0.19,0.32},	//B3
    };
    const double sMEOCoef[3][10]={		/* m */
            {-0.47,-0.38,-0.32,-0.23,-0.11,0.06,0.34,0.69,0.97,1.05},	//B1
            {-0.40,-0.31,-0.26,-0.18,-0.06,0.09,0.28,0.48,0.64,0.69},	//B2
            {-0.22,-0.15,-0.13,-0.10,-0.04,0.05,0.14,0.27,0.36,0.47},	//B3
    };
};


class ReadGnssResultBin: public ReadFile{
public:
    ReadGnssResultBin()  = default;
    ~ReadGnssResultBin() = default;

    virtual bool Read();
    map<double, SmoothSolution> GetSolutionData();

private:
    SmoothSolution mSol_t;               /* solution of one epoch */
    map<double, SmoothSolution> mSols;   /* solution of all epoch */

};

#endif //READGNSSDATA_H
