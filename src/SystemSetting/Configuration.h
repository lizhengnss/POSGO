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

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <iomanip>
#include <fstream>
#include <unistd.h>
#include "../Global.h"
#include "../CommonFunction/Parameter.h"

using namespace std;

class Config {
public:
    Config()  = default;
    ~Config() = default;
    bool PosGoPasteCommand(char *argv[]);        /* get config info from command line of POSGO */
    bool ReadConfigFile();                       /* read config file and set option            */

private:
    vector<string> opt_Switch      = {"off", "on"};
    vector<string> opt_Solver      = {"lsq", "ekf", "go"};
    vector<string> opt_SolveType   = {"forward", "backward", "smooth"};

    vector<string> opt_Iono        = {"off", "brdc", "IF", "GIM"};
    vector<string> opt_Trop        = {"off", "saas"};
    vector<string> opt_TropMap     = {"nmf", "gmf", "vmf"};
    vector<string> opt_Code        = {"off", "tgd", "dcb"};
    vector<string> opt_Robust      = {"off", "IGG3", "huber"};
    vector<string> opt_Velocity    = {"off", "doppler", "tdcp"};
    vector<string> opt_WeightMode  = {"elevation", "snr"};
    vector<string> opt_BdsOption   = {"bd2", "bd3", "bd23", "bd2-3"};
    vector<string> opt_PsrSmooth   = {"off", "doppler", "phase"};
    vector<string> opt_LossFun     = {"off", "huber", "cauchy", "arctan", "tukey", "softone"};

    vector<string> opt_Coord       = {"xyz", "blh"};
    vector<string> opt_TimeFormat  = {"tow", "hms"};
    vector<string> opt_TimeSystem  = {"gpst", "utc", "bst"};
    vector<string> opt_Heigth      = {"ellipsoidal", "geodetic"};
    vector<string> opt_BLHFormat   = {"deg", "dms"};

private:
    ifstream mInfile;
    string mFilename;
    string mBuff;
    vector<string> mTempdata;
    string sSysCodes = "GCREJSI";             /* satellite system codes */
    string mFreqStr[4][MAXFREQ]={
            {"L1",  "L2",  "L5",  "",    "",     "",    ""},       /*GPS*/
            {"B1I", "B2I", "B3I", "B1C", "B2a",  "B2ab", "B2b"},   /*BDS*/
            {"G1",  "G2",  "G3",  "G1a", "G2a",  "",    ""},       /*GLO*/  // todo G1a and G2b
            {"E1",  "E5b", "E5a", "E6",  "E5ab", "",    ""},       /*GAL*/
    };

    void PrintPosGoUsage();         /* print usage of command line for POSGO app          */
    void InitConfig();              /* set default option                                 */
    void DeleteComments();          /* delete Comments after identifier "#"               */
    bool CheckConfigSetting();      /* check for configuration errors                     */
    bool GetVaildSystem();          /* set GNSS constellation used according command line */
    bool GetSloveMode();            /* set slove mode according command line              */

    double GetConfigInformation(vector<string> info);    /* get config information one by one    */
    int GetConfigInformation(vector<string> info, const vector<string> opt);
    void GetConfigInformation(vector<string> info, double *cfg, int num);
    void GetConfigInformation(vector<string> info, Vector3d& cfg);
    void GetConfigInformation(vector<string> info, VectorXd& cfg);
    string GetConfigFilename(vector<string> info);               /* get filename from config one by one  */

    bool SetFequencyIndex(int sys, const vector<string> freq);   /* set observation frequency index      */
    int GetFequencyIndex(int sys, string freq);                  /* get index from preset                */
    double GetFequency(int sys, int freqID);                     /* get frequency from preset            */
    bool SetFequency(int sys);                                   /* set frequency of each observation    */
    int GetOptionID(string opt, const vector<string> candidate); /* get option index in preset candidate */
    void GetStartEndTime(vector<string> info);                   /* get start time and end time          */

    bool ReadLeapSecond();          /* read leap second file */
};


#endif //CONFIGURATION_H
