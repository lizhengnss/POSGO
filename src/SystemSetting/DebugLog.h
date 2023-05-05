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
 * Created by lizhen on 2022/8/18.
 *-------------------------------------------------------------------------------*/
#ifndef DEBUGLOG_H
#define DEBUGLOG_H

#include <fstream>
#include <iostream>
#include <iomanip>
#include <Eigen/Eigen>
#include "../CommonFunction/Parameter.h"
#include "../CommonFunction/CTimes.h"

using namespace std;

class DebugLog {
public:
    DebugLog()  = default;
    ~DebugLog() = default;

    void TraceLevel(int level);                                  /* set trace debug level                 */
    bool TraceOpen(string filename);                             /* open trace debug file                 */
    void TraceClose();                                           /* close trace debug file                */
    void Trace(int level, string info);                          /* export trace debug information        */
    void Trace(int level, STimes tTime, string info);            /* export trace debug information        */
    void TraceMat(int level, string info, const MatrixXd& mat);  /* export trace debug information of mat */
    void TraceVec(int level, string info, const VectorXd& vec);  /* export trace debug information of vec */
    void ShowState(string msg, int stat, int epoch);
private:
    int mLevel = 2;        /* level of debug trace       */
    ofstream mFile;        /* iostream of output file    */
    time_t mTime = 0;      /* time of program now moment */
    long mStartTime = 0;   /* program start time         */
    long mEndTime = 0;     /* program end   time         */
    double mTotalTime = 0; /* total program running time */
    char mSTime[256] = {}; /* string of time for output  */
};


#endif //DEBUGLOG_H
