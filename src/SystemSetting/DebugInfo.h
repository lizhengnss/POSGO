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

#ifndef DEBUGINFO_H
#define DEBUGINFO_H

#include <iostream>
#include "../CommonFunction/CState.h"
#include "../CommonFunction/CTimes.h"


class DebugResidual{
public:
    DebugResidual(const string filename, int type);
    ~DebugResidual();
    void TraceRes(const STimes tTime, const vector<Equation>& pri, const vector<Equation>& post);
    void TraceRes(const STimes tTime, const vector<Equation>& pri, const vector<Equation>& post, const int ref[][NFREQ]);
private:
    ofstream mResLog;      /* iostream of Residual log file    */
};


class DebugRobust{
public:
    DebugRobust(const string filename);
    ~DebugRobust();
    void TraceTime(const STimes tTime);
    void TraceRobust(const STimes tTime, const Equation& eq, int type, double v0, double fact);
public:
    bool mExportTime = true;
private:
    ofstream mRobLog;      /* iostream of Robust log file    */
};


#endif //DEBUGINFO_H
