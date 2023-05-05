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
 * Created by lizhen on 2021/7/27.
 *-------------------------------------------------------------------------------*/

#ifndef SOLVER_H
#define SOLVER_H

#include <string>
#include <Eigen/Eigen>
#include "../Global.h"
#include "../CommonFunction/CState.h"

class Solver {
public:
    Solver()  = default;
    ~Solver() = default;

protected:
    int mSysMask[5] = {0};          /* Whether to use GPS,BD2,GLO,GAL,BD3    */
    virtual bool InitSolving(const vector<Equation>& eq){
        Log.Trace(TERROR, "*InitSolving function in Solver class overload error, please check!");
        return false;
    }

    void RemarkVaildSystem(string prn){
        int system = CString::satsys(prn);
        if (config.BdsOption == BDSOPT_BD2_3)
        {
            if(system == SYS_BDS && CString::str2num(prn, 1, 2) > 18) system = SYS_BD3;
        }
        switch (system)
        {
            case SYS_BDS: mSysMask[1] = 1; break;
            case SYS_GLO: mSysMask[2] = 1; break;
            case SYS_GAL: mSysMask[3] = 1; break;
            case SYS_BD3: mSysMask[4] = 1; break;
            default:      mSysMask[0] = 1; break;
        }
    }
};


#endif //SOLVER_H
