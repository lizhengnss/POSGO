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
 * Created by lizhen on 2021/8/25.
 *-------------------------------------------------------------------------------*/

#ifndef RESULTEXPORT_H
#define RESULTEXPORT_H

#include <fstream>
#include <iomanip>
#include "../Global.h"


class ResultExport {
public:
    ResultExport()  = default;
    ~ResultExport() = default;

    virtual bool Export(Solution& sol) {
        Log.Trace(TERROR, "*Export function in ResultExport class overload error, please check!");
        return false;
    };

    virtual void Close() {
        Log.Trace(TERROR, "*Close function in ResultExport class overload error, please check!");
    };

protected:
    string mTime;
    vector<string> opt_Process     = {"SPP", "RTK", "PPP"};
    vector<string> opt_Slover      = {"LSQ", "EKF", "GO"};
    vector<string> opt_SloveMode   = {"forward", "backward", "smooth"};
    vector<string> opt_Robust      = {"off", "IGG3", "Huber"};
    vector<string> opt_Iono        = {"off", "BRDC", "IF", "GIM"};
    vector<string> opt_Trop        = {"off", "Saastamoinen"};
    vector<string> opt_Velocity    = {"off", "Doppler", "TDCP"};
    vector<string> opt_Code        = {"off", "TGD", "DCB"};
    vector<string> opt_LossFun     = {"off", "huber", "cauchy", "arctan", "tukey", "softone"};
    vector<string> opt_WeightMode  = {"elevation", "snr"};
    vector<string> opt_Switch      = {"off", "on"};

};


#endif //RESULTEXPORT_H
