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
 * Created by lizhen on 2021/7/13.
 *-------------------------------------------------------------------------------*/

#include <memory>
#include "../src/CommonFunction/Parameter.h"
#include "../src/MultiPosition/SppPosition.h"
#include "../src/MultiPosition/RtkPosition.h"
#include "../src/SystemSetting/Configuration.h"


MultiPosition* MP;

bool SelectProcessMode(){
    if      (config.ProcessMode == PMODE_SINGLE)
    {
        MP = new SppPosition();
        return true;
    }
    else if (config.ProcessMode == PMODE_RTK)
    {
        MP = new RtkPosition();
        return true;
    }
    else
    {
        Log.Trace(TERROR, "*Process mode set error, please check");
        return false;
    }
}


bool Process(){

    if(!SelectProcessMode())  return false;

    if      (config.SolveType == MODE_FORWARD)
    {
        if(!MP->Processing()) return false;
    }
    else if (config.SolveType == MODE_BACKWARD)
    {
        MP->mSolveType = MODE_BACKWARD;
        if(!MP->Processing()) return false;
    }
    else if (config.SolveType == MODE_SMOOTH)
    {
        if (!MP->Processing())
        {
            Log.Trace(TERROR, "*Forward process error, please check!");
            return false;
        }

        delete MP;
        config.Temp_File = "backward.bin";
        if(!SelectProcessMode())  return false;

        MP->mSolveType = MODE_BACKWARD;
        if (!MP->Processing())
        {
            Log.Trace(TERROR, "*Backward process error, please check!");
            return false;
        }

        auto SP = std::make_shared<SmoothPosition>();
        if (!SP->Smooth()) return false;
        MP->mSolveType = MODE_FORWARD;
    }

    delete MP;

    return true; // smooth success;
}


int main(int argc, char *argv[]) {
    Config conf;
    if (argc < 3)
    {
        Log.Trace(TERROR, "*Usage: POSGO -C [config] -S ['GREC'] -M [SolveMode]  -L [TraceLevel]");
        return false;
    }

    if(!conf.PosGoPasteCommand(argv)) return false;
    if(!conf.ReadConfigFile()) return false;

    if (!Log.TraceOpen(config.DebugFile))
    {
        Log.Trace(TERROR, "*DebugFile: " + config.DebugFile + " cann't open, please check");
        return false;
    }

    if (!Process())  return false;

    Log.TraceClose();
    return true;
}
