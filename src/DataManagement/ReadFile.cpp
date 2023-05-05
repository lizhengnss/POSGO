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

#include "ReadFile.h"


bool ReadFile::OpenFile(string& filename) {
    mInfile.open(filename, ios::in);
    if (!mInfile.is_open())
    {
        Log.Trace(TERROR, "*File: " + filename + " doesn't exit, please check");
        return false;
    }
    return true;
}


bool ReadFile::OpenBinFile(string& filename) {
    mInfile.open(filename, ios::in | ios::binary);
    if (!mInfile.is_open())
    {
        Log.Trace(TERROR, "*File: " + filename + " doesn't exit, please check");
        return false;
    }
    return true;
}


void ReadFile::CloseFile() {
    if (mInfile.is_open())
    {
        mInfile.close();
    }
}


bool ReadFile::EndFile() {
    if (mInfile.eof())
    {
        return true;
    }
    else
    {
        return false;
    }
}
