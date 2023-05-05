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

#ifndef READFILE_H
#define READFILE_H


#include <fstream>
#include <iomanip>
#include "../Global.h"
#include "../CommonFunction/Parameter.h"
#include "../CommonFunction/CState.h"

class ReadFile {
public:
    ReadFile()  = default;
    ~ReadFile() = default;

    bool OpenFile(string& file_path);    /* open infilename                 */
    bool OpenBinFile(string& file_path); /* open binary infilename          */
    void CloseFile();                    /* close both infile and outfile   */
    bool EndFile();                      /* whether read to the end of file */

    virtual bool ReadHead() {            /* read obs file head      */
        Log.Trace(TERROR, "*ReadHead function in ReadFile class overload error, please check!");
        return false;
    };

    virtual bool Reading() {             /* read one epoch data     */
        Log.Trace(TERROR, "*Reading function in ReadFile class overload error, please check!");
        return false;
    };

    virtual bool Read() {                /* read all data from file */
        Log.Trace(TERROR, "*Read function in ReadFile class overload error, please check!");
        return false;
    };

protected:
    ifstream mInfile;                    /* iostream of input file       */
    ofstream mOutfile;                   /* iostream of output file      */
    string mBuff;                        /* data of each line            */
    vector<string> mTempdata;            /* container of split each line */
};


#endif //READFILE_H
