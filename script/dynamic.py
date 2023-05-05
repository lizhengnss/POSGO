
#!/usr/bin/python3
import math
from math import radians, sin, sqrt
from com import std, rms, mean, maxabs
import matplotlib.pyplot as plt

RE = 6378137

class cStat(object):
    def __init__(self):
        self.dx, self.dy, self.dz = [], [], []
        self.dvx, self.dvy, self.dvz = [], [], []
        self.dr, self.dp, self.dh = [], [], []
        self.rms, self.vrms, self.arms = [], [], []
        self.std, self.vstd, self.astd = [], [], []
        self.max, self.vstd, self.astd = [], [], []
        self.mean, self.vmean , self.amean= [], [], []


def ReadMyResult(_file, _isvel, _isatt):
    result = {}
    with open(_file, 'r') as file:
        content = file.readlines()
        for eachLine in content:
            if eachLine.startswith("%"):  # ignore comment line
                continue
            eachData = eachLine.split(",")
            if (_isvel and _isatt):
                result.update({round(float(eachData[1]), 2):
                                   { 'b':float(eachData[2]),   'l':float(eachData[3]),    'h':float(eachData[4]),
                                    'vb':float(eachData[13]), 'vl':float(eachData[12]), 'vh':float(eachData[14]),
                                     "r":float(eachData[18]), "p":float(eachData[19]), "y":float(eachData[20]),
                                   'num':float(eachData[9]),
                                # 'ratio':float(eachData[11]),
                                  'stat':float(eachData[8])  } })
            elif _isvel:
                result.update({round(float(eachData[1]), 2):
                                   { 'b':float(eachData[2]),   'l':float(eachData[3]),    'h':float(eachData[4]),
                                    'vb':float(eachData[13]), 'vl':float(eachData[12]), 'vh':float(eachData[14]),
                                   'num':float(eachData[9]),
                                # 'ratio':float(eachData[11]),
                                  'stat':float(eachData[8])  } })
            else:
                result.update({round(float(eachData[1])):
                                   { 'b':float(eachData[2]),   'l':float(eachData[3]),    'h':float(eachData[4]),
                                   'num':float(eachData[9]),
                               #  'ratio':float(eachData[11]),
                                  'stat':float(eachData[8])  } })
    return result


def ReadLeadorResult(_file, _isvel):
    result = {}
    with open(_file, 'r') as file:
        content = file.readlines()
        for eachLine in content:
            if eachLine.startswith("%"):  # ignore comment line
                continue
            eachData = eachLine.split()
#            if (float(eachData[8]) > 7):
#                continue
            result.update({ float(eachData[0]): {   'b':float(eachData[5]),   'l':float(eachData[6]),   'h':float(eachData[7]),
                                                   'vb':float(eachData[11]), 'vl':float(eachData[12]), 'vh':float(eachData[13]),
                                                 'stat':float(eachData[8])} })   #pos
    return result


def ReadIERefResult(_file, _isvel, _isatt):
    result = {}
    with open(_file, 'r') as file:
        content = file.readlines()
        for eachLine in content:
            if eachLine.startswith("%"):  # ignore comment line
                continue
            eachData = eachLine.split(",")
#            if (float(eachData[8]) > 7):
#                continue

            if _isatt:
                result.update({round(float(eachData[1]), 2): {'b': float(eachData[2]), 'l': float(eachData[3]), 'h': float(eachData[4]),
                                                              'vb': float(eachData[6]), 'vl': float(eachData[5]),'vh': float(eachData[7]),
                                                              "r": float(eachData[8]), "p": float(eachData[9]), "y": float(eachData[10]),
                                                              'stat': float(eachData[13])}})  # note stat shoule be 14
            else:
                result.update({round(float(eachData[1]), 2): {'b': float(eachData[2]), 'l': float(eachData[3]),
                                                              'h': float(eachData[4]),
                                                              'vb': float(eachData[6]), 'vl': float(eachData[5]),
                                                              'vh': float(eachData[7]),
                                                              'stat': float(eachData[13])}})  # note stat shoule be 14
    return result


def ReadRtklibResult(_file, _isvel):
    result = {}
    with open(_file, 'r') as file:
        content = file.readlines()
        for eachLine in content:
            if eachLine.startswith("%"):  # ignore comment line
                continue
            eachData = eachLine.split()
            result.update({float(eachData[1]):
                               {'b': float(eachData[2]), 'l': float(eachData[3]), 'h': float(eachData[4]),
                                'num': float(eachData[6]),
                              #  'ratio': float(eachData[13]),
                                'stat': float(eachData[5])}})
    return result


def ReadGlabResult(_file, _isvel):
    result = {}
    with open(_file, 'r') as file:
        content = file.readlines()
        for eachLine in content:
            if eachLine.startswith("%"):  # ignore comment line
                continue
            eachData = eachLine.split()
            result.update({float(eachData[1]):
                               {'b': float(eachData[3]), 'l': float(eachData[2]), 'h': float(eachData[4]),
                                'num': 0,
                                'stat': 0}})
    return result


def ReadPandaResult(_file, _isvel):
    result = {}
    with open(_file, 'r') as file:
        content = file.readlines()
        for eachLine in content:
            if eachLine.startswith("%"):  # ignore comment line
                continue
            eachData = eachLine.split()
            result.update({float(eachData[7]):
                               {'b': float(eachData[15]) , 'l': float(eachData[16]), 'h': float(eachData[17]),
                                'num': 0,
                                'stat': 0}})
    return result


def CalDifference(_cal, _ref, _isvel, _isatt):
    result = {}
    for time, cal in _cal.items():
        if time in _ref.keys():

         #   if (int(cal['stat']) == 20):
         #      continue
         #   if (int(cal['stat']) == 50):
         #       continue

            if (_isvel and _isatt):
                if _ref[time]["y"] < 0:
                    _ref[time]["y"] += 360
                result.update(
                    {time: {'b': radians(cal["b"] - _ref[time]["b"]) * RE,
                            'l': radians(cal["l"] - _ref[time]["l"]) * RE / sin(radians(cal["b"])),
                            'h': cal["h"] - _ref[time]["h"],
                            'vb': cal["vb"] - _ref[time]["vb"],
                            'vl': cal["vl"] - _ref[time]["vl"],
                            'vh': cal["vh"] - _ref[time]["vh"],
                            'r':  cal["r"] - _ref[time]["r"],
                            'p': cal["p"] - _ref[time]["p"],
                            'y': cal["y"] - _ref[time]["y"],
                            'stat': cal['stat'],
                            #   'ratio': cal["ratio"]
                            }})

            elif _isvel:
                result.update(
                    {time: {'b': radians(cal["b"] - _ref[time]["b"]) * RE,
                            'l': radians(cal["l"] - _ref[time]["l"]) * RE / sin(radians(cal["b"])),
                            'h': cal["h"] - _ref[time]["h"],
                            'vb': cal["vb"] - _ref[time]["vb"],
                            'vl': cal["vl"] - _ref[time]["vl"],
                            'vh': cal["vh"] - _ref[time]["vh"],
                            'stat': cal['stat'],
                            #   'ratio': cal["ratio"]
                            }})
            else:
                result.update(
                    {time: {'b': radians(cal["b"] - _ref[time]["b"]) * RE,
                            'l': radians(cal["l"] - _ref[time]["l"]) * RE / sin(radians(cal["b"])),
                            'h': cal["h"] - _ref[time]["h"],
                            'stat': cal['stat'],
                          #  'ratio': cal["ratio"]
                            }})

    return result


def StatisticResult(_det, _isvel, _isatt):
    all, fix = 0, 0
    _stat = cStat()
    for time, det in detValue.items():
        all += 1;
        if abs(det["b"]) < 200:
            _stat.dx.append(det["b"])
        if abs(det["l"]) < 200:
            _stat.dy.append(det["l"])
        if abs(det["h"]) < 200:
            _stat.dz.append(det["h"])
        if det["stat"] == 1 or det["stat"] == 3:
            fix += 1

    _stat.max  = [maxabs(_stat.dx)*100, maxabs(_stat.dy)*100, maxabs(_stat.dz)*100]
    _stat.std  = [std(_stat.dx)*100,    std(_stat.dy)*100,    std(_stat.dz)*100]
    _stat.rms  = [rms(_stat.dx)*100,    rms(_stat.dy)*100,    rms(_stat.dz)*100]
    _stat.mean = [mean(_stat.dx)*100,  mean(_stat.dy)*100,   mean(_stat.dz)*100]

    print("Position Error (BLH cm):")
    print("RMS   %9.3f    %9.3f    %9.3f" % (_stat.rms[0], _stat.rms[1], _stat.rms[2]))
    print("STD   %9.3f    %9.3f    %9.3f" % (_stat.std[0], _stat.std[1], _stat.std[2]))
    print("MAX   %9.3f    %9.3f    %9.3f" % (_stat.max[0], _stat.max[1], _stat.max[2]))
    print("MEAN  %9.3f    %9.3f    %9.3f" % (_stat.mean[0], _stat.mean[1], _stat.mean[2]))

    if _isvel:
        for time, det in detValue.items():
            if abs(det["vb"]) < 2:
                _stat.dvx.append(det["vb"])
            if abs(det["vl"]) < 2:
                _stat.dvy.append(det["vl"])
            if abs(det["vh"]) < 2:
                _stat.dvz.append(det["vh"])
        _stat.vmax = [maxabs(_stat.dvx)*100, maxabs(_stat.dvy)*100, maxabs(_stat.dvz)*100]
        _stat.vstd = [std(_stat.dvx)*100,    std(_stat.dvy)*100,    std(_stat.dvz)*100]
        _stat.vrms = [rms(_stat.dvx)*100,    rms(_stat.dvy)*100,    rms(_stat.dvz)*100]
        _stat.vmean = [mean(_stat.dvx)*100,  mean(_stat.dvy)*100,   mean(_stat.dvz)*100]
        print("velocity Error (BLH cm/s):")
        print("RMS   %9.3f    %9.3f    %9.3f" % (_stat.vrms[0], _stat.vrms[1], _stat.vrms[2]))
        print("STD   %9.3f    %9.3f    %9.3f" % (_stat.vstd[0], _stat.vstd[1], _stat.vstd[2]))
        print("MAX   %9.3f    %9.3f    %9.3f" % (_stat.vmax[0], _stat.vmax[1], _stat.vmax[2]))
        print("MEAN  %9.3f    %9.3f    %9.3f" % (_stat.vmean[0], _stat.vmean[1], _stat.vmean[2]))



    if _isatt:
        for time, det in detValue.items():
            if abs(det["r"]) < 2:
                _stat.dr.append(det["r"])
            if abs(det["p"]) < 2:
                _stat.dp.append(det["p"])
            if abs(det["y"]) < 2:
                _stat.dh.append(det["y"])
        _stat.amax = [maxabs(_stat.dr), maxabs(_stat.dp), maxabs(_stat.dh)]
        _stat.astd = [std(_stat.dr),    std(_stat.dp),    std(_stat.dh)]
        _stat.arms = [rms(_stat.dr),    rms(_stat.dp),    rms(_stat.dh)]
        _stat.amean = [mean(_stat.dr),  mean(_stat.dp),   mean(_stat.dh)]
        print("Attitede Error (BLH °):")
        print("RMS   %9.3f    %9.3f    %9.3f" % (_stat.arms[0], _stat.arms[1], _stat.arms[2]))
        print("STD   %9.3f    %9.3f    %9.3f" % (_stat.astd[0], _stat.astd[1], _stat.astd[2]))
        print("MAX   %9.3f    %9.3f    %9.3f" % (_stat.amax[0], _stat.amax[1], _stat.amax[2]))
        print("MEAN  %9.3f    %9.3f    %9.3f" % (_stat.amean[0], _stat.amean[1], _stat.amean[2]))

    print("All epoch:%7d, Fix epoch:%7d, Percentage:%7.3f" % (all, fix, fix / all * 100))

    return _stat


def ExportDifference(_file, _det):
    with open(_file, 'w') as file:
        for time, each in _det.items():
            file.write('%10.3f, %9.3f, %9.3f, %9.3f, %2d \n' %
                       (time, each["b"], each["l"], each["h"], each["stat"])
                      )


def DrawFigure(_stat, _isvel, _isatt):
    ## draw position
    plt.figure(dpi = 100, figsize=(10, 6.18))
    plt.plot(_stat.dz, 'c', label = 'rmsh:' + str(round(_stat.rms[2], 5)))
    plt.plot(_stat.dx, 'blue', label = 'rmsb:' + str(round(_stat.rms[0], 5)))
    plt.plot(_stat.dy, 'red', label = 'rmsl:' + str(round(_stat.rms[1], 5)))
    plt.xlabel('epoch')
    plt.ylabel('m')
    plt.ylim(-10,10)
    plt.legend()
    plt.grid(True)
    plt.show()

    ## draw velocity
    if _isvel:
        plt.figure(dpi = 100, figsize=(10, 6.18))
        plt.plot(_stat.dvz, 'c', label = 'rmsh:' + str(round(_stat.vrms[2], 5)))
        plt.plot(_stat.dvx, 'blue', label = 'rmsb:' + str(round(_stat.vrms[0], 5)))
        plt.plot(_stat.dvy, 'red', label = 'rmsl:' + str(round(_stat.vrms[1], 5)))
        plt.xlabel('epoch')
        plt.ylabel('m/s')
        plt.ylim(- 1, 1)
        plt.legend()
        plt.grid(True)
        plt.show()

    if _isatt:
        plt.figure(dpi = 100, figsize=(10, 6.18))
        plt.plot(_stat.dr, 'c', label = 'rmsr:' + str(round(_stat.arms[2], 5)))
        plt.plot(_stat.dp, 'blue', label = 'rmsp:' + str(round(_stat.arms[0], 5)))
        plt.plot(_stat.dh, 'red', label = 'rmsh:' + str(round(_stat.arms[1], 5)))
        plt.xlabel('epoch')
        plt.ylabel('°')
        plt.ylim(-0.5,0.5)
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == '__main__':

    filepath = "E:\\VirtualFile\\data\\paper\\Alloy\\"

    calFile = filepath + "my2.txt"
    refFile = filepath + "ref.txt"
    detFile = filepath + "det.txt"

    isVel = True
    isAtt = False

    calValue = ReadMyResult(calFile, isVel, isAtt)
 #   calValue = ReadPandaResult(calFile, isVel)
 #   calValue = ReadRtklibResult(calFile, isVel)
#   calValue = ReadIERefResult(calFile, isVel)

  #  refValue = ReadLeadorResult(refFile, isVel)
    refValue = ReadIERefResult(refFile, isVel, isAtt)
 #   refValue = ReadMyResult(refFile, isVel)


    detValue = CalDifference(calValue, refValue, isVel, isAtt)
    Stat = StatisticResult(detValue, isVel, isAtt)

    ExportDifference(detFile, detValue)

    DrawFigure(Stat, isVel, isAtt)




