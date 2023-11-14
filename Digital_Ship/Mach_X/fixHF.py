#!/usr/bin/python3

import shutil
import os
hffile= input("Input file name: ")
shutil.copy(hffile, hffile+'.orig')
with open(hffile+'.orig', "r") as fin, open(hffile, "w") as fout:
    idx = 0
    for row in fin:
        if idx < 21:
            fout.write(row)
            if idx == 14:
                splt = row.rstrip().split()
                section = int(splt[1])
        else:
            if (idx > 20 and idx < 21+section):
                splt = row.rstrip().split()
                fout.write('{} {:11.4f} {:11.4f} {:11.4f} {:11.4f} {:11.4f} {:11.4f}\n'.format(splt[0].rjust(3,' '),float(splt[1]),float(splt[2]),float(splt[3]),float(splt[4]),float(splt[5]),float(splt[6])))
            else:
                if 'RADIUS' in row:
                    splt2 = row.rstrip().split()
                    #print(splt2)
                    fout.write(' RADIUS :{:11.4f} PUNKTZAHL :{:4d}   r/R =  {:.4f}    XC/C     YS/C     YP/C\n'.format(float(splt2[1]),int(splt2[3]),float(splt2[5])))
                else:
                    if 'HF' in row:
                        fout.write(row)
                    else:
                        splt = row.rstrip().split()
                        try:
                             fout.write('{:4d}{:12.4f}{:12.4f}{:12.4f}{:12.4f}{:12.4f}{:12.4f}\n'.format(int(splt[0]),float(splt[1]),float(splt[2]),float(splt[3]),float(splt[4]),float(splt[5]),float(splt[6])))
                        except:
                            fout.write(row)
        idx += 1
os.remove(hffile+'.orig')
print("Writing Complete: " +hffile)