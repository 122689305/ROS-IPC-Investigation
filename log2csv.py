#!/usr/bin/env python3
import json
import sys
import csv

def log2csv_rrate(logname):
    f = open(logname, 'r')
    cf = open(logname+'.csv', 'w')
    data = [json.loads(line) for line in f]
    head = ["run_time(sec)", "data_size(Byte)", "comm_freq(Hz)", "queue_size"]
    tktn = [list(map(lambda s: func+"_"+s, sorted(data[0]['talk'].keys()))) for func in ["talk", "lstn"]]
    head += tktn[0] + tktn[1]
    def d2l(d):
        return list(map(lambda x:x[1], sorted(d.items())))
    def l2il(l):
        return list(map(lambda x:int(x), l))
    cw = csv.writer(cf)
    cw.writerow(head)
    for stat in data:
        newline = l2il(stat['setting']) + d2l(stat['talk']) + d2l(stat['lstn'])
        cw.writerow(newline)

def log2csv_delay(lognam):
    head = ["interval", "delay", "data_size", "talk_n", "lstn_n"]
    f = open(logname, 'r')
    cf = open(logname+'.csv', 'w')
    data = [json.loads(line) for line in f]
    cw = csv.DictWriter(cf, fieldnames=head)
    cw.writeheader()
    for stat in data:
        if stat['interval'] < 100:
            cw.writerow(stat)

if __name__ == '__main__':
    logname, logtype = sys.argv[1:3]
    if logtype == 'recv_rate':
        log2csv_rrate(logname)
    elif logtype == 'delay':
        log2csv_delay(logname)
