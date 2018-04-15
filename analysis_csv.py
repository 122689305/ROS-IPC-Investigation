#!/usr/bin/env python3

import csv
import sys
import itertools

def load_csv(csv_name):
    cf = open(csv_name, 'r')
    cw = csv.DictReader(cf)
    return list(cw)

def groupby(data, gpkeys, varkey, score_func):
    def mapf (args):
        k,g = args
        def mapg(g):
            return (g[varkey], score_func(g))
        return (k, list(map(mapg, g)))

    gp = itertools.groupby(data, lambda x: [x[k] for k in gpkeys])
    return list(map(mapf, gp))

def main():
    csv_name = sys.argv[1]
    data = load_csv(csv_name)
    gb = groupby(data, ["run_time(sec)", "data_size(Byte)", "comm_freq(Hz)", "queue_size"] + ["talk_length"], "lstn_length", lambda x: float(x["lstn_sum"])/(float(x["talk_sum"])*float(x["lstn_length"])))
    for g in gb:
        print(g)

if __name__ == '__main__':
    main()

