#!/usr/bin/env python3

import csv
import sys
import itertools
import matplotlib
from matplotlib import pyplot as plt
import scipy

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
    keyfunc = lambda x: [x[k] for k in gpkeys]
    data = sorted(data, key=keyfunc)
    gp = itertools.groupby(data, keyfunc)
    return list(map(mapf, gp))

def round_agg(data, keys, score_func):
    rdagg = {}
    for key in filter(lambda k: k != "queue_size", keys):
        gpkeys = list(filter(lambda k: k != key, keys))
        varkey = key
        gps = groupby(data, gpkeys, varkey, score_func)
        rdagg[varkey] = gps
    return rdagg

def plot_rdagg(rdagg):
    for varkey, gps in rdagg.items():
        gps = list(gps)
        print(varkey)
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

        def plot_fig(ax, x, y):
            if varkey in ["data_size(Byte)", "comm_freq(Hz)"]:
                ax.semilogx(x,y)
            else:
                ax.plot(x, y)


        def plot_mul(ax):
            ax.set_title(varkey)
            for gp in gps:
                setting, scores = gp
                x,y = zip(*scores)
                plot_fig(ax, x, y)


        def plot_avg(ax):
            ax.set_title(varkey)
            scores = list(reduce(lambda a,b: a+b[1], gps, []))
            scores = sorted(scores)
            scoregp = itertools.groupby(scores, lambda x:x[0])

            scores = map(lambda x: (x[0], scipy.mean([y[1] for y in x[1]])) , scoregp)
            x,y = zip(*scores)
            plot_fig(ax, x, y)

        plot_mul(ax1)
        plot_avg(ax2)

        plt.show()

#(x[0], scipy.mean(list(x[1])[1]))
def analyze_recv_rate(csv_name):
    data = load_csv(csv_name)
    rdagg = round_agg(data, ["run_time(sec)", "data_size(Byte)", "comm_freq(Hz)", "queue_size", "talk_length", "lstn_length"], lambda x: float(x["lstn_sum"])/(float(x["talk_sum"])*float(x["lstn_length"])))
    plot_rdagg(rdagg)

def analyze_delay(csv_name):
    data = load_csv(csv_name)
    y = [d['delay'] for d in data]
    for x_type in ["interval", "data_size", "lstn_n", "talk_n"]:
        x = [d[x_type] for d in data]
        plt.figure()
        plt.title(x_type)
        plt.scatter(x,y)
        plt.show()

def main():
    csv_name, csv_type = sys.argv[1:3]
    if csv_type == "recv_rate":
        analyze_recv_rate(csv_name)
    elif csv_type == "delay":
        analyze_delay(csv_name)

if __name__ == '__main__':
    main()

