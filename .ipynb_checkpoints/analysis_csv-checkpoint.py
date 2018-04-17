#!/usr/bin/env python3

import csv
import sys
import itertools
import matplotlib
from matplotlib import pyplot as plt
import scipy
import numpy as np
import sphviewer as sph
import math

import sys
sys.path += ['/home/yt113/.local/lib/python3.5/site-packages', '/usr/local/lib/python3.5/dist-packages']

import seaborn as sns
from collections import defaultdict

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

def heatplot(x, y, nb=32, xsize=500, ysize=500):   
    xmin = np.min(x)
    xmax = np.max(x)
    ymin = np.min(y)
    ymax = np.max(y)

    x0 = (xmin+xmax)/2.
    y0 = (ymin+ymax)/2.

    pos = np.zeros([3, len(x)])
    pos[0,:] = x
    pos[1,:] = y
    w = np.ones(len(x))

    P = sph.Particles(pos, w, nb=nb)
    S = sph.Scene(P)
    S.update_camera(r='infinity', x=x0, y=y0, z=0, 
                    xsize=xsize, ysize=ysize)
    R = sph.Render(S)
    R.set_logscale()
    img = R.get_image()
    extent = R.get_extent()
    for i, j in zip(xrange(4), [x0,x0,y0,y0]):
        extent[i] += j
    return img, extent
        
#(x[0], scipy.mean(list(x[1])[1]))
def analyze_recv_rate(csv_name):
    data = load_csv(csv_name)
    rdagg = round_agg(data, ["run_time(sec)", "data_size(Byte)", "comm_freq(Hz)", "queue_size", "talk_length", "lstn_length"], lambda x: float(x["lstn_sum"])/(float(x["talk_sum"])*float(x["lstn_length"])))
    plot_rdagg(rdagg)

def analyze_delay(csv_name):
    data = load_csv(csv_name)
    data = [{k: float(v) for k,v in d.items()} for d in data if float(d["interval"]) > 0]
    y = [float(d['delay']) for d in data]
    xs = {}
    for x_type in ["interval", "data_size", "lstn_n", "talk_n"]:
        xs[x_type] = [float(d[x_type]) for d in data]

    x_interval = xs["interval"]
    #print([x for x in x_interval if x < 0])
    x_data_size = xs["data_size"]
    xs["speed"] = [ds / intv for intv, ds  in zip(x_interval, x_data_size)]
    print(min(xs["speed"]))
    print(max(xs["speed"]))
    print(min(xs["interval"]))
    print(max(xs["interval"]))
    print(min(xs["data_size"]))
    print(max(xs["data_size"]))
    
    x = xs["data_size"]
    heatmap_16, extent_16 = heatplot(x,y, nb=64)
    plt.figure()
    ax = plt.gca()
    ax.imshow(heatmap_16, extent=extent_16, origin='lower', aspect='auto')
    ax.set_title("Smoothing over 16 neighbors")
    
    x = xs["data_size"]
    x_sub = [x[i] for i in range(len(y)) if y[i] < 0.001]
    y_sub = [y[i] for i in range(len(y)) if y[i] < 0.001]
    heatmap_16, extent_16 = heatplot(x_sub,y_sub, nb=128)
    plt.figure()
    ax = plt.gca()
    ax.imshow(heatmap_16, extent=extent_16, origin='lower', aspect='auto')
    ax.set_title("Smoothing over 16 neighbors")
    
    x = xs["speed"]
    #x_sub = [x[i] for i in range(len(y)) if y[i] < 0.001]
    #y_sub = [y[i] for i in range(len(y)) if y[i] < 0.001]
    heatmap_16, extent_16 = heatplot(x,y, nb=32)
    plt.figure()
    ax = plt.gca()
    ax.imshow(heatmap_16, extent=extent_16, origin='lower', aspect='auto')
    ax.set_title("speed")
    
    # x = xs["data_size"]
    # heatmap, xedges, yedges = np.histogram2d(x, y, bins=50)
    # extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]
    # plt.clf()
    # plt.imshow(heatmap.T, extent=extent, origin='lower')
    # plt.show()
    
    # ax = sns.heatmap(zip(xs["data_size"], y), linewidth=0.5)
    # plt.show()
    
    # plt.figure()
    # plt.hexbin(xs["data_size"], y)
    # plt.show()  

    # from mpl_toolkits.mplot3d import Axes3D
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(xs["interval"], xs["data_size"], y, edgecolors='none', alpha=0.5)
    # plt.show()

    for x_type, x in xs.items():
        plt.figure()
        ax = plt.gca()
        ax.set_title(x_type)
        ax.scatter(x,y, edgecolors='none', alpha=0.5)
        if x_type in ["speed"]:
            ax.set_xscale('log')
        plt.show()
        
def scatter_plot(ax, x,y, title, x_label, y_label, x_log=False):
    if ax is None:
        plt.figure()
        ax = plt.gca()
    ax.set_title(title)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.scatter(x,y, edgecolors='none', alpha=0.5)
    if x_log:
        ax.set_xscale('log')
    #plt.show()

def heat_plot(ax, x, y, title, x_label, y_label, nb=64):
    heatmap, extent = heatplot(x,y, nb=nb)
    if ax is None:
        plt.figure()
        ax = plt.gca()
    ax.imshow(heatmap, extent=extent, origin='lower', aspect='auto')
    ax.set_title(title)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)

    
def analyze_delay_in_nodenums(csv_name):
    data = load_csv(csv_name)
    data = [{k: float(v) for k,v in d.items()} for d in data if float(d["interval"]) > 0]
    
    data_in_nodenums = defaultdict(list)
    for d in data:
        data_in_nodenums[(int(d["talk_n"]), int(d["lstn_n"]))].append(d)
        
    nn = data_in_nodenums.keys()
    max_talkn = max(np[0] for np in nn)
    max_lstnn = max(np[1] for np in nn)
    figs = {}
    for x_type in ["interval", "data_size", "speed", "interval_heat"]:
        fig, axes = plt.subplots(max_talkn, max_lstnn, sharex=True, sharey=True, figsize=(15,15))
        figs[x_type] = {"fig":fig, "axes":axes}
        
    for nn, sub_data in data_in_nodenums.items():
        talk_n, lstn_n = nn
        title_base = "talk_n=%d lstn_n=%d "%(talk_n, lstn_n)
        y_shared = [float(d['delay']) * 1000 for d in sub_data] #ms
        y_label = "delay (ms)"
        xs = {}
        xs["interval"] = np.array([float(d["interval"]) * 1000 for d in sub_data]) # ms
        xs["data_size"] = np.array([float(d["data_size"]) / 1000 for d in sub_data]) # KB
        xs["speed"] = xs["data_size"] / xs["interval"] # KB / ms
        xs["interval_heat"] = np.log10(xs["interval"]) # 10^(x) KB
        
        xlbl = {}
        xlbl["interval"] = "interval (ms)"
        xlbl["data_size"] = "data_size (KB)"
        xlbl["speed"] = "speed (KB/ms)"
        xlbl["interval_heat"] = "interval (10^(x) KB)"
        
        # new_xs = {}
        # new_xs["interval_heat"] = xs["interval_heat"]
        # xs = new_xs
        for x_type, x in xs.items():
            axes = figs[x_type]["axes"]
            talk_n, lstn_n = nn
            ax = axes[talk_n-1, lstn_n-1]
            y = y_shared
            title = title_base + x_type + " - " + "delay" 
            x_label = xlbl[x_type]
            x_log = False
            if x_type in ["interval", "data_size", "speed"]:
                x_log = True
            
            if x_type in ["interval", "data_size", "speed"]:
                scatter_plot(ax, x, y, title, x_label, y_label, x_log)
            elif x_type in ["interval_heat"]:
                heat_plot(ax, x, y, title, x_label, y_label, 128)

def main():
    csv_name, csv_type = sys.argv[1:3]
    if csv_type == "recv_rate":
        analyze_recv_rate(csv_name)
    elif csv_type == "delay":
        #analyze_delay_in_nodenums(csv_name)
        analyze_delay(csv_name)

if __name__ == '__main__':
    main()

