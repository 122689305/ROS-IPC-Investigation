#!/usr/bin/env python3

from multiprocessing import Pool
import subprocess
import statistics as Stat
import json

cmd_base = ["rosrun", "beginner_tutorials"]

def nat_range(x):
    return range(1, x+1)

def run(args):
    output = subprocess.check_output(args)
    return output

def run_talker(run_time, data_size, comm_freq, queue_size):
    cmd = cmd_base + ["ipctest-talker.py", run_time, data_size, comm_freq, queue_size]
    output = run(cmd)
    return int(output)

def run_listener(run_time):
    data_size, comm_freq, queue_size = ("-1", "-1", "-1") #trivial parameter
    cmd = cmd_base + ["ipctest-listener.py", run_time, data_size, comm_freq, queue_size]
    output = run(cmd)
    return int(output)

def unit(f):
    f()

def launch(args):
    func = args[0]
    args = args[1]
    if func == "talk":
        output = run_talker(*args)
    elif func == "lstn":
        output = run_listener(*args)
    else:
        output = run(args)
    return (func, output)

def gen_talk(run_time, data_size, comm_freq, queue_size):
    return ("talk", [run_time, data_size, comm_freq, queue_size])

def gen_lstn(run_time, data_size="", comm_freq="", queue_size=""):
    return ("lstn", [run_time])

def collect_stat(setting, res_list):
    stat = {"setting":setting}
    talk = []
    lstn = []
    for res in res_list:
        func, val = res
        if func == 'talk':
            talk.append(val)
        elif func == 'lstn':
            lstn.append(val)
    def l2stat(l):
        return {"length": len(l),
                "mean": Stat.mean(l),
                "mdeian": Stat.median(l),
                "stdev": Stat.stdev(l) if len(l) > 1 else 0,
                "sum" : sum(l)
                }
    stat["talk"] = l2stat(talk)
    stat["lstn"] = l2stat(lstn)
    return stat

def test_mul_talk_mul_lstn(talk_n, lstn_n, args = None):
    p = Pool(talk_n + lstn_n)
    if args is None:
        args = ["1", "10", "100", "10"]
    talk = gen_talk(*args)
    lstn = gen_lstn(*args)
    stat = []

    for talk_i in nat_range(talk_n):
        for lstn_i in nat_range(lstn_n):
            res_list = p.map(launch, [talk]*talk_i + [lstn]*lstn_i)
            stat.append(collect_stat(args, res_list))
    return stat

def main_test():
    queue_size = 10
    stat = []
    for (run_time, data_size, comm_rate) in zip(range(1, 11, 2), [10, 100, 1000, 10000, 100000, 1000000], [1, 10, 100, 1000]):
        stat += test_mul_talk_mul_lstn(5, 5, list(map(str, [run_time, data_size, comm_rate, queue_size])))
    return stat



def simple_test():
    p = Pool(20)
    talk = ("talk", ["1", "100", "10", "10"])
    lstn = ("lstn", ["1"])

    print(p.map(launch, [talk, talk, lstn, lstn]))

if __name__ == '__main__':
    #stat = test_mul_talk_mul_lstn(1, 1)
    stat = main_test()
    print(json.dumps(stat))
