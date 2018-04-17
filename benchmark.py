#!/usr/bin/env python3

from multiprocessing import Pool
import subprocess
import statistics as Stat
import json
import sys

cmd_base = ["rosrun", "beginner_tutorials"]
#cmd_base = ["python"]

def init():
    import os, subprocess as sp, json
    source = 'source ./catkin_ws/devel/setup.bash'
    #source += ' && source ./catkin_ws/devel/setup.zsh'
    dump = '/usr/bin/python -c "import os, json;print json.dumps(dict(os.environ))"'
    pipe = sp.Popen(['/bin/bash', '-c', '%s && %s' %(source,dump)], stdout=sp.PIPE)
    env_b = pipe.stdout.read()
    env = json.loads(env_b.decode('utf-8'))
    os.environ = env

def nat_range(x):
    return range(1, x+1)

def run(args):
    output = subprocess.check_output(args)
    return output

def run_talker(run_time, data_size, comm_freq, queue_size):
    cmd = cmd_base + ["ipctest-talker.py", run_time, data_size, comm_freq, queue_size]
    output = run(cmd)
    return int(output)

def run_listener(run_time, data_size, comm_freq, queue_size):
    cmd = cmd_base + ["ipctest-listener.py", run_time, data_size, comm_freq, queue_size]
    output = run(cmd)
    output = output.decode().strip()
    output = output.split('\n')
    data_recv = int(output[0])
    recv_flow = [x.split() for x in output[1:]]
    recv_flow = list(map(lambda x: (float(x[0]), float(x[1]), int(x[2])), recv_flow))
    return (data_recv, recv_flow)

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

def gen_lstn(run_time, data_size, comm_freq, queue_size):
    return ("lstn", [run_time, data_size, comm_freq, queue_size])

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
                "median": Stat.median(l),
                "stdev": Stat.stdev(l) if len(l) > 1 else 0,
                "sum" : sum(l)
                }
    stat["talk"] = l2stat(talk)
    stat["lstn"] = l2stat([datarecv_dataflow[0] for datarecv_dataflow in lstn])
    return stat

def test_mul_talk_mul_lstn(talk_n, lstn_n, args = None):
    p = Pool(talk_n + lstn_n)
    if args is None:
        args = ["1", "10", "100", "10"]
    talk = gen_talk(*args)
    lstn = gen_lstn(*args)

    for talk_i in nat_range(talk_n):
        for lstn_i in nat_range(lstn_n):
            res_list = p.map(launch, [talk]*talk_i + [lstn]*lstn_i)
            stat = collect_stat(args, res_list)
            yield stat

def main_test():
    from itertools import product
    queue_size = 10
    for (run_time, data_size, comm_freq) in product(range(1, 6), [1, 10, 100, 1000], [1, 10, 100, 1000]):
        for stat in test_mul_talk_mul_lstn(5, 5, list(map(str, [run_time, data_size, comm_freq, queue_size]))):
            yield stat

def delay_test(talk_n, lstn_n, run_time, data_size, comm_freq, queue_size):
    p = Pool(talk_n + lstn_n)
    talk_args = [str(run_time), str(data_size), str(comm_freq), str(queue_size)]
    lstn_args = [str(run_time), str(data_size), str(comm_freq), str(queue_size)]
    talk = gen_talk(*talk_args)
    lstn = gen_lstn(*lstn_args)

    def process_res_list(res_list):
        from functools import reduce
        data = [x[1][1] for x in res_list if x[0] == 'lstn']
        data = list(reduce(lambda a,b: b+a, data, []))
        for interval, delay, data_size in data:
            yield {"interval":interval,
                    "delay"  :delay,
                    "data_size":data_size}

    for talk_i in nat_range(talk_n):
        for lstn_i in nat_range(lstn_n):
            res_list = p.map(launch, [talk]*talk_i + [lstn]*lstn_i)
            for entry in process_res_list(res_list):
                entry["talk_n"] = talk_i
                entry["lstn_n"] = lstn_i
                yield entry

def main_delay_test():
    from itertools import product
    queue_size = 10
    for (run_time, data_size, comm_freq) in product(range(1, 6), [1, 10, 100, 1000], [1, 10, 100, 1000]):
        for stat in delay_test(5, 5, run_time, data_size, comm_freq, queue_size):
            yield stat

def small_delay_test():
    from itertools import product
    queue_size = 10
    for (run_time, data_size, comm_freq) in product(range(1, 3), [100, 1000], [100, 1000]):
        for stat in delay_test(2, 2, run_time, data_size, comm_freq, queue_size):
            yield stat

def uniform_delay_test():
    queue_size = 10
    settings = []
    for run_time in [1, 10, 100]:
        comm_freq = int(1000 / run_time)
        for data_size in [10, 1000, 1000000]:
            settings.append((run_time, data_size, comm_freq))
    for (run_time, data_size, comm_freq) in settings:
        for stat in delay_test(2, 2, run_time, data_size, comm_freq, queue_size):
            yield stat


def simple_test():
    p = Pool(20)
    talk = ("talk", ["1", "100", "10", "10"])
    lstn = ("lstn", ["1"])

    print(p.map(launch, [talk, talk, lstn, lstn]))

if __name__ == '__main__':
    init()
    #stats = test_mul_talk_mul_lstn(1, 1)
    #stats = main_test()
    #stats = delay_test(1, 1, 1, 100, 10, 10)
    #stats = small_delay_test()
    #stats = main_delay_test()
    stats = uniform_delay_test()
    cnt = 0
    for stat in stats:
        print(json.dumps(stat))
        cnt += 1
        if (cnt % 1000 == 0):
            sys.stdout.flush()
    sys.stdout.flush()
