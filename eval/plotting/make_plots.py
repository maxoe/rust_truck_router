#!/usr/bin/python3

from cProfile import label, run
from math import log2
from multiprocessing.pool import RUN
from os.path import join
import subprocess
import os
import glob
from collections import OrderedDict
import importlib
import itertools
import pickle
import hashlib
import argparse
import time
import argparse

import matplotlib.pyplot as plt
from matplotlib import style
import matplotlib.ticker as ticker
import matplotlib.patches as mpatches
import pandas as pd
import numpy as np
import math

from pandas.core.base import DataError

ggPlotColors = importlib.import_module("ggplot_colors").__dict__["ggColorSlice"]
perfprof = importlib.import_module("perfprof").__dict__["perfprof"]
style.use("ggplot")
plt.rcParams["lines.linewidth"] = 1


# all paths are relative to this directory
DATA_PATH = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "../data")
)

if not os.path.exists(DATA_PATH):
    os.makedirs(DATA_PATH)

OUTPUT_PATH = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "../plots")
)

if not os.path.exists(OUTPUT_PATH):
    os.makedirs(OUTPUT_PATH)

PLOTDATA_PATH = os.path.normpath(os.path.join(DATA_PATH, ".plotdata"))

REPO_PATH = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../")
)

BIN_PATH = os.path.normpath(os.path.join(REPO_PATH, "target/release"))

GRAPH_PATH = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../")
)

RUN_ALL_MEASUREMENTS = False
PROHIBIT_MEASUREMENTS = False


def is_bin(fpath):
    return os.path.isfile(fpath) and os.access(fpath, os.X_OK)


def to_local_os_binary_file_name(file):
    if os.name != "nt" and file[-4:] == ".exe":
        return file[:-4]
    elif os.name == "nt" and file[-4:] != ".exe":
        return file + ".exe"

    return file


def load_bin_hashes():
    if not os.path.isfile(PLOTDATA_PATH):
        hashes = dict()
    else:
        with open(PLOTDATA_PATH, "rb") as f:
            hashes = pickle.load(f)
            f.close()

    return hashes


def load_bin_hash(bin):
    hashes = load_bin_hashes()

    if bin in hashes.keys():
        return hashes[bin]
    else:
        None


def update_file_hash(bin):
    hashes = load_bin_hashes()
    hashes[bin] = create_file_hash(bin)

    with open(PLOTDATA_PATH, "wb") as f:
        pickle.dump(hashes, f)


def create_file_hash(bin):
    with open(to_local_os_binary_file_name(os.path.join(BIN_PATH, bin)), "rb") as f:
        h = hashlib.md5(f.read()).digest()
        f.close()
        return h


def is_hash_up_to_date(bin):
    fpath = to_local_os_binary_file_name(os.path.join(BIN_PATH, bin))
    return is_bin(fpath) and create_file_hash(bin) == load_bin_hash(bin)


def run_bin(bin, args=[]):
    start = time.time()
    popen = subprocess.Popen(
        ["cargo", "run", "--release", "--bin", bin] + args,
        stdout=subprocess.PIPE,
        cwd=DATA_PATH,
    )
    popen.communicate()
    print('"' + bin + '" ran in {:.2f}'.format(time.time() - start))


def run_build(bin):
    popen = subprocess.Popen(
        ["cargo", "build", "--release", "--bin", bin],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    stream_data = popen.communicate()

    if popen.returncode is not 0:
        print('Build of "' + bin + '" failed with exit code ' + str(popen.returncode))
        return False

    return True


def exists_measurement(bin, graph):
    return os.path.isfile(os.path.join(DATA_PATH, bin + "-" + graph + ".txt"))


def run_measurement_conditionally(bin, graph):
    build_successful = run_build(bin)
    should_skip = (
        is_hash_up_to_date(bin)
        and exists_measurement(bin, graph)
        and not RUN_ALL_MEASUREMENTS
    ) or PROHIBIT_MEASUREMENTS

    if not build_successful or should_skip:
        print('Skipping measurement of "' + bin + '" with "' + graph + '"')
    elif not should_skip:
        run_measurement(bin, graph)


def run_measurement(bin, graph):
    run_bin(bin, ["--", os.path.join(GRAPH_PATH, graph)])
    update_file_hash(bin)


def build_measurement(bin, graph):
    run_bin(bin, ["--", os.path.join(GRAPH_PATH, graph)])


def read_measurement(bin, *args, **kwargs):
    return pd.read_csv(os.path.join(DATA_PATH, bin + ".txt"), *args, **kwargs)


def write_plt(filename, graph):
    path = os.path.join(os.path.join(OUTPUT_PATH, graph))

    if not os.path.exists(path):
        os.makedirs(path)

    plt.savefig(
        os.path.join(
            path,
            filename,
        ),
        dpi=1200,
    )
    plt.close()


def exp2_ticks(x):
    if x == 0:
        return "$0$"

    exponent = int(np.log2(x))
    coeff = x / 2 ** exponent

    if coeff == 1:
        return r"$2^{{ {:2d} }}$".format(exponent)

    return r"${:2.0f} \times 2^{{ {:2d} }}$".format(coeff, exponent)


def exp2_ticks_from_exponent(exponent):
    return r"$2^{{ {:2d} }}$".format(exponent)


def make_dijkstra_rank_tick_labels_from_exponent(ax_axis, exponents):
    ax_axis.set_ticklabels([exp2_ticks_from_exponent(int(i)) for i in exponents])


def make_dijkstra_rank_tick_labels_from_number(ax_axis, exponents):
    ax_axis.set_ticklabels([exp2_ticks(int(i)) for i in exponents])


def get_boxplot_outliers(df, by_column_name):
    q1 = df[by_column_name].quantile(0.25)
    q3 = df[by_column_name].quantile(0.75)
    iqr = q3 - q1
    filter = np.invert(
        (df[by_column_name] >= q1 - 1.5 * iqr) & (df[by_column_name] <= q3 + 1.5 * iqr)
    )
    return df.loc[filter]


def plot_all_rank_times(problem, graph):
    name = "measure_all_" + problem + "_1000_queries_rank_times"

    run_measurement_conditionally(name, graph)

    queries_all = read_measurement(
        name + "-" + graph,
    )

    # plot only 2^10 and larger
    queries_all = queries_all.loc[queries_all["dijkstra_rank_exponent"] >= 10]

    colors = ggPlotColors(4)

    to_plot = [
        ("time_ms", "log"),
    ]

    # algos = ["astar_chpot", "astar_bidir_chpot", "core_ch", "core_ch_chpot"]
    algos = ["astar_chpot", "core_ch", "core_ch_chpot"]

    for algo in algos:
        queries = queries_all.loc[queries_all["algo"] == algo]
        # queries = queries.loc[queries["path_distance"] == -1]
        for (column_name, plot_scale) in to_plot:
            if column_name in queries.columns:
                fig, ax = plt.subplots(figsize=(10, 5))
                bp = queries.boxplot(
                    ax=ax, by="dijkstra_rank_exponent", column=column_name
                )

                bp.get_figure().gca().set_title("")
                fig.suptitle("")

                ax.set_xlabel("dijkstra rank")
                ax.set_ylabel(column_name)
                ax.set_yscale(plot_scale)

                if plot_scale == "linear":
                    ax.set_ylim(bottom=-0.1)
                    ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))

                make_dijkstra_rank_tick_labels_from_exponent(
                    ax.xaxis, queries["dijkstra_rank_exponent"].unique()
                )

                plt.title(name + "-" + algo + ": " + graph)
                fig.tight_layout()

                write_plt(name + "-" + algo + "-" + column_name + ".png", graph)


def plot_rank_times_perf_profile(problem, graph):
    algos = ["astar_chpot", "core_ch_chpot"]
    algo_times = np.ndarray([])
    linespecs = ["r-", "b-"]  # , "g-"]

    run_measurement_conditionally(
        "measure_all_" + problem + "_1000_queries_rank_times", g
    )

    algo_results = read_measurement(
        "measure_all_" + problem + "_1000_queries_rank_times-" + g,
    )

    algo_results = algo_results.loc[algo_results["dijkstra_rank_exponent"] >= 10]

    algo_times = []
    for a in algos:
        algo_times = algo_times + [
            list(algo_results.loc[algo_results["algo"] == a]["time_ms"])
        ]

    algo_times = np.asarray(algo_times).T
    perfprof(algo_times, linespecs=linespecs, legendnames=algos)
    write_plt("measure_all_" + problem + "_1000_queries_rank_times-perfprofile.png", g)


def plot_core_size_experiments(problem, graph):
    name = "measure_" + problem + "_core_sizes"

    run_measurement_conditionally(name, graph)

    queries = read_measurement(
        name + "-" + graph,
    )

    # plot only 2^10 and larger
    # queries_all = queries_all.loc[queries_all["dijkstra_rank_exponent"] >= 10]

    colors = ggPlotColors(4)

    fig, ax = plt.subplots(figsize=(10, 5))
    bp = queries.boxplot(ax=ax, by="core_size", column="time_ms")

    bp.get_figure().gca().set_title("")
    fig.suptitle("")

    ax.set_xlabel("relative core size")
    ax.set_ylabel("time [ms]")
    ax.set_yscale("log")

    # if plot_scale == "linear":
    #     ax.set_ylim(bottom=-0.1)
    #     ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))

    # make_dijkstra_rank_tick_labels_from_exponent(
    #     ax.xaxis, queries["dijkstra_rank_exponent"].unique()
    # )

    plt.title("Core CH A* with CH Potentials for " + problem + ": " + graph)
    fig.tight_layout()

    write_plt(name, graph)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-g",
        "--graph",
        nargs="+",
        required=True,
        dest="graph",
        help="the graph directory in GRAPH_PATH",
    )
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        help="force rerun of all measurements",
    )
    parser.add_argument(
        "-p",
        "--plot",
        action="store_true",
        help="prohibit rerun of any measurement",
    )
    args = parser.parse_args()

    RUN_ALL_MEASUREMENTS = args.force
    PROHIBIT_MEASUREMENTS = args.plot

    if RUN_ALL_MEASUREMENTS and PROHIBIT_MEASUREMENTS:
        print("Cannot force and prohibit measurements and the same time")
        exit(0)

    for g in args.graph:
        for p in ["csp", "csp_2"]:
            # plot_all_rank_times(p, g)
            # plot_rank_times_perf_profile(p, g)
            plot_core_size_experiments(p, g)
