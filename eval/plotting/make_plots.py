#!/usr/bin/python3

from cProfile import label, run
from math import log2
from multiprocessing.pool import RUN
from os.path import join
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt
from matplotlib import style
import matplotlib.ticker as ticker
import matplotlib.patches as mpatches
import pandas as pd
import numpy as np
import math

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

from pandas.core.base import DataError

ggPlotColors = importlib.import_module("ggplot_colors").__dict__["ggColorSlice"]
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
    )

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


def make_dijkstra_rank_tick_labels(ax_axis, series):
    ax_axis.set_ticks(series + 1)
    ax_axis.set_ticklabels([exp2_ticks_from_exponent(int(i)) for i in series])


def get_boxplot_outliers(df, by_column_name):
    q1 = df[by_column_name].quantile(0.25)
    q3 = df[by_column_name].quantile(0.75)
    iqr = q3 - q1
    filter = np.invert(
        (df[by_column_name] >= q1 - 1.5 * iqr) & (df[by_column_name] <= q3 + 1.5 * iqr)
    )
    return df.loc[filter]


def plot_variable_max_driving_time():
    run_measurement_conditionally(
        "measure_csp_variable_max_driving_time", "parking_ger_hgv"
    )

    run_measurement_conditionally(
        "measure_csp_variable_max_driving_time", "parking_ger"
    )

    csp_parking_hgv = (
        read_measurement("measure_csp_variable_max_driving_time-parking_ger_hgv")
        .set_index("max_driving_time_ms")
        .fillna(-1)
    )

    csp_parking = (
        read_measurement("measure_csp_variable_max_driving_time-parking_ger")
        .set_index("max_driving_time_ms")
        .fillna(-1)
    )

    fig, ax = plt.subplots(figsize=(10, 5))
    csp_parking_hgv[["path_number_flagged_nodes", "time_ms"]].boxplot(
        by="path_number_flagged_nodes", ax=ax
    )
    ax.set_yscale("log")
    ax.set_xlabel("number of parkings on path (-1: no path)")
    ax.set_ylabel("time [ms]")

    # reduce tick labels
    every_nth = 4
    for n, label in enumerate(ax.xaxis.get_ticklabels()):
        if n % every_nth != 0:
            label.set_visible(False)

    write_plt()
    plt.savefig(
        os.path.join(OUTPUT_PATH, "variable_number_parkings.png"),
        dpi=1200,
    )
    plt.close()

    fig, ax = plt.subplots(figsize=(10, 5))
    csp_parking_hgv[["path_number_pauses", "time_ms"]].boxplot(
        by="path_number_pauses", ax=ax
    )
    ax.set_yscale("log")
    ax.set_xlabel("number of breaks (-1: no path)")
    ax.set_ylabel("time [ms]")

    plt.savefig(
        os.path.join(OUTPUT_PATH, "variable_number_breaks.png"),
        dpi=1200,
    )
    plt.close()

    fig, ax = plt.subplots(figsize=(10, 5))
    csp_parking[["path_number_pauses", "time_ms"]].boxplot(
        by="path_number_pauses", ax=ax
    )
    ax.set_yscale("log")
    ax.set_xlabel("number of breaks (-1: no path)")
    ax.set_ylabel("time [ms]")

    plt.savefig(
        os.path.join(OUTPUT_PATH, "variable_number_breaks_all_parkings.png"),
        dpi=1200,
    )
    plt.close()

    fig, ax = plt.subplots(figsize=(10, 5))
    csp_parking_hgv[["path_number_pauses", "num_nodes_searched"]].boxplot(
        by="path_number_pauses", ax=ax
    )
    ax.set_yscale("log")
    ax.set_xlabel("number of breaks (-1: no path)")
    ax.set_ylabel("#nodes visited")

    plt.savefig(
        os.path.join(OUTPUT_PATH, "variable_number_breaks_search_space.png"),
        dpi=1200,
    )
    plt.close()

    fig, ax = plt.subplots(figsize=(10, 5))
    csp_parking_hgv[["num_labels_reset", "time_ms"]].boxplot(
        by="num_labels_reset", ax=ax
    )
    ax.set_yscale("log")
    ax.set_xlabel("number of labels which were reset")
    ax.set_ylabel("time [ms]")

    # reduce tick labels
    every_nth = 50
    for n, label in enumerate(ax.xaxis.get_ticklabels()):
        if n % every_nth != 0:
            label.set_visible(False)

    plt.savefig(
        os.path.join(OUTPUT_PATH, "variable_number_labels_reset.png"),
        dpi=1200,
    )
    plt.close()


def plot_variable_pause_time():
    run_measurement_conditionally("measure_csp_variable_pause_time", "parking_ger_hgv")

    csp_parking = read_measurement(
        "measure_csp_variable_pause_time-parking_ger_hgv"
    ).fillna(-1)
    fig, ax = plt.subplots(figsize=(10, 5))
    csp_parking.set_index("pause_duration_ms")["time_ms"].plot(ax=ax)
    ax.set_yscale("log")
    ax.set_xlabel("pause duration [ms]")
    ax.set_ylabel("time [ms]")

    plt.savefig(
        os.path.join(OUTPUT_PATH, "variable_pause_duration.png"),
        dpi=1200,
    )
    plt.close()

    fig, ax = plt.subplots(figsize=(10, 5))
    csp_parking.set_index("pause_duration_ms")["num_labels_propagated"].plot(ax=ax)
    ax.set_yscale("log")
    ax.set_xlabel("pause duration [ms]")
    ax.set_ylabel("labels propagated")

    plt.savefig(
        os.path.join(OUTPUT_PATH, "variable_pause_duration_labels_propagated.png"),
        dpi=1200,
    )
    plt.close()

    fig, ax = plt.subplots(figsize=(10, 5))
    csp_parking.set_index("pause_duration_ms")["num_nodes_searched"].plot(ax=ax)
    ax.set_yscale("log")
    ax.set_xlabel("pause duration [ms]")
    ax.set_ylabel("nodes searched")

    plt.savefig(
        os.path.join(OUTPUT_PATH, "variable_pause_duration_nodes_searched.png"),
        dpi=1200,
    )
    plt.close()


def plot_1000_csp_queries(graph):
    run_measurement_conditionally("measure_csp_1000_queries", graph)

    csp_1000_queries = read_measurement("measure_csp_1000_queries-" + graph).fillna(-1)

    fig, ax = plt.subplots(figsize=(10, 5))
    csp_1000_queries[["path_number_pauses", "time_ms"]].boxplot(
        by="path_number_pauses", ax=ax
    )

    ax.set_xlabel("number of pauses on path (-1: no path)")
    ax.set_ylabel("time [ms]")

    write_plt("1000_queries_csp.png", graph)


def plot_rank_times(name, graph):
    run_measurement_conditionally(name, graph)

    csp_1000_queries = read_measurement(
        name + "-" + graph,
    )

    fig, ax = plt.subplots(figsize=(10, 5))
    bp = csp_1000_queries.boxplot(ax=ax, by="dijkstra_rank_exponent", column="time_ms")

    bp.get_figure().gca().set_title("")
    fig.suptitle("")

    ax.set_xlabel("dijkstra rank")
    ax.set_ylabel("query time [ms]")
    ax.set_yscale("log")

    make_dijkstra_rank_tick_labels(
        ax.xaxis, csp_1000_queries["dijkstra_rank_exponent"].unique()
    )

    ax2 = ax.twinx()
    colors = ggPlotColors(3)

    # DIRTY HACK BUT SOMEWHERE IT IS SHIFTED AGAINST EACH OTHER
    csp_1000_queries["dijkstra_rank_exponent"] = (
        csp_1000_queries["dijkstra_rank_exponent"] + 1
    )

    if "path_number_pauses" in csp_1000_queries.columns:
        csp_1000_queries.groupby("dijkstra_rank_exponent")[
            "path_number_pauses"
        ].mean().plot(
            ax=ax2,
            color=colors[1],
            style=".-",
            x=csp_1000_queries["dijkstra_rank_exponent"] + 1,
        )

        patch1 = mpatches.Patch(color=colors[0], label="query time")
        patch2 = mpatches.Patch(color=colors[1], label="mean number of breaks")
        ax2.legend(handles=[patch1, patch2], loc=0)

    else:
        gb = csp_1000_queries.groupby("dijkstra_rank_exponent").mean()

        gb["path_number_short_pauses"].plot(
            ax=ax2,
            color=colors[1],
            style=".-",
        )
        gb["path_number_long_pauses"].plot(ax=ax2, color=colors[2], style=".-")

        ax2.set_ylabel("avg number of short breaks")

        patch1 = mpatches.Patch(color=colors[0], label="query time")
        patch2 = mpatches.Patch(color=colors[1], label="mean number of short breaks")
        patch3 = mpatches.Patch(color=colors[2], label="mean number of long breaks")
        ax2.legend(handles=[patch1, patch2, patch3], loc=0)

    ax2.set_ylim(bottom=0)
    ax2.set_ylabel("#breaks")
    ax2.grid(False)
    ax2.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))
    plt.title("1000 queries on " + graph)

    write_plt(name + ".png", graph)


def plot_1000_csp_2_queries(graph):
    run_measurement_conditionally("measure_csp_2_1000_queries", graph)

    csp_1000_queries = read_measurement("measure_csp_2_1000_queries-" + graph).fillna(
        -1
    )

    fig, ax = plt.subplots(figsize=(10, 5))
    csp_1000_queries[["path_number_pauses", "time_ms"]].boxplot(
        by="path_number_pauses", ax=ax
    )

    ax.set_xlabel("number of pauses on path (-1: no path)")
    ax.set_ylabel("time [ms]")

    write_plt("1000_queries_csp_2.png", graph)


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
    args = parser.parse_args()

    RUN_ALL_MEASUREMENTS = args.force
    # plot_variable_max_driving_time()
    # plot_variable_pause_time()
    # plot_1000_csp_queries(args.graph)
    # plot_1000_csp_2_queries(args.graph)

    for g in args.graph:
        plot_rank_times("measure_csp_1000_queries_rank_times", g)
        plot_rank_times("measure_csp_2_1000_queries_rank_times", g)
