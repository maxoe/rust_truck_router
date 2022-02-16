#!/usr/bin/python3

from cProfile import label
from math import log2
from os.path import join
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt
from matplotlib import style
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
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


def is_bin(fpath):
    return os.path.isfile(fpath) and os.access(fpath, os.X_OK)


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
    with open(os.path.join(BIN_PATH, bin + ".exe"), "rb") as f:
        h = hashlib.md5(f.read()).digest()
        f.close()
        return h


def is_up_to_data(bin):
    fpath = os.path.join(BIN_PATH, bin + ".exe")
    return is_bin(fpath) and create_file_hash(bin) == load_bin_hash(bin)


def run_bin(bin, args=[]):
    popen = subprocess.Popen(
        ["cargo", "run", "--release", "--bin", bin] + args,
        stdout=subprocess.PIPE,
        cwd=DATA_PATH,
    )
    popen.wait()


def exists_measurement(bin, graph):
    return os.path.isfile(os.path.join(DATA_PATH, bin + "-" + graph + ".txt"))


def run_measurement_conditionally(bin, graph):
    if is_up_to_data(bin) and exists_measurement(bin, graph):
        print('Skipping "' + bin + '" with "' + graph + '"')
    else:
        run_measurement(bin, graph)


def run_measurement(bin, graph):
    run_bin(bin, ["--", os.path.join(GRAPH_PATH, graph)])
    update_file_hash(bin)


def read_measurement(bin):
    return pd.read_csv(os.path.join(DATA_PATH, bin + ".txt"))


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


if __name__ == "__main__":
    plot_variable_max_driving_time()
    plot_variable_pause_time()
