#!/usr/bin/python3

import sys

import matplotlib  # noqa: F401
import matplotlib.pyplot as plt

import numpy as np  # noqa: F401

import pandas as pd


def plot_result(name_physics):
    dataframe_data = pd.DataFrame()
    names = []
    for i in range(7):
        name = name_physics + str(i) + '.csv'
        try:
            dataframe = pd.read_csv(sys.argv[1] + name, skiprows=0, sep=',', engine='python')
            dataframe.columns = [name]
            dataframe_data[name] = dataframe[name]
            names.append(name)
        except Exception as e:
            pass
    ax = dataframe_data[names].plot()
    ax.set_ylabel('Realtime factor')
    ax.set_title(name_physics)


plot_result('unthrottled')
plot_result('split_unthrottled')
plt.show()
