import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap


def plot_road(
    history: np.ndarray,
    vmax: int,
):
    if history.ndim != 3:
        raise ValueError("history must have shape (time, lane, length)")
    _, lanes, _ = history.shape

    colors = plt.get_cmap()(np.linspace(0, 1, vmax + 1))
    cmap = ListedColormap(np.vstack(([1, 1, 1, 1], colors)))

    fig, axes = plt.subplots(1, lanes, squeeze=False)
    for lane, ax in enumerate(axes[0]):
        image = ax.imshow(
            history[:, lane, :],
            cmap=cmap,
            origin="upper",
            interpolation="nearest",
        )
        ax.set(title=f"Lane {lane}", xlabel="Cell Coordinate", ylabel="Time Iteration")

    fig.colorbar(image, ax=axes[0], ticks=range(-1, vmax + 1), label="Velocity")


def plot_study(values, results, metric, ax):
    labels = {
        "flow_rate": "Flow Rate",
        "avg_velocity": "Average Velocity",
        "usage": "Lane Usage",
        "lane_change_frequency": "Lane swaps / car / timestep",
    }

    plot_lanes = metric in ("flow_rate", "avg_velocity", "usage")

    if plot_lanes:
        lane_count = len(results[0].lanes)

        for lane in range(lane_count):
            lane_values = [getattr(result.lanes[lane], metric) for result in results]
            ax.plot(values, lane_values, label=f"Lane {lane}")

        if (metric == "flow_rate" or metric == "avg_velocity") and lane_count > 1:
            ax.plot(
                values,
                [getattr(result, metric) for result in results],
                "--",
                label="Total",
            )
        ax.legend()
    else:
        ax.plot(values, [getattr(result, metric) for result in results])

    ax.set(xlabel="Car Density", ylabel=labels[metric])
