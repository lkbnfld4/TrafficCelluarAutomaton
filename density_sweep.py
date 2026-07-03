import matplotlib.pyplot as plt
import numpy as np

from traffic import SimParams, run_experiment
from traffic.plotting import plot_study


def main():
    densities = np.linspace(0, 0.4, 20 + 1)[1:]
    results = []

    print(f"{'Density':>7}  {'Mean flow':>9}")
    for density in densities:
        params = SimParams(
            lanes=3,
            length=10_000,
            vMax=5,
            brakeProb=0.2,
            density=float(density),
        )
        result = run_experiment(params, steps=5_000, warmup=1_000)
        results.append(result)
        print(f"{density:7.2f}  {result.flow_rate:9.5f}")

    _, axes = plt.subplots(1, 3, figsize=(12, 4), layout="constrained")
    for ax in axes:
        ax.set_box_aspect(1)
    plot_study(densities, results, "flow_rate", ax=axes[0])
    plot_study(densities, results, "usage", ax=axes[1])
    plot_study(densities, results, "lane_change_frequency", ax=axes[2])
    plt.show()


if __name__ == "__main__":
    main()
