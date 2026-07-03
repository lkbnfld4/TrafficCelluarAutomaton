import argparse

from traffic.core import SimParams, TrafficSim
from traffic.experiment import ExperimentResult, LaneResult, run_experiment
from traffic.plotting import plot_road, plot_study

__all__ = [
    "ExperimentResult",
    "LaneResult",
    "SimParams",
    "TrafficSim",
    "main",
    "plot_road",
    "plot_study",
    "run_experiment",
]


def main():
    import matplotlib.pyplot as plt
    import numpy as np

    parser = argparse.ArgumentParser(prog="traffic", allow_abbrev=False)
    parser.add_argument("--lanes", type=int, default=1, help="the number of lanes")
    parser.add_argument("length", type=int, help="N, the length of the road")
    parser.add_argument("density", type=float, help="rho, the car density")
    parser.add_argument("vMax", type=int, help="v_{max}, the maximum velocity of car")
    parser.add_argument(
        "brakeProb", type=float, help="p, the probability of a car braking"
    )
    parser.add_argument("steps", type=int, help="T, the number of timesteps")
    parser.add_argument(
        "--seed", type=int, default=0, help="seed for rng; 0 draws a random seed"
    )
    parser.add_argument("--plot", action=argparse.BooleanOptionalAction)
    args = parser.parse_args()

    try:
        params = SimParams(
            lanes=args.lanes,
            length=args.length,
            density=args.density,
            vMax=args.vMax,
            brakeProb=args.brakeProb,
            seed=args.seed,
        )
        params.validate()
    except (OverflowError, TypeError, ValueError) as err:
        parser.error(str(err))

    sim = TrafficSim(params)
    sim.setup()

    if args.plot:
        history = np.empty((args.steps, args.lanes, args.length))

    for i in range(args.steps):
        sim.step()
        sim.print()

        if args.plot:
            history[i] = sim.road()

    print("Final timestep")
    print(f"Average Velocity (total): {sim.avgVelocity():g}")
    print(f"Flow Rate (total): {sim.flowrate():g}")
    print(f"Lane Changes: {sim.laneChangeFrequency():g}")
    print()
    for lane in range(sim.params_.lanes):
        print(f"Lane {lane}")
        print(f"  Average Velocity: {sim.avgVelocity(lane):g}")
        print(f"  Lane Usage: {sim.laneUsage(lane):g}")
        print(f"  Flow Rate: {sim.flowrate(lane):g}")

    if args.plot:
        plot_road(history, args.vMax)
        plt.show()
