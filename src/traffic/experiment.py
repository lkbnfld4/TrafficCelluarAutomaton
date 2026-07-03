from dataclasses import dataclass

from traffic.core import TrafficSim


@dataclass(frozen=True, slots=True)
class LaneResult:
    avg_velocity: float
    usage: float
    flow_rate: float


@dataclass(frozen=True, slots=True)
class ExperimentResult:
    avg_velocity: float
    flow_rate: float
    lanes: tuple[LaneResult]
    lane_change_frequency: float


def run_experiment(params, steps, warmup):
    if not 0 <= warmup < steps:
        raise ValueError("warmup must satisfy 0 <= warmup < steps")

    sim = TrafficSim(params)
    sim.setup()
    for _ in range(warmup):
        sim.step()

    count = steps - warmup
    lanes = range(sim.params_.lanes)
    velocity = flow = lane_changes = 0.0
    lane_velocity = [0.0] * len(lanes)
    lane_usage = [0.0] * len(lanes)
    lane_flow = [0.0] * len(lanes)
    for _ in range(count):
        sim.step()
        velocity += sim.avgVelocity()
        flow += sim.flowrate()
        lane_changes += sim.laneChangeFrequency()
        for lane in lanes:
            lane_velocity[lane] += sim.avgVelocity(lane)
            lane_usage[lane] += sim.laneUsage(lane)
            lane_flow[lane] += sim.flowrate(lane)

    return ExperimentResult(
        avg_velocity=velocity / count,
        flow_rate=flow / count,
        lanes=tuple(
            LaneResult(
                avg_velocity=lane_velocity[lane] / count,
                usage=lane_usage[lane] / count,
                flow_rate=lane_flow[lane] / count,
            )
            for lane in lanes
        ),
        lane_change_frequency=lane_changes / count,
    )
