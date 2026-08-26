from mavcore.mav_protocol import MAVProtocol
from mavcore.messages import CommandAck
from gnc.waypoint_laps.min_snap import MinSnap


class PursuitNavigationProtocol(MAVProtocol):
    def __init__(
        self,
        vehicle,
        geofence,
        waypoints,
        log_func=lambda msg: print(msg),
        v_max: float | None = None,
        a_max: float | None = None,
        hz: float = 2.0,
        straightness_weight: float = 0.0,
        acceleration_weight: float = 0.0,
        pursuit_lookahead_time: float = 1.5,
        pursuit_lookahead_min: float = 3.0,
        feedforward_time: float = 0.15,
        curvature_margin_reserve: float = 0.3,
        pursuit_bias_bound_m: float = 0.5,
    ):
        super().__init__()
        self.vehicle = vehicle
        self.geofence = geofence
        self.waypoints = waypoints
        self.log_func = log_func

        min_snap_kwargs = dict(
            hz=hz,
            straightness_weight=straightness_weight,
            acceleration_weight=acceleration_weight,
            pursuit_lookahead_time=pursuit_lookahead_time,
            pursuit_lookahead_min=pursuit_lookahead_min,
            feedforward_time=feedforward_time,
            curvature_margin_reserve=curvature_margin_reserve,
            pursuit_bias_bound_m=pursuit_bias_bound_m,
        )
        if v_max is not None:
            min_snap_kwargs["v_max"] = v_max
        if a_max is not None:
            min_snap_kwargs["a_max"] = a_max

        self.min_snap = MinSnap(
            self.vehicle,
            self.geofence,
            self.waypoints,
            **min_snap_kwargs,
        )

        self.ack_msg = CommandAck()
        self.last_lap_distance: float = 0.0

    def run(self, sender, receiver) -> float:
        self.last_lap_distance = self.min_snap.run_single_lap()
        return self.last_lap_distance
