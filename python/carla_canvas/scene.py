"""The world as the renderer sees it: CARLA's actors and their transforms,
kept up to date from the client's Render calls."""

from __future__ import annotations

import fnmatch
from dataclasses import dataclass, field

import numpy as np

from .geometry import Transform

# carla.CityObjectLabel (CARLA UE5).
class CityObjectLabel:
    NONE = 0
    Roads = 1
    Sidewalks = 2
    Buildings = 3
    Walls = 4
    Fences = 5
    Poles = 6
    TrafficLight = 7
    TrafficSigns = 8
    Vegetation = 9
    Terrain = 10
    Sky = 11
    Pedestrians = 12
    Rider = 13
    Car = 14
    Truck = 15
    Bus = 16
    Train = 17
    Motorcycle = 18
    Bicycle = 19
    Static = 20
    Dynamic = 21
    Other = 22
    Water = 23
    RoadLines = 24
    Ground = 25
    Bridge = 26
    RailTrack = 27
    GuardRail = 28


_VEHICLE_BASE_TYPES = {
    "car": CityObjectLabel.Car,
    "van": CityObjectLabel.Car,
    "truck": CityObjectLabel.Truck,
    "bus": CityObjectLabel.Bus,
    "motorcycle": CityObjectLabel.Motorcycle,
    "bicycle": CityObjectLabel.Bicycle,
}


@dataclass
class BoundingBox:
    location: tuple[float, float, float] = (0.0, 0.0, 0.0)  # center, actor-local
    extent: tuple[float, float, float] = (0.0, 0.0, 0.0)  # half size
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class ActorInfo:
    """carla.Actor: what does not change over its life."""

    id: int
    type_id: str
    parent_id: int = 0
    attributes: dict[str, str] = field(default_factory=dict)
    bounding_box: BoundingBox = field(default_factory=BoundingBox)

    @staticmethod
    def from_proto(a) -> "ActorInfo":
        b = a.bounding_box
        return ActorInfo(
            id=a.id, type_id=a.type_id, parent_id=a.parent_id,
            attributes={x.id: x.value for x in a.attributes},
            bounding_box=BoundingBox((b.location.x, b.location.y, b.location.z),
                                     (b.extent.x, b.extent.y, b.extent.z),
                                     (b.rotation.pitch, b.rotation.yaw, b.rotation.roll)))

    def matches(self, pattern: str) -> bool:
        return fnmatch.fnmatchcase(self.type_id, pattern)

    @property
    def semantic_tag(self) -> int:
        """The CityObjectLabel a segmentation camera gives the actor."""
        t = self.type_id
        if t.startswith("vehicle."):
            return _VEHICLE_BASE_TYPES.get(self.attributes.get("base_type", "").lower(),
                                           CityObjectLabel.Car)
        if t.startswith("walker.") and not t.startswith("walker.controller"):
            return CityObjectLabel.Pedestrians
        if t == "traffic.traffic_light":
            return CityObjectLabel.TrafficLight
        if t.startswith("traffic."):
            return CityObjectLabel.TrafficSigns
        if t.startswith("static.prop"):
            return CityObjectLabel.Static
        return CityObjectLabel.NONE

    @property
    def is_visible_body(self) -> bool:
        """Whether the actor has a body cameras and LiDARs see (sensors,
        controllers and the spectator do not)."""
        t = self.type_id
        return not (t.startswith("sensor.") or t.startswith("controller.")
                    or t.startswith("walker.controller") or t == "spectator"
                    or t.startswith("traffic.unknown"))


@dataclass
class ActorState:
    transform: Transform = field(default_factory=Transform)
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)


class Scene:
    """Actors (info + latest snapshot) of the current episode."""

    def __init__(self) -> None:
        self.map_name = ""
        self.episode_id = 0
        self.frame = 0
        self.elapsed_seconds = 0.0
        self.delta_seconds = 0.0
        self.actors: dict[int, ActorInfo] = {}
        self.states: dict[int, ActorState] = {}

    def reset(self, map_name: str, episode_id: int) -> None:
        self.__init__()
        self.map_name = map_name
        self.episode_id = episode_id

    def add_actor(self, info: ActorInfo) -> None:
        self.actors[info.id] = info

    def apply(self, request) -> None:
        """A RenderRequest's snapshot and actor changes."""
        for a in request.new_actors:
            self.add_actor(ActorInfo.from_proto(a))
        for actor_id in request.removed_actors:
            self.actors.pop(actor_id, None)
            self.states.pop(actor_id, None)
        snap = request.snapshot
        self.frame = snap.timestamp.frame
        self.elapsed_seconds = snap.timestamp.elapsed_seconds
        self.delta_seconds = snap.timestamp.delta_seconds
        for s in snap.actors:
            self.states[s.id] = ActorState(Transform.from_proto(s.transform),
                                           (s.velocity.x, s.velocity.y, s.velocity.z))

    def transform(self, actor_id: int) -> Transform:
        state = self.states.get(actor_id)
        if state is None:
            raise KeyError(f"actor {actor_id} is not in the snapshot")
        return state.transform

    def parent_transform(self, actor_id: int) -> Transform | None:
        """The world transform of the actor's parent (a sensor's vehicle), or
        None when it has none or the parent is not in the snapshot."""
        info = self.actors.get(actor_id)
        state = self.states.get(info.parent_id) if info is not None and info.parent_id else None
        return state.transform if state is not None else None

    def bodies(self, exclude: set[int] = frozenset()) -> list[tuple[ActorInfo, Transform]]:
        """Visible actors with a non-empty bounding box and a known transform."""
        out = []
        for actor_id, info in self.actors.items():
            if actor_id in exclude or not info.is_visible_body:
                continue
            state = self.states.get(actor_id)
            if state is None or not any(info.bounding_box.extent):
                continue
            out.append((info, state.transform))
        return out

    def ancestors(self, actor_id: int) -> list[int]:
        """The actor's parent, grandparent, ... ids."""
        out = []
        info = self.actors.get(actor_id)
        while info is not None and info.parent_id and info.parent_id not in out:
            out.append(info.parent_id)
            info = self.actors.get(info.parent_id)
        return out

    def box_corners_world(self, info: ActorInfo, transform: Transform) -> np.ndarray:
        b = info.bounding_box
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
        local = Transform(b.location, b.rotation).transform_points(signs * np.asarray(b.extent))
        return transform.transform_points(local)
