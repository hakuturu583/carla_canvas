"""Middlewares for carla_canvas's ROS 2 output.

* ``cyclonedds``: plain DDS (the ``cyclonedds`` package, ``pip install
  carla-canvas[ros]``) with ROS 2's wire types and topic names ("rt/..."):
  ROS 2 nodes on any RMW that speaks standard DDS (Fast DDS, Cyclone DDS)
  receive the messages, and no ROS installation is needed.
* ``rclpy``: a ROS 2 node, when the program runs in a sourced ROS 2
  environment.
* ``auto``: rclpy if it imports, else cyclonedds, else none.
* ``memory``: records messages (tests).
"""

from __future__ import annotations

import dataclasses
import logging
import os
import threading
from abc import ABC, abstractmethod
from typing import Any

from . import messages as m

log = logging.getLogger(__name__)


class TransportError(RuntimeError):
    pass


class Publisher(ABC):
    @abstractmethod
    def publish(self, message: Any) -> None: ...


class Transport(ABC):
    name = "transport"

    @abstractmethod
    def publisher(self, topic: str, msg_type: type) -> Publisher:
        """`topic` is the ROS 2 name ("/carla/hero/rgb/image"), `msg_type` a
        class of carla_canvas.ros.messages."""

    def close(self) -> None:
        pass


# --- memory ---------------------------------------------------------------------------


class MemoryTransport(Transport):
    name = "memory"

    def __init__(self) -> None:
        self.messages: dict[str, list[Any]] = {}
        self._lock = threading.Lock()

    def publisher(self, topic: str, msg_type: type) -> Publisher:
        transport = self

        class _Pub(Publisher):
            def publish(self, message: Any) -> None:
                with transport._lock:
                    transport.messages.setdefault(topic, []).append(message)

        return _Pub()


# --- CycloneDDS --------------------------------------------------------------------------


def _dds_types():
    """IDL types with ROS 2's DDS type names (carla_canvas.ros.dds_types: one
    class per message class, of the same name)."""
    from . import dds_types

    return {cls: getattr(dds_types, cls.__name__) for cls in m.ROS_TYPES}


def _convert(value: Any, types: dict) -> Any:
    """A carla_canvas.ros.messages value as the matching `types` value."""
    if dataclasses.is_dataclass(value):
        target = types[type(value)]
        return target(**{f.name: _convert(getattr(value, f.name), types)
                         for f in dataclasses.fields(value)})
    if isinstance(value, list):
        return [_convert(v, types) for v in value]
    return value


def dds_topic_name(topic: str) -> str:
    """ROS 2's DDS topic name: "rt" + the ROS name."""
    return "rt" + (topic if topic.startswith("/") else "/" + topic)


class CycloneDdsTransport(Transport):
    name = "cyclonedds"

    def __init__(self, domain_id: int | None = None) -> None:
        try:
            from cyclonedds.core import Qos, Policy
            from cyclonedds.domain import DomainParticipant
            from cyclonedds.util import duration
        except ImportError as e:
            raise TransportError("the cyclonedds package is not installed "
                                 "(pip install 'carla-canvas[ros]')") from e
        if domain_id is None:
            domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0") or 0)
        self.domain_id = domain_id
        self._types = _dds_types()
        self._participant = DomainParticipant(domain_id)
        # ROS 2's default QoS: reliable, volatile, keep last 10.
        self._qos = Qos(Policy.Reliability.Reliable(duration(seconds=1)),
                        Policy.Durability.Volatile, Policy.History.KeepLast(10))
        self._topics: dict[str, Any] = {}

    def _topic(self, topic: str, msg_type: type):
        from cyclonedds.topic import Topic

        name = dds_topic_name(topic)
        t = self._topics.get(name)
        if t is None:
            t = self._topics[name] = Topic(self._participant, name, self._types[msg_type],
                                           qos=self._qos)
        return t

    def publisher(self, topic: str, msg_type: type) -> Publisher:
        from cyclonedds.pub import DataWriter

        writer = DataWriter(self._participant, self._topic(topic, msg_type), qos=self._qos)
        types = self._types

        class _Pub(Publisher):
            def publish(self, message: Any) -> None:
                writer.write(_convert(message, types))

        return _Pub()

    def subscribe(self, topic: str, msg_type: type):
        """A DataReader of the topic (for tests and tools)."""
        from cyclonedds.sub import DataReader

        return DataReader(self._participant, self._topic(topic, msg_type), qos=self._qos)


# --- rclpy -------------------------------------------------------------------------------


class RclpyTransport(Transport):
    name = "rclpy"

    def __init__(self, domain_id: int | None = None, node_name: str = "carla_canvas") -> None:
        try:
            import rclpy
        except ImportError as e:
            raise TransportError("rclpy is not available (source a ROS 2 environment)") from e
        import importlib

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(domain_id=domain_id) if domain_id is not None else rclpy.init()
        self._node = rclpy.create_node(node_name)
        self._classes = {}
        for cls, ros_type in m.ROS_TYPES.items():
            pkg, _, name = ros_type.split("/")
            self._classes[cls] = getattr(importlib.import_module(f"{pkg}.msg"), name)

    def _to_ros(self, value: Any) -> Any:
        if dataclasses.is_dataclass(value):
            out = self._classes[type(value)]()
            for f in dataclasses.fields(value):
                setattr(out, f.name, self._to_ros(getattr(value, f.name)))
            return out
        if isinstance(value, list):
            return [self._to_ros(v) for v in value]
        return value

    def publisher(self, topic: str, msg_type: type) -> Publisher:
        pub = self._node.create_publisher(self._classes[msg_type], topic, 10)
        to_ros = self._to_ros

        class _Pub(Publisher):
            def publish(self, message: Any) -> None:
                pub.publish(to_ros(message))

        return _Pub()

    def close(self) -> None:
        self._node.destroy_node()


def make_transport(backend: str = "auto", domain_id: int | None = None) -> Transport | None:
    """The configured transport; None for "none"."""
    if backend == "none":
        return None
    if backend == "memory":
        return MemoryTransport()
    if backend == "rclpy":
        return RclpyTransport(domain_id)
    if backend == "cyclonedds":
        return CycloneDdsTransport(domain_id)
    if backend == "auto":
        # rclpy in a sourced ROS 2 environment, else CycloneDDS, else none
        # (enable_for_ros() then fails with this message)
        try:
            return RclpyTransport(domain_id)
        except TransportError:
            pass
        try:
            return CycloneDdsTransport(domain_id)
        except TransportError as e:
            log.info("no ROS 2 transport: %s", e)
            return None
    raise TransportError(f"unknown ROS backend {backend!r}")
