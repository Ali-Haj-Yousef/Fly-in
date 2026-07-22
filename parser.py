"""
parser.py
=========

Parser for the "Fly-in" drone network input file format.

This module reads a map description file and converts it into a fully
object-oriented in-memory representation: :class:`Zone` objects connected
by :class:`Connection` objects, all owned by a :class:`Graph`.

File format summary
--------------------
nb_drones: <positive_integer>
start_hub: <name> <x> <y> [metadata]
end_hub: <name> <x> <y> [metadata]
hub: <name> <x> <y> [metadata]
connection: <name1>-<name2> [metadata]

Metadata is optional, enclosed in brackets, space-separated key=value pairs.
Lines starting with '#' are comments and are ignored.

Author: (your name / login here)
"""

from __future__ import annotations

import re
from typing import Dict, Optional
from connection import Connection
from graph import Graph
from zone import Zone, ZoneType, HubRole


class ParserError(Exception):
    """Raised whenever the input file does not respect the expected format.

    Attributes:
        line_number: The 1-indexed line number where the error occurred.
        message: A human-readable description of the error.
    """

    def __init__(self, line_number: int, message: str) -> None:
        """Initializes the parser error with line context and message.

        Args:
            line_number: The 1-indexed line number where the error occurred.
            message: A human-readable description of the error.
        """
        self.line_number = line_number
        self.message = message
        super().__init__(f"Line {line_number}: {message}")


class MapParser:
    """Parses a Fly-in map file into a :class:`Graph` of zones and connections.

    Typical usage:
        >>> parser = MapParser()
        >>> graph = parser.parse_file("map_easy_1.txt")
    """

    # Regex building blocks, compiled once at class-definition time.
    _ZONE_NAME_RE = re.compile(r"^[^\s\-]+$")
    _METADATA_BLOCK_RE = re.compile(r"\[(.*)\]")
    _HUB_LINE_RE = re.compile(
        r"^(?P<prefix>start_hub|end_hub|hub):\s*"
        r"(?P<name>[^\s]+)\s+(?P<x>-?\d+)\s+(?P<y>-?\d+)\s*"
        r"(?P<meta>\[.*\])?\s*$"
    )
    _CONNECTION_LINE_RE = re.compile(
        r"^connection:\s*"
        r"(?P<name1>[^\s\-]+)-(?P<name2>[^\s\-]+)\s*(?P<meta>\[.*\])?\s*$"
    )
    _NB_DRONES_RE = re.compile(r"^nb_drones:\s*(?P<value>\d+)\s*$")

    _VALID_ZONE_TYPES = {z.value for z in ZoneType}

    def __init__(self) -> None:
        """Initializes parser state for a new map file parse."""
        self._graph = Graph()
        self._nb_drones_set = False
        self._start_seen = False
        self._end_seen = False

    def parse_file(self, file_name: str) -> Graph:
        """Parses a map file from disk and returns the graph.

        Args:
            file_name: Name of the map file to open.

        Returns:
            Graph: The fully constructed network of zones and connections.

        Raises:
            ParserError: If the file content violates the expected format.
            OSError: If the file cannot be opened or read.
        """
        with open(file_name) as f:
            lines = f.readlines()

        self._graph = Graph()
        self._nb_drones_set = False
        self._start_seen = False
        self._end_seen = False

        # First pass: zones and the drone count must be known before
        # connections are validated (connections may only reference
        # already-defined zones, as required by the parser constraints).
        for line_number, raw_line in enumerate(lines, start=1):
            line = self._strip_comment(raw_line).strip()
            if not line:
                continue
            if line.startswith("nb_drones:"):
                self._parse_nb_drones(line, line_number)
            elif line.startswith(("start_hub:", "end_hub:", "hub:")):
                self._parse_hub_line(line, line_number)
            elif line.startswith("connection:"):
                self._parse_connection_line(line, line_number)
            else:
                raise ParserError(
                    line_number, f"Unrecognized line: {raw_line!r}")

        self._validate_post_conditions(len(lines))
        return self._graph

    @staticmethod
    def _strip_comment(raw_line: str) -> str:
        """Removes a trailing '#' comment from a line, if present.

        Args:
            raw_line: The raw line as read from the file.

        Returns:
            str: The line with any comment portion removed.
        """
        hash_index = raw_line.find("#")
        if hash_index == -1:
            return raw_line
        return raw_line[:hash_index]

    def _parse_nb_drones(self, line: str, line_number: int) -> None:
        """Parses the ``nb_drones: <number>`` directive.

        Args:
            line: The cleaned line content.
            line_number: 1-indexed line number, for error reporting.

        Raises:
            ParserError: If the directive is malformed, duplicated, or
                not a positive integer.
        """
        if self._nb_drones_set:
            raise ParserError(line_number, "Duplicate nb_drones directive")

        match = self._NB_DRONES_RE.match(line)
        if not match:
            raise ParserError(
                line_number, f"Malformed nb_drones directive: {line!r}")

        value = int(match.group("value"))
        if value <= 0:
            raise ParserError(
                line_number, "nb_drones must be a positive integer")

        self._graph.nb_drones = value
        self._nb_drones_set = True

    def _parse_metadata(
            self, meta_block: Optional[str], line_number: int
    ) -> Dict[str, str]:
        """Parses a bracketed metadata block into a key/value dictionary.

        Args:
            meta_block: The raw ``[...]`` string, or None if absent.
            line_number: 1-indexed line number, for error reporting.

        Returns:
            Dict[str, str]: Parsed key/value metadata pairs (values as raw
            strings; type-specific conversion/validation happens by the
            caller).

        Raises:
            ParserError: If a token inside the brackets is not a valid
                ``key=value`` pair.
        """
        if not meta_block:
            return {}

        match = self._METADATA_BLOCK_RE.search(meta_block)
        if not match:
            raise ParserError(
                line_number, f"Malformed metadata block: {meta_block!r}")

        content = match.group(1).strip()
        if not content:
            return {}

        metadata: Dict[str, str] = {}
        for token in content.split():
            if "=" not in token:
                raise ParserError(
                    line_number,
                    f"Invalid metadata token (expected key=value): {token!r}"
                )
            key, _, value = token.partition("=")
            key, value = key.strip(), value.strip()
            if not key or not value:
                raise ParserError(
                    line_number, f"Invalid metadata token: {token!r}")
            if key in metadata:
                raise ParserError(
                    line_number, f"Duplicate metadata key: {key!r}")
            metadata[key] = value
        return metadata

    def _parse_hub_line(self, line: str, line_number: int) -> None:
        """Parses a ``start_hub:``, ``end_hub:`` or ``hub:`` definition line.

        Args:
            line: The cleaned line content.
            line_number: 1-indexed line number, for error reporting.

        Raises:
            ParserError: If the line is malformed, the zone name is
                duplicated/invalid, the coordinates are invalid, the zone
                type is unknown, metadata values are invalid, or a second
                start/end hub is declared.
        """
        match = self._HUB_LINE_RE.match(line)
        if not match:
            raise ParserError(
                line_number, f"Malformed hub definition: {line!r}")

        prefix = match.group("prefix")
        name = match.group("name")
        x_str, y_str = match.group("x"), match.group("y")
        meta_block = match.group("meta")

        if not self._ZONE_NAME_RE.match(name):
            raise ParserError(
                line_number,
                f"Zone name {name!r} contains forbidden characters"
            )
        if name in self._graph.zones:
            raise ParserError(line_number, f"Duplicate zone name: {name!r}")

        try:
            x, y = int(x_str), int(y_str)
        except ValueError as exc:
            raise ParserError(
                line_number, f"Invalid integer coordinates: {exc}") from exc

        metadata = self._parse_metadata(meta_block, line_number)

        zone_type = self._extract_zone_type(metadata, line_number)
        color = metadata.pop("color", None)
        max_drones = self._extract_max_drones(metadata, line_number)

        if metadata:
            unknown_keys = ", ".join(metadata.keys())
            raise ParserError(
                line_number, f"Unknown metadata key(s): {unknown_keys}")

        role = self._resolve_role(prefix, line_number)

        zone = Zone(
            name=name,
            x=x,
            y=y,
            zone_type=zone_type,
            color=color,
            max_drones=max_drones,
            role=role,
        )
        self._graph.add_zone(zone)

    def _resolve_role(self, prefix: str, line_number: int) -> HubRole:
        """Validates and resolves the hub role, enforcing start/end uniqueness.

        Args:
            prefix: The raw line prefix
            (``start_hub``, ``end_hub`` or ``hub``).
            line_number: 1-indexed line number, for error reporting.

        Raises:
            ParserError: If a second ``start_hub`` or ``end_hub`` is declared.
        """
        if prefix == "start_hub":
            if self._start_seen:
                raise ParserError(
                    line_number, "Multiple start_hub zones defined")
            self._start_seen = True
            return HubRole.START
        if prefix == "end_hub":
            if self._end_seen:
                raise ParserError(
                    line_number, "Multiple end_hub zones defined")
            self._end_seen = True
            return HubRole.END
        return HubRole.REGULAR

    def _extract_zone_type(
            self, metadata: Dict[str, str], line_number: int) -> ZoneType:
        """Pops and validates the ``zone`` metadata key.

        Args:
            metadata: Mutable metadata dictionary
            (the key is removed if present).
            line_number: 1-indexed line number, for error reporting.

        Raises:
            ParserError: If the zone type is not one of the allowed values.
        """
        raw_type = metadata.pop("zone", ZoneType.NORMAL.value)
        if raw_type not in self._VALID_ZONE_TYPES:
            raise ParserError(line_number, f"Invalid zone type: {raw_type!r}")
        return ZoneType(raw_type)  # type: ignore[call-arg]
        # Enum value lookup, not __new__

    def _extract_max_drones(
            self, metadata: Dict[str, str], line_number: int) -> int:
        """Pops and validates the ``max_drones`` metadata key.

        Args:
            metadata: Mutable metadata dictionary
            (the key is removed if present).
            line_number: 1-indexed line number, for error reporting.

        Raises:
            ParserError: If the value is not a positive integer.
        """
        raw_value = metadata.pop("max_drones", "1")
        if not raw_value.isdigit() or int(raw_value) <= 0:
            raise ParserError(
                line_number,
                f"max_drones must be a positive integer, got {raw_value!r}"
            )
        return int(raw_value)

    def _extract_max_link_capacity(
        self, metadata: Dict[str, str], line_number: int
    ) -> int:
        """Pops and validates the ``max_link_capacity`` metadata key.

        Args:
            metadata: Mutable metadata dictionary
            (the key is removed if present).
            line_number: 1-indexed line number, for error reporting.

        Raises:
            ParserError: If the value is not a positive integer.
        """
        raw_value = metadata.pop("max_link_capacity", "1")
        if not raw_value.isdigit() or int(raw_value) <= 0:
            raise ParserError(
                line_number,
                (
                    "max_link_capacity must be a positive integer, "
                    f"got {raw_value!r}"
                )
            )
        return int(raw_value)

    def _parse_connection_line(self, line: str, line_number: int) -> None:
        """Parses a ``connection: <name1>-<name2> [metadata]`` line.

        Args:
            line: The cleaned line content.
            line_number: 1-indexed line number, for error reporting.

        Raises:
            ParserError: If the line is malformed, references an undefined
                zone, is a self-loop, is a duplicate of an existing
                connection, or has invalid metadata.
        """
        match = self._CONNECTION_LINE_RE.match(line)
        if not match:
            raise ParserError(
                line_number, f"Malformed connection definition: {line!r}")

        name1, name2 = match.group("name1"), match.group("name2")
        meta_block = match.group("meta")

        if name1 not in self._graph.zones:
            raise ParserError(
                line_number,
                f"Connection references undefined zone: {name1!r}")
        if name2 not in self._graph.zones:
            raise ParserError(
                line_number,
                f"Connection references undefined zone: {name2!r}")
        if name1 == name2:
            raise ParserError(
                line_number, f"Self-loop connection is not allowed: {name1!r}")

        if self._connection_already_exists(name1, name2):
            raise ParserError(
                line_number,
                f"Duplicate connection between {name1!r} and {name2!r}"
            )

        metadata = self._parse_metadata(meta_block, line_number)
        max_link_capacity = self._extract_max_link_capacity(
            metadata, line_number)

        if metadata:
            unknown_keys = ", ".join(metadata.keys())
            raise ParserError(
                line_number, f"Unknown metadata key(s): {unknown_keys}")

        connection = Connection(
            zone_a=self._graph.zones[name1],
            zone_b=self._graph.zones[name2],
            max_link_capacity=max_link_capacity,
        )
        self._graph.add_connection(connection)

    def _connection_already_exists(self, name1: str, name2: str) -> bool:
        """Checks whether a connection between two zone names already exists.

        Both orderings (``a-b`` and ``b-a``) are treated as duplicates,
        since connections are bidirectional.

        Args:
            name1: Name of the first zone.
            name2: Name of the second zone.

        Returns:
            bool: True if an equivalent connection is already registered.
        """
        for connection in self._graph.connections:
            endpoints = {connection.zone_a.name, connection.zone_b.name}
            if endpoints == {name1, name2}:
                return True
        return False

    def _validate_post_conditions(self, total_lines: int) -> None:
        """Validates global file-level constraints after the parse is complete.

        Args:
            total_lines: Total number of lines in the source file, used as
                a fallback line number for file-level errors.

        Raises:
            ParserError: If ``nb_drones`` was never set, or if no/multiple
                start or end hubs exist.
        """
        if not self._nb_drones_set:
            raise ParserError(
                total_lines, "Missing mandatory nb_drones directive")
        if not self._start_seen:
            raise ParserError(total_lines, "Missing mandatory start_hub zone")
        if not self._end_seen:
            raise ParserError(total_lines, "Missing mandatory end_hub zone")
