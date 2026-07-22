"""Runtime IR — the single, system-facing representation of constraints.

The IR is built once at ingest (cold-start path) and is what the Prefix Compiler
and Safety Shield consume; they never re-parse the DSL. It carries:

* a canonical dict + ``policy_hash`` (the reproducibility key),
* shapely polygons in a local ENU metre plane plus an STRtree spatial index,
* per-polygon signed-distance / nearest-boundary queries (for lateral projection),
* altitude-envelope bounds.

This satisfies the "single IR contract" invariant from overview.md.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from shapely import STRtree
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points
from vlaguard_common import policy_hash

from policy_dsl.models import AltitudeEnvelope, PolicyDoc, PolygonFence
from policy_dsl.projection import LocalProjection


@dataclass(frozen=True)
class PolygonRecord:
    id: str
    constraint_type: str
    priority: str
    violation_action: str
    altitude_floor_m: float
    altitude_ceiling_m: float
    polygon: Polygon  # in local ENU metres


@dataclass(frozen=True)
class EnvelopeRecord:
    id: str
    constraint_type: str
    priority: str
    violation_action: str
    altitude_min_m: float
    altitude_max_m: float


@dataclass
class PolicyIR:
    policy_id: str
    version: str
    generation: int
    policy_hash: str
    canonical: dict[str, Any]
    projection: LocalProjection
    polygons: list[PolygonRecord]
    envelopes: list[EnvelopeRecord]
    _tree: STRtree = field(init=False)

    def __post_init__(self) -> None:
        self._tree = STRtree([r.polygon for r in self.polygons])

    # --- geometry queries used by the Safety Shield checker / repair ---------

    def polygons_near(self, lat: float, lon: float, radius_m: float) -> list[PolygonRecord]:
        """Spatial-index lookup: polygons whose bbox is within ``radius_m`` of the point."""
        if not self.polygons:
            return []
        x, y = self.projection.to_xy(lat, lon)
        probe = Point(x, y).buffer(radius_m)
        return [self.polygons[i] for i in self._tree.query(probe)]

    def signed_distance(self, rec: PolygonRecord, lat: float, lon: float) -> float:
        """Distance (m) from point to polygon boundary; negative if inside."""
        x, y = self.projection.to_xy(lat, lon)
        p = Point(x, y)
        d = float(p.distance(rec.polygon.exterior))
        return -d if rec.polygon.contains(p) else d

    def nearest_exterior_latlon(
        self, rec: PolygonRecord, lat: float, lon: float, margin_m: float = 0.0
    ) -> tuple[float, float]:
        """Nearest point just *outside* the polygon boundary, in lat/lon.

        Used by ``LateralProjection`` to pull a violating predicted point back to
        a feasible point. ``margin_m`` nudges it outside the boundary by a buffer.
        """
        x, y = self.projection.to_xy(lat, lon)
        p = Point(x, y)
        boundary_pt = nearest_points(rec.polygon.exterior, p)[0]
        bx, by = boundary_pt.x, boundary_pt.y
        if margin_m > 0:
            ox, oy = self.outward_normal_enu(rec, lat, lon)
            bx += ox * margin_m
            by += oy * margin_m
        return self.projection.to_latlon(bx, by)

    def outward_normal_enu(
        self, rec: PolygonRecord, lat: float, lon: float
    ) -> tuple[float, float]:
        """Unit outward normal ``(east, north)`` toward the nearest way out of the polygon.

        Points from the query point toward the nearest boundary point — i.e. the
        direction to fly to *leave* the polygon. Degenerate (query point on the
        boundary, norm ~ 0) falls back to the direction away from the centroid.

        This is the geometric primitive the trend-aware checker and the
        ``GeofenceEscape`` repair operator share; keeping it on the IR preserves
        the "one geometry code path" invariant.
        """
        x, y = self.projection.to_xy(lat, lon)
        p = Point(x, y)
        boundary_pt = nearest_points(rec.polygon.exterior, p)[0]
        ox, oy = boundary_pt.x - x, boundary_pt.y - y
        norm = (ox * ox + oy * oy) ** 0.5
        if norm < 1e-9:
            c = rec.polygon.centroid
            ox, oy = x - c.x, y - c.y
            norm = (ox * ox + oy * oy) ** 0.5 or 1.0
        return (ox / norm, oy / norm)

    def outward_from_centre_enu(
        self, rec: PolygonRecord, lat: float, lon: float
    ) -> tuple[float, float]:
        """Unit ``(east, north)`` from the polygon centroid toward the query point.

        Unambiguous "which way is out" for a vehicle already inside: the centroid
        is a single fixed point, so the direction is well-defined even when the
        vehicle sits exactly at the centre (where several equidistant boundary
        walls compete and ``outward_normal_enu`` can flip between them). Used by
        the trend-aware checker to decide whether an inside vehicle is escaping.
        """
        x, y = self.projection.to_xy(lat, lon)
        c = rec.polygon.centroid
        ox, oy = x - c.x, y - c.y
        norm = (ox * ox + oy * oy) ** 0.5
        if norm < 1e-9:
            return (1.0, 0.0)  # exactly on centroid: pick an arbitrary out-axis
        return (ox / norm, oy / norm)


def build_ir(doc: PolicyDoc) -> PolicyIR:
    """Cold-start ingest: authored doc -> indexed, hashed runtime IR."""
    canonical = doc.model_dump(mode="json")
    phash = policy_hash(canonical)

    polygon_models = [c for c in doc.constraints if isinstance(c, PolygonFence)]
    origin = _choose_origin(polygon_models)
    proj = LocalProjection(*origin)

    polygons: list[PolygonRecord] = []
    for c in polygon_models:
        ring = [proj.to_xy(v.lat, v.lon) for v in c.geometry.vertices]
        polygons.append(
            PolygonRecord(
                id=c.id,
                constraint_type=c.constraint_type,
                priority=c.priority,
                violation_action=c.violation_action,
                altitude_floor_m=c.geometry.altitude_floor_m,
                altitude_ceiling_m=c.geometry.altitude_ceiling_m,
                polygon=Polygon(ring),
            )
        )

    envelopes = [
        EnvelopeRecord(
            id=c.id,
            constraint_type=c.constraint_type,
            priority=c.priority,
            violation_action=c.violation_action,
            altitude_min_m=c.altitude_min_m,
            altitude_max_m=c.altitude_max_m,
        )
        for c in doc.constraints
        if isinstance(c, AltitudeEnvelope)
    ]

    return PolicyIR(
        policy_id=doc.policy_id,
        version=doc.version,
        generation=doc.generation,
        policy_hash=phash,
        canonical=canonical,
        projection=proj,
        polygons=polygons,
        envelopes=envelopes,
    )


def _choose_origin(polygons: list[PolygonFence]) -> tuple[float, float]:
    """Origin for the local plane: centroid of the first polygon, else (0, 0)."""
    if not polygons:
        return (0.0, 0.0)
    verts = polygons[0].geometry.vertices
    lat = sum(v.lat for v in verts) / len(verts)
    lon = sum(v.lon for v in verts) / len(verts)
    return (lat, lon)
