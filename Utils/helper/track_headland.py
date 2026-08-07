"""
track_headland.py

Associates each track segment with the headland boundary segment it terminates
on, and provides headland-aware turn routing.

Terminology
-----------
A track   : [start_pt, end_pt]  where each point is [lat, lon]
h_gcpp    : list of [pt0, pt1]  – the headland boundary as ordered segments
            (output of GenerateHeadland.gen_headland).  The segments form a
            closed polygon when taken in order.

End "A"   : track[0]   – one terminus of the track
End "B"   : track[-1]  – the other terminus

For every track we find the nearest headland segment for each end.
When a turn is generated between two tracks:
  • Same headland segment at the turn-end  → normal flatturn arc is used.
  • Different headland segments            → the turn must be rerouted to
      follow the headland boundary between the two segments.
"""

from math import radians, degrees, sin, cos, asin, atan2, sqrt, pi
from .geodesy import Geodesy


# ---------------------------------------------------------------------------
# Data container
# ---------------------------------------------------------------------------

class TrackWithHeadland:
    """
    Wraps one track segment and records which headland segment each end
    belongs to.

    Attributes
    ----------
    track          : [[lat,lon],[lat,lon]]
    headland_end_a : [[lat,lon],[lat,lon]]  h_gcpp entry closest to track[0]
    headland_idx_a : int
    headland_end_b : [[lat,lon],[lat,lon]]  h_gcpp entry closest to track[-1]
    headland_idx_b : int
    """

    def __init__(self, track, headland_end_a, headland_idx_a,
                 headland_end_b, headland_idx_b):
        self.track          = track
        self.headland_end_a = headland_end_a
        self.headland_idx_a = headland_idx_a
        self.headland_end_b = headland_end_b
        self.headland_idx_b = headland_idx_b

    def endpoint_a(self):
        return self.track[0]

    def endpoint_b(self):
        return self.track[-1]

    def __repr__(self):
        return (f"TrackWithHeadland(a_idx={self.headland_idx_a}, "
                f"b_idx={self.headland_idx_b})")


# ---------------------------------------------------------------------------
# Builder
# ---------------------------------------------------------------------------

def assign_headlands(tracks, h_gcpp):
    """
    For every track build a TrackWithHeadland that records which headland
    segment each endpoint is closest to.

    Parameters
    ----------
    tracks  : list of [pt_start, pt_end]
    h_gcpp  : list of [pt0, pt1]  (headland boundary segments, ordered)

    Returns
    -------
    list[TrackWithHeadland]
    """
    result = []
    for track in tracks:
        idx_a, seg_a = _nearest_headland_segment(track[0],  h_gcpp)
        idx_b, seg_b = _nearest_headland_segment(track[-1], h_gcpp)
        result.append(TrackWithHeadland(track, seg_a, idx_a, seg_b, idx_b))
    return result


def _nearest_headland_segment(point, h_gcpp):
    """Return (index, segment) of the h_gcpp entry closest to *point*."""
    best_idx  = 0
    best_dist = float('inf')

    for idx, seg in enumerate(h_gcpp):
        try:
            d = abs(Geodesy.cross_track_distance(point, seg[0], seg[1]))
        except Exception:
            d = min(Geodesy.distancebet(point, seg[0]),
                    Geodesy.distancebet(point, seg[1]))
        if d < best_dist:
            best_dist = d
            best_idx  = idx

    return best_idx, h_gcpp[best_idx]


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _project_onto_segment(point, seg_pt0, seg_pt1):
    """
    Return the point on the geodesic seg_pt0→seg_pt1 that is closest to
    *point* (i.e. the foot of the perpendicular from *point* onto the segment).

    Uses cross-track / along-track distances on a spherical earth.
    Falls back to the nearer endpoint if the foot is outside the segment.
    """
    R = Geodesy.EARTH_RADIUS

    d13   = Geodesy.distancebet(seg_pt0, point) / R          # angular dist A→P
    d12   = Geodesy.distancebet(seg_pt0, seg_pt1) / R        # angular dist A→B
    bear12 = radians(Geodesy.angle(seg_pt0, seg_pt1)[0])
    bear13 = radians(Geodesy.angle(seg_pt0, point)[0])

    # Along-track distance  (positive = towards seg_pt1)
    cross = asin(sin(d13) * sin(bear13 - bear12))
    along = asin(sqrt(max(0.0, sin(d13)**2 - sin(cross)**2))
                 / cos(cross))

    # Check foot is within the segment
    if along < 0 or along > d12:
        # Clamp to nearer endpoint
        if Geodesy.distancebet(point, seg_pt0) <= Geodesy.distancebet(point, seg_pt1):
            return seg_pt0
        return seg_pt1

    # Compute the foot point
    lat0 = radians(seg_pt0[0])
    lon0 = radians(seg_pt0[1])
    lat_f = asin(sin(lat0) * cos(along) +
                 cos(lat0) * sin(along) * cos(bear12))
    lon_f = lon0 + atan2(sin(bear12) * sin(along) * cos(lat0),
                         cos(along) - sin(lat0) * sin(lat_f))
    return [degrees(lat_f), degrees(lon_f)]


def _headland_corners_between(h_gcpp, idx_from, idx_to):
    """
    Return the list of headland *corner* points that must be visited when
    travelling along the headland boundary from segment idx_from to idx_to.

    h_gcpp is a list of [pt0, pt1] segments where consecutive segments share
    a corner: h_gcpp[i][1] ≈ h_gcpp[i+1][0]  (they are the intersected
    polygon vertices from GenerateHeadland).

    Strategy: try both directions (CW and CCW around the closed polygon) and
    pick the shorter one.

    Returns
    -------
    list of [lat, lon]   – the corner points (NOT including the foot-points
                           on the start/end segments, those are added by the
                           caller)
    """
    n = len(h_gcpp)

    if idx_from == idx_to:
        return []   # Same segment – no corners needed

    # Build corner sequence in forward direction (idx_from → idx_to)
    corners_fwd = []
    i = idx_from
    while True:
        # The "exit corner" of segment i going forward is h_gcpp[i][1]
        corners_fwd.append(h_gcpp[i][1])
        i = (i + 1) % n
        if i == idx_to:
            break
        if len(corners_fwd) > n:   # safety – full loop
            break

    # Build corner sequence in backward direction (idx_from → idx_to going CCW)
    corners_bwd = []
    i = idx_from
    while True:
        # The "exit corner" going backward is h_gcpp[i][0]
        corners_bwd.append(h_gcpp[i][0])
        i = (i - 1) % n
        if i == idx_to:
            break
        if len(corners_bwd) > n:
            break

    # Pick the shorter route (fewer corner points = shorter path)
    if len(corners_fwd) <= len(corners_bwd):
        return corners_fwd
    else:
        return list(reversed(corners_bwd))


def _densify_segment(pt_a, pt_b, step_m=1.0):
    """
    Interpolate points along the great-circle from pt_a to pt_b at ~step_m
    metre intervals.  Returns a list that includes pt_a and pt_b.
    """
    dist = Geodesy.distancebet(pt_a, pt_b)
    if dist < step_m:
        return [pt_a, pt_b]

    n = max(2, int(dist / step_m))
    pts = []
    R = Geodesy.EARTH_RADIUS
    lat1, lon1 = radians(pt_a[0]), radians(pt_a[1])
    lat2, lon2 = radians(pt_b[0]), radians(pt_b[1])
    bear = radians(Geodesy.angle(pt_a, pt_b)[0])

    for k in range(n + 1):
        d = (dist * k / n) / R
        lat = asin(sin(lat1) * cos(d) + cos(lat1) * sin(d) * cos(bear))
        lon = lon1 + atan2(sin(bear) * sin(d) * cos(lat1),
                           cos(d) - sin(lat1) * sin(lat))
        pts.append([degrees(lat), degrees(lon)])

    return pts


# ---------------------------------------------------------------------------
# Public API – headland-aware turn routing
# ---------------------------------------------------------------------------

def headland_turn(track_end_pt, track_start_pt,
                  twh_end, twh_start,
                  h_gcpp,
                  step_m=1.0):
    """
    Build a turn path from *track_end_pt* to *track_start_pt* that stays on
    the headland boundary.

    The algorithm:
      1. Project track_end_pt  onto its nearest headland segment  → foot_end
      2. Project track_start_pt onto its nearest headland segment → foot_start
      3. Collect the headland corner points between the two segments (shortest
         path around the headland polygon).
      4. Densify each leg so the path follows the headland line.

    Parameters
    ----------
    track_end_pt    : [lat, lon]  – endpoint of the current track (where the
                                    tractor leaves the track)
    track_start_pt  : [lat, lon]  – start point of the next track (where it
                                    must arrive)
    twh_end         : TrackWithHeadland of the current track
    twh_start       : TrackWithHeadland of the next track
    h_gcpp          : full headland segment list
    step_m          : interpolation step in metres along headland edges

    Returns
    -------
    list of [lat, lon]   – waypoints of the headland turn, from track_end_pt
                           to track_start_pt (inclusive at both ends)
    """

    # Which headland segments are involved at the turn end?
    # "turn end" = end B for even-index tracks, end A for odd-index tracks
    # (the caller passes the right endpoint already as track_end_pt / track_start_pt)
    # We use the headland index that corresponds to the track endpoint passed.

    # Determine headland segment index for each endpoint
    idx_end,   seg_end   = _nearest_headland_segment(track_end_pt,   h_gcpp)
    idx_start, seg_start = _nearest_headland_segment(track_start_pt, h_gcpp)

    # Project endpoints onto their headland segments
    foot_end   = _project_onto_segment(track_end_pt,   seg_end[0],   seg_end[1])
    foot_start = _project_onto_segment(track_start_pt, seg_start[0], seg_start[1])

    # Collect headland corner points between the two segments
    corners = _headland_corners_between(h_gcpp, idx_end, idx_start)

    # Build the full waypoint sequence
    waypoints = []

    # 1. Start at the track endpoint
    waypoints.append(track_end_pt)

    # 2. Move to the foot on the headland
    if Geodesy.distancebet(track_end_pt, foot_end) > 0.5:
        waypoints += _densify_segment(track_end_pt, foot_end, step_m)[1:]

    # 3. Walk along headland corners
    prev = foot_end
    for corner in corners:
        if Geodesy.distancebet(prev, corner) > 0.5:
            waypoints += _densify_segment(prev, corner, step_m)[1:]
        prev = corner

    # 4. Walk from last corner (or foot_end) to foot_start
    if Geodesy.distancebet(prev, foot_start) > 0.5:
        waypoints += _densify_segment(prev, foot_start, step_m)[1:]

    # 5. Move from headland foot to the next track start
    if Geodesy.distancebet(foot_start, track_start_pt) > 0.5:
        waypoints += _densify_segment(foot_start, track_start_pt, step_m)[1:]

    return waypoints


def same_headland_end(twh_a, twh_b, end='b'):
    """
    Return True when two tracks share the same headland segment at the
    specified end ('a' or 'b').
    """
    if end == 'b':
        return twh_a.headland_idx_b == twh_b.headland_idx_b
    return twh_a.headland_idx_a == twh_b.headland_idx_a
