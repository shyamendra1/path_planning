"""
Modern Headland Generation using Shapely Buffer

This module provides industry-standard polygon offsetting for agricultural
field headland generation. It handles:
- Closed/narrow points
- Bottleneck regions
- Acute angles
- Self-intersecting polygons

Used by QGIS, PostGIS, and professional agriculture software.

CUSTOMIZATION NOTES:
- To offset INWARD (inside polygon): use NEGATIVE offset_distance
- To offset OUTWARD (outside polygon): use POSITIVE offset_distance
"""

import math
from shapely.geometry import Polygon, MultiPolygon, LinearRing
from shapely.ops import unary_union
import numpy as np
from Utils.helper.geodesy import Geodesy


class ModernHeadlandGenerator:
    """
    Industry-standard headland generation using geometric algorithms.
    Handles closed points, bottlenecks, and acute angles robustly.
    
    Uses Shapely for offsetting, which is proven in CAD/GIS software worldwide.
    
    OFFSET DIRECTION:
    - Negative buffer = INWARD (inside the field boundary)
    - Positive buffer = OUTWARD (outside the field boundary)
    """
    
    MIN_OFFSET_DISTANCE = 0.1  # 10cm minimum safe offset
    MIN_VERTEX_SEPARATION = 0.5  # 50cm minimum between vertices
    MAX_OFFSET_SPIKE = 5.0  # Reject offsets > 5x the requested offset
    SIMPLIFICATION_TOLERANCE = 0.05  # 5cm simplification
    
    # ✅ KEY SETTING: Set to True for INWARD (inside), False for OUTWARD (outside)
    OFFSET_INWARD = True
    
    @staticmethod
    def buffer_polygon_shapely(polygon_coords, offset_distance, resolution=8):
        """
        Offset polygon using Shapely buffer.
        This is the industry standard - used by QGIS, PostGIS, Autodesk, etc.
        
        Parameters:
        - polygon_coords: list of [lat, lon] in GCS (geographic)
        - offset_distance: meters (converted to degrees approx)
        - resolution: quality of buffer (8=smooth, 16=smoother)
        
        Returns:
        - list of [lat, lon] vertices of offset polygon
        
        NOTE: Negative offset = inward, Positive offset = outward
        """
        
        if len(polygon_coords) < 3:
            return []
        
        # Convert GCS (lat/lon) to local coordinates for Shapely operations
        # Use UTM-like approximation: 1 degree latitude ≈ 111 km
        coords_xy = []
        for lat, lon in polygon_coords:
            x = lon * 111000  # approximate meters
            y = lat * 111000
            coords_xy.append((x, y))
        
        try:
            # Create Shapely polygon
            poly = Polygon(coords_xy)
            
            # Validate polygon
            if not poly.is_valid:
                poly = poly.buffer(0)  # Fix self-intersections
            
            # ✅ INVERT SIGN for INWARD offset
            actual_offset = offset_distance
            if ModernHeadlandGenerator.OFFSET_INWARD:
                actual_offset = -offset_distance  # Negative = inward
            
            # Buffer (positive = outward, negative = inward)
            buffered = poly.buffer(
                actual_offset,
                resolution=resolution,
                join_style=2,  # Mitre join (sharp corners)
                mitre_limit=2.0
            )
            
            # Handle MultiPolygon result (can happen with complex shapes)
            if isinstance(buffered, MultiPolygon):
                # Take the largest polygon
                buffered = max(buffered.geoms, key=lambda x: x.area)
            
            # Extract coordinates and convert back to lat/lon
            if buffered.is_empty or buffered.geom_type != 'Polygon':
                return []
            
            coords_latlng = []
            for x, y in buffered.exterior.coords[:-1]:  # Exclude closing point
                lat = y / 111000
                lon = x / 111000
                coords_latlng.append([lat, lon])
            
            return coords_latlng
            
        except Exception as e:
            print(f"⚠️  Shapely buffer failed: {e}")
            return []
    
    
    @staticmethod
    def offset_polygon_geodetic(polygon, track_heading, width, turning_radius):
        """
        Offset each edge independently, then resolve vertices.
        This is a FALLBACK when Shapely isn't available.
        More robust than basic offset approaches.
        
        ✅ For INWARD offset: negate the offset_distance direction
        """
        
        if len(polygon) < 3:
            return []
        
        n = len(polygon)
        offset_distance = width / 2 + turning_radius
        
        # ✅ INVERT for INWARD offset
        if ModernHeadlandGenerator.OFFSET_INWARD:
            offset_distance = -offset_distance
        
        # Step 1: Offset each edge perpendicular to track
        offset_edges = []
        
        for i in range(n):
            edge_start = polygon[i]
            edge_end = polygon[(i + 1) % n]
            
            edge_heading = Geodesy.angle(edge_start, edge_end)[0]
            normal_heading = (edge_heading + 90) % 360
            
            # Offset edge endpoints
            os_start = Geodesy.points(edge_start, offset_distance, normal_heading)
            os_end = Geodesy.points(edge_end, offset_distance, normal_heading)
            
            offset_edges.append({
                'start': os_start,
                'end': os_end,
                'heading': edge_heading,
                'length': Geodesy.distancebet(os_start, os_end)
            })
        
        # Step 2: Resolve vertices with robust corner handling
        headland = []
        
        for i in range(n):
            prev_edge = offset_edges[(i - 1) % n]
            curr_edge = offset_edges[i]
            
            # Get the corner angle
            angle = ModernHeadlandGenerator.get_interior_angle(
                polygon[(i - 1) % n],
                polygon[i],
                polygon[(i + 1) % n]
            )
            
            # Try intersection for convex corners
            intersection_pt = ModernHeadlandGenerator.compute_corner(
                prev_edge['start'],
                prev_edge['end'],
                prev_edge['heading'],
                curr_edge['start'],
                curr_edge['end'],
                curr_edge['heading'],
                angle,
                abs(offset_distance)  # Use absolute value for distance checks
            )
            
            if intersection_pt:
                headland.append(intersection_pt)
            else:
                # Fallback: use edge endpoints for acute/tight corners
                headland.append(prev_edge['end'])
                headland.append(curr_edge['start'])
        
        # Step 3: Clean duplicates and short segments
        headland = ModernHeadlandGenerator.clean_path(headland)
        
        return headland
    
    
    @staticmethod
    def get_interior_angle(prev_pt, curr_pt, next_pt):
        """Calculate interior angle at vertex (0-180 degrees)."""
        bearing1 = Geodesy.angle(curr_pt, prev_pt)[0]
        bearing2 = Geodesy.angle(curr_pt, next_pt)[0]
        
        angle = (bearing2 - bearing1) % 360
        if angle > 180:
            angle = 360 - angle
        
        return angle
    
    
    @staticmethod
    def compute_corner(prev_start, prev_end, prev_heading,
                      next_start, next_end, next_heading,
                      angle_deg, offset_distance):
        """
        Compute corner point with intelligent fallback.
        
        Returns:
        - Intersection point if valid
        - None if too acute, parallel, or would create a spike
        """
        
        # Check for parallel edges
        heading_diff = abs((prev_heading - next_heading + 180) % 360 - 180)
        if heading_diff < 2.0:  # Nearly parallel
            return None
        
        # Reject very acute angles (< 15 degrees)
        if angle_deg < 15:
            return None
        
        # Try intersection
        try:
            intersection = Geodesy.intersection(
                prev_start[0], prev_start[1], prev_heading,
                next_start[0], next_start[1], next_heading
            )
            
            # Validate intersection quality
            d1 = Geodesy.distancebet(prev_end, intersection)
            d2 = Geodesy.distancebet(next_start, intersection)
            
            # Reject spikes (intersection too far from edge endpoints)
            if d1 > ModernHeadlandGenerator.MAX_OFFSET_SPIKE * offset_distance:
                return None
            if d2 > ModernHeadlandGenerator.MAX_OFFSET_SPIKE * offset_distance:
                return None
            
            return list(intersection)
            
        except (ValueError, Exception):
            return None
    
    
    @staticmethod
    def clean_path(path, min_separation=None):
        """
        Remove degenerate vertices, duplicates, and short segments.
        """
        
        if min_separation is None:
            min_separation = ModernHeadlandGenerator.MIN_VERTEX_SEPARATION
        
        if len(path) < 3:
            return path
        
        cleaned = [path[0]]
        
        for i in range(1, len(path)):
            curr = path[i]
            prev = cleaned[-1]
            
            # Skip if too close to previous
            dist = Geodesy.distancebet(prev, curr)
            if dist < min_separation:
                continue
            
            cleaned.append(curr)
        
        # Remove if final point is too close to first
        if len(cleaned) > 2:
            if Geodesy.distancebet(cleaned[-1], cleaned[0]) < min_separation:
                cleaned.pop()
        
        return cleaned
    
    
    @staticmethod
    def build_headland_robust(polygon, track_heading, width, turning_radius):
        """
        Main entry point: robust headland generation.
        Uses Shapely first, falls back to geodetic method.
        
        Parameters:
        - polygon: list of [lat, lon] vertices
        - track_heading: primary track direction (degrees)
        - width: implement width (meters)
        - turning_radius: tractor turning radius (meters)
        
        Returns:
        - tuple: (headland_points, headland_pass_points)
        
        ⚙️  CONFIGURATION:
        Change ModernHeadlandGenerator.OFFSET_INWARD to:
        - True = Generate headland INSIDE the field boundary
        - False = Generate headland OUTSIDE the field boundary
        """
        
        if len(polygon) < 3:
            return [], []
        
        offset_distance = width / 2 + turning_radius
        
        # Ensure minimum offset
        if offset_distance < ModernHeadlandGenerator.MIN_OFFSET_DISTANCE:
            offset_distance = ModernHeadlandGenerator.MIN_OFFSET_DISTANCE
        
        print(f"📍 Generating headland with offset={offset_distance:.2f}m (INWARD={ModernHeadlandGenerator.OFFSET_INWARD})")
        
        # Try Shapely-based method (most robust)
        try:
            headland = ModernHeadlandGenerator.buffer_polygon_shapely(
                polygon,
                offset_distance
            )
            
            if headland and len(headland) >= 3:
                print(f"✓ Headland generated using Shapely buffer ({len(headland)} points)")
                return headland, headland  # Use same for both headland and pass
        except ImportError:
            print("⚠️  Shapely not available, using geodetic method...")
        except Exception as e:
            print(f"⚠️  Shapely failed ({e}), trying geodetic method...")
        
        # Fallback to geodetic method
        print("→ Using geodetic offset algorithm...")
        headland = ModernHeadlandGenerator.offset_polygon_geodetic(
            polygon,
            track_heading,
            width,
            turning_radius
        )
        
        if headland and len(headland) >= 3:
            return headland, headland
        
        print("❌ Headland generation completely failed!")
        return [], []


# ============================================================================
# Backward compatibility function
# ============================================================================

def build_headland_production(polygon, track_heading, width, turning_radius):
    """
    Drop-in replacement for Headland.build_headland().
    Use this function in your path generation code.
    """
    return ModernHeadlandGenerator.build_headland_robust(
        polygon,
        track_heading,
        width,
        turning_radius
    )
