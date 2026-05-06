#!/usr/bin/env python3
"""Fetch OSM road network via Overpass API and render to 1-bit tile binaries.

One vector download covers all zoom levels. Roads are drawn as white lines
on black -- purpose-built for 1-bit output on the Sharp Memory LCD rather
than thresholding colour web-map tiles.

Usage:
    python scripts/fetch_tiles.py <lat> <lon> [options]
    pip install pillow   # only dependency

Output: <out>/<zoom>/<x>/<y>.bin  (256x256, 1-bit packed MSB-first = 8192 bytes)
Also writes <out>/preview_z<zoom>.png for visual verification before SD insertion.
Re-running is safe: tiles are always overwritten with freshly rendered content.
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import time
import urllib.request
from PIL import Image, ImageDraw

TILE_PX    = 256
TILE_BYTES = TILE_PX * TILE_PX // 8  # 8192

# Line widths in pixels at zoom 15 (reference). Scaled by 2^(zoom-15).
HIGHWAY_WIDTH: dict[str, int] = {
    "motorway":      5,  "motorway_link":  3,
    "trunk":         4,  "trunk_link":     3,
    "primary":       4,  "primary_link":   3,
    "secondary":     3,  "secondary_link": 2,
    "tertiary":      2,  "tertiary_link":  2,
    "unclassified":  2,  "residential":    2,
    "living_street": 2,  "pedestrian":     2,
    "service":       1,  "track":          1,
    "cycleway":      2,  "path":           1,
    "footway":       1,  "bridleway":      1,
    "steps":         1,  "road":           1,
}
WATERWAY_WIDTH = 2  # river, stream, canal


# ---------------------------------------------------------------------------
# Tile / projection math
# ---------------------------------------------------------------------------

def tile_xy(lat: float, lon: float, zoom: int) -> tuple[int, int]:
    """Slippy map tile coords for a lat/lon at a given zoom."""
    n = 1 << zoom
    x = int((lon + 180.0) / 360.0 * n)
    r = math.radians(lat)
    y = int((1.0 - math.log(math.tan(r) + 1.0 / math.cos(r)) / math.pi) / 2.0 * n)
    return x, y


def latlon_to_px(lat: float, lon: float, zoom: int) -> tuple[float, float]:
    """Global pixel coordinate (float) at this zoom level."""
    n = 1 << zoom
    px = (lon + 180.0) / 360.0 * n * TILE_PX
    r  = math.radians(lat)
    py = (1.0 - math.log(math.tan(r) + 1.0 / math.cos(r)) / math.pi) / 2.0 * n * TILE_PX
    return px, py


# ---------------------------------------------------------------------------
# Overpass fetch
# ---------------------------------------------------------------------------

def fetch_overpass(lat_min: float, lon_min: float,
                   lat_max: float, lon_max: float) -> list[dict]:
    """Download all highway + waterway ways in the bounding box."""
    query = (
        f"[out:json]"
        f"[bbox:{lat_min:.6f},{lon_min:.6f},{lat_max:.6f},{lon_max:.6f}]"
        f"[timeout:90];\n"
        f"(\n"
        f'  way["highway"];\n'
        f'  way["waterway"~"^(river|stream|canal|drain)$"];\n'
        f");\n"
        f"out geom;\n"
    )
    req = urllib.request.Request(
        "https://overpass-api.de/api/interpreter",
        data=query.encode(),
        headers={
            "Content-Type": "application/x-www-form-urlencoded",
            "User-Agent":   "bike-computer/1.0 (personal embedded nav project)",
        },
    )
    print("Querying Overpass API ... ", end="", flush=True)
    t0 = time.monotonic()
    with urllib.request.urlopen(req, timeout=120) as resp:
        raw = resp.read()
    elements = json.loads(raw)["elements"]
    print(f"{len(elements)} ways  {len(raw)//1024} KB  {time.monotonic()-t0:.1f}s")
    return elements


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def render_canvas(ways: list[dict], zoom: int,
                  tx_min: int, tx_max: int,
                  ty_min: int, ty_max: int) -> Image.Image:
    """Draw all ways onto a single canvas covering the tile range."""
    scale  = 2.0 ** (zoom - 15)
    w      = (tx_max - tx_min + 1) * TILE_PX
    h      = (ty_max - ty_min + 1) * TILE_PX
    canvas = Image.new("L", (w, h), 0)       # black background
    draw   = ImageDraw.Draw(canvas)
    off_x  = tx_min * TILE_PX
    off_y  = ty_min * TILE_PX

    for way in ways:
        geom = way.get("geometry") or []
        if len(geom) < 2:
            continue
        tags = way.get("tags", {})
        hw   = tags.get("highway")
        ww   = tags.get("waterway")
        if hw:
            base = HIGHWAY_WIDTH.get(hw, 1)
        elif ww:
            base = WATERWAY_WIDTH
        else:
            continue

        width = max(1, round(base * scale))
        pts   = []
        for node in geom:
            gx, gy = latlon_to_px(node["lat"], node["lon"], zoom)
            pts.append((gx - off_x, gy - off_y))

        for i in range(len(pts) - 1):
            draw.line([pts[i], pts[i + 1]], fill=255, width=width)

    return canvas


# ---------------------------------------------------------------------------
# Tile writing
# ---------------------------------------------------------------------------

def write_tiles(canvas: Image.Image, zoom: int,
                tx_min: int, ty_min: int,
                out_dir: pathlib.Path) -> int:
    """Slice canvas into 256x256 tiles and write as 1-bit packed binary."""
    cols    = canvas.width  // TILE_PX
    rows    = canvas.height // TILE_PX
    written = 0
    for row in range(rows):
        for col in range(cols):
            x   = tx_min + col
            y   = ty_min + row
            box = (col * TILE_PX, row * TILE_PX,
                   (col + 1) * TILE_PX, (row + 1) * TILE_PX)
            # convert() to "1" thresholds at 128; our canvas is pure 0/255
            # so dithering has no effect regardless of PIL version.
            tile = canvas.crop(box).convert("1")
            path = out_dir / str(zoom) / str(x) / f"{y}.bin"
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(tile.tobytes())
            written += 1
    return written


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("lat",         type=float,          help="centre latitude")
    ap.add_argument("lon",         type=float,          help="centre longitude")
    ap.add_argument("--zoom-min",  type=int,   default=14)
    ap.add_argument("--zoom-max",  type=int,   default=16)
    ap.add_argument("--radius-km", type=float, default=5.0,
                    help="coverage radius in km (default 5)")
    ap.add_argument("--out", type=pathlib.Path, default=pathlib.Path("tiles"),
                    help="output directory (copy this to SD card root)")
    args = ap.parse_args()

    # Bounding box with 20%% margin so roads crossing the edge are fully drawn.
    dlat    = args.radius_km / 111.0 * 1.2
    dlon    = args.radius_km / (111.0 * math.cos(math.radians(args.lat))) * 1.2
    lat_min = args.lat - dlat;  lat_max = args.lat + dlat
    lon_min = args.lon - dlon;  lon_max = args.lon + dlon
    print(f"Area: ({lat_min:.4f},{lon_min:.4f}) → ({lat_max:.4f},{lon_max:.4f})")

    # Single Overpass fetch reused for every zoom level.
    ways = fetch_overpass(lat_min, lon_min, lat_max, lon_max)

    for zoom in range(args.zoom_min, args.zoom_max + 1):
        # NW corner → smallest tile coords; SE corner → largest.
        tx_min, ty_min = tile_xy(lat_max, lon_min, zoom)
        tx_max, ty_max = tile_xy(lat_min, lon_max, zoom)
        n_tiles = (tx_max - tx_min + 1) * (ty_max - ty_min + 1)
        print(f"\nzoom {zoom}: {tx_max-tx_min+1}×{ty_max-ty_min+1} = {n_tiles} tiles",
              flush=True)

        print("  rendering ... ", end="", flush=True)
        t0     = time.monotonic()
        canvas = render_canvas(ways, zoom, tx_min, tx_max, ty_min, ty_max)
        print(f"{time.monotonic()-t0:.1f}s")

        # Thumbnail preview written alongside the tiles so you can sanity-check
        # the render in Preview.app before plugging the SD card in.
        preview = canvas.copy()
        preview.thumbnail((1024, 1024))
        preview_path = args.out / f"preview_z{zoom}.png"
        preview_path.parent.mkdir(parents=True, exist_ok=True)
        preview.save(str(preview_path))
        print(f"  preview  → {preview_path}")

        print("  writing tiles ... ", end="", flush=True)
        t0        = time.monotonic()
        n_written = write_tiles(canvas, zoom, tx_min, ty_min, args.out)
        print(f"{n_written} tiles  {time.monotonic()-t0:.1f}s")

    print(f"\nDone → {args.out}/")


if __name__ == "__main__":
    main()
