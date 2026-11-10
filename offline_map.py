"""
offline_map.py - Offline PMTiles map widget for the ResQNet LoRa dashboard.

What it does
------------
* Starts `pmtiles serve` on a free local port (reads sar_area.pmtiles).
* Starts a tiny local web server (127.0.0.1 only) that serves the map page,
  the vendored Leaflet / protomaps-leaflet files, and proxies /tiles/* to
  `pmtiles serve`, so the page and tiles share one origin.
* Shows the page inside a QWebEngineView.

Public API (used by the dashboard)
----------------------------------
* set_base_position(lat, lon, label="Base")
      Fixed Base marker (red square "B" with a permanent label).
* set_position(lat, lon, message="", name=None, fix="LIVE", fix_age=None)
      Node marker + trail. Blue "N" = live fix, amber "N" = last known
      position (node lost GPS). Called for every GPS beacon and every
      message; the view only moves when the node nears the edge. The popup shows name, message, coordinates,
      fix state, receive time and distance from the Base.
* shutdown()
      Stops both servers (called from the dashboard's closeEvent).

All map state is kept in Python and replayed into the page whenever it
(re)loads, so nothing is lost if the page loads late or the browser
render process crashes and is restarted.

Nothing here touches the internet.
"""

import atexit
import json
import math
import shutil
import socket
import subprocess
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

# QtWebEngine must be imported BEFORE QApplication is created.
from PyQt6.QtCore import Qt, QCoreApplication, QTimer, QUrl
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtWebEngineCore import QWebEnginePage
from PyQt6.QtWebEngineWidgets import QWebEngineView

QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)


# ============================================================
# CONFIG - change these if your paths differ
# ============================================================

PROJECT_DIR = Path("/srv/sar-cyberdeck")
MAPS_DIR = PROJECT_DIR / "maps"          # contains sar_area.pmtiles
WEB_DIR = PROJECT_DIR / "webapp"         # contains leaflet.js, leaflet.css, ...
MAP_NAME = "sar_area"                    # file name without .pmtiles

PMTILES_BIN = shutil.which("pmtiles") or "/usr/local/bin/pmtiles"

# Where the map opens before the first GPS fix (centre of your bbox)
START_LAT = 6.9169
START_LON = 79.8898
START_ZOOM = 12

# Zoom used when the first node fix arrives (and the max zoom when
# framing Base + node together)
FIX_ZOOM = 16

MAX_TRAIL_POINTS = 500

# The node reports every few seconds. A stationary NEO-6M drifts a
# few metres, so only add a trail point after moving this far.
TRAIL_MIN_METRES = 5

# How many times to retry if the page fails to load
MAX_LOAD_RETRIES = 5


# ============================================================
# MAP PAGE (served at "/")
# ============================================================

MAP_HTML = """<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>ResQNet Map</title>
  <link rel="stylesheet" href="/leaflet.css">
  <style>
    html, body, #map { height: 100%; margin: 0; background: #e5e7eb; }

    /* Markers are pure CSS, so no Leaflet marker PNGs are needed */
    .pin {
      width: 22px; height: 22px;
      border: 2px solid #ffffff;
      border-radius: 50%;
      box-shadow: 0 0 4px rgba(0, 0, 0, 0.6);
      color: #ffffff;
      font: bold 12px Arial, sans-serif;
      display: flex; align-items: center; justify-content: center;
      box-sizing: border-box;
    }
    .pin-base { background: #dc2626; border-radius: 4px; }
    .pin-live { background: #2563eb; }
    .pin-last { background: #f59e0b; }

    .pin-label { font: bold 12px Arial, sans-serif; }
  </style>
</head>
<body>
<div id="map"></div>
<script src="/leaflet.js"></script>
<script src="/protomaps-leaflet.js"></script>
<script>
  const MAX_TRAIL = __MAX_TRAIL__;
  const FIX_ZOOM = __FIX_ZOOM__;
  const TRAIL_MIN_M = __TRAIL_MIN__;

  const map = L.map('map', { maxZoom: 18 }).setView([__LAT__, __LON__], __ZOOM__);

  protomapsL.leafletLayer({
    url: '/tiles/__TILE_NAME__/{z}/{x}/{y}.mvt',
    flavor: 'light',
    attribution: '\\u00a9 OpenStreetMap contributors'
  }).addTo(map);

  let baseMarker = null;
  let baseLatLng = null;
  let baseName = 'Base';

  let marker = null;
  let trail = null;
  let firstFix = true;
  const points = [];

  // ---------- helpers (all text goes in as textContent, never HTML) ----------

  function makeIcon(cls, letter) {
    return L.divIcon({
      className: '',
      html: '<div class="pin ' + cls + '">' + letter + '</div>',
      iconSize: [22, 22],
      iconAnchor: [11, 11],
      popupAnchor: [0, -12],
      tooltipAnchor: [12, 0]
    });
  }

  function textSpan(text) {
    const span = document.createElement('span');
    span.className = 'pin-label';
    span.textContent = text;
    return span;
  }

  function textBlock(lines) {
    const div = document.createElement('div');
    lines.forEach(function (line, i) {
      if (i > 0) { div.appendChild(document.createElement('br')); }
      if (i === 0) {
        const b = document.createElement('b');
        b.textContent = line;
        div.appendChild(b);
      } else {
        div.appendChild(document.createTextNode(line));
      }
    });
    return div;
  }

  function setLabel(layer, text) {
    if (layer.getTooltip()) {
      layer.setTooltipContent(textSpan(text));
    } else {
      layer.bindTooltip(textSpan(text), { permanent: true, direction: 'right' });
    }
  }

  // ---------- Base ----------

  function setBase(lat, lon, name) {
    baseLatLng = L.latLng(lat, lon);
    baseName = name || 'Base';

    if (!baseMarker) {
      baseMarker = L.marker(baseLatLng, {
        icon: makeIcon('pin-base', 'B'),
        zIndexOffset: 1000
      }).addTo(map);
    } else {
      baseMarker.setLatLng(baseLatLng);
    }

    setLabel(baseMarker, baseName);
    baseMarker.bindPopup(textBlock([
      baseName,
      lat.toFixed(6) + ', ' + lon.toFixed(6)
    ]));
  }

  // ---------- Node ----------

  function applyPosition(p) {
    const ll = L.latLng(p.lat, p.lon);
    const lost = (p.fix === 'LAST');

    // Trail: skip repeats and GPS jitter of a few metres
    const prev = points[points.length - 1];
    if (!prev || map.distance(prev, ll) >= TRAIL_MIN_M) {
      points.push(ll);
      if (points.length > MAX_TRAIL) { points.shift(); }
    }

    const icon = makeIcon(lost ? 'pin-last' : 'pin-live', 'N');

    if (!marker) {
      marker = L.marker(ll, { icon: icon }).addTo(map);
    } else {
      marker.setLatLng(ll);
      marker.setIcon(icon);
    }

    setLabel(marker, lost ? p.name + ' (last known)' : p.name);

    const lines = [p.name];
    if (p.msg) { lines.push(p.msg); }
    lines.push(p.lat.toFixed(6) + ', ' + p.lon.toFixed(6));
    lines.push(lost
      ? 'LAST KNOWN position' + (p.age ? ' (' + p.age + ' old)' : '')
      : 'LIVE GPS fix');
    lines.push('Received ' + p.stamp);
    if (baseLatLng) {
      lines.push((map.distance(baseLatLng, ll) / 1000).toFixed(2) +
                 ' km from ' + baseName);
    }
    marker.bindPopup(textBlock(lines));

    if (!trail) {
      trail = L.polyline(points, { color: '#2563eb', weight: 3 }).addTo(map);
    } else {
      trail.setLatLngs(points);
    }
  }

  // Show Base and node together the first time, follow the node after
  function frameView() {
    if (!marker) { return; }
    const ll = marker.getLatLng();
    if (baseLatLng && !baseLatLng.equals(ll)) {
      map.fitBounds(L.latLngBounds([baseLatLng, ll]),
                    { padding: [40, 40], maxZoom: FIX_ZOOM });
    } else {
      map.setView(ll, FIX_ZOOM);
    }
  }

  function setPosition(p) {
    applyPosition(p);
    if (firstFix) {
      frameView();
      firstFix = false;
    } else if (!map.getBounds().pad(-0.1).contains(L.latLng(p.lat, p.lon))) {
      // Only move the map when the node is near/over the edge, so the
      // operator's own panning and zooming isn't undone every update
      map.panTo(L.latLng(p.lat, p.lon));
    }
  }

  // Called by Python after every page (re)load
  function restore(state) {
    if (state.base) {
      setBase(state.base.lat, state.base.lon, state.base.name);
    }
    points.length = 0;
    state.track.forEach(applyPosition);
    if (marker) {
      frameView();
      firstFix = false;
    }
  }
</script>
</body>
</html>
"""


# ============================================================
# HELPERS
# ============================================================

def _free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _wait_for_port(port, proc, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if proc.poll() is not None:
            raise RuntimeError(
                "'pmtiles serve' exited immediately "
                "(check the map file and the pmtiles binary)."
            )
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                return
        except OSError:
            time.sleep(0.1)
    raise RuntimeError("'pmtiles serve' did not start listening in time.")


def _valid_coords(latitude, longitude):
    """Return (lat, lon) as floats, or None if they are not usable."""

    try:
        lat = float(latitude)
        lon = float(longitude)
    except (TypeError, ValueError):
        return None

    if not (math.isfinite(lat) and math.isfinite(lon)):
        return None

    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return None

    # 0,0 is what a GPS module reports before it has a real fix
    if lat == 0.0 and lon == 0.0:
        return None

    return lat, lon


def _format_age(seconds):
    s = int(max(0, seconds))
    if s < 60:
        return "{}s".format(s)
    m, s = divmod(s, 60)
    if m < 60:
        return "{}m {}s".format(m, s)
    h, m = divmod(m, 60)
    return "{}h {}m".format(h, m)


def _make_handler(web_dir, tile_port, index_html):
    """Build a request handler class bound to our directories/ports."""

    class Handler(SimpleHTTPRequestHandler):

        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=web_dir, **kwargs)

        def log_message(self, fmt, *args):
            pass  # keep the terminal quiet

        def _send(self, status, body, content_type, extra=None):
            self.send_response(status)

            # 204 / 304 must not carry a body
            no_body = status in (204, 304)

            if not no_body:
                self.send_header("Content-Type", content_type)

            for key, value in (extra or {}).items():
                self.send_header(key, value)

            if not no_body:
                self.send_header("Content-Length", str(len(body)))

            self.end_headers()

            if no_body:
                return

            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _proxy_tile(self, path):
            # /tiles/sar_area/z/x/y.mvt -> http://127.0.0.1:<port>/sar_area/z/x/y.mvt
            url = "http://127.0.0.1:{}{}".format(
                tile_port, path[len("/tiles"):]
            )
            try:
                with urllib.request.urlopen(url, timeout=5) as r:
                    status, body, headers = r.status, r.read(), r.headers
            except urllib.error.HTTPError as e:
                status, body, headers = e.code, e.read(), e.headers
            except Exception:
                self.send_error(502, "Tile server unavailable")
                return

            extra = {}
            for name in ("Content-Encoding", "Cache-Control"):
                value = headers.get(name)
                if value:
                    extra[name] = value

            self._send(
                status,
                body,
                headers.get("Content-Type", "application/octet-stream"),
                extra,
            )

        def do_GET(self):
            path = self.path.split("?", 1)[0]

            if path in ("/", "/index.html"):
                self._send(200, index_html, "text/html; charset=utf-8")
            elif path.startswith("/tiles/"):
                self._proxy_tile(path)
            else:
                super().do_GET()

    return Handler


class _Page(QWebEnginePage):
    """Prints JavaScript console messages to the terminal for debugging."""

    def javaScriptConsoleMessage(self, level, message, line, source):
        print("[map js] {} ({}:{})".format(message, source, line))


# ============================================================
# WIDGET
# ============================================================

class OfflineMapWidget(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)

        self._tile_proc = None
        self._httpd = None
        self._closed = False
        self._loaded = False
        self._load_retries = 0
        self.view = None

        # Map state, kept here so it can be replayed into the page
        self._base = None                                  # dict
        self._track = deque(maxlen=MAX_TRAIL_POINTS)       # list of dicts

        self.setMinimumHeight(250)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        try:
            url = self._start_servers()
        except Exception as e:
            print("[map] Offline map unavailable: {}".format(e))
            self.shutdown()
            label = QLabel("Offline map unavailable:\n{}".format(e))
            label.setWordWrap(True)
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(label)
            return

        self.view = QWebEngineView()
        self.view.setPage(_Page(self.view))
        self.view.loadFinished.connect(self._on_load_finished)
        self.view.page().renderProcessTerminated.connect(
            self._on_render_process_terminated
        )
        layout.addWidget(self.view)
        self.view.load(QUrl(url))

        atexit.register(self.shutdown)

    # --------------------------------------------------------
    # servers
    # --------------------------------------------------------

    def _start_servers(self):
        map_file = MAPS_DIR / "{}.pmtiles".format(MAP_NAME)

        if not map_file.exists():
            raise FileNotFoundError("Map file not found: {}".format(map_file))

        for name in ("leaflet.js", "leaflet.css", "protomaps-leaflet.js"):
            if not (WEB_DIR / name).exists():
                raise FileNotFoundError(
                    "Missing web file: {}".format(WEB_DIR / name)
                )

        if not Path(PMTILES_BIN).exists():
            raise FileNotFoundError(
                "pmtiles binary not found: {}".format(PMTILES_BIN)
            )

        tile_port = _free_port()

        self._tile_proc = subprocess.Popen(
            [PMTILES_BIN, "serve", str(MAPS_DIR), "--port", str(tile_port)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        _wait_for_port(tile_port, self._tile_proc)

        html = (
            MAP_HTML
            .replace("__TILE_NAME__", MAP_NAME)
            .replace("__LAT__", str(START_LAT))
            .replace("__LON__", str(START_LON))
            .replace("__ZOOM__", str(START_ZOOM))
            .replace("__FIX_ZOOM__", str(FIX_ZOOM))
            .replace("__MAX_TRAIL__", str(MAX_TRAIL_POINTS))
            .replace("__TRAIL_MIN__", str(TRAIL_MIN_METRES))
        ).encode("utf-8")

        handler = _make_handler(str(WEB_DIR), tile_port, html)
        self._httpd = ThreadingHTTPServer(("127.0.0.1", 0), handler)

        threading.Thread(
            target=self._httpd.serve_forever, daemon=True
        ).start()

        return "http://127.0.0.1:{}/".format(self._httpd.server_address[1])

    # --------------------------------------------------------
    # page events
    # --------------------------------------------------------

    def _run_js(self, js):
        if self.view is not None and self._loaded and not self._closed:
            self.view.page().runJavaScript(js)

    def _on_load_finished(self, ok):
        if self._closed:
            return

        if not ok:
            self._loaded = False

            if self._load_retries < MAX_LOAD_RETRIES:
                self._load_retries += 1
                print("[map] Page failed to load, retry {}/{}".format(
                    self._load_retries, MAX_LOAD_RETRIES
                ))
                QTimer.singleShot(2000, self._reload)
            else:
                print("[map] Page failed to load - giving up.")
            return

        self._loaded = True
        self._load_retries = 0

        # Replay everything we know into the fresh page
        state = {
            "base": self._base,
            "track": list(self._track),
        }
        self._run_js("restore({});".format(json.dumps(state)))

    def _on_render_process_terminated(self, status, exit_code):
        if self._closed:
            return

        print("[map] Browser render process stopped ({}), reloading.".format(
            exit_code
        ))
        self._loaded = False
        QTimer.singleShot(1000, self._reload)

    def _reload(self):
        if self.view is not None and not self._closed:
            self.view.reload()

    # --------------------------------------------------------
    # public API (used by the dashboard)
    # --------------------------------------------------------

    def set_base_position(self, latitude, longitude, label="Base"):
        """Place (or move) the fixed Base marker."""

        coords = _valid_coords(latitude, longitude)

        if coords is None:
            print("[map] Ignoring invalid Base position: {}, {}".format(
                latitude, longitude
            ))
            return

        self._base = {
            "lat": coords[0],
            "lon": coords[1],
            "name": str(label or "Base"),
        }

        self._run_js("setBase({lat}, {lon}, {name});".format(
            lat=json.dumps(self._base["lat"]),
            lon=json.dumps(self._base["lon"]),
            name=json.dumps(self._base["name"]),
        ))

    def set_position(self, latitude, longitude, message="",
                     name=None, fix="LIVE", fix_age=None):
        """
        Pin / move the node marker to a GPS fix received over LoRa.

        fix     : "LIVE" (fresh fix) or "LAST" (last known position)
        fix_age : seconds since the node had that fix (for "LAST")
        """

        coords = _valid_coords(latitude, longitude)

        if coords is None:
            print("[map] Ignoring invalid node position: {}, {}".format(
                latitude, longitude
            ))
            return

        point = {
            "lat": coords[0],
            "lon": coords[1],
            "name": str(name or "Node"),
            "msg": str(message or ""),
            "fix": "LAST" if fix == "LAST" else "LIVE",
            "age": _format_age(fix_age) if fix_age else "",
            "stamp": time.strftime("%H:%M:%S"),
        }

        # Same spot and same state as last time (e.g. a stationary node
        # beaconing): update that entry instead of filling the history
        last = self._track[-1] if self._track else None

        if (
            last is not None
            and last["lat"] == point["lat"]
            and last["lon"] == point["lon"]
            and last["fix"] == point["fix"]
        ):
            self._track[-1] = point
        else:
            self._track.append(point)

        self._run_js("setPosition({});".format(json.dumps(point)))

    def shutdown(self):
        """Stop the local web server and `pmtiles serve`."""

        if self._closed:
            return

        self._closed = True
        self._loaded = False

        if self._httpd is not None:
            try:
                self._httpd.shutdown()
                self._httpd.server_close()
            except Exception:
                pass
            self._httpd = None

        if self._tile_proc is not None:
            try:
                if self._tile_proc.poll() is None:
                    self._tile_proc.terminate()
                    try:
                        self._tile_proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        self._tile_proc.kill()
            except Exception:
                pass
            self._tile_proc = None


# ============================================================
# STANDALONE TEST:  python offline_map.py
# ============================================================

if __name__ == "__main__":

    import sys
    from PyQt6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    window = OfflineMapWidget()
    window.resize(900, 600)
    window.show()

    # Base is set before the page loads, to test the replay
    window.set_base_position(6.9126422, 79.8509647, "Base")

    QTimer.singleShot(
        3000, lambda: window.set_position(
            6.9169, 79.8898, "Test fix 1", name="HIKER-01", fix="LIVE")
    )
    QTimer.singleShot(
        6000, lambda: window.set_position(
            6.9200, 79.8950, "Test fix 2", name="HIKER-01", fix="LIVE")
    )
    QTimer.singleShot(
        9000, lambda: window.set_position(
            6.9200, 79.8950, "Lost GPS", name="HIKER-01",
            fix="LAST", fix_age=95)
    )

    code = app.exec()
    window.shutdown()
    sys.exit(code)
