#include "track.h"

#include <SD.h>
#include <cmath>
#include <cstring>
#include <cstdlib>

#include "logging.h"

// --- Hardware ---
static constexpr int TRACK_BUTTON_PIN = 0;
static constexpr uint32_t BUTTON_DEBOUNCE_MS = 40;

// --- Detection thresholds ---
static constexpr double TRACK_RADIUS_M        = 12.0;   // proximity for RECORDING circuit detection
static constexpr double MIN_CROSSING_DIST_M   = 50.0;   // cumulative distance required between crossings
static constexpr double GATE_HALF_WIDTH_M     = 15.0;   // half-width of start/finish gate (30m total)
// TODO: increase to 25.0 km/h for production — 5 km/h is for neighborhood testing only
static constexpr double MIN_SPEED_KMH         = 5.0;
static constexpr uint32_t MIN_LAP_MS          = 5000;
static constexpr double AUTO_DETECT_RADIUS_M  = 1000.0; // wide enough to catch paddock areas
static constexpr double CIRCUIT_HEADING_TOL_DEG = 90.0; // reject wrong-way passes during RECORDING

// --- Multi-track storage ---
static constexpr int MAX_TRACKS = 16;
static const char *TRACKS_FILE  = "/tracks.csv";
static const char *OLD_META_FILE = "/track_meta.csv";

struct SavedTrack {
  double   lat;
  double   lon;
  double   alt;
  double   courseDeg;
  uint32_t id;
  char     name[32];
};

static SavedTrack savedTracks[MAX_TRACKS];
static int numSavedTracks = 0;

// --- Active track state ---
static uint8_t  trackState      = TRACK_STATE_IDLE;
static bool     lapActive       = false;
static bool     startPending    = false;
// --- Previous GPS position (for line-segment crossing) ---
static double   prevGpsLat      = NAN;
static double   prevGpsLon      = NAN;

// --- Cumulative distance tracking (replaces leave-radius) ---
static double   cumulativeDistM = 0.0;

static double   startLat        = NAN;
static double   startLon        = NAN;
static double   startAlt        = NAN;
static double   startCourseDeg  = NAN;
static uint32_t trackId         = 0;

static uint32_t lapStartMs      = 0;
static uint32_t currentLapMs    = 0;
static uint32_t lastLapMs       = 0;
static uint32_t bestLapMs       = 0;
static uint32_t lapCount        = 0;

// --- Button ---
static bool     buttonStableState  = true;
static bool     buttonLastReading  = true;
static uint32_t buttonLastChangeMs = 0;
static bool     buttonPressedFlag  = false;

static uint8_t  trackEventFlags = 0;

// ===================== Math helpers =====================

static double deg2rad(double deg) {
  return deg * (PI / 180.0);
}

static double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  static constexpr double EARTH_RADIUS_M = 6371000.0;
  double dLat = deg2rad(lat2 - lat1);
  double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return EARTH_RADIUS_M * c;
}

static double courseDelta(double a, double b) {
  double diff = fabs(a - b);
  if (diff > 180.0) diff = 360.0 - diff;
  return diff;
}

static uint32_t fnv1a(const uint8_t *data, size_t len) {
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < len; ++i) {
    hash ^= data[i];
    hash *= 16777619u;
  }
  return hash;
}

static uint32_t computeTrackId(double lat, double lon, double /* alt */) {
  // Quantize to 4 decimal places (~11m resolution) for stable hashing.
  // Altitude excluded — GPS alt varies 20-50m between sessions, causing
  // the same track to get different IDs.
  int32_t lat_i = static_cast<int32_t>(lround(lat * 1e4));
  int32_t lon_i = static_cast<int32_t>(lround(lon * 1e4));
  uint8_t buf[8];
  memcpy(buf, &lat_i, sizeof(lat_i));
  memcpy(buf + 4, &lon_i, sizeof(lon_i));
  return fnv1a(buf, sizeof(buf));
}

// ===================== Line-segment crossing math =====================

// Project lat/lon to local X/Y meters relative to a reference point.
// Equirectangular approximation — accurate within ~10km of reference.
static void latLonToXY(double lat, double lon,
                       double refLat, double refLon,
                       double &xM, double &yM) {
  static constexpr double R = 6371000.0;
  double latRad = deg2rad(refLat);
  xM = deg2rad(lon - refLon) * R * cos(latRad);
  yM = deg2rad(lat - refLat) * R;
}

// 2D cross product: (B-A) x (C-A). Positive = C is left of A->B.
static double cross2D(double ax, double ay,
                      double bx, double by,
                      double cx, double cy) {
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

// Test whether trajectory P1->P2 crosses gate Q1->Q2.
// checkDirection: if true, only accept crossings where P1 is on the "right"
// side of Q1->Q2 (approaching from the expected direction).
// crossFrac: fractional position along P1->P2 where crossing occurs [0..1].
static bool segmentsCross(double p1x, double p1y, double p2x, double p2y,
                          double q1x, double q1y, double q2x, double q2y,
                          bool checkDirection,
                          double &crossFrac) {
  double d1 = cross2D(q1x, q1y, q2x, q2y, p1x, p1y);
  double d2 = cross2D(q1x, q1y, q2x, q2y, p2x, p2y);
  if (d1 * d2 > 0.0) return false;  // same side — no crossing

  double d3 = cross2D(p1x, p1y, p2x, p2y, q1x, q1y);
  double d4 = cross2D(p1x, p1y, p2x, p2y, q2x, q2y);
  if (d3 * d4 > 0.0) return false;  // trajectory misses gate

  // Direction: d1 > 0 means P1 is on the approach side of the gate
  // (behind the start line relative to the racing direction)
  if (checkDirection && d1 <= 0.0) return false;

  double denom = d1 - d2;
  crossFrac = (fabs(denom) < 1e-12) ? 0.5 : (d1 / denom);
  return true;
}

// Compute gate endpoints perpendicular to startCourseDeg, in local XY.
// Start point is at origin (0,0).
static void computeGateEndpoints(double courseDeg,
                                  double &q1x, double &q1y,
                                  double &q2x, double &q2y) {
  double perpRad = deg2rad(courseDeg + 90.0);
  double dx = GATE_HALF_WIDTH_M * sin(perpRad);
  double dy = GATE_HALF_WIDTH_M * cos(perpRad);
  q1x =  dx;  q1y =  dy;   // left endpoint
  q2x = -dx;  q2y = -dy;   // right endpoint
}

// ===================== Track persistence =====================

static void saveTracks();  // forward declaration for migration path

static bool parseCsvLine(const String &line, SavedTrack &t) {
  memset(&t, 0, sizeof(t));
  int idx1 = line.indexOf(',');
  int idx2 = line.indexOf(',', idx1 + 1);
  int idx3 = line.indexOf(',', idx2 + 1);
  int idx4 = line.indexOf(',', idx3 + 1);
  if (idx1 < 0 || idx2 < 0 || idx3 < 0 || idx4 < 0) return false;

  t.lat       = line.substring(0, idx1).toDouble();
  t.lon       = line.substring(idx1 + 1, idx2).toDouble();
  t.alt       = line.substring(idx2 + 1, idx3).toDouble();
  t.courseDeg = line.substring(idx3 + 1, idx4).toDouble();

  // 6th column (name) is optional — backward compatible with old files
  int idx5 = line.indexOf(',', idx4 + 1);
  if (idx5 >= 0) {
    t.id = static_cast<uint32_t>(strtoul(line.substring(idx4 + 1, idx5).c_str(), nullptr, 10));
    String name = line.substring(idx5 + 1);
    name.trim();
    strlcpy(t.name, name.c_str(), sizeof(t.name));
  } else {
    t.id = static_cast<uint32_t>(strtoul(line.substring(idx4 + 1).c_str(), nullptr, 10));
  }
  return t.id != 0;
}

static bool loadTracks() {
  if (!storageReady) return false;
  numSavedTracks = 0;

  // Try new multi-track file first
  if (SD.exists(TRACKS_FILE)) {
    File f = SD.open(TRACKS_FILE, "r");
    if (!f) return false;
    f.readStringUntil('\n'); // skip header
    while (f.available() && numSavedTracks < MAX_TRACKS) {
      String line = f.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) continue;
      SavedTrack t;
      if (parseCsvLine(line, t)) {
        savedTracks[numSavedTracks++] = t;
      }
    }
    f.close();
    Serial.print("[TRACK] Loaded ");
    Serial.print(numSavedTracks);
    Serial.println(" saved tracks");
    return numSavedTracks > 0;
  }

  // Migrate from old single-track format
  if (SD.exists(OLD_META_FILE)) {
    File f = SD.open(OLD_META_FILE, "r");
    if (!f) return false;
    f.readStringUntil('\n'); // skip header
    String line = f.readStringUntil('\n');
    f.close();
    line.trim();
    SavedTrack t;
    if (line.length() > 0 && parseCsvLine(line, t)) {
      savedTracks[0] = t;
      numSavedTracks = 1;
      saveTracks();
      SD.remove(OLD_META_FILE);
      Serial.println("[TRACK] Migrated old track_meta.csv -> tracks.csv");
      return true;
    }
  }

  return false;
}

static void saveTracks() {
  if (!storageReady) return;
  File f = SD.open(TRACKS_FILE, "w");
  if (!f) return;
  f.println("start_lat,start_lon,start_alt,start_course_deg,track_id,name");
  for (int i = 0; i < numSavedTracks; ++i) {
    const SavedTrack &t = savedTracks[i];
    f.print(String(t.lat, 6));       f.print(",");
    f.print(String(t.lon, 6));       f.print(",");
    f.print(String(t.alt, 2));       f.print(",");
    f.print(String(t.courseDeg, 2)); f.print(",");
    f.print(String(t.id));           f.print(",");
    f.println(t.name);
  }
  f.close();
}

static void saveOrUpdateTrack(double lat, double lon, double alt,
                               double courseDeg, uint32_t id,
                               const char *name = "") {
  for (int i = 0; i < numSavedTracks; ++i) {
    if (savedTracks[i].id == id) {
      savedTracks[i].lat = lat;
      savedTracks[i].lon = lon;
      savedTracks[i].alt = alt;
      savedTracks[i].courseDeg = courseDeg;
      if (name[0] != '\0') strlcpy(savedTracks[i].name, name, sizeof(savedTracks[i].name));
      saveTracks();
      return;
    }
  }
  if (numSavedTracks < MAX_TRACKS) {
    SavedTrack &t = savedTracks[numSavedTracks++];
    t.lat = lat;
    t.lon = lon;
    t.alt = alt;
    t.courseDeg = courseDeg;
    t.id = id;
    strlcpy(t.name, name, sizeof(t.name));
    saveTracks();
  }
}

// ===================== Auto-detection =====================

static int findNearestTrack(double lat, double lon) {
  int bestIdx = -1;
  double bestDist = AUTO_DETECT_RADIUS_M;
  for (int i = 0; i < numSavedTracks; ++i) {
    double d = distanceMeters(lat, lon, savedTracks[i].lat, savedTracks[i].lon);
    if (d < bestDist) {
      bestDist = d;
      bestIdx = i;
    }
  }
  return bestIdx;
}

static void loadTrackByIndex(int idx) {
  const SavedTrack &t = savedTracks[idx];
  startLat      = t.lat;
  startLon      = t.lon;
  startAlt      = t.alt;
  startCourseDeg = t.courseDeg;
  trackId       = t.id;
  trackState      = TRACK_STATE_READY;
  lapActive       = false;
  startPending    = false;
  cumulativeDistM = 0.0;
  prevGpsLat      = NAN;
  prevGpsLon      = NAN;
  lapStartMs      = 0;
  currentLapMs    = 0;
  lastLapMs       = 0;
  bestLapMs       = 0;
  lapCount        = 0;
  Serial.print("[TRACK] Auto-detected track ID ");
  Serial.println(trackId);
}

// ===================== Start line crossing =====================

static uint32_t lastCrossDebugMs = 0;

// Circuit detection during RECORDING: proximity + cumulative distance + heading.
// Heading tolerance is 90° — generous enough for GPS noise at 5 km/h,
// restrictive enough to reject wrong-way passes (180° delta).
static bool isCircuitComplete(double lat, double lon,
                               double spdKmh, double courseDeg) {
  if (isnan(startLat) || isnan(startLon)) return false;
  if (spdKmh < MIN_SPEED_KMH) return false;
  if (cumulativeDistM < MIN_CROSSING_DIST_M) return false;

  // Heading check: reject opposite-direction passes through start.
  // Both initial and current course are above MIN_SPEED_KMH, so GPS course
  // is reliable enough to distinguish same-way (~0° delta) from wrong-way (~180°).
  if (!isnan(startCourseDeg)) {
    double hdgDelta = courseDelta(courseDeg, startCourseDeg);
    if (hdgDelta > CIRCUIT_HEADING_TOL_DEG) return false;
  }

  double dist = distanceMeters(lat, lon, startLat, startLon);

  // Debug print every 2 seconds when near start
  uint32_t now = millis();
  if (dist < 100.0 && now - lastCrossDebugMs > 2000) {
    lastCrossDebugMs = now;
    Serial.print("[REC-CROSS] dist=");
    Serial.print(dist, 1);
    Serial.print("m spd=");
    Serial.print(spdKmh, 1);
    Serial.print("km/h cumDist=");
    Serial.print(cumulativeDistM, 0);
    Serial.print("m hdgDelta=");
    Serial.print(courseDelta(courseDeg, startCourseDeg), 1);
    Serial.println("deg");
  }

  return dist <= TRACK_RADIUS_M;
}

// Start-line crossing during READY/RACING: line-segment intersection.
// Tests trajectory (prevGps -> currentGps) against the start/finish gate.
static bool isCrossingStartLine(double lat, double lon,
                                 double pLat, double pLon,
                                 double spdKmh, double &crossFrac) {
  if (isnan(startLat) || isnan(startLon) || isnan(startCourseDeg)) return false;
  if (isnan(pLat) || isnan(pLon)) return false;
  if (spdKmh < MIN_SPEED_KMH) return false;
  if (cumulativeDistM < MIN_CROSSING_DIST_M) return false;

  // Project into local XY relative to start point
  double p1x, p1y, p2x, p2y;
  latLonToXY(pLat, pLon, startLat, startLon, p1x, p1y);
  latLonToXY(lat, lon, startLat, startLon, p2x, p2y);

  // Quick reject: both points far from gate
  double maxDist = fmax(fabs(p1x) + fabs(p1y), fabs(p2x) + fabs(p2y));
  if (maxDist > GATE_HALF_WIDTH_M * 3.0) return false;

  // Debug print every 2 seconds when near start
  uint32_t now = millis();
  if (now - lastCrossDebugMs > 2000) {
    lastCrossDebugMs = now;
    Serial.print("[CROSS] p1=(");
    Serial.print(p1x, 1); Serial.print(","); Serial.print(p1y, 1);
    Serial.print(") p2=(");
    Serial.print(p2x, 1); Serial.print(","); Serial.print(p2y, 1);
    Serial.print(") cumDist=");
    Serial.print(cumulativeDistM, 0);
    Serial.print("m spd=");
    Serial.print(spdKmh, 1);
    Serial.println("km/h");
  }

  // Compute gate endpoints
  double q1x, q1y, q2x, q2y;
  computeGateEndpoints(startCourseDeg, q1x, q1y, q2x, q2y);

  return segmentsCross(p1x, p1y, p2x, p2y,
                       q1x, q1y, q2x, q2y,
                       true, crossFrac);
}

// ===================== Recording =====================

// Start point is deferred until we have a valid course while moving
static bool startLocked = false;

static void beginTrackRecording() {
  trackState      = TRACK_STATE_RECORDING;
  lapActive       = false;
  startPending    = false;
  cumulativeDistM = 0.0;
  prevGpsLat      = NAN;
  prevGpsLon      = NAN;
  startLocked     = false;
  lapStartMs      = 0;
  currentLapMs    = 0;
  lastLapMs       = 0;
  bestLapMs       = 0;
  lapCount        = 0;
  startLat        = NAN;
  startLon        = NAN;
  startAlt        = NAN;
  startCourseDeg  = NAN;
  trackId         = 0;
  Serial.println("[TRACK] Recording — waiting for movement to lock start");
}

static void lockStartPoint(double lat, double lon, double alt, double courseDeg) {
  startLat       = lat;
  startLon       = lon;
  startAlt       = alt;
  startCourseDeg = courseDeg;
  trackId        = computeTrackId(lat, lon, alt);
  startLocked    = true;
  startTrackLog(trackId);
  Serial.print("[TRACK] Start locked at heading ");
  Serial.print(courseDeg, 1);
  Serial.print("° — ID ");
  Serial.println(trackId);
}

static void finishTrackRecording() {
  stopTrackLog();
  saveOrUpdateTrack(startLat, startLon, startAlt, startCourseDeg, trackId);
  trackState      = TRACK_STATE_READY;
  lapActive       = false;
  cumulativeDistM = 0.0;
  startPending    = false;
  Serial.print("[TRACK] Recording finished, track ID ");
  Serial.println(trackId);
}

// ===================== Public API =====================

void initTrack() {
  pinMode(TRACK_BUTTON_PIN, INPUT_PULLUP);
  buttonStableState = digitalRead(TRACK_BUTTON_PIN);
  buttonLastReading = buttonStableState;
  buttonLastChangeMs = millis();
  buttonPressedFlag = false;
  loadTracks();
}

void pollTrackButton(uint32_t nowMs) {
  bool reading = digitalRead(TRACK_BUTTON_PIN);
  if (reading != buttonLastReading) {
    buttonLastChangeMs = nowMs;
    buttonLastReading = reading;
  }
  if (nowMs - buttonLastChangeMs > BUTTON_DEBOUNCE_MS) {
    if (reading != buttonStableState) {
      buttonStableState = reading;
      if (buttonStableState == LOW) {
        buttonPressedFlag = true;
        Serial.println("[TRACK] Button press detected");
      }
    }
  }
}

void updateTrack(uint32_t nowMs,
                 bool gpsUpdated,
                 bool gpsValid,
                 bool courseValid,
                 double lat,
                 double lon,
                 double alt_m,
                 double spd_kmph,
                 double course_deg,
                 double hdop,
                 uint32_t sats,
                 uint32_t epoch) {
  trackEventFlags = 0;

  // --- Button handling ---
  if (buttonPressedFlag) {
    buttonPressedFlag = false;
    if (trackState == TRACK_STATE_RECORDING) {
      // Cancel recording
      if (startLocked) stopTrackLog();
      trackState = TRACK_STATE_IDLE;
      startPending = false;
      startLocked = false;
      Serial.println("[TRACK] Recording cancelled");
    } else if (trackState == TRACK_STATE_RACING) {
      // Finalize session — stop racing, return to READY
      trackState = TRACK_STATE_READY;
      lapActive = false;
      cumulativeDistM = 0.0;
      Serial.println("[TRACK] Session finalized by button");
      diagLog("Session finalized by button");
    } else if (trackState == TRACK_STATE_IDLE ||
               trackState == TRACK_STATE_READY) {
      // Start recording — start point locked later when moving
      beginTrackRecording();
    }
  }

  // Lock the start point once we're moving with a valid course
  if (trackState == TRACK_STATE_RECORDING && !startLocked &&
      gpsUpdated && gpsValid && courseValid && spd_kmph >= MIN_SPEED_KMH) {
    lockStartPoint(lat, lon, alt_m, course_deg);
  }

  // --- Auto-detect known tracks when IDLE ---
  if (trackState == TRACK_STATE_IDLE && gpsUpdated && gpsValid && numSavedTracks > 0) {
    int idx = findNearestTrack(lat, lon);
    if (idx >= 0) {
      loadTrackByIndex(idx);
      diagLogf("Track auto-detected ID=%lu", (unsigned long)trackId);
    }
  }

  // --- Accumulate distance traveled + save previous GPS for crossing checks ---
  // Save previous position BEFORE updating (crossing checks need prev→current trajectory)
  double trajPrevLat = prevGpsLat;
  double trajPrevLon = prevGpsLon;
  if (gpsUpdated && gpsValid) {
    if (!isnan(prevGpsLat) && !isnan(prevGpsLon)) {
      double stepDist = distanceMeters(prevGpsLat, prevGpsLon, lat, lon);
      if (stepDist < 50.0) {  // reject implausible GPS jumps
        cumulativeDistM += stepDist;
      }
    }
    prevGpsLat = lat;
    prevGpsLon = lon;
  }

  // --- Recording state: log GPS and watch for circuit completion ---
  if (trackState == TRACK_STATE_RECORDING) {
    if (!startLocked) return;  // waiting for movement to lock start
    if (gpsUpdated && gpsValid) {
      appendTrackLog(epoch, lat, lon, alt_m, spd_kmph, course_deg, hdop, sats);
    }
    if (gpsUpdated && gpsValid && courseValid &&
        isCircuitComplete(lat, lon, spd_kmph, course_deg)) {
      // Update heading from current GPS course — reliable at speed.
      // The initial heading from lockStartPoint was at ~5 km/h and unreliable.
      startCourseDeg = course_deg;
      Serial.print("[TRACK] Updated start heading to ");
      Serial.print(course_deg, 1);
      Serial.println(" deg (captured at circuit completion speed)");
      finishTrackRecording();

      // Go straight to RACING — the circuit completion crossing IS the first start.
      // lapCount=1 so the next crossing records Lap 1 time (no wasted out-lap).
      trackState = TRACK_STATE_RACING;
      lapActive = true;
      lapStartMs = nowMs;
      currentLapMs = 0;
      lastLapMs = 0;
      bestLapMs = 0;
      lapCount = 1;
      cumulativeDistM = 0.0;
      trackEventFlags |= TRACK_EVENT_RECOGNIZED | TRACK_EVENT_LAP_START;
      Serial.println("[TRACK] >>> RACING started — timing from circuit detection");
      diagLog("RECORDING->RACING direct start");
    }
    return;
  }

  // --- Ready state: waiting for first start-line crossing ---
  if (trackState == TRACK_STATE_READY) {
    if (gpsUpdated && gpsValid) {
      double crossFrac = 0.0;
      if (isCrossingStartLine(lat, lon, trajPrevLat, trajPrevLon, spd_kmph, crossFrac)) {
        trackState = TRACK_STATE_RACING;
        lapActive = true;
        lapStartMs = nowMs;
        currentLapMs = 0;
        lastLapMs = 0;
        bestLapMs = 0;
        lapCount = 0;
        cumulativeDistM = 0.0;
        trackEventFlags |= TRACK_EVENT_RECOGNIZED | TRACK_EVENT_LAP_START;
        Serial.println("[TRACK] >>> RACING started — crossed start line");
        diagLog("READY->RACING crossed start line");
      }
    }
  }

  // --- Racing state: counting laps ---
  if (trackState == TRACK_STATE_RACING) {
    if (lapActive) {
      currentLapMs = nowMs - lapStartMs;
    }
    if (gpsUpdated && gpsValid) {
      double crossFrac = 0.0;
      if (isCrossingStartLine(lat, lon, trajPrevLat, trajPrevLon, spd_kmph, crossFrac)) {
        if (nowMs - lapStartMs >= MIN_LAP_MS) {
          uint32_t thisLapMs = nowMs - lapStartMs;

          if (lapCount > 0) {
            // Completed a full lap (not the out-lap)
            lastLapMs = thisLapMs;
            if (bestLapMs == 0 || lastLapMs < bestLapMs) {
              bestLapMs = lastLapMs;
            }
            trackEventFlags |= TRACK_EVENT_LAP_END;
            char lapBuf[64];
            snprintf(lapBuf, sizeof(lapBuf), "[TRACK] >>> LAP %lu completed — %lu.%03lus",
                     static_cast<unsigned long>(lapCount),
                     static_cast<unsigned long>(lastLapMs / 1000),
                     static_cast<unsigned long>(lastLapMs % 1000));
            Serial.println(lapBuf);
          } else {
            // Out-lap completed — don't record time, just note it
            Serial.println("[TRACK] >>> Out-lap completed");
          }

          lapCount++;
          lapStartMs = nowMs;
          currentLapMs = 0;
          cumulativeDistM = 0.0;
          trackEventFlags |= TRACK_EVENT_LAP_START;
        }
      }
    }
  }
}

TrackUiState getTrackUiState() {
  TrackUiState state = {};
  state.trackState   = trackState;
  state.lapActive    = lapActive;
  state.startPending = startPending;
  state.lapCount     = lapCount;
  state.currentLapMs = currentLapMs;
  state.lastLapMs    = lastLapMs;
  state.bestLapMs    = bestLapMs;
  state.trackId      = trackId;
  // Find name for the active track
  state.trackName[0] = '\0';
  for (int i = 0; i < numSavedTracks; ++i) {
    if (savedTracks[i].id == trackId) {
      strlcpy(state.trackName, savedTracks[i].name, sizeof(state.trackName));
      break;
    }
  }
  return state;
}

uint8_t getTrackEventFlags() {
  return trackEventFlags;
}

bool getTrackReference(double &lat, double &lon, double &alt,
                       double &courseDeg, uint32_t &outTrackId) {
  if (isnan(startLat) || isnan(startLon)) return false;
  lat = startLat;
  lon = startLon;
  alt = startAlt;
  courseDeg = startCourseDeg;
  outTrackId = trackId;
  return true;
}

void resetTrack() {
  trackState      = TRACK_STATE_IDLE;
  lapActive       = false;
  startPending    = false;
  cumulativeDistM = 0.0;
  prevGpsLat      = NAN;
  prevGpsLon      = NAN;
  startLat        = NAN;
  startLon        = NAN;
  startAlt        = NAN;
  startCourseDeg  = NAN;
  trackId         = 0;
  lapStartMs      = 0;
  currentLapMs    = 0;
  lastLapMs       = 0;
  bestLapMs       = 0;
  lapCount        = 0;
}

int getSavedTrackCount() {
  return numSavedTracks;
}

bool getSavedTrack(int index, SavedTrackInfo &info) {
  if (index < 0 || index >= numSavedTracks) return false;
  const SavedTrack &t = savedTracks[index];
  info.lat = t.lat;
  info.lon = t.lon;
  info.alt = t.alt;
  info.courseDeg = t.courseDeg;
  info.id = t.id;
  strlcpy(info.name, t.name, sizeof(info.name));
  return true;
}

bool setTrackName(uint32_t tid, const char *name) {
  for (int i = 0; i < numSavedTracks; ++i) {
    if (savedTracks[i].id == tid) {
      strlcpy(savedTracks[i].name, name, sizeof(savedTracks[i].name));
      saveTracks();
      return true;
    }
  }
  return false;
}

bool addTrackFromApi(double lat, double lon, double alt,
                     double courseDeg, const char *name,
                     uint32_t &outTrackId) {
  uint32_t id = computeTrackId(lat, lon, alt);
  saveOrUpdateTrack(lat, lon, alt, courseDeg, id, name);
  outTrackId = id;
  return true;
}

bool deleteTrack(uint32_t tid) {
  int found = -1;
  for (int i = 0; i < numSavedTracks; ++i) {
    if (savedTracks[i].id == tid) { found = i; break; }
  }
  if (found < 0) return false;

  // Remove track recording CSV if it exists
  char recPath[32];
  snprintf(recPath, sizeof(recPath), "/track_%08lX.csv", (unsigned long)tid);
  if (SD.exists(recPath)) SD.remove(recPath);

  // Shift remaining tracks down
  for (int i = found; i < numSavedTracks - 1; ++i) {
    savedTracks[i] = savedTracks[i + 1];
  }
  numSavedTracks--;
  saveTracks();

  // If this was the active track, reset to IDLE
  if (trackId == tid) {
    resetTrack();
  }

  Serial.printf("[TRACK] Deleted track ID %lu\n", (unsigned long)tid);
  return true;
}

int countTrackRecordingPoints(uint32_t tid) {
  char recPath[32];
  snprintf(recPath, sizeof(recPath), "/track_%08lX.csv", (unsigned long)tid);
  File f = SD.open(recPath, "r");
  if (!f) return 0;

  int lines = 0;
  bool firstLine = true;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (firstLine) { firstLine = false; continue; }  // skip header
    if (line.length() > 0) lines++;
  }
  f.close();
  return lines;
}
