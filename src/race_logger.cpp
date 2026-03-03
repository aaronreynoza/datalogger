#include "race_logger.h"

#include <cmath>

#include "atp_writer.h"
#include "device_config.h"
#include "logging.h"
#include "track.h"

static constexpr uint32_t MAX_INTERP_MS = 200;
static constexpr uint32_t REAL_FIX_WINDOW_MS = 10;
static constexpr size_t IMU_BUFFER_SIZE = 600;
static constexpr double EARTH_RADIUS_M = 6371000.0;

struct ReferenceFrame {
  bool valid;
  double lat0_deg;
  double lon0_deg;
  double alt0_m;
  double lat0_rad;
  double cos_lat0;
};

static ReferenceFrame refFrame = {false, 0, 0, 0, 0, 1};
static uint32_t refTrackId = 0;

static ImuSample imuBuffer[IMU_BUFFER_SIZE];
static size_t imuHead = 0;
static size_t imuTail = 0;
static size_t imuCount = 0;

static bool havePrevFix = false;
static GpsFix prevFix;

static bool raceActive = false;
static bool lastPosValid = false;
static double lastX = 0.0;
static double lastY = 0.0;
static float lapDistanceM = 0.0f;

static AtpSession atpSession;
static uint8_t lastTrackState = TRACK_STATE_IDLE;

static double deg2rad(double deg) {
  return deg * (PI / 180.0);
}

static double rad2deg(double rad) {
  return rad * (180.0 / PI);
}

static void llaToEnu(double lat, double lon, double alt,
                     double &x_m, double &y_m, double &z_m) {
  double dLat = deg2rad(lat - refFrame.lat0_deg);
  double dLon = deg2rad(lon - refFrame.lon0_deg);
  x_m = dLon * refFrame.cos_lat0 * EARTH_RADIUS_M;
  y_m = dLat * EARTH_RADIUS_M;
  z_m = alt - refFrame.alt0_m;
}

static void enuToLla(double x_m, double y_m, double z_m,
                     double &lat, double &lon, double &alt) {
  double dLat = y_m / EARTH_RADIUS_M;
  double dLon = x_m / (EARTH_RADIUS_M * refFrame.cos_lat0);
  lat = refFrame.lat0_deg + rad2deg(dLat);
  lon = refFrame.lon0_deg + rad2deg(dLon);
  alt = refFrame.alt0_m + z_m;
}

static void computeVelocity(double spd_kmh, double course_deg,
                            double &vn_mps, double &ve_mps) {
  if (isnan(spd_kmh) || isnan(course_deg)) {
    vn_mps = NAN;
    ve_mps = NAN;
    return;
  }
  double spd_mps = spd_kmh / 3.6;
  double course_rad = deg2rad(course_deg);
  vn_mps = spd_mps * cos(course_rad);
  ve_mps = spd_mps * sin(course_rad);
}

static void pushSample(const ImuSample &sample) {
  if (imuCount == IMU_BUFFER_SIZE) {
    imuTail = (imuTail + 1) % IMU_BUFFER_SIZE;
    imuCount--;
  }
  imuBuffer[imuHead] = sample;
  imuHead = (imuHead + 1) % IMU_BUFFER_SIZE;
  imuCount++;
}

static bool popSample(ImuSample &sample) {
  if (imuCount == 0) return false;
  sample = imuBuffer[imuTail];
  imuTail = (imuTail + 1) % IMU_BUFFER_SIZE;
  imuCount--;
  return true;
}

static bool peekSample(ImuSample &sample) {
  if (imuCount == 0) return false;
  sample = imuBuffer[imuTail];
  return true;
}

// ===================== ATP output (replaces CSV writeRaceRow) =====================

static void writeRaceRow(const ImuSample &sample,
                         double lat, double lon, double alt_m,
                         double spd_kmh, double course_deg,
                         double hdop, uint32_t sats,
                         double x_m, double y_m, double z_m,
                         double vn_mps, double ve_mps,
                         uint8_t gpsSource,
                         uint16_t eventFlags) {
  // Lap distance tracking (unchanged)
  if (!sample.lapActive) {
    lapDistanceM = 0.0f;
    lastPosValid = false;
  }
  if (sample.eventFlags & TRACK_EVENT_LAP_START) {
    lapDistanceM = 0.0f;
    lastPosValid = false;
  }
  if (sample.lapActive && gpsSource > 0 && !isnan(x_m) && !isnan(y_m)) {
    if (lastPosValid) {
      double dx = x_m - lastX;
      double dy = y_m - lastY;
      lapDistanceM += static_cast<float>(sqrt(dx * dx + dy * dy));
    }
    lastX = x_m;
    lastY = y_m;
    lastPosValid = true;
  }

  // Push IMU sample to ATP
  if (sample.imu.hasData) {
    atpPushImu(atpSession, sample.ms,
               sample.imu.acc.x, sample.imu.acc.y, sample.imu.acc.z,
               sample.imu.gyr.x, sample.imu.gyr.y, sample.imu.gyr.z,
               sample.imu.tempC);
  }

  // Push GPS fix (only on real GPS fix rows, not every IMU row)
  if (gpsSource >= 1 && !isnan(lat) && !isnan(lon)) {
    float speed_mps = isnan(spd_kmh) ? 0.0f : (float)(spd_kmh / 3.6);
    float course_f = isnan(course_deg) ? 0.0f : (float)course_deg;
    float hdop_f = isnan(hdop) ? 99.0f : (float)hdop;
    atpPushGps(atpSession, sample.ms,
               lat, lon, (float)alt_m,
               speed_mps, course_f,
               hdop_f, (uint8_t)sats,
               (float)x_m, (float)y_m, (float)z_m,
               (float)vn_mps, (float)ve_mps, lapDistanceM);
  }

  // State change detection
  if (sample.trackState != lastTrackState) {
    atpPushStateChange(atpSession, sample.ms, lastTrackState, sample.trackState);
    lastTrackState = sample.trackState;
  }

  // Lap events
  if (eventFlags & TRACK_EVENT_LAP_END) {
    atpPushLapEvent(atpSession, sample.ms,
                    (uint16_t)sample.lapCount, sample.lastLapMs,
                    lapDistanceM, eventFlags);
  }
}

// ===================== Public API =====================

void initRaceLogger() {
  imuHead = 0;
  imuTail = 0;
  imuCount = 0;
  havePrevFix = false;
  raceActive = false;
  lastPosValid = false;
  lapDistanceM = 0.0f;
  lastTrackState = TRACK_STATE_IDLE;
}

void setRaceReference(double lat, double lon, double alt, uint32_t trackId) {
  refFrame.valid = true;
  refFrame.lat0_deg = lat;
  refFrame.lon0_deg = lon;
  refFrame.alt0_m = alt;
  refFrame.lat0_rad = deg2rad(lat);
  refFrame.cos_lat0 = cos(refFrame.lat0_rad);
  refTrackId = trackId;
}

bool hasRaceReference() {
  return refFrame.valid;
}

uint32_t getRaceTrackId() {
  return refTrackId;
}

bool startRaceLogger(uint32_t epoch, uint32_t nowMs,
                     const char *trackName) {
  if (!refFrame.valid) return false;
  if (!storageReady || !loggingEnabled) return false;

  // Load track recording points if available
  // (for embedding in ATP track definition)
  // TODO: read track_XXXXXXXX.csv and pass points to atpOpen
  // For now, pass no track recording points
  bool ok = atpOpen(atpSession, epoch, nowMs, refTrackId,
                    getConfig(),
                    refFrame.lat0_deg, refFrame.lon0_deg, refFrame.alt0_m,
                    0.0,  // start heading — we could get this from track
                    trackName,
                    nullptr, 0);
  if (!ok) return false;

  raceActive = true;
  havePrevFix = false;
  imuHead = 0;
  imuTail = 0;
  imuCount = 0;
  lastPosValid = false;
  lapDistanceM = 0.0f;
  lastTrackState = TRACK_STATE_IDLE;
  return true;
}

bool isRaceLoggerActive() {
  return raceActive && atpIsActive(atpSession);
}

void stopRaceLogger() {
  if (raceActive) {
    atpClose(atpSession);
  }
  raceActive = false;
}

const char *getRaceSessionFilename() {
  if (!raceActive || !atpIsActive(atpSession)) return nullptr;
  return atpSession.filename;
}

void pushImuSample(const ImuSample &sample) {
  if (!raceActive) return;
  pushSample(sample);
}

void onGpsFix(const GpsFix &fixIn) {
  if (!raceActive) return;
  if (!refFrame.valid) return;

  GpsFix fix = fixIn;
  if (fix.valid) {
    llaToEnu(fix.lat, fix.lon, fix.alt_m, fix.x_m, fix.y_m, fix.z_m);
    computeVelocity(fix.spd_kmh, fix.course_deg, fix.vn_mps, fix.ve_mps);
  } else {
    fix.x_m = NAN;
    fix.y_m = NAN;
    fix.z_m = NAN;
    fix.vn_mps = NAN;
    fix.ve_mps = NAN;
  }

  if (!havePrevFix) {
    prevFix = fix;
    havePrevFix = true;
    return;
  }

  uint32_t dtMs = fix.ms - prevFix.ms;
  bool interpOk = dtMs > 0 && dtMs <= MAX_INTERP_MS && prevFix.good && fix.good;

  ImuSample sample;
  while (peekSample(sample)) {
    if (sample.ms > fix.ms) break;
    popSample(sample);

    bool useRealFix = fix.valid && (sample.ms + REAL_FIX_WINDOW_MS >= fix.ms);
    uint8_t gpsSource = 0;
    double lat = NAN, lon = NAN, alt_m = NAN, spd_kmh = NAN;
    double course_deg = NAN, hdop = NAN;
    uint32_t sats = 0;
    double x_m = NAN, y_m = NAN, z_m = NAN;
    double vn_mps = NAN, ve_mps = NAN;
    uint16_t eventFlags = sample.eventFlags;

    if (interpOk) {
      double t = 0.0;
      if (sample.ms > prevFix.ms) {
        t = static_cast<double>(sample.ms - prevFix.ms) / dtMs;
      }
      t = constrain(t, 0.0, 1.0);
      x_m = prevFix.x_m + t * (fix.x_m - prevFix.x_m);
      y_m = prevFix.y_m + t * (fix.y_m - prevFix.y_m);
      z_m = prevFix.z_m + t * (fix.z_m - prevFix.z_m);
      vn_mps = prevFix.vn_mps + t * (fix.vn_mps - prevFix.vn_mps);
      ve_mps = prevFix.ve_mps + t * (fix.ve_mps - prevFix.ve_mps);
      enuToLla(x_m, y_m, z_m, lat, lon, alt_m);
      spd_kmh = sqrt(vn_mps * vn_mps + ve_mps * ve_mps) * 3.6;
      if (!isnan(vn_mps) && !isnan(ve_mps)) {
        course_deg = rad2deg(atan2(ve_mps, vn_mps));
        if (course_deg < 0) course_deg += 360.0;
      }
      hdop = prevFix.hdop;
      sats = prevFix.sats;
      gpsSource = 1;

      if (useRealFix) {
        lat = fix.lat;
        lon = fix.lon;
        alt_m = fix.alt_m;
        spd_kmh = fix.spd_kmh;
        course_deg = fix.course_deg;
        hdop = fix.hdop;
        sats = fix.sats;
        x_m = fix.x_m;
        y_m = fix.y_m;
        z_m = fix.z_m;
        vn_mps = fix.vn_mps;
        ve_mps = fix.ve_mps;
        gpsSource = 2;
      }
    } else {
      eventFlags |= RACE_EVENT_GPS_GAP;
      if (useRealFix && fix.valid) {
        lat = fix.lat;
        lon = fix.lon;
        alt_m = fix.alt_m;
        spd_kmh = fix.spd_kmh;
        course_deg = fix.course_deg;
        hdop = fix.hdop;
        sats = fix.sats;
        x_m = fix.x_m;
        y_m = fix.y_m;
        z_m = fix.z_m;
        vn_mps = fix.vn_mps;
        ve_mps = fix.ve_mps;
        gpsSource = 2;
      }
    }

    if (fix.degraded && gpsSource == 2) {
      eventFlags |= RACE_EVENT_GPS_DEGRADED;
    }
    if (!fix.valid) {
      eventFlags |= RACE_EVENT_GPS_INVALID;
    }

    writeRaceRow(sample, lat, lon, alt_m, spd_kmh, course_deg, hdop, sats,
                 x_m, y_m, z_m, vn_mps, ve_mps,
                 gpsSource, eventFlags);
  }

  prevFix = fix;
  havePrevFix = true;
}

void flushRaceLogger(uint32_t nowMs) {
  if (!raceActive || !havePrevFix) return;

  uint32_t cutoffMs = prevFix.ms + MAX_INTERP_MS;
  ImuSample sample;
  while (peekSample(sample)) {
    if (sample.ms > cutoffMs) break;
    popSample(sample);

    uint16_t eventFlags = sample.eventFlags | RACE_EVENT_GPS_GAP;
    writeRaceRow(sample,
                 NAN, NAN, NAN, NAN, NAN, NAN, 0,
                 NAN, NAN, NAN, NAN, NAN,
                 0, eventFlags);
  }

  atpFlushChunk(atpSession, false);
}
