#include <unity.h>
#include <cmath>
#include <cstring>

static constexpr double EARTH_RADIUS_M = 6371000.0;

static double deg2rad(double d) { return d * M_PI / 180.0; }

static double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
    double dLat = deg2rad(lat2 - lat1);
    double dLon = deg2rad(lon2 - lon1);
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return EARTH_RADIUS_M * c;
}

static double courseDelta(double a, double b) {
    double diff = fabs(a - b);
    if (diff > 180.0) diff = 360.0 - diff;
    return diff;
}

static uint32_t fnv1a(const uint8_t *data, size_t len) {
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        hash ^= data[i];
        hash *= 16777619u;
    }
    return hash;
}

static uint32_t computeTrackId(double lat, double lon, double alt) {
    int32_t lat_i = (int32_t)round(lat * 1e6);
    int32_t lon_i = (int32_t)round(lon * 1e6);
    int32_t alt_i = (int32_t)round(alt * 100.0);
    uint8_t buf[12];
    memcpy(buf + 0, &lat_i, 4);
    memcpy(buf + 4, &lon_i, 4);
    memcpy(buf + 8, &alt_i, 4);
    return fnv1a(buf, 12);
}

void setUp(void) {}
void tearDown(void) {}

void test_haversine_known_distance() {
    double d = distanceMeters(13.6929, -89.2182, 13.4833, -88.1833);
    TEST_ASSERT_FLOAT_WITHIN(5000.0, 114257.0, d);
}

void test_haversine_same_point() {
    double d = distanceMeters(13.778436, -89.191724, 13.778436, -89.191724);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, d);
}

void test_haversine_short_distance() {
    double d = distanceMeters(13.778436, -89.191724, 13.778700, -89.191724);
    TEST_ASSERT_FLOAT_WITHIN(5.0, 29.3, d);
}

void test_course_delta_simple() {
    TEST_ASSERT_FLOAT_WITHIN(0.01, 10.0, courseDelta(100.0, 110.0));
    TEST_ASSERT_FLOAT_WITHIN(0.01, 10.0, courseDelta(110.0, 100.0));
}

void test_course_delta_wraparound() {
    TEST_ASSERT_FLOAT_WITHIN(0.01, 20.0, courseDelta(350.0, 10.0));
    TEST_ASSERT_FLOAT_WITHIN(0.01, 20.0, courseDelta(10.0, 350.0));
}

void test_fnv1a_deterministic() {
    uint32_t id1 = computeTrackId(13.778436, -89.191724, 492.0);
    uint32_t id2 = computeTrackId(13.778436, -89.191724, 492.0);
    TEST_ASSERT_EQUAL_UINT32(id1, id2);
}

void test_fnv1a_different_locations() {
    uint32_t id1 = computeTrackId(13.778436, -89.191724, 492.0);
    uint32_t id2 = computeTrackId(44.344236, 11.713606, 47.0);
    TEST_ASSERT_NOT_EQUAL(id1, id2);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_haversine_known_distance);
    RUN_TEST(test_haversine_same_point);
    RUN_TEST(test_haversine_short_distance);
    RUN_TEST(test_course_delta_simple);
    RUN_TEST(test_course_delta_wraparound);
    RUN_TEST(test_fnv1a_deterministic);
    RUN_TEST(test_fnv1a_different_locations);
    UNITY_END();
    return 0;
}
