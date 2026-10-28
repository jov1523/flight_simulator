/*******************************************************************************
 * CESSNA 172 FLIGHT SIMULATOR — Educational Training Tool
 * ==========================================================================
 * A complete flight simulator with realistic aerodynamics, glass cockpit HUD,
 * instructor panel, and full keyboard controls.
 *
 * Built with: C++17, OpenGL (FreeGLUT + GLEW)
 * Target:     Cessna 172S Skyhawk
 *
 * Modules:
 *   1. Includes, Constants & Global State
 *   2. Vec3/Matrix Math Utilities
 *   3. Aerodynamics Engine (Physics)
 *   4. Terrain & Environment Rendering
 *   5. Aircraft Model Rendering
 *   6. Camera System
 *   7. HUD / Glass Cockpit (PFD)
 *   8. Controls, Instructor Panel & Main Loop
 ******************************************************************************/

// ============================================================================
// MODULE 1: INCLUDES, CONSTANTS & GLOBAL STATE
// ============================================================================

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <algorithm>
#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

// ---------- Math Constants ----------
static const double PI        = 3.14159265358979323846;
static const double DEG2RAD   = PI / 180.0;
static const double RAD2DEG   = 180.0 / PI;
static const double GRAVITY   = 9.80665;            // m/s^2

// Utility: clamp
inline double clampd(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Utility: lerp
inline double lerpd(double a, double b, double t) {
    return a + (b - a) * t;
}

// ---------- Cessna 172S Physical Constants ----------
// NACA 2412 airfoil, Lycoming IO-360-L2A 180hp engine
static const double WING_AREA       = 16.2;          // m^2
static const double WING_SPAN       = 11.0;          // m
static const double ASPECT_RATIO    = 7.52;          // b^2/S
static const double MEAN_CHORD      = 1.49;          // m
static const double MASS            = 1043.0;        // kg (typical loaded)
static const double ENGINE_POWER_W  = 134226.0;      // watts (180 hp)
static const double MAX_THRUST      = 3300.0;        // N (static thrust)
static const double CL_0            = 0.307;         // zero-AoA lift coefficient
static const double CL_ALPHA        = 5.7;           // lift slope (/rad)
static const double CL_MAX          = 1.56;          // max lift (stall, clean)
static const double STALL_AOA_DEG   = 16.0;          // degrees
static const double STALL_AOA_RAD   = STALL_AOA_DEG * DEG2RAD;
static const double CD_0            = 0.032;         // parasitic drag coefficient
static const double OSWALD_E        = 0.73;          // Oswald efficiency
static const double AIR_DENSITY     = 1.225;         // kg/m^3 (sea level ISA)
static const double IX              = 1285.0;        // kg*m^2 roll inertia
static const double IY              = 1825.0;        // kg*m^2 pitch inertia
static const double IZ              = 2667.0;        // kg*m^2 yaw inertia

// Stability derivatives
static const double CL_P            = -0.5;          // roll damping
static const double CM_Q            = -12.4;         // pitch damping
static const double CN_R            = -0.15;         // yaw damping
static const double CN_DA           = -0.003;        // adverse yaw due to aileron

// Speed limits (m/s)
static const double VR_SPEED        = 28.0;          // rotation speed (~55 kts)
static const double VNE_SPEED       = 82.0;          // never exceed (~160 kts)

// Physics timestep
static const double PHYSICS_DT      = 1.0 / 120.0;  // 120 Hz fixed step

// Runway dimensions
static const double RUNWAY_LENGTH   = 1500.0;        // m
static const double RUNWAY_WIDTH    = 30.0;           // m
static const double RUNWAY_X        = 0.0;            // centered on X
static const double RUNWAY_Z_START  = 0.0;            // start Z
static const double RUNWAY_Z_END    = RUNWAY_LENGTH;  // end Z

// ---------- Window State ----------
static int windowWidth  = 1280;
static int windowHeight = 720;

// ---------- Global Structs ----------

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

struct AircraftState {
    // Position (world frame, meters)
    double px, py, pz;
    // Velocity (world frame, m/s)
    double vx, vy, vz;
    // Euler angles (radians): pitch, roll, yaw
    double pitch, roll, yaw;
    // Angular rates (rad/s)
    double pitchRate, rollRate, yawRate;
    // Engine
    double throttle;      // 0.0 to 1.0
    bool   engineRunning;
    double propellerAngle; // visual only
    // State flags
    bool   onGround;
    bool   brakes;
    bool   stallWarning;
    double gLoad;          // current G-force
    // Airspeed and derived
    double airspeed;       // m/s (TAS)
    double aoa;            // angle of attack (rad)
    double sideslip;       // sideslip angle (rad)
    double verticalSpeed;  // m/s (positive = climb)

    // Flaps
    double flaps;          // current flaps angle in degrees (0, 10, 20, 30)
    double flapsTarget;    // target flaps angle in degrees

    // Autopilot
    bool   apActive;
    bool   apHdgHold;
    bool   apAltHold;
    double apTargetHdg;    // degrees (0-360)
    double apTargetAlt;    // feet

    // Control surface deflections (visual animation)
    double elevatorDeflection; // -1.0 to 1.0
    double aileronDeflection;  // -1.0 to 1.0
    double rudderDeflection;   // -1.0 to 1.0
};

struct EnvironmentState {
    double windX, windY, windZ;   // wind vector (m/s)
    double windSpeed;             // magnitude
    double windDirection;         // degrees (from)
    bool   fogEnabled;
    double fogDensity;
    bool   engineFailure;
    int    turbulenceLevel;       // 0=none, 1=low, 2=medium
};

struct SimConfig {
    int    cameraMode;         // 0=cockpit, 1=chase
    bool   paused;
    bool   hudVisible;
    bool   instructorPanelVisible;
    double simTime;
    int    fps;
    double fpsTimer;
    int    frameCount;
};

// ---------- Scenery State ----------
struct TreePos {
    double x, z;
    double height;
};
static const int NUM_TREES = 300;
static TreePos treePositions[NUM_TREES];

struct CloudPos {
    double x, y, z;
    double scaleX, scaleY, scaleZ;
};
static const int NUM_CLOUDS = 20;
static CloudPos cloudPositions[NUM_CLOUDS];

// ---------- Global Instances ----------
static AircraftState   aircraft;
static EnvironmentState environment;
static SimConfig       config;

// Keyboard state array
static bool keyStates[256]    = {false};
static bool specialKeys[256]  = {false};

// Timing
static double lastTime     = 0.0;
static double accumulator  = 0.0;

// Camera smoothing (chase cam)
static double camX = 0, camY = 5, camZ = -15;

// Helper: random range
inline double randRange(double lo, double hi) {
    return lo + (hi - lo) * ((double)rand() / RAND_MAX);
}

// ---------- Terrain & Rivers ----------
inline bool isRiver(double x, double z, double &riverDepth) {
    // Keep it away from the runway! (Runway is at X=0, Z=0..1500)
    if (fabs(x) < 250.0 && z > -500.0 && z < 2000.0) return false;

    // Curved river path running along X
    double riverCenter = 600.0 + 800.0 * sin(z * 0.0004);
    double dist = fabs(x - riverCenter);
    double riverWidth = 70.0; // wider river
    if (dist < riverWidth) {
        riverDepth = (riverWidth - dist) / riverWidth; // 1 at center, 0 at edge
        return true;
    }
    return false;
}

inline double getTerrainHeight(double x, double z) {
    // Runway is centered at X=0, Z from 0 to RUNWAY_LENGTH (1500m).
    // Flat zone around the runway: within X = -180 to 180, and Z = -300 to 1800.
    double flatRadiusX = 180.0;
    double flatRadiusZStart = -300.0;
    double flatRadiusZEnd = RUNWAY_LENGTH + 300.0;
    
    // Smooth transition factor
    double dX = 0.0;
    if (fabs(x) < flatRadiusX) {
        dX = (flatRadiusX - fabs(x)) / flatRadiusX; // 1 at center, 0 at edge
    }
    double dZ = 0.0;
    if (z > flatRadiusZStart && z < flatRadiusZEnd) {
        double distStart = z - flatRadiusZStart;
        double distEnd = flatRadiusZEnd - z;
        double minDist = std::min(distStart, distEnd);
        double transitionZone = 300.0;
        if (minDist < transitionZone) {
            dZ = minDist / transitionZone;
        } else {
            dZ = 1.0;
        }
    }
    double runwayFactor = dX * dZ; // 1.0 near runway, 0.0 far away
    
    // Procedural terrain height: sum of sine/cosine waves
    // Larger scale mountains
    double h = 280.0 * sin(x * 0.00025) * cos(z * 0.0002);
    // Medium hills
    h += 70.0 * sin(x * 0.0008 + 1.0) * sin(z * 0.0007 + 2.0);
    // Small features
    h += 15.0 * cos(x * 0.004) * sin(z * 0.0035);
    
    h = std::max(0.0, h);
    
    // River carving
    double rDepth = 0.0;
    if (isRiver(x, z, rDepth)) {
        // Blend down to water level (near 0)
        double waterLevel = 1.0;
        h = lerpd(h, waterLevel, rDepth * 0.95);
    }
    
    return h * (1.0 - runwayFactor);
}

// ---------- Initialization ----------
void initAircraft() {
    memset(&aircraft, 0, sizeof(aircraft));
    aircraft.px = 0.0;
    aircraft.py = 0.0;          // on the ground
    aircraft.pz = 50.0;        // near runway threshold
    aircraft.yaw = 0.0;        // facing down runway (positive Z)
    aircraft.throttle = 0.0;
    aircraft.engineRunning = true;
    aircraft.onGround = true;
    aircraft.brakes = true;
    aircraft.gLoad = 1.0;
    aircraft.propellerAngle = 0.0;
    aircraft.flaps = 0.0;
    aircraft.flapsTarget = 0.0;
    aircraft.apActive = false;
    aircraft.apHdgHold = false;
    aircraft.apAltHold = false;
    aircraft.apTargetHdg = 0.0;
    aircraft.apTargetAlt = 0.0;
    aircraft.elevatorDeflection = 0.0;
    aircraft.aileronDeflection = 0.0;
    aircraft.rudderDeflection = 0.0;
}

void initTrees() {
    srand(12345);
    for (int i = 0; i < NUM_TREES; i++) {
        double tx, tz;
        double rDepth;
        bool valid = false;
        while (!valid) {
            tx = randRange(-3000.0, 3000.0);
            tz = randRange(-3000.0, 4500.0);
            
            // Avoid runway flat zone
            bool nearRunway = (fabs(tx) < 180.0 && tz > -300.0 && tz < 1800.0);
            // Avoid river
            bool inRiver = isRiver(tx, tz, rDepth);
            
            if (!nearRunway && !inRiver) {
                valid = true;
            }
        }
        treePositions[i].x = tx;
        treePositions[i].z = tz;
        treePositions[i].height = randRange(8.0, 18.0);
    }
}

void initClouds() {
    srand(54321);
    for (int i = 0; i < NUM_CLOUDS; i++) {
        cloudPositions[i].x = randRange(-4000.0, 4000.0);
        cloudPositions[i].y = randRange(400.0, 700.0);
        cloudPositions[i].z = randRange(-4000.0, 6000.0);
        cloudPositions[i].scaleX = randRange(150.0, 350.0);
        cloudPositions[i].scaleY = randRange(30.0, 70.0);
        cloudPositions[i].scaleZ = randRange(150.0, 350.0);
    }
}

void initEnvironment() {
    memset(&environment, 0, sizeof(environment));
    environment.fogDensity = 0.002;
    environment.turbulenceLevel = 0;
    initTrees();
    initClouds();
}

void initConfig() {
    memset(&config, 0, sizeof(config));
    config.cameraMode = 1;           // start in chase view
    config.hudVisible = true;
    config.instructorPanelVisible = false;
    config.paused = false;
}


// ============================================================================
// MODULE 2: VEC3 / MATRIX MATH UTILITIES
// ============================================================================

// ----- Vec3 operations -----
inline Vec3 vec3Add(const Vec3& a, const Vec3& b) {
    return Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline Vec3 vec3Sub(const Vec3& a, const Vec3& b) {
    return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline Vec3 vec3Scale(const Vec3& v, double s) {
    return Vec3(v.x * s, v.y * s, v.z * s);
}
inline double vec3Dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Vec3 vec3Cross(const Vec3& a, const Vec3& b) {
    return Vec3(a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x);
}
inline double vec3Mag(const Vec3& v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
inline Vec3 vec3Normalize(const Vec3& v) {
    double m = vec3Mag(v);
    if (m < 1e-12) return Vec3(0, 0, 0);
    return Vec3(v.x / m, v.y / m, v.z / m);
}

// ----- Rotation helpers -----
// Rotate a vector from BODY frame to WORLD frame using Euler angles (yaw, pitch, roll)
// Convention: Rz(yaw) * Rx(pitch) * Ry(roll) — but we use Aerospace convention:
//   World X = East, Y = Up, Z = North (runway direction)
//   Aircraft heading = yaw around Y, pitch = rotation about body X, roll = rotation about body Z

Vec3 bodyToWorld(const Vec3& bodyVec, double pitch, double roll, double yaw) {
    // Build rotation matrix: R = Ry(yaw) * Rx(pitch) * Rz(roll)
    double cy = cos(yaw),   sy = sin(yaw);
    double cp = cos(pitch),  sp = sin(pitch);
    double cr = cos(roll),   sr = sin(roll);

    Vec3 result;
    // Row 0 of combined rotation matrix
    result.x = (cy * cr + sy * sp * sr) * bodyVec.x
             + (sy * sp * cr - cy * sr) * bodyVec.y
             + (sy * cp)                * bodyVec.z;
    // Row 1 (up direction)
    result.y = (cp * sr)                * bodyVec.x
             + (cp * cr)                * bodyVec.y
             + (-sp)                    * bodyVec.z;
    // Row 2
    result.z = (cy * sp * sr - sy * cr) * bodyVec.x
             + (sy * sr + cy * sp * cr) * bodyVec.y
             + (cy * cp)                * bodyVec.z;

    return result;
}

Vec3 worldToBody(const Vec3& worldVec, double pitch, double roll, double yaw) {
    // Transpose of the bodyToWorld rotation matrix
    double cy = cos(yaw),   sy = sin(yaw);
    double cp = cos(pitch),  sp = sin(pitch);
    double cr = cos(roll),   sr = sin(roll);

    Vec3 result;
    result.x = (cy * cr + sy * sp * sr) * worldVec.x
             + (cp * sr)                * worldVec.y
             + (cy * sp * sr - sy * cr) * worldVec.z;
    result.y = (sy * sp * cr - cy * sr) * worldVec.x
             + (cp * cr)                * worldVec.y
             + (sy * sr + cy * sp * cr) * worldVec.z;
    result.z = (sy * cp)                * worldVec.x
             + (-sp)                    * worldVec.y
             + (cy * cp)                * worldVec.z;

    return result;
}

// Get forward, up, and right vectors in world frame
Vec3 getForwardVec() { return bodyToWorld(Vec3(0, 0, 1), aircraft.pitch, aircraft.roll, aircraft.yaw); }
Vec3 getUpVec()      { return bodyToWorld(Vec3(0, 1, 0), aircraft.pitch, aircraft.roll, aircraft.yaw); }
Vec3 getRightVec()   { return bodyToWorld(Vec3(1, 0, 0), aircraft.pitch, aircraft.roll, aircraft.yaw); }

// clampd and lerpd relocated to the top of the file




// ============================================================================
// MODULE 3: AERODYNAMICS ENGINE (Physics)
// ============================================================================

void updatePhysics(double dt) {
    if (config.paused) return;

    // Transit flaps smoothly
    double flapSpeed = 10.0; // degrees per second
    if (aircraft.flaps < aircraft.flapsTarget) {
        aircraft.flaps = std::min(aircraft.flapsTarget, aircraft.flaps + flapSpeed * dt);
    } else if (aircraft.flaps > aircraft.flapsTarget) {
        aircraft.flaps = std::max(aircraft.flapsTarget, aircraft.flaps - flapSpeed * dt);
    }

    // ---- Process control inputs from keyboard state ----
    double pitchInput = 0.0;
    double rollInput  = 0.0;
    double yawInput   = 0.0;

    if (keyStates['w'] || keyStates['W']) pitchInput -= 1.0;  // nose down
    if (keyStates['s'] || keyStates['S']) pitchInput += 1.0;  // nose up
    if (keyStates['a'] || keyStates['A']) rollInput  -= 1.0;  // roll left
    if (keyStates['d'] || keyStates['D']) rollInput  += 1.0;  // roll right
    if (keyStates['q'] || keyStates['Q']) yawInput   -= 1.0;  // yaw left (left rudder)
    if (keyStates['e'] || keyStates['E']) yawInput   += 1.0;  // yaw right (right rudder)

    // Autopilot overrides
    bool manualPitch = (keyStates['w'] || keyStates['W'] || keyStates['s'] || keyStates['S']);
    bool manualRoll = (keyStates['a'] || keyStates['A'] || keyStates['d'] || keyStates['D']);
    bool manualYaw = (keyStates['q'] || keyStates['Q'] || keyStates['e'] || keyStates['E']);

    if (aircraft.apActive) {
        if (aircraft.apHdgHold && !manualRoll) {
            double hdgError = aircraft.apTargetHdg - (aircraft.yaw * RAD2DEG);
            while (hdgError > 180.0) hdgError -= 360.0;
            while (hdgError < -180.0) hdgError += 360.0;
            // Target bank proportional to heading error
            double targetRoll = clampd(hdgError * 1.5, -25.0, 25.0) * DEG2RAD;
            rollInput = clampd((targetRoll - aircraft.roll) * 2.0, -1.0, 1.0);
            
            if (!manualYaw) {
                // Coordinate rudder to counter sideslip
                yawInput = clampd(-aircraft.sideslip * 3.0, -0.5, 0.5);
            }
        }
        if (aircraft.apAltHold && !manualPitch) {
            double currentAltFt = aircraft.py * 3.28084;
            double altError = aircraft.apTargetAlt - currentAltFt;
            // Target vertical speed proportional to altitude error
            double targetVS_fpm = clampd(altError * 5.0, -800.0, 800.0);
            double targetVS_mps = targetVS_fpm / 196.85;
            double vsError = targetVS_mps - aircraft.verticalSpeed;
            // Target pitch proportional to VS error
            double targetPitch = clampd(vsError * 0.1, -10.0, 15.0) * DEG2RAD;
            pitchInput = clampd((targetPitch - aircraft.pitch) * 2.0, -1.0, 1.0);
        }
    }

    // Throttle (gradual)
    // Using number keys 'z' and 'x' as alternatives if shift/ctrl don't register
    bool throttleUp   = keyStates['+'] || specialKeys[GLUT_KEY_UP];
    bool throttleDown = keyStates['-'] || specialKeys[GLUT_KEY_DOWN];

    // Check for shift/ctrl via glutGetModifiers in the keyboard callback
    // We store them in special array positions
    if (keyStates[1])  throttleUp = true;    // 1 = shift flag set in callback
    if (keyStates[2])  throttleDown = true;  // 2 = ctrl flag set in callback

    if (throttleUp)   aircraft.throttle = std::min(1.0, aircraft.throttle + 0.4 * dt);
    if (throttleDown) aircraft.throttle = std::max(0.0, aircraft.throttle - 0.4 * dt);

    // ---- Engine failure override ----
    double effectiveThrottle = aircraft.throttle;
    if (environment.engineFailure || !aircraft.engineRunning) {
        effectiveThrottle = 0.0;
    }

    // ---- Get body frame vectors in world ----
    Vec3 fwd   = getForwardVec();
    Vec3 up    = getUpVec();
    Vec3 right = getRightVec();

    // ---- Relative airspeed (accounting for wind) ----
    Vec3 windVec(environment.windX, environment.windY, environment.windZ);
    Vec3 velWorld(aircraft.vx, aircraft.vy, aircraft.vz);
    Vec3 relVel = vec3Sub(velWorld, windVec);
    double airspeed = vec3Mag(relVel);
    aircraft.airspeed = airspeed;

    // ---- Angle of Attack ----
    Vec3 relVelBody = worldToBody(relVel, aircraft.pitch, aircraft.roll, aircraft.yaw);
    double alpha = 0.0;
    if (fabs(relVelBody.z) > 0.1) {
        alpha = atan2(-relVelBody.y, relVelBody.z);
    }
    aircraft.aoa = alpha;

    // Sideslip angle
    if (fabs(relVelBody.z) > 0.1) {
        aircraft.sideslip = atan2(relVelBody.x, relVelBody.z);
    } else {
        aircraft.sideslip = 0.0;
    }

    // ---- Compute aerodynamic forces ----

    double qbar = 0.5 * AIR_DENSITY * airspeed * airspeed;  // dynamic pressure

    // Flaps lift and drag increments
    double delta_CL_flaps = 0.15 * (aircraft.flaps / 10.0);
    double delta_CD_flaps = 0.015 * (aircraft.flaps / 10.0) + 0.01 * (aircraft.flaps / 10.0) * (aircraft.flaps / 10.0);

    // -- Lift --
    double CL;
    if (fabs(alpha) < STALL_AOA_RAD) {
        CL = CL_0 + CL_ALPHA * alpha + delta_CL_flaps;
    } else {
        // Post-stall: sharp drop-off
        double excessAoA = fabs(alpha) - STALL_AOA_RAD;
        double CLat_stall = CL_0 + CL_ALPHA * STALL_AOA_RAD + delta_CL_flaps;
        CL = CLat_stall * (1.0 - 3.0 * excessAoA * excessAoA);
        if (CL < 0.1) CL = 0.1;  // residual lift
        if (alpha < 0) CL = -CL;
    }

    // Stall warning
    aircraft.stallWarning = (fabs(alpha) > STALL_AOA_RAD * 0.85 && airspeed > 5.0);

    double liftMag = qbar * WING_AREA * CL;
    Vec3 liftForce = vec3Scale(up, liftMag);

    // -- Drag --
    double CDi = (CL * CL) / (PI * ASPECT_RATIO * OSWALD_E);  // induced drag
    double CD  = CD_0 + CDi + delta_CD_flaps;

    // Ground effect: reduce induced drag when close to ground
    double heightAboveGround = aircraft.py - getTerrainHeight(aircraft.px, aircraft.pz);
    if (heightAboveGround < WING_SPAN && heightAboveGround >= 0) {
        double phi_ge = 1.0 - (0.5 * WING_SPAN) / (WING_SPAN + 2.0 * heightAboveGround);
        CD = CD_0 + CDi * phi_ge;  // reduced induced drag in ground effect
    }

    double dragMag = qbar * WING_AREA * CD;
    Vec3 dragDir = (airspeed > 0.5) ? vec3Scale(vec3Normalize(relVel), -1.0) : Vec3(0, 0, 0);
    Vec3 dragForce = vec3Scale(dragDir, dragMag);

    // -- Thrust --
    // Power-based model: thrust decreases with airspeed
    double thrustMag = effectiveThrottle * MAX_THRUST;
    if (airspeed > 1.0) {
        thrustMag *= std::max(0.3, 1.0 - 0.4 * (airspeed / VNE_SPEED));
    }
    Vec3 thrustForce = vec3Scale(fwd, thrustMag);

    // -- Weight --
    Vec3 weightForce(0.0, -MASS * GRAVITY, 0.0);

    // ---- Sum forces ----
    Vec3 netForce = vec3Add(vec3Add(vec3Add(thrustForce, weightForce), liftForce), dragForce);

    // ---- G-load calculation ----
    Vec3 accelWorld = vec3Scale(netForce, 1.0 / MASS);
    Vec3 accelBody = worldToBody(accelWorld, aircraft.pitch, aircraft.roll, aircraft.yaw);
    aircraft.gLoad = (accelBody.y + GRAVITY) / GRAVITY;

    // ---- Angular rates (control response) ----
    // Control authority scales with dynamic pressure
    double controlAuthority = clampd(qbar / 500.0, 0.05, 1.0);

    // Target angular rates from input
    double targetPitchRate = pitchInput * 1.2 * controlAuthority;  // rad/s max
    double targetRollRate  = rollInput  * 2.0 * controlAuthority;
    double targetYawRate   = yawInput   * 0.8 * controlAuthority;

    // Damping (stability derivatives)
    double rollDamping  = CL_P * (aircraft.rollRate * WING_SPAN / (2.0 * std::max(airspeed, 1.0)));
    double pitchDamping = CM_Q * (aircraft.pitchRate * MEAN_CHORD / (2.0 * std::max(airspeed, 1.0)));
    double yawDamping   = CN_R * (aircraft.yawRate * WING_SPAN / (2.0 * std::max(airspeed, 1.0)));

    // Adverse yaw: rolling creates opposing yaw
    double adverseYaw = CN_DA * rollInput * controlAuthority * qbar * WING_AREA * WING_SPAN / IZ;

    // Smooth rate changes
    double rollAccel  = (targetRollRate - aircraft.rollRate) * 4.0
                      + rollDamping * qbar * WING_AREA * WING_SPAN / IX;
    double pitchAccel = (targetPitchRate - aircraft.pitchRate) * 3.0
                      + pitchDamping * qbar * WING_AREA * MEAN_CHORD / IY;
    double yawAccel   = (targetYawRate - aircraft.yawRate) * 3.0
                      + yawDamping * qbar * WING_AREA * WING_SPAN / IZ
                      + adverseYaw;

    aircraft.rollRate  += rollAccel * dt;
    aircraft.pitchRate += pitchAccel * dt;
    aircraft.yawRate   += yawAccel * dt;

    // Turbulence
    if (environment.turbulenceLevel > 0 && !aircraft.onGround) {
        double turbStr = environment.turbulenceLevel * 0.015;
        aircraft.pitchRate += randRange(-turbStr, turbStr);
        aircraft.rollRate  += randRange(-turbStr * 1.5, turbStr * 1.5);
        aircraft.yawRate   += randRange(-turbStr * 0.5, turbStr * 0.5);
    }

    // Integrate angles
    aircraft.pitch += aircraft.pitchRate * dt;
    aircraft.roll  += aircraft.rollRate * dt;
    aircraft.yaw   += aircraft.yawRate * dt;

    // Clamp pitch to avoid extreme values
    aircraft.pitch = clampd(aircraft.pitch, -85.0 * DEG2RAD, 85.0 * DEG2RAD);

    // Normalize yaw to [0, 2*PI)
    while (aircraft.yaw < 0)       aircraft.yaw += 2.0 * PI;
    while (aircraft.yaw >= 2.0 * PI) aircraft.yaw -= 2.0 * PI;

    // ---- Integrate velocity ----
    Vec3 accel = vec3Scale(netForce, 1.0 / MASS);
    aircraft.vx += accel.x * dt;
    aircraft.vy += accel.y * dt;
    aircraft.vz += accel.z * dt;

    // ---- Ground interaction ----
    double terrainH = getTerrainHeight(aircraft.px, aircraft.pz);
    if (aircraft.py <= terrainH) {
        aircraft.onGround = true;
        aircraft.py = terrainH;

        // Ground prevents sinking
        if (aircraft.vy < 0.0) {
            // Landing detection
            if (aircraft.vy < -3.0) {
                // Hard landing! Could add crash logic
            }
            aircraft.vy = 0.0;
        }

        // Ground friction
        if (aircraft.brakes) {
            double groundSpeed = sqrt(aircraft.vx * aircraft.vx + aircraft.vz * aircraft.vz);
            if (groundSpeed > 0.1) {
                double frictionForce = 0.04 * MASS * GRAVITY; // rolling + brake friction
                double decel = frictionForce / MASS;
                double ratio = std::max(0.0, 1.0 - decel * dt / groundSpeed);
                aircraft.vx *= ratio;
                aircraft.vz *= ratio;
            } else {
                aircraft.vx = 0.0;
                aircraft.vz = 0.0;
            }
        } else {
            // Rolling friction only
            double groundSpeed = sqrt(aircraft.vx * aircraft.vx + aircraft.vz * aircraft.vz);
            if (groundSpeed > 0.1) {
                double frictionForce = 0.015 * MASS * GRAVITY;
                double decel = frictionForce / MASS;
                double ratio = std::max(0.0, 1.0 - decel * dt / groundSpeed);
                aircraft.vx *= ratio;
                aircraft.vz *= ratio;
            }
        }

        // Prevent rolling/banking on ground
        aircraft.roll = aircraft.roll * 0.95; // decay toward level
        aircraft.rollRate *= 0.9;

        // Nose wheel steering: yaw follows rudder input on ground
        if (sqrt(aircraft.vx * aircraft.vx + aircraft.vz * aircraft.vz) > 1.0) {
            aircraft.yawRate = yawInput * 0.5;
        }

        // Check for takeoff
        if (airspeed > VR_SPEED && aircraft.pitch > 5.0 * DEG2RAD) {
            aircraft.onGround = false;
        }
    } else {
        aircraft.onGround = false;
    }

    // ---- Integrate position ----
    aircraft.px += aircraft.vx * dt;
    aircraft.py += aircraft.vy * dt;
    aircraft.pz += aircraft.vz * dt;

    // Prevent going underground
    double finalTerrainH = getTerrainHeight(aircraft.px, aircraft.pz);
    if (aircraft.py < finalTerrainH) {
        aircraft.py = finalTerrainH;
        if (aircraft.vy < 0.0) aircraft.vy = 0.0;
    }

    // ---- Derived values ----
    aircraft.verticalSpeed = aircraft.vy;

    // Propeller visual
    aircraft.propellerAngle += effectiveThrottle * 50.0 * dt;
    if (aircraft.propellerAngle > 2.0 * PI) aircraft.propellerAngle -= 2.0 * PI;

    // Save control deflections for visual model
    aircraft.elevatorDeflection = pitchInput;
    aircraft.aileronDeflection = rollInput;
    aircraft.rudderDeflection = yawInput;

    // Update sim time
    config.simTime += dt;
}


// ============================================================================
// MODULE 4: TERRAIN & ENVIRONMENT RENDERING
// ============================================================================

// Helper: draw text at 3D position
void drawText3D(double x, double y, double z, const char* text, void* font = GLUT_BITMAP_HELVETICA_12) {
    glRasterPos3d(x, y, z);
    for (const char* c = text; *c; c++) {
        glutBitmapCharacter(font, *c);
    }
}

// Draw the sky dome (large colored quad behind everything)
void renderSky() {
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    // Save current matrices
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-1, 1, -1, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Sky gradient quad
    glBegin(GL_QUADS);
    // Top: deep blue
    glColor3f(0.15f, 0.30f, 0.70f);
    glVertex2f(-1.0f, 1.0f);
    glVertex2f(1.0f, 1.0f);
    // Bottom: light blue/white horizon
    glColor3f(0.60f, 0.78f, 0.95f);
    glVertex2f(1.0f, -0.1f);
    glVertex2f(-1.0f, -0.1f);
    glEnd();

    // Below horizon: ground color fade
    glBegin(GL_QUADS);
    glColor3f(0.60f, 0.78f, 0.95f);
    glVertex2f(-1.0f, -0.1f);
    glVertex2f(1.0f, -0.1f);
    glColor3f(0.35f, 0.55f, 0.25f);
    glVertex2f(1.0f, -1.0f);
    glVertex2f(-1.0f, -1.0f);
    glEnd();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glEnable(GL_DEPTH_TEST);
}

// Draw the ground plane with grid
void setVertexColor(double x, double y, double z) {
    double rDepth = 0.0;
    if (isRiver(x, z, rDepth)) {
        // Deep blue to light blue/turquoise based on depth
        glColor3f(0.1f + 0.1f * (1.0 - rDepth), 0.4f + 0.2f * (1.0 - rDepth), 0.8f);
        return;
    }
    
    // Color based on height y
    if (y < 12.0) {
        // Lush green valley
        glColor3f(0.28f, 0.55f, 0.22f);
    } else if (y < 70.0) {
        // Forested hills - darker green
        double t = (y - 12.0) / (70.0 - 12.0);
        glColor3f(lerpd(0.28, 0.18, t), lerpd(0.55, 0.38, t), lerpd(0.22, 0.15, t));
    } else if (y < 180.0) {
        // Mountain slopes - brown/gray rock
        double t = (y - 70.0) / (180.0 - 70.0);
        glColor3f(lerpd(0.18, 0.45, t), lerpd(0.38, 0.45, t), lerpd(0.15, 0.45, t));
    } else {
        // High peaks - snow capped white
        double t = std::min(1.0, (y - 180.0) / 100.0);
        glColor3f(lerpd(0.45, 0.95, t), lerpd(0.45, 0.95, t), lerpd(0.45, 0.95, t));
    }
}

void renderGround() {
    double cellSize = 120.0; // 120m per grid cell
    int gridRadius = 45;     // 45 cells in each direction (90x90 grid -> ~10.8km span)

    // Center the grid around the aircraft's position (rounded to cell size)
    double centerValX = round(aircraft.px / cellSize) * cellSize;
    double centerValZ = round(aircraft.pz / cellSize) * cellSize;

    glDisable(GL_LIGHTING);
    glBegin(GL_QUADS);
    for (int i = -gridRadius; i < gridRadius; i++) {
        double x1 = centerValX + i * cellSize;
        double x2 = x1 + cellSize;
        for (int j = -gridRadius; j < gridRadius; j++) {
            double z1 = centerValZ + j * cellSize;
            double z2 = z1 + cellSize;

            // Four corners heights
            double y11 = getTerrainHeight(x1, z1);
            double y21 = getTerrainHeight(x2, z1);
            double y22 = getTerrainHeight(x2, z2);
            double y12 = getTerrainHeight(x1, z2);

            // Draw quad
            setVertexColor(x1, y11, z1); glVertex3d(x1, y11, z1);
            setVertexColor(x2, y21, z1); glVertex3d(x2, y21, z1);
            setVertexColor(x2, y22, z2); glVertex3d(x2, y22, z2);
            setVertexColor(x1, y12, z2); glVertex3d(x1, y12, z2);
        }
    }
    glEnd();
    glEnable(GL_LIGHTING);
}

// Draw the runway with markings
void renderRunway() {
    double hw = RUNWAY_WIDTH / 2.0;

    // Runway surface (dark gray asphalt)
    glColor3f(0.25f, 0.25f, 0.27f);
    glBegin(GL_QUADS);
    glVertex3d(-hw, 0.01, RUNWAY_Z_START);
    glVertex3d( hw, 0.01, RUNWAY_Z_START);
    glVertex3d( hw, 0.01, RUNWAY_Z_END);
    glVertex3d(-hw, 0.01, RUNWAY_Z_END);
    glEnd();

    // Runway edge lines (white)
    glColor3f(0.9f, 0.9f, 0.9f);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    // Left edge
    glVertex3d(-hw + 0.5, 0.02, RUNWAY_Z_START);
    glVertex3d(-hw + 0.5, 0.02, RUNWAY_Z_END);
    // Right edge
    glVertex3d(hw - 0.5, 0.02, RUNWAY_Z_START);
    glVertex3d(hw - 0.5, 0.02, RUNWAY_Z_END);
    glEnd();

    // Centerline dashes (every 50m, 30m long, 0.5m wide)
    glColor3f(0.95f, 0.95f, 0.95f);
    for (double z = RUNWAY_Z_START + 50.0; z < RUNWAY_Z_END - 50.0; z += 50.0) {
        glBegin(GL_QUADS);
        glVertex3d(-0.25, 0.02, z);
        glVertex3d( 0.25, 0.02, z);
        glVertex3d( 0.25, 0.02, z + 30.0);
        glVertex3d(-0.25, 0.02, z + 30.0);
        glEnd();
    }

    // Threshold markings ("piano keys") at start
    glColor3f(0.95f, 0.95f, 0.95f);
    for (int i = -5; i <= 5; i++) {
        if (i == 0) continue; // skip center
        double stripeX = i * 2.0;
        // Start threshold
        glBegin(GL_QUADS);
        glVertex3d(stripeX - 0.6, 0.02, RUNWAY_Z_START + 5.0);
        glVertex3d(stripeX + 0.6, 0.02, RUNWAY_Z_START + 5.0);
        glVertex3d(stripeX + 0.6, 0.02, RUNWAY_Z_START + 35.0);
        glVertex3d(stripeX - 0.6, 0.02, RUNWAY_Z_START + 35.0);
        glEnd();
        // End threshold
        glBegin(GL_QUADS);
        glVertex3d(stripeX - 0.6, 0.02, RUNWAY_Z_END - 35.0);
        glVertex3d(stripeX + 0.6, 0.02, RUNWAY_Z_END - 35.0);
        glVertex3d(stripeX + 0.6, 0.02, RUNWAY_Z_END - 5.0);
        glVertex3d(stripeX - 0.6, 0.02, RUNWAY_Z_END - 5.0);
        glEnd();
    }

    // PAPI lights (left side, 4 lights)
    for (int i = 0; i < 4; i++) {
        double lx = -hw - 5.0;
        double lz = RUNWAY_Z_START + 300.0 + i * 3.0;

        // Determine red/white based on glide slope
        // 3-degree glideslope from touchdown point
        double distToTdz = aircraft.pz - (RUNWAY_Z_START + 300.0);
        double idealAlt = -distToTdz * tan(3.0 * DEG2RAD);  // positive when approaching
        double altError = aircraft.py - idealAlt;

        // Each PAPI light transitions at different angles
        double thresholds[] = {2.5, 2.8, 3.2, 3.5};
        double actualAngle = 0.0;
        if (fabs(distToTdz) > 10.0) {
            actualAngle = atan2(aircraft.py, fabs(distToTdz)) * RAD2DEG;
        }

        if (actualAngle > thresholds[i]) {
            glColor3f(1.0f, 1.0f, 1.0f); // white = too high
        } else {
            glColor3f(1.0f, 0.0f, 0.0f); // red = too low
        }

        glPushMatrix();
        glTranslated(lx, 0.5, lz);
        glutSolidSphere(0.4, 8, 8);
        glPopMatrix();
    }

    // Runway edge lights (small yellow/orange points)
    glColor3f(0.95f, 0.80f, 0.2f);
    glPointSize(4.0f);
    glBegin(GL_POINTS);
    for (double z = RUNWAY_Z_START; z <= RUNWAY_Z_END; z += 30.0) {
        glVertex3d(-hw, 0.3, z);
        glVertex3d( hw, 0.3, z);
    }
    glEnd();

    // Approach lighting (green threshold lights)
    glColor3f(0.0f, 1.0f, 0.0f);
    glPointSize(5.0f);
    glBegin(GL_POINTS);
    for (int i = -5; i <= 5; i++) {
        glVertex3d(i * 2.5, 0.3, RUNWAY_Z_START);
    }
    glEnd();

    // End of runway lights (red)
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);
    for (int i = -5; i <= 5; i++) {
        glVertex3d(i * 2.5, 0.3, RUNWAY_Z_END);
    }
    glEnd();
}

// Windsock indicator near runway
void renderWindsock() {
    if (environment.windSpeed < 0.1) return;

    double sockX = RUNWAY_WIDTH / 2.0 + 15.0;
    double sockZ = RUNWAY_Z_START + 200.0;

    glPushMatrix();
    glTranslated(sockX, 0.0, sockZ);

    // Pole
    glColor3f(0.6f, 0.6f, 0.6f);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glVertex3d(0, 0, 0);
    glVertex3d(0, 5.0, 0);
    glEnd();

    // Sock (points in wind direction)
    glTranslated(0, 5.0, 0);
    double windAngle = atan2(environment.windX, environment.windZ);
    double sockExtension = clampd(environment.windSpeed / 15.0, 0.1, 1.0);

    glRotated(windAngle * RAD2DEG, 0, 1, 0);

    // Draw sock as tapered cone shape
    glBegin(GL_TRIANGLE_FAN);
    glColor3f(1.0f, 0.5f, 0.0f); // orange
    glVertex3d(0, 0, 0);
    double sockLen = 3.0 * sockExtension;
    glColor3f(1.0f, 1.0f, 1.0f); // white tip
    for (int i = 0; i <= 12; i++) {
        double a = i * 2.0 * PI / 12.0;
        glVertex3d(sin(a) * 0.2, cos(a) * 0.2 - 0.5 * sockExtension, sockLen);
    }
    glEnd();

    glPopMatrix();
}

void renderTrees() {
    glDisable(GL_LIGHTING);
    for (int i = 0; i < NUM_TREES; i++) {
        double tx = treePositions[i].x;
        double tz = treePositions[i].z;
        double th = treePositions[i].height;
        double ty = getTerrainHeight(tx, tz);
        
        glPushMatrix();
        glTranslated(tx, ty, tz);
        
        // Trunk
        glColor3f(0.4f, 0.25f, 0.15f); // brown
        glPushMatrix();
        glTranslated(0, th * 0.25, 0);
        glScaled(th * 0.1, th * 0.5, th * 0.1);
        glutSolidCube(1.0);
        glPopMatrix();
        
        // Foliage (cone)
        glColor3f(0.15f, 0.45f, 0.2f); // forest green
        glPushMatrix();
        glTranslated(0, th * 0.4, 0);
        glRotated(-90.0, 1.0, 0.0, 0.0);
        glutSolidCone(th * 0.25, th * 0.8, 8, 2);
        glPopMatrix();
        
        glPopMatrix();
    }
    glEnable(GL_LIGHTING);
}

void renderClouds() {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_LIGHTING);
    
    for (int i = 0; i < NUM_CLOUDS; i++) {
        glPushMatrix();
        glTranslated(cloudPositions[i].x, cloudPositions[i].y, cloudPositions[i].z);
        glScaled(cloudPositions[i].scaleX, cloudPositions[i].scaleY, cloudPositions[i].scaleZ);
        
        // Draw soft cloud (semi-transparent sphere)
        glColor4f(0.95f, 0.95f, 0.95f, 0.3f);
        glutSolidSphere(0.5, 8, 8);
        glPopMatrix();
    }
    
    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);
}

void renderAirportBuildings() {
    glDisable(GL_LIGHTING);
    
    // --- CONTROL TOWER ---
    double towerX = 40.0;
    double towerZ = 400.0;
    
    glPushMatrix();
    glTranslated(towerX, 0.0, towerZ);
    
    // Tower base shaft
    glColor3f(0.8f, 0.8f, 0.82f); // light gray
    glPushMatrix();
    glTranslated(0, 10.0, 0);
    glScaled(4.0, 20.0, 4.0);
    glutSolidCube(1.0);
    glPopMatrix();
    
    // Tower cab (glass section at top)
    glEnable(GL_BLEND);
    glColor4f(0.2f, 0.4f, 0.6f, 0.5f); // blue tinted glass
    glPushMatrix();
    glTranslated(0, 21.0, 0);
    glScaled(6.0, 4.0, 6.0);
    glutSolidCube(1.0);
    glPopMatrix();
    glDisable(GL_BLEND);
    
    // Tower roof
    glColor3f(0.7f, 0.2f, 0.2f); // red roof
    glPushMatrix();
    glTranslated(0, 23.0, 0);
    glRotated(-90.0, 1.0, 0.0, 0.0);
    glutSolidCone(4.5, 2.0, 8, 2);
    glPopMatrix();
    
    glPopMatrix();
    
    // --- HANGAR ---
    double hangarX = 65.0;
    double hangarZ = 500.0;
    glPushMatrix();
    glTranslated(hangarX, 0.0, hangarZ);
    
    // Main body
    glColor3f(0.5f, 0.6f, 0.65f); // corrugated metal blue-gray
    glPushMatrix();
    glTranslated(0, 5.0, 0);
    glScaled(18.0, 10.0, 25.0);
    glutSolidCube(1.0);
    glPopMatrix();
    
    // Roof (half-cylinder or wedge shape)
    glColor3f(0.4f, 0.45f, 0.5f);
    glPushMatrix();
    glTranslated(0, 10.5, 0);
    glScaled(18.0, 2.0, 25.0);
    glutSolidCube(1.0);
    glPopMatrix();
    
    // Hangar door (big dark gray door)
    glColor3f(0.2f, 0.2f, 0.2f);
    glPushMatrix();
    glTranslated(-8.9, 3.5, 0);
    glScaled(0.1, 7.0, 15.0);
    glutSolidCube(1.0);
    glPopMatrix();
    
    glPopMatrix();
    
    // --- RUNWAY MARKS / SIGNS ---
    // Distance remaining signs along runway sides (at 200m, 500m, 800m, 1100m, 1400m)
    for (int i = 0; i < 5; i++) {
        double signZ = 200.0 + i * 250.0;
        
        // Left sign
        glPushMatrix();
        glTranslated(-RUNWAY_WIDTH/2 - 3.0, 0.5, signZ);
        glColor3f(0.1f, 0.1f, 0.1f);
        glScaled(1.0, 1.0, 0.5);
        glutSolidCube(1.0);
        glPopMatrix();
        
        glColor3f(1.0f, 1.0f, 1.0f);
        char signText[8];
        snprintf(signText, sizeof(signText), "%d", 5 - i);
        drawText3D(-RUNWAY_WIDTH/2 - 2.8, 0.3, signZ - 0.1, signText, GLUT_BITMAP_HELVETICA_12);
        
        // Right sign
        glPushMatrix();
        glTranslated(RUNWAY_WIDTH/2 + 3.0, 0.5, signZ);
        glColor3f(0.1f, 0.1f, 0.1f);
        glScaled(1.0, 1.0, 0.5);
        glutSolidCube(1.0);
        glPopMatrix();
        
        glColor3f(1.0f, 1.0f, 1.0f);
        drawText3D(RUNWAY_WIDTH/2 + 2.4, 0.3, signZ - 0.1, signText, GLUT_BITMAP_HELVETICA_12);
    }
    glEnable(GL_LIGHTING);
}

// Apply fog settings
void setupFog() {
    if (environment.fogEnabled) {
        glEnable(GL_FOG);
        GLfloat fogColor[] = {0.7f, 0.75f, 0.8f, 1.0f};
        glFogfv(GL_FOG_COLOR, fogColor);
        glFogi(GL_FOG_MODE, GL_EXP2);
        glFogf(GL_FOG_DENSITY, (float)environment.fogDensity);
    } else {
        glDisable(GL_FOG);
    }
}


// ============================================================================
// MODULE 5: AIRCRAFT MODEL RENDERING
// ============================================================================

void renderAircraftModel() {
    glPushMatrix();

    // Transform to aircraft position and orientation
    glTranslated(aircraft.px, aircraft.py, aircraft.pz);
    glRotated(aircraft.yaw * RAD2DEG, 0, 1, 0);
    glRotated(aircraft.pitch * RAD2DEG, 1, 0, 0);
    glRotated(aircraft.roll * RAD2DEG, 0, 0, 1);

    // Scale: Cessna 172 is about 8.3m long, 11m wingspan
    // Forward = +Z in body frame

    // ---- FUSELAGE ----
    glColor3f(0.95f, 0.95f, 0.95f); // white base
    glPushMatrix();
    glScaled(0.5, 0.5, 4.0);    // elongated box
    glutSolidCube(1.0);
    glPopMatrix();

    // Fuselage nose (tapered cowling)
    glColor3f(0.92f, 0.92f, 0.92f);
    glPushMatrix();
    glTranslated(0, 0, 2.5);
    glScaled(0.4, 0.4, 1.5);
    glutSolidSphere(0.5, 12, 12);
    glPopMatrix();

    // Cowling Dual Air Intakes (left & right grills)
    glPushMatrix();
    glDisable(GL_LIGHTING);
    glColor3f(0.08f, 0.08f, 0.08f); // black intake color
    // Left grill
    glPushMatrix();
    glTranslated(-0.16, -0.05, 2.85);
    glScaled(0.12, 0.07, 0.05);
    glutSolidSphere(1.0, 8, 8);
    glPopMatrix();
    // Right grill
    glPushMatrix();
    glTranslated(0.16, -0.05, 2.85);
    glScaled(0.12, 0.07, 0.05);
    glutSolidSphere(1.0, 8, 8);
    glPopMatrix();
    // Center lower air intake filter
    glPushMatrix();
    glTranslated(0.0, -0.22, 2.82);
    glScaled(0.18, 0.06, 0.05);
    glutSolidCube(1.0);
    glPopMatrix();
    glEnable(GL_LIGHTING);
    glPopMatrix();

    // Fuselage tail taper
    glColor3f(0.92f, 0.92f, 0.92f);
    glPushMatrix();
    glTranslated(0, 0.1, -2.5);
    glScaled(0.35, 0.35, 1.5);
    glutSolidSphere(0.5, 12, 12);
    glPopMatrix();

    // Navy Blue stripe (decorative)
    glColor3f(0.08f, 0.18f, 0.36f); // dark navy
    glBegin(GL_QUADS);
    glVertex3d(-0.26, 0.1, -2.0);
    glVertex3d(-0.26, 0.1, 3.0);
    glVertex3d(-0.26, 0.18, 3.0);
    glVertex3d(-0.26, 0.18, -2.0);
    // Other side
    glVertex3d(0.26, 0.1, -2.0);
    glVertex3d(0.26, 0.1, 3.0);
    glVertex3d(0.26, 0.18, 3.0);
    glVertex3d(0.26, 0.18, -2.0);
    glEnd();

    // Gold/Orange stripe below navy blue
    glColor3f(0.85f, 0.60f, 0.15f); // warm gold
    glBegin(GL_QUADS);
    glVertex3d(-0.26, 0.02, -2.0);
    glVertex3d(-0.26, 0.02, 3.0);
    glVertex3d(-0.26, 0.1, 3.0);
    glVertex3d(-0.26, 0.1, -2.0);
    glVertex3d(0.26, 0.02, -2.0);
    glVertex3d(0.26, 0.02, 3.0);
    glVertex3d(0.26, 0.1, 3.0);
    glVertex3d(0.26, 0.1, -2.0);
    glEnd();

    // ---- WINGS (high-wing configuration with moving surfaces) ----
    glColor3f(0.93f, 0.93f, 0.93f);
    
    // -- Left Wing main box (Z: 0.0 to 0.8) --
    glBegin(GL_QUADS);
    glVertex3d(-0.3, 0.30, 0.0);
    glVertex3d(-5.5, 0.30, 0.0);
    glVertex3d(-5.5, 0.30,  0.8);
    glVertex3d(-0.3, 0.30,  0.8);
    // wing underside
    glVertex3d(-0.3, 0.25, 0.0);
    glVertex3d(-5.5, 0.25, 0.0);
    glVertex3d(-5.5, 0.25,  0.8);
    glVertex3d(-0.3, 0.25,  0.8);
    glEnd();
    // Wing tip cap left
    glBegin(GL_QUADS);
    glVertex3d(-5.5, 0.25, 0.0);
    glVertex3d(-5.5, 0.30, 0.0);
    glVertex3d(-5.5, 0.30,  0.8);
    glVertex3d(-5.5, 0.25,  0.8);
    glEnd();

    // -- Left Flap (hinged at trailing edge Z:0.0, X: -0.3 to -2.9) --
    glPushMatrix();
    glTranslated(0.0, 0.275, 0.0); // center of wing box thickness
    glRotated(-aircraft.flaps, 1.0, 0.0, 0.0); // rotates down around X-axis
    glBegin(GL_QUADS);
    glVertex3d(-0.3, 0.025, -0.2);
    glVertex3d(-2.9, 0.025, -0.2);
    glVertex3d(-2.9, 0.025, 0.0);
    glVertex3d(-0.3, 0.025, 0.0);
    
    glVertex3d(-0.3, -0.025, -0.2);
    glVertex3d(-2.9, -0.025, -0.2);
    glVertex3d(-2.9, -0.025, 0.0);
    glVertex3d(-0.3, -0.025, 0.0);
    glEnd();
    glPopMatrix();

    // -- Left Aileron (hinged outer trailing edge Z:0.0, X: -2.9 to -5.5) --
    glPushMatrix();
    glTranslated(0.0, 0.275, 0.0);
    glRotated(-aircraft.aileronDeflection * 15.0, 1.0, 0.0, 0.0);
    glBegin(GL_QUADS);
    glVertex3d(-2.9, 0.025, -0.2);
    glVertex3d(-5.5, 0.025, -0.2);
    glVertex3d(-5.5, 0.025, 0.0);
    glVertex3d(-2.9, 0.025, 0.0);
    
    glVertex3d(-2.9, -0.025, -0.2);
    glVertex3d(-5.5, -0.025, -0.2);
    glVertex3d(-5.5, -0.025, 0.0);
    glVertex3d(-2.9, -0.025, 0.0);
    glEnd();
    glPopMatrix();

    // -- Right Wing main box (Z: 0.0 to 0.8) --
    glBegin(GL_QUADS);
    glVertex3d(0.3, 0.30, 0.0);
    glVertex3d(5.5, 0.30, 0.0);
    glVertex3d(5.5, 0.30,  0.8);
    glVertex3d(0.3, 0.30,  0.8);
    // wing underside
    glVertex3d(0.3, 0.25, 0.0);
    glVertex3d(5.5, 0.25, 0.0);
    glVertex3d(5.5, 0.25,  0.8);
    glVertex3d(0.3, 0.25,  0.8);
    glEnd();
    // Wing tip cap right
    glBegin(GL_QUADS);
    glVertex3d(5.5, 0.25, 0.0);
    glVertex3d(5.5, 0.30, 0.0);
    glVertex3d(5.5, 0.30,  0.8);
    glVertex3d(5.5, 0.25,  0.8);
    glEnd();

    // -- Right Flap (hinged trailing edge Z:0.0, X: 0.3 to 2.9) --
    glPushMatrix();
    glTranslated(0.0, 0.275, 0.0);
    glRotated(-aircraft.flaps, 1.0, 0.0, 0.0); // rotates down symmetrically
    glBegin(GL_QUADS);
    glVertex3d(0.3, 0.025, -0.2);
    glVertex3d(2.9, 0.025, -0.2);
    glVertex3d(2.9, 0.025, 0.0);
    glVertex3d(0.3, 0.025, 0.0);
    
    glVertex3d(0.3, -0.025, -0.2);
    glVertex3d(2.9, -0.025, -0.2);
    glVertex3d(2.9, -0.025, 0.0);
    glVertex3d(0.3, -0.025, 0.0);
    glEnd();
    glPopMatrix();

    // -- Right Aileron (hinged outer trailing edge Z:0.0, X: 2.9 to 5.5, rotates opposite) --
    glPushMatrix();
    glTranslated(0.0, 0.275, 0.0);
    glRotated(aircraft.aileronDeflection * 15.0, 1.0, 0.0, 0.0);
    glBegin(GL_QUADS);
    glVertex3d(2.9, 0.025, -0.2);
    glVertex3d(5.5, 0.025, -0.2);
    glVertex3d(5.5, 0.025, 0.0);
    glVertex3d(2.9, 0.025, 0.0);
    
    glVertex3d(2.9, -0.025, -0.2);
    glVertex3d(5.5, -0.025, -0.2);
    glVertex3d(5.5, -0.025, 0.0);
    glVertex3d(2.9, -0.025, 0.0);
    glEnd();
    glPopMatrix();

    // Wing struts (left and right)
    glColor3f(0.7f, 0.7f, 0.7f);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glVertex3d(-0.4, -0.1, 0.3);
    glVertex3d(-3.5, 0.25, 0.3);
    glVertex3d(0.4, -0.1, 0.3);
    glVertex3d(3.5, 0.25, 0.3);
    glEnd();

    // ---- HORIZONTAL STABILIZER & ELEVATORS ----
    glColor3f(0.93f, 0.93f, 0.93f);
    glBegin(GL_QUADS);
    // Left static stabilizer
    glVertex3d(-0.2, 0.1, -3.5);
    glVertex3d(-2.0, 0.1, -3.5);
    glVertex3d(-2.0, 0.1, -3.0);
    glVertex3d(-0.2, 0.1, -3.0);
    // Right static stabilizer
    glVertex3d(0.2, 0.1, -3.5);
    glVertex3d(2.0, 0.1, -3.5);
    glVertex3d(2.0, 0.1, -3.0);
    glVertex3d(0.2, 0.1, -3.0);
    glEnd();

    // Moving Elevators (hinged at Z = -3.5, extending back by 0.3m)
    glPushMatrix();
    glTranslated(0.0, 0.1, -3.5);
    glRotated(-aircraft.elevatorDeflection * 20.0, 1.0, 0.0, 0.0); // rotates up/down
    glBegin(GL_QUADS);
    // Left elevator flap
    glVertex3d(-0.2, 0.0, -0.3);
    glVertex3d(-2.0, 0.0, -0.3);
    glVertex3d(-2.0, 0.0, 0.0);
    glVertex3d(-0.2, 0.0, 0.0);
    // Right elevator flap
    glVertex3d(0.2, 0.0, -0.3);
    glVertex3d(2.0, 0.0, -0.3);
    glVertex3d(2.0, 0.0, 0.0);
    glVertex3d(0.2, 0.0, 0.0);
    glEnd();
    glPopMatrix();

    // ---- VERTICAL STABILIZER & RUDDER ----
    glColor3f(0.93f, 0.93f, 0.93f);
    glBegin(GL_TRIANGLES);
    // Left side of static fin
    glVertex3d(0, 0.1, -3.5);
    glVertex3d(0, 2.0, -3.5);
    glVertex3d(0, 0.1, -2.5);
    glEnd();
    // Right side of static fin
    glColor3f(0.88f, 0.88f, 0.88f);
    glBegin(GL_TRIANGLES);
    glVertex3d(0.02, 0.1, -3.5);
    glVertex3d(0.02, 2.0, -3.5);
    glVertex3d(0.02, 0.1, -2.5);
    glEnd();

    // Moving Rudder (hinged at Z = -3.5, deflecting left/right)
    glPushMatrix();
    glTranslated(0.0, 0.0, -3.5);
    glRotated(aircraft.rudderDeflection * 25.0, 0.0, 1.0, 0.0); // rotates around Y-axis
    glColor3f(0.93f, 0.93f, 0.93f);
    glBegin(GL_TRIANGLES);
    glVertex3d(0.0, 0.1, -0.3);
    glVertex3d(0.0, 2.0, -0.3);
    glVertex3d(0.0, 0.1, 0.0);
    glEnd();
    glPopMatrix();

    // ---- PROPELLER ----
    glPushMatrix();
    glTranslated(0, 0, 3.2);
    glRotated(aircraft.propellerAngle * RAD2DEG, 0, 0, 1);

    glColor3f(0.2f, 0.2f, 0.2f);
    // Blade 1
    glBegin(GL_QUADS);
    glVertex3d(-0.08, -0.8, 0);
    glVertex3d( 0.08, -0.8, 0);
    glVertex3d( 0.08,  0.8, 0);
    glVertex3d(-0.08,  0.8, 0);
    glEnd();
    // Blade 2
    glBegin(GL_QUADS);
    glVertex3d(-0.8, -0.08, 0);
    glVertex3d( 0.8, -0.08, 0);
    glVertex3d( 0.8,  0.08, 0);
    glVertex3d(-0.8,  0.08, 0);
    glEnd();

    // Spinner
    glColor3f(0.85f, 0.85f, 0.85f);
    glutSolidSphere(0.15, 8, 8);
    glPopMatrix();

    // ---- COCKPIT INTERIOR (SEATS) ----
    glColor3f(0.35f, 0.35f, 0.35f); // charcoal seats
    // Left Seat
    glPushMatrix();
    glTranslated(-0.15, -0.05, 0.35);
    glScaled(0.18, 0.25, 0.18);
    glutSolidCube(1.0);
    glPopMatrix();
    // Right Seat
    glPushMatrix();
    glTranslated(0.15, -0.05, 0.35);
    glScaled(0.18, 0.25, 0.18);
    glutSolidCube(1.0);
    glPopMatrix();

    // ---- LANDING GEAR (detailed with metallic struts and wheel fairings) ----
    glColor3f(0.6f, 0.6f, 0.62f); // metal gear struts
    glLineWidth(4.0f);

    // -- Nose Gear --
    glBegin(GL_LINES);
    glVertex3d(0, -0.25, 1.8);
    glVertex3d(0, -1.2, 1.8);
    glEnd();
    // Nose Wheel Fairing & Wheel
    glPushMatrix();
    glTranslated(0, -1.2, 1.8);
    // White fairing
    glPushMatrix();
    glScaled(0.18, 0.24, 0.38);
    glColor3f(0.95f, 0.95f, 0.95f);
    glutSolidSphere(1.0, 8, 8);
    glPopMatrix();
    // Tire
    glColor3f(0.15f, 0.15f, 0.15f);
    glutSolidSphere(0.15, 8, 8);
    glPopMatrix();

    // -- Left Main Gear --
    glColor3f(0.6f, 0.6f, 0.62f);
    glBegin(GL_LINES);
    glVertex3d(-0.8, -0.25, -0.3);
    glVertex3d(-0.8, -1.2, -0.3);
    glEnd();
    // Wheel Fairing & Wheel
    glPushMatrix();
    glTranslated(-0.8, -1.2, -0.3);
    // White fairing
    glPushMatrix();
    glScaled(0.2, 0.28, 0.44);
    glColor3f(0.95f, 0.95f, 0.95f);
    glutSolidSphere(1.0, 8, 8);
    glPopMatrix();
    // Tire
    glColor3f(0.15f, 0.15f, 0.15f);
    glutSolidSphere(0.18, 8, 8);
    glPopMatrix();

    // -- Right Main Gear --
    glColor3f(0.6f, 0.6f, 0.62f);
    glBegin(GL_LINES);
    glVertex3d(0.8, -0.25, -0.3);
    glVertex3d(0.8, -1.2, -0.3);
    glEnd();
    // Wheel Fairing & Wheel
    glPushMatrix();
    glTranslated(0.8, -1.2, -0.3);
    // White fairing
    glPushMatrix();
    glScaled(0.2, 0.28, 0.44);
    glColor3f(0.95f, 0.95f, 0.95f);
    glutSolidSphere(1.0, 8, 8);
    glPopMatrix();
    // Tire
    glColor3f(0.15f, 0.15f, 0.15f);
    glutSolidSphere(0.18, 8, 8);
    glPopMatrix();

    // ---- COCKPIT WINDOW (tinted glass) ----
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.3f, 0.5f, 0.7f, 0.4f);  // tinted blue glass
    glBegin(GL_QUADS);
    // Front windshield
    glVertex3d(-0.22, 0.25, 1.0);
    glVertex3d( 0.22, 0.25, 1.0);
    glVertex3d( 0.20, 0.35, 1.8);
    glVertex3d(-0.20, 0.35, 1.8);
    // Side windows
    glVertex3d(-0.26, 0.10, 0.0);
    glVertex3d(-0.26, 0.25, 0.0);
    glVertex3d(-0.24, 0.35, 1.0);
    glVertex3d(-0.24, 0.10, 1.0);
    glVertex3d(0.26, 0.10, 0.0);
    glVertex3d(0.26, 0.25, 0.0);
    glVertex3d(0.24, 0.35, 1.0);
    glVertex3d(0.24, 0.10, 1.0);
    glEnd();
    glDisable(GL_BLEND);

    // ---- NAVIGATION & STROBE LIGHTS ----
    // Left Wingtip Nav Light (Red)
    glPushMatrix();
    glTranslated(-5.5, 0.28, 0.7);
    glDisable(GL_LIGHTING);
    glColor3f(1.0f, 0.0f, 0.0f);
    glutSolidSphere(0.08, 8, 8);
    glEnable(GL_LIGHTING);
    glPopMatrix();

    // Right Wingtip Nav Light (Green)
    glPushMatrix();
    glTranslated(5.5, 0.28, 0.7);
    glDisable(GL_LIGHTING);
    glColor3f(0.0f, 1.0f, 0.0f);
    glutSolidSphere(0.08, 8, 8);
    glEnable(GL_LIGHTING);
    glPopMatrix();

    // Tail Strobe Light (Flashing White)
    glPushMatrix();
    glTranslated(0.0, 2.0, -3.75);
    glDisable(GL_LIGHTING);
    double strobePeriod = 0.8;
    double phase = fmod(config.simTime, strobePeriod);
    if (phase < 0.08) {
        glColor3f(1.0f, 1.0f, 1.0f); // bright white flash
    } else {
        glColor3f(0.3f, 0.3f, 0.3f); // dim/off state
    }
    glutSolidSphere(0.06, 6, 6);
    glEnable(GL_LIGHTING);
    glPopMatrix();

    glPopMatrix();
}


// ============================================================================
// MODULE 6: CAMERA SYSTEM
// ============================================================================

void setupCamera() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double aspect = (double)windowWidth / (double)windowHeight;
    gluPerspective(60.0, aspect, 0.5, 20000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    if (config.cameraMode == 0) {
        // ---- COCKPIT VIEW ----
        // Eye at pilot position (slightly above and behind nose, looking forward)
        Vec3 eyeOffset(0.0, 0.7, 1.5);  // body frame offset (up, fwd from CG)
        Vec3 eyeWorld = bodyToWorld(eyeOffset, aircraft.pitch, aircraft.roll, aircraft.yaw);
        double eyeX = aircraft.px + eyeWorld.x;
        double eyeY = aircraft.py + eyeWorld.y;
        double eyeZ = aircraft.pz + eyeWorld.z;

        // Look point: far ahead
        Vec3 lookOffset(0.0, 0.5, 100.0);
        Vec3 lookWorld = bodyToWorld(lookOffset, aircraft.pitch, aircraft.roll, aircraft.yaw);
        double lookX = aircraft.px + lookWorld.x;
        double lookY = aircraft.py + lookWorld.y;
        double lookZ = aircraft.pz + lookWorld.z;

        // Up vector from aircraft
        Vec3 upVec = getUpVec();

        gluLookAt(eyeX, eyeY, eyeZ,
                  lookX, lookY, lookZ,
                  upVec.x, upVec.y, upVec.z);

    } else {
        // ---- CHASE VIEW ----
        // Smooth chase camera: follows behind and above
        double chaseDist   = 18.0;
        double chaseHeight = 6.0;

        Vec3 behindOffset(0.0, chaseHeight, -chaseDist);
        Vec3 behind = bodyToWorld(behindOffset, aircraft.pitch * 0.3, 0, aircraft.yaw);

        double targetX = aircraft.px + behind.x;
        double targetY = aircraft.py + behind.y;
        double targetZ = aircraft.pz + behind.z;

        // Smooth interpolation
        double smoothing = 0.04;
        camX += (targetX - camX) * smoothing;
        camY += (targetY - camY) * smoothing;
        camZ += (targetZ - camZ) * smoothing;

        // Ensure camera doesn't go underground
        if (camY < 1.5) camY = 1.5;

        gluLookAt(camX, camY, camZ,
                  aircraft.px, aircraft.py + 1.0, aircraft.pz,
                  0, 1, 0);
    }
}


// ============================================================================
// MODULE 7: HUD / GLASS COCKPIT (PFD)
// ============================================================================

// Helper: draw text at 2D screen position (in orthographic mode)
void drawText2D(float x, float y, const char* text, void* font = GLUT_BITMAP_HELVETICA_12) {
    glRasterPos2f(x, y);
    for (const char* c = text; *c; c++) {
        glutBitmapCharacter(font, *c);
    }
}

// Helper: draw a filled rectangle
void drawRect(float x1, float y1, float x2, float y2) {
    glBegin(GL_QUADS);
    glVertex2f(x1, y1);
    glVertex2f(x2, y1);
    glVertex2f(x2, y2);
    glVertex2f(x1, y2);
    glEnd();
}

// Helper: draw a line
void drawLine2D(float x1, float y1, float x2, float y2) {
    glBegin(GL_LINES);
    glVertex2f(x1, y1);
    glVertex2f(x2, y2);
    glEnd();
}

// Draw the attitude indicator
void drawAttitudeIndicator(float cx, float cy, float radius) {
    double pitchDeg = aircraft.pitch * RAD2DEG;
    double rollDeg  = aircraft.roll * RAD2DEG;

    // Clipping circle (scissor approximation)
    // We'll draw within the circle

    float pixPerDeg = radius / 20.0f;  // 20 degrees visible range

    // Save state and set up clipping
    glEnable(GL_SCISSOR_TEST);
    int scX = (int)(cx - radius);
    int scY = (int)(cy - radius);
    int scW = (int)(radius * 2);
    int scH = (int)(radius * 2);
    glScissor(scX, scY, scW, scH);

    glPushMatrix();
    glTranslatef(cx, cy, 0);
    glRotatef((float)rollDeg, 0, 0, 1);

    float pitchOffset = (float)pitchDeg * pixPerDeg;

    // Sky (upper half)
    glColor3f(0.2f, 0.45f, 0.85f);
    drawRect(-radius * 3, pitchOffset, radius * 3, radius * 3 + pitchOffset);

    // Ground (lower half)
    glColor3f(0.55f, 0.35f, 0.15f);
    drawRect(-radius * 3, -radius * 3 + pitchOffset, radius * 3, pitchOffset);

    // Horizon line
    glColor3f(0.95f, 0.95f, 0.95f);
    glLineWidth(2.0f);
    drawLine2D(-radius * 2, pitchOffset, radius * 2, pitchOffset);

    // Pitch ladder lines
    glLineWidth(1.0f);
    for (int deg = -40; deg <= 40; deg += 5) {
        if (deg == 0) continue;
        float ly = pitchOffset - deg * pixPerDeg;
        float halfLen = (deg % 10 == 0) ? radius * 0.35f : radius * 0.2f;

        glColor3f(0.95f, 0.95f, 0.95f);
        drawLine2D(-halfLen, ly, halfLen, ly);

        // Degree labels on 10-degree lines
        if (deg % 10 == 0) {
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", abs(deg));
            drawText2D(-halfLen - 20, ly - 4, buf, GLUT_BITMAP_HELVETICA_10);
            drawText2D(halfLen + 5, ly - 4, buf, GLUT_BITMAP_HELVETICA_10);
        }
    }

    glPopMatrix();

    // Aircraft reference symbol (fixed, not rotating)
    glColor3f(1.0f, 1.0f, 0.0f);  // yellow
    glLineWidth(2.5f);
    // Center dot
    drawLine2D(cx - 2, cy, cx + 2, cy);
    // Left wing
    drawLine2D(cx - radius * 0.4f, cy, cx - radius * 0.15f, cy);
    drawLine2D(cx - radius * 0.15f, cy, cx - radius * 0.15f, cy - 5);
    // Right wing
    drawLine2D(cx + radius * 0.4f, cy, cx + radius * 0.15f, cy);
    drawLine2D(cx + radius * 0.15f, cy, cx + radius * 0.15f, cy - 5);

    // Roll pointer/bank scale (arc at top)
    glColor3f(0.95f, 0.95f, 0.95f);
    glLineWidth(1.0f);
    float bankArcR = radius * 0.9f;
    int bankMarks[] = {0, 10, 20, 30, 45, 60};
    for (int i = 0; i < 6; i++) {
        for (int sign = -1; sign <= 1; sign += 2) {
            if (bankMarks[i] == 0 && sign == -1) continue;
            float angle = (90.0f + sign * bankMarks[i]) * (float)DEG2RAD;
            float tickLen = (bankMarks[i] % 30 == 0) ? 12.0f : 7.0f;
            float x1 = cx + cos(angle) * bankArcR;
            float y1 = cy + sin(angle) * bankArcR;
            float x2 = cx + cos(angle) * (bankArcR - tickLen);
            float y2 = cy + sin(angle) * (bankArcR - tickLen);
            drawLine2D(x1, y1, x2, y2);
        }
    }

    // Bank pointer (triangle at top, rotates with roll)
    float ptrAngle = (float)(90.0 - rollDeg) * (float)DEG2RAD;
    float px = cx + cos(ptrAngle) * (bankArcR - 14);
    float py2 = cy + sin(ptrAngle) * (bankArcR - 14);
    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_TRIANGLES);
    glVertex2f(px, py2);
    glVertex2f(px - 5 * cos(ptrAngle + 0.3f), py2 - 5 * sin(ptrAngle + 0.3f));
    glVertex2f(px - 5 * cos(ptrAngle - 0.3f), py2 - 5 * sin(ptrAngle - 0.3f));
    glEnd();

    // Draw ILS localizer and glideslope needles if active
    bool ilsActive = (aircraft.pz < 2000.0 && aircraft.pz > -8000.0);
    if (ilsActive) {
        float dotSpacing = radius * 0.15f;

        // --- Localizer (Horizontal scale at bottom of AI) ---
        float locY = cy - radius * 0.7f;
        
        // Scale tick marks (representing dots)
        glColor3f(0.8f, 0.8f, 0.8f);
        glLineWidth(1.0f);
        drawLine2D(cx - dotSpacing * 2, locY - 4, cx - dotSpacing * 2, locY + 4);
        drawLine2D(cx - dotSpacing, locY - 4, cx - dotSpacing, locY + 4);
        
        // Center circle outline
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i <= 8; i++) {
            float a = (float)i * 2.0f * (float)PI / 8.0f;
            glVertex2f(cx + 3.0f * cos(a), locY + 3.0f * sin(a));
        }
        glEnd();
        
        drawLine2D(cx + dotSpacing, locY - 4, cx + dotSpacing, locY + 4);
        drawLine2D(cx + dotSpacing * 2, locY - 4, cx + dotSpacing * 2, locY + 4);
        
        // Calculate deflection (deviation from runway centerline)
        double distToTdz = 300.0 - aircraft.pz;
        double locAngle = 0.0;
        if (distToTdz > 50.0) {
            locAngle = atan2(aircraft.px, distToTdz) * RAD2DEG;
        }
        double locDeflection = clampd(locAngle / 2.5, -1.0, 1.0); // full scale is 2.5 degrees
        
        // Draw diamond pointer (magenta)
        float diaX = cx + (float)locDeflection * (dotSpacing * 2);
        glColor3f(0.85f, 0.0f, 0.85f); // Magenta
        glLineWidth(2.0f);
        glBegin(GL_LINE_LOOP);
        glVertex2f(diaX, locY + 5);
        glVertex2f(diaX + 4, locY);
        glVertex2f(diaX, locY - 5);
        glVertex2f(diaX - 4, locY);
        glEnd();
        glLineWidth(1.0f);

        // --- Glide Slope (Vertical scale on right of AI) ---
        float gsX = cx + radius * 0.75f;
        
        // Scale tick marks (representing dots)
        glColor3f(0.8f, 0.8f, 0.8f);
        drawLine2D(gsX - 4, cy + dotSpacing * 2, gsX + 4, cy + dotSpacing * 2);
        drawLine2D(gsX - 4, cy + dotSpacing, gsX + 4, cy + dotSpacing);
        
        // Center circle outline
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i <= 8; i++) {
            float a = (float)i * 2.0f * (float)PI / 8.0f;
            glVertex2f(gsX + 3.0f * cos(a), cy + 3.0f * sin(a));
        }
        glEnd();
        
        drawLine2D(gsX - 4, cy - dotSpacing, gsX + 4, cy - dotSpacing);
        drawLine2D(gsX - 4, cy - dotSpacing * 2, gsX + 4, cy - dotSpacing * 2);
        
        // Calculate GS deflection
        double gsDeflection = 0.0;
        if (distToTdz > 50.0) {
            double actualGsAngle = atan2(aircraft.py, distToTdz) * RAD2DEG;
            double gsErrorAngle = actualGsAngle - 3.0; // 3 degree glide path
            gsDeflection = clampd(gsErrorAngle / 0.7, -1.0, 1.0); // full scale is 0.7 degrees
        }
        
        // Draw diamond pointer (deflected down when we are high, meaning GS is below us)
        float diaY = cy - (float)gsDeflection * (dotSpacing * 2);
        glColor3f(0.85f, 0.0f, 0.85f); // Magenta
        glLineWidth(2.0f);
        glBegin(GL_LINE_LOOP);
        glVertex2f(gsX, diaY + 5);
        glVertex2f(gsX + 4, diaY);
        glVertex2f(gsX, diaY - 5);
        glVertex2f(gsX - 4, diaY);
        glEnd();
        glLineWidth(1.0f);
        
        // Label G & L
        glColor3f(0.85f, 0.0f, 0.85f);
        drawText2D(gsX - 12, cy + radius * 0.45f, "G", GLUT_BITMAP_HELVETICA_10);
        drawText2D(cx - radius * 0.55f, locY - 12, "L", GLUT_BITMAP_HELVETICA_10);
    }

    glDisable(GL_SCISSOR_TEST);
}

// Draw the airspeed tape
void drawAirspeedTape(float x, float y, float w, float h) {
    double speedKts = aircraft.airspeed * 1.94384;  // m/s to knots

    // Background
    glColor4f(0.1f, 0.1f, 0.12f, 0.85f);
    glEnable(GL_BLEND);
    drawRect(x, y, x + w, y + h);

    // Speed range bands
    float pixPerKt = h / 60.0f;  // show ±30 knots range
    float centerY = y + h / 2.0f;

    glEnable(GL_SCISSOR_TEST);
    glScissor((int)x, (int)y, (int)w, (int)h);

    // Draw speed scale
    for (int spd = 0; spd <= 200; spd += 5) {
        float yPos = centerY + (float)(spd - speedKts) * pixPerKt;

        if (yPos < y - 20 || yPos > y + h + 20) continue;

        // Color bands on left edge
        float bandX = x + 2;
        float bandW = 4;
        if (spd <= 40) {
            glColor3f(1.0f, 1.0f, 1.0f);   // white arc (flap range)
        } else if (spd <= 129) {
            glColor3f(0.0f, 0.8f, 0.0f);   // green arc (normal)
        } else if (spd <= 163) {
            glColor3f(1.0f, 1.0f, 0.0f);   // yellow arc (caution)
        } else {
            glColor3f(1.0f, 0.0f, 0.0f);   // red line (VNE)
        }
        drawRect(bandX, yPos - pixPerKt * 2.5f, bandX + bandW, yPos + pixPerKt * 2.5f);

        // Tick marks
        glColor3f(0.8f, 0.8f, 0.8f);
        if (spd % 10 == 0) {
            drawLine2D(x + w - 15, yPos, x + w, yPos);
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", spd);
            drawText2D(x + 8, yPos - 4, buf, GLUT_BITMAP_HELVETICA_10);
        } else {
            drawLine2D(x + w - 8, yPos, x + w, yPos);
        }
    }

    glDisable(GL_SCISSOR_TEST);

    // Current speed pointer
    glColor3f(0.0f, 0.0f, 0.0f);
    drawRect(x + 2, centerY - 10, x + w - 2, centerY + 10);
    glColor3f(0.0f, 1.0f, 0.0f);
    glLineWidth(1.5f);
    char spdBuf[16];
    snprintf(spdBuf, sizeof(spdBuf), "%3.0f", speedKts);
    drawText2D(x + 8, centerY - 5, spdBuf, GLUT_BITMAP_HELVETICA_12);

    // Pointer arrow
    glBegin(GL_TRIANGLES);
    glVertex2f(x + w, centerY);
    glVertex2f(x + w + 8, centerY + 6);
    glVertex2f(x + w + 8, centerY - 6);
    glEnd();

    // Label
    glColor3f(0.7f, 0.7f, 0.7f);
    drawText2D(x + 3, y + h + 3, "IAS", GLUT_BITMAP_HELVETICA_10);
    drawText2D(x + 3, y - 12, "KTS", GLUT_BITMAP_HELVETICA_10);

    glDisable(GL_BLEND);
}

// Draw the altimeter tape
void drawAltimeterTape(float x, float y, float w, float h) {
    double altFt = aircraft.py * 3.28084;  // meters to feet

    // Background
    glEnable(GL_BLEND);
    glColor4f(0.1f, 0.1f, 0.12f, 0.85f);
    drawRect(x, y, x + w, y + h);

    float pixPerFt = h / 400.0f;  // ±200 ft visible range
    float centerY = y + h / 2.0f;

    glEnable(GL_SCISSOR_TEST);
    glScissor((int)x, (int)y, (int)w, (int)h);

    // Draw altitude scale
    for (int alt = -100; alt <= 15000; alt += 20) {
        float yPos = centerY + (float)(alt - altFt) * pixPerFt;
        if (yPos < y - 20 || yPos > y + h + 20) continue;

        glColor3f(0.8f, 0.8f, 0.8f);
        if (alt % 100 == 0) {
            drawLine2D(x, yPos, x + 15, yPos);
            char buf[16];
            snprintf(buf, sizeof(buf), "%d", alt);
            drawText2D(x + 18, yPos - 4, buf, GLUT_BITMAP_HELVETICA_10);
        } else if (alt % 50 == 0) {
            drawLine2D(x, yPos, x + 10, yPos);
        }
    }

    glDisable(GL_SCISSOR_TEST);

    // Current altitude pointer
    glColor3f(0.0f, 0.0f, 0.0f);
    drawRect(x + 2, centerY - 10, x + w - 2, centerY + 10);
    glColor3f(0.0f, 1.0f, 0.0f);
    char altBuf[16];
    snprintf(altBuf, sizeof(altBuf), "%5.0f", altFt);
    drawText2D(x + 5, centerY - 5, altBuf, GLUT_BITMAP_HELVETICA_12);

    // Pointer arrow
    glBegin(GL_TRIANGLES);
    glVertex2f(x, centerY);
    glVertex2f(x - 8, centerY + 6);
    glVertex2f(x - 8, centerY - 6);
    glEnd();

    // Label
    glColor3f(0.7f, 0.7f, 0.7f);
    drawText2D(x + 3, y + h + 3, "ALT", GLUT_BITMAP_HELVETICA_10);
    drawText2D(x + 3, y - 12, "FT", GLUT_BITMAP_HELVETICA_10);

    glDisable(GL_BLEND);
}

// Draw the Vertical Speed Indicator
void drawVSI(float x, float y, float w, float h) {
    double vsiFpm = aircraft.verticalSpeed * 196.85;  // m/s to ft/min

    // Background
    glEnable(GL_BLEND);
    glColor4f(0.1f, 0.1f, 0.12f, 0.85f);
    drawRect(x, y, x + w, y + h);

    float centerY = y + h / 2.0f;
    float pixPerHundred = h / 40.0f;  // ±2000 fpm range

    // Scale markings
    glColor3f(0.7f, 0.7f, 0.7f);
    glLineWidth(1.0f);
    for (int vs = -2000; vs <= 2000; vs += 500) {
        float yPos = centerY + (float)vs * pixPerHundred / 100.0f;
        drawLine2D(x, yPos, x + 8, yPos);
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", abs(vs / 100));
        drawText2D(x + 10, yPos - 4, buf, GLUT_BITMAP_HELVETICA_10);
    }

    // Center line (zero)
    glColor3f(0.6f, 0.6f, 0.6f);
    drawLine2D(x, centerY, x + w, centerY);

    // VSI pointer
    float clampedVsi = (float)clampd(vsiFpm, -2000.0, 2000.0);
    float pointerY = centerY + clampedVsi * pixPerHundred / 100.0f;
    glColor3f(0.0f, 1.0f, 0.0f);
    glLineWidth(2.0f);
    drawLine2D(x, centerY, x + w - 5, pointerY);

    // Value readout
    glColor3f(0.0f, 1.0f, 0.0f);
    char vsiBuf[16];
    snprintf(vsiBuf, sizeof(vsiBuf), "%+.0f", vsiFpm);
    drawText2D(x + 2, y + 5, vsiBuf, GLUT_BITMAP_HELVETICA_10);

    // Label
    glColor3f(0.7f, 0.7f, 0.7f);
    drawText2D(x + 2, y + h + 3, "VS", GLUT_BITMAP_HELVETICA_10);

    glDisable(GL_BLEND);
}

// Draw heading indicator bar
void drawHeadingIndicator(float x, float y, float w, float h) {
    double headingDeg = aircraft.yaw * RAD2DEG;

    // Background
    glEnable(GL_BLEND);
    glColor4f(0.1f, 0.1f, 0.12f, 0.85f);
    drawRect(x, y, x + w, y + h);

    float centerX = x + w / 2.0f;
    float pixPerDeg = w / 60.0f;  // ±30 degrees visible

    glEnable(GL_SCISSOR_TEST);
    glScissor((int)x, (int)y, (int)w, (int)h);

    // Draw heading scale
    for (int hdg = 0; hdg < 360; hdg += 5) {
        double diff = hdg - headingDeg;
        // Wrap to -180..180
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;

        float xPos = centerX + (float)diff * pixPerDeg;
        if (xPos < x - 20 || xPos > x + w + 20) continue;

        glColor3f(0.8f, 0.8f, 0.8f);
        if (hdg % 10 == 0) {
            drawLine2D(xPos, y, xPos, y + 10);
            const char* label = nullptr;
            char buf[8];
            switch (hdg) {
                case 0:   label = "N"; break;
                case 90:  label = "E"; break;
                case 180: label = "S"; break;
                case 270: label = "W"; break;
                default:
                    snprintf(buf, sizeof(buf), "%d", hdg / 10);
                    label = buf;
            }
            if (label) {
                float textX = xPos - 4;
                if (hdg % 90 == 0) {
                    glColor3f(1.0f, 0.8f, 0.0f); // cardinal directions in gold
                }
                drawText2D(textX, y + 12, label, GLUT_BITMAP_HELVETICA_10);
                glColor3f(0.8f, 0.8f, 0.8f);
            }
        } else {
            drawLine2D(xPos, y, xPos, y + 6);
        }
    }

    glDisable(GL_SCISSOR_TEST);

    // Center pointer (triangle at top)
    glColor3f(1.0f, 1.0f, 0.0f);
    glBegin(GL_TRIANGLES);
    glVertex2f(centerX, y + h);
    glVertex2f(centerX - 5, y + h + 8);
    glVertex2f(centerX + 5, y + h + 8);
    glEnd();

    // Heading readout
    char hdgBuf[8];
    snprintf(hdgBuf, sizeof(hdgBuf), "%03.0f", headingDeg);
    glColor3f(0.0f, 1.0f, 0.0f);
    drawText2D(centerX - 10, y + h + 10, hdgBuf, GLUT_BITMAP_HELVETICA_10);

    glDisable(GL_BLEND);
}

// Draw slip/skid ball indicator
void drawSlipBall(float cx, float cy, float w) {
    // Background tube
    glColor4f(0.1f, 0.1f, 0.12f, 0.85f);
    glEnable(GL_BLEND);
    drawRect(cx - w / 2.0f, cy - 6, cx + w / 2.0f, cy + 6);

    // Center marks (coordinated flight zone)
    glColor3f(0.6f, 0.6f, 0.6f);
    glLineWidth(1.5f);
    drawLine2D(cx - 8, cy - 6, cx - 8, cy + 6);
    drawLine2D(cx + 8, cy - 6, cx + 8, cy + 6);

    // Ball position based on sideslip
    float ballOffset = (float)clampd(aircraft.sideslip * RAD2DEG * 5.0, -w / 2.0 + 8.0, w / 2.0 - 8.0);

    // Draw ball
    glColor3f(0.95f, 0.95f, 0.95f);  // white ball
    float bx = cx + ballOffset;

    // Filled circle approximation
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(bx, cy);
    for (int i = 0; i <= 16; i++) {
        float a = (float)i * 2.0f * (float)PI / 16.0f;
        glVertex2f(bx + 5.0f * cos(a), cy + 5.0f * sin(a));
    }
    glEnd();

    // Ball outline
    glColor3f(0.3f, 0.3f, 0.3f);
    glLineWidth(1.0f);
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i <= 16; i++) {
        float a = (float)i * 2.0f * (float)PI / 16.0f;
        glVertex2f(bx + 5.0f * cos(a), cy + 5.0f * sin(a));
    }
    glEnd();

    glDisable(GL_BLEND);
}

// Draw the status bar at top of HUD
void drawStatusBar(float x, float y, float w, float h) {
    glEnable(GL_BLEND);
    glColor4f(0.08f, 0.08f, 0.1f, 0.9f);
    drawRect(x, y, x + w, y + h);

    float textY = y + 5;
    char buf[64];

    // Throttle
    glColor3f(0.7f, 0.9f, 0.7f);
    snprintf(buf, sizeof(buf), "THR: %3.0f%%", aircraft.throttle * 100.0);
    drawText2D(x + w * 0.02f, textY, buf, GLUT_BITMAP_HELVETICA_12);

    // Ground speed
    double gsKts = sqrt(aircraft.vx * aircraft.vx + aircraft.vz * aircraft.vz) * 1.94384;
    snprintf(buf, sizeof(buf), "GS: %3.0f kts", gsKts);
    drawText2D(x + w * 0.14f, textY, buf, GLUT_BITMAP_HELVETICA_12);

    // Altitude
    snprintf(buf, sizeof(buf), "ALT: %5.0f ft", aircraft.py * 3.28084);
    drawText2D(x + w * 0.28f, textY, buf, GLUT_BITMAP_HELVETICA_12);

    // VS
    snprintf(buf, sizeof(buf), "VS: %+5.0f fpm", aircraft.verticalSpeed * 196.85);
    drawText2D(x + w * 0.42f, textY, buf, GLUT_BITMAP_HELVETICA_12);

    // Gear
    glColor3f(0.0f, 1.0f, 0.0f);
    drawText2D(x + w * 0.56f, textY, "GEAR: DOWN", GLUT_BITMAP_HELVETICA_12);

    // Brakes
    if (aircraft.brakes) {
        glColor3f(1.0f, 0.5f, 0.0f);
        drawText2D(x + w * 0.70f, textY, "BRK", GLUT_BITMAP_HELVETICA_12);
    }

    // Flaps
    glColor3f(0.7f, 0.9f, 0.7f);
    snprintf(buf, sizeof(buf), "FLP: %2.0f deg", aircraft.flaps);
    drawText2D(x + w * 0.76f, textY, buf, GLUT_BITMAP_HELVETICA_12);

    // Autopilot
    if (aircraft.apActive) {
        glColor3f(0.0f, 1.0f, 0.0f); // Green
        snprintf(buf, sizeof(buf), "AP:%s%s", aircraft.apHdgHold ? " HDG" : "", aircraft.apAltHold ? " ALT" : "");
        drawText2D(x + w * 0.88f, textY, buf, GLUT_BITMAP_HELVETICA_12);
    } else {
        glColor3f(0.5f, 0.5f, 0.5f); // Gray
        drawText2D(x + w * 0.88f, textY, "AP: OFF", GLUT_BITMAP_HELVETICA_12);
    }

    glDisable(GL_BLEND);
}

// Stall warning overlay
void drawStallWarning() {
    if (!aircraft.stallWarning) return;

    // Flashing red border
    double flash = sin(config.simTime * 8.0);
    if (flash > 0) {
        glColor4f(1.0f, 0.0f, 0.0f, 0.4f);
        glEnable(GL_BLEND);
        // Top
        drawRect(0, (float)windowHeight - 30, (float)windowWidth, (float)windowHeight);
        // Bottom
        drawRect(0, 0, (float)windowWidth, 30);
        // Left
        drawRect(0, 0, 30, (float)windowHeight);
        // Right
        drawRect((float)windowWidth - 30, 0, (float)windowWidth, (float)windowHeight);
        glDisable(GL_BLEND);

        // STALL text
        glColor3f(1.0f, 0.0f, 0.0f);
        drawText2D((float)windowWidth / 2.0f - 30, (float)windowHeight - 50, "!! STALL !!", GLUT_BITMAP_TIMES_ROMAN_24);
    }

    // Audio beep (Windows only)
#ifdef _WIN32
    static double lastBeep = 0.0;
    if (config.simTime - lastBeep > 0.5) {
        Beep(1200, 100);
        lastBeep = config.simTime;
    }
#endif
}

// Main HUD render function
void renderHUD() {
    if (!config.hudVisible) return;

    // Switch to 2D orthographic projection
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, windowWidth, 0, windowHeight, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // --- HUD Layout ---
    float hudBottom = 10;
    float hudHeight = (float)windowHeight * 0.35f;
    float hudTop    = hudBottom + hudHeight;
    float hudLeft   = (float)windowWidth * 0.15f;
    float hudRight  = (float)windowWidth * 0.85f;
    float hudWidth  = hudRight - hudLeft;

    // Background panel
    glColor4f(0.05f, 0.05f, 0.08f, 0.65f);
    drawRect(hudLeft - 5, hudBottom - 5, hudRight + 5, hudTop + 30);

    // Panel border
    glColor3f(0.3f, 0.5f, 0.3f);
    glLineWidth(1.5f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(hudLeft - 5, hudBottom - 5);
    glVertex2f(hudRight + 5, hudBottom - 5);
    glVertex2f(hudRight + 5, hudTop + 30);
    glVertex2f(hudLeft - 5, hudTop + 30);
    glEnd();

    // Calculate instrument positions
    float aiCenterX = hudLeft + hudWidth * 0.5f;
    float aiCenterY = hudBottom + hudHeight * 0.55f;
    float aiRadius  = hudHeight * 0.42f;

    float spdTapeX = hudLeft + 5;
    float spdTapeW = 55;
    float spdTapeY = hudBottom + 15;
    float spdTapeH = hudHeight - 40;

    float altTapeX = hudRight - 60;
    float altTapeW = 55;
    float altTapeY = hudBottom + 15;
    float altTapeH = hudHeight - 40;

    float vsiX = hudRight - 100;
    float vsiW = 35;
    float vsiY = hudBottom + 15;
    float vsiH = hudHeight - 40;

    // Draw instruments
    drawAirspeedTape(spdTapeX, spdTapeY, spdTapeW, spdTapeH);
    drawAttitudeIndicator(aiCenterX, aiCenterY, aiRadius);
    drawAltimeterTape(altTapeX, altTapeY, altTapeW, altTapeH);
    drawVSI(vsiX, vsiY, vsiW, vsiH);

    // Heading indicator below AI
    float hdgX = aiCenterX - hudWidth * 0.25f;
    float hdgW = hudWidth * 0.5f;
    drawHeadingIndicator(hdgX, hudBottom, hdgW, 25);

    // Slip ball below AI
    drawSlipBall(aiCenterX, hudBottom + 30, 60);

    // Status bar at top of HUD
    drawStatusBar(hudLeft, hudTop + 2, hudWidth, 25);

    // Stall warning
    drawStallWarning();

    // Engine failure warning
    if (environment.engineFailure) {
        glColor3f(1.0f, 0.3f, 0.0f);
        drawText2D((float)windowWidth / 2.0f - 50, (float)windowHeight - 80, "ENGINE FAILURE", GLUT_BITMAP_TIMES_ROMAN_24);
    }

    // FPS counter
    glColor3f(0.5f, 0.5f, 0.5f);
    char fpsBuf[32];
    snprintf(fpsBuf, sizeof(fpsBuf), "FPS: %d", config.fps);
    drawText2D(5, (float)windowHeight - 15, fpsBuf, GLUT_BITMAP_HELVETICA_10);

    // AoA display
    char aoaBuf[32];
    snprintf(aoaBuf, sizeof(aoaBuf), "AoA: %.1f deg", aircraft.aoa * RAD2DEG);
    drawText2D(5, (float)windowHeight - 30, aoaBuf, GLUT_BITMAP_HELVETICA_10);

    // G-load display
    char gBuf[32];
    snprintf(gBuf, sizeof(gBuf), "G: %.2f", aircraft.gLoad);
    drawText2D(5, (float)windowHeight - 45, gBuf, GLUT_BITMAP_HELVETICA_10);

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}


// ============================================================================
// MODULE 8: CONTROLS, INSTRUCTOR PANEL & MAIN LOOP
// ============================================================================

// Draw the Instructor Panel overlay
// Draw the Instructor Panel overlay
void renderInstructorPanel() {
    if (!config.instructorPanelVisible) return;

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, windowWidth, 0, windowHeight, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    float panelW = 230;
    float panelH = 530;
    float panelX = (float)windowWidth - panelW - 15;
    float panelY = (float)windowHeight - panelH - 30;

    // Panel background
    glColor4f(0.08f, 0.08f, 0.12f, 0.92f);
    drawRect(panelX, panelY, panelX + panelW, panelY + panelH);

    // Panel border
    glColor3f(0.3f, 0.6f, 0.9f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(panelX, panelY);
    glVertex2f(panelX + panelW, panelY);
    glVertex2f(panelX + panelW, panelY + panelH);
    glVertex2f(panelX, panelY + panelH);
    glEnd();

    // Title
    glColor3f(0.3f, 0.7f, 1.0f);
    drawText2D(panelX + 45, panelY + panelH - 22, "INSTRUCTOR PANEL", GLUT_BITMAP_HELVETICA_12);

    // Divider line
    glColor3f(0.3f, 0.4f, 0.5f);
    drawLine2D(panelX + 10, panelY + panelH - 30, panelX + panelW - 10, panelY + panelH - 30);

    float lineY = panelY + panelH - 45;
    float lineH = 17;
    char buf[64];

    // Crosswind
    glColor3f(0.8f, 0.8f, 0.8f);
    snprintf(buf, sizeof(buf), "Crosswind: %.0f kts", environment.windSpeed * 1.94384);
    drawText2D(panelX + 15, lineY, buf, GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    // Wind direction
    snprintf(buf, sizeof(buf), "Direction: %.0f deg", environment.windDirection);
    drawText2D(panelX + 15, lineY, buf, GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    // Crosswind bar
    glColor3f(0.3f, 0.3f, 0.3f);
    drawRect(panelX + 15, lineY + 3, panelX + panelW - 15, lineY + 11);
    glColor3f(0.0f, 0.7f, 1.0f);
    float windBar = (float)clampd(environment.windSpeed / 15.0, 0, 1.0);
    drawRect(panelX + 15, lineY + 3, panelX + 15 + (panelW - 30) * windBar, lineY + 11);
    lineY -= lineH + 2;

    // Controls hint
    glColor3f(0.5f, 0.5f, 0.6f);
    drawText2D(panelX + 15, lineY, "[1/2] Wind -/+", GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    // Divider
    glColor3f(0.3f, 0.4f, 0.5f);
    drawLine2D(panelX + 10, lineY + 5, panelX + panelW - 10, lineY + 5);
    lineY -= 8;

    // Fog
    glColor3f(0.8f, 0.8f, 0.8f);
    snprintf(buf, sizeof(buf), "Fog: %s", environment.fogEnabled ? "ON" : "OFF");
    drawText2D(panelX + 15, lineY, buf, GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    snprintf(buf, sizeof(buf), "Density: %.3f", environment.fogDensity);
    drawText2D(panelX + 15, lineY, buf, GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    glColor3f(0.5f, 0.5f, 0.6f);
    drawText2D(panelX + 15, lineY, "[O] Toggle  [/] Density", GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    // Divider
    glColor3f(0.3f, 0.4f, 0.5f);
    drawLine2D(panelX + 10, lineY + 5, panelX + panelW - 10, lineY + 5);
    lineY -= 8;

    // Engine failure
    if (environment.engineFailure) {
        glColor3f(1.0f, 0.3f, 0.0f);
    } else {
        glColor3f(0.0f, 0.8f, 0.0f);
    }
    snprintf(buf, sizeof(buf), "Engine: %s", environment.engineFailure ? "FAILED" : "RUNNING");
    drawText2D(panelX + 15, lineY, buf, GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    glColor3f(0.5f, 0.5f, 0.6f);
    drawText2D(panelX + 15, lineY, "[3] Toggle failure", GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    // Turbulence
    glColor3f(0.8f, 0.8f, 0.8f);
    const char* turbLabels[] = {"NONE", "LOW", "MEDIUM"};
    snprintf(buf, sizeof(buf), "Turbulence: %s", turbLabels[environment.turbulenceLevel]);
    drawText2D(panelX + 15, lineY, buf, GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    glColor3f(0.5f, 0.5f, 0.6f);
    drawText2D(panelX + 15, lineY, "[4] Cycle level", GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    // Divider
    glColor3f(0.3f, 0.4f, 0.5f);
    drawLine2D(panelX + 10, lineY + 5, panelX + panelW - 10, lineY + 5);
    lineY -= 8;

    // Flaps
    glColor3f(0.8f, 0.8f, 0.8f);
    snprintf(buf, sizeof(buf), "Flaps: %.0f deg (Target: %.0f)", aircraft.flaps, aircraft.flapsTarget);
    drawText2D(panelX + 15, lineY, buf, GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    glColor3f(0.5f, 0.5f, 0.6f);
    drawText2D(panelX + 15, lineY, "[ [ / ] ] Flaps retract/extend", GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    // Divider
    glColor3f(0.3f, 0.4f, 0.5f);
    drawLine2D(panelX + 10, lineY + 5, panelX + panelW - 10, lineY + 5);
    lineY -= 8;

    // Autopilot
    if (aircraft.apActive) {
        glColor3f(0.0f, 0.8f, 0.0f); // Green
        snprintf(buf, sizeof(buf), "AP: ACTIVE (%s%s)", aircraft.apHdgHold ? "HDG " : "", aircraft.apAltHold ? "ALT" : "");
    } else {
        glColor3f(0.6f, 0.6f, 0.6f); // Gray
        snprintf(buf, sizeof(buf), "AP: OFF");
    }
    drawText2D(panelX + 15, lineY, buf, GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    glColor3f(0.8f, 0.8f, 0.8f);
    snprintf(buf, sizeof(buf), "AP Target HDG: %03.0f deg (F5/F6)", aircraft.apTargetHdg);
    drawText2D(panelX + 15, lineY, buf, GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    snprintf(buf, sizeof(buf), "AP Target ALT: %5.0f ft (F7/F8)", aircraft.apTargetAlt);
    drawText2D(panelX + 15, lineY, buf, GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    glColor3f(0.5f, 0.5f, 0.6f);
    drawText2D(panelX + 15, lineY, "[T] AP Master  [Y] HDG Hold  [U] ALT Hold", GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    // Divider
    glColor3f(0.3f, 0.4f, 0.5f);
    drawLine2D(panelX + 10, lineY + 5, panelX + panelW - 10, lineY + 5);
    lineY -= 8;

    // Scenarios
    glColor3f(0.3f, 0.7f, 1.0f);
    drawText2D(panelX + 15, lineY, "Training Scenarios:", GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;
    glColor3f(0.85f, 0.85f, 0.9f);
    drawText2D(panelX + 15, lineY, "[5] Takeoff Roll  [6] 3-Mile Final", GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;
    drawText2D(panelX + 15, lineY, "[7] Cruise Flight Preset", GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    // Divider
    glColor3f(0.3f, 0.4f, 0.5f);
    drawLine2D(panelX + 10, lineY + 5, panelX + panelW - 10, lineY + 5);
    lineY -= 8;

    // Telemetry
    glColor3f(0.3f, 0.7f, 1.0f);
    drawText2D(panelX + 60, lineY, "-- Telemetry --", GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    glColor3f(0.7f, 0.9f, 0.7f);
    snprintf(buf, sizeof(buf), "AoA: %.1f deg", aircraft.aoa * RAD2DEG);
    drawText2D(panelX + 15, lineY, buf, GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    snprintf(buf, sizeof(buf), "G-Load: %.2f", aircraft.gLoad);
    drawText2D(panelX + 15, lineY, buf, GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    snprintf(buf, sizeof(buf), "Stall: %s", aircraft.stallWarning ? "YES!" : "NO");
    if (aircraft.stallWarning) glColor3f(1.0f, 0.0f, 0.0f);
    drawText2D(panelX + 15, lineY, buf, GLUT_BITMAP_HELVETICA_10);
    lineY -= lineH;

    snprintf(buf, sizeof(buf), "On Ground: %s", aircraft.onGround ? "YES" : "NO");
    glColor3f(0.7f, 0.9f, 0.7f);
    drawText2D(panelX + 15, lineY, buf, GLUT_BITMAP_HELVETICA_10);

    // Reset button hint
    glColor3f(0.9f, 0.5f, 0.2f);
    drawText2D(panelX + 50, panelY + 10, "[R] Reset Aircraft", GLUT_BITMAP_HELVETICA_10);

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

// Draw controls help overlay (shown briefly at start)
void renderControlsHelp() {
    // Only show for first 10 seconds or when paused
    if (config.simTime > 10.0 && !config.paused) return;

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, windowWidth, 0, windowHeight, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    float alpha = 1.0f;
    if (config.simTime > 7.0 && !config.paused) {
        alpha = 1.0f - (float)(config.simTime - 7.0) / 3.0f;
    }

    float px = 15;
    float py = (float)windowHeight / 2.0f + 80;

    glColor4f(0.05f, 0.05f, 0.1f, 0.8f * alpha);
    drawRect(px - 5, py - 245, px + 215, py + 25);

    glColor4f(0.3f, 0.7f, 1.0f, alpha);
    drawText2D(px + 40, py, "FLIGHT CONTROLS", GLUT_BITMAP_HELVETICA_12);

    glColor4f(0.85f, 0.85f, 0.85f, alpha);
    float ly = py - 22;
    float lh = 15;
    drawText2D(px + 5, ly, "W/S   - Pitch down/up");     ly -= lh;
    drawText2D(px + 5, ly, "A/D   - Roll left/right");    ly -= lh;
    drawText2D(px + 5, ly, "Q/E   - Rudder left/right");  ly -= lh;
    drawText2D(px + 5, ly, "+/-   - Throttle up/down");   ly -= lh;
    drawText2D(px + 5, ly, "[ / ] - Flaps up/down");       ly -= lh;
    drawText2D(px + 5, ly, "T/Y/U - AP / HDG / ALT");     ly -= lh;
    drawText2D(px + 5, ly, "5/6/7 - Training Scenarios");  ly -= lh;
    drawText2D(px + 5, ly, "B     - Toggle brakes");      ly -= lh;
    drawText2D(px + 5, ly, "V     - Toggle camera");      ly -= lh;
    drawText2D(px + 5, ly, "H     - Toggle HUD");         ly -= lh;
    drawText2D(px + 5, ly, "I     - Instructor panel");   ly -= lh;
    drawText2D(px + 5, ly, "P     - Pause");              ly -= lh;
    drawText2D(px + 5, ly, "R     - Reset aircraft");     ly -= lh;
    drawText2D(px + 5, ly, "ESC   - Exit");

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void triggerScenario(int id) {
    if (id == 1) {
        // Takeoff Roll
        initAircraft();
        aircraft.px = 0.0;
        aircraft.py = 0.0;
        aircraft.pz = 50.0;
        aircraft.yaw = 0.0;
        aircraft.vx = 0.0;
        aircraft.vy = 0.0;
        aircraft.vz = 0.0;
        aircraft.throttle = 1.0; // full throttle
        aircraft.brakes = false;
        aircraft.flaps = 0.0;
        aircraft.flapsTarget = 0.0;
        aircraft.apActive = false;
    } else if (id == 2) {
        // 3-Mile Final Approach
        initAircraft();
        aircraft.px = 0.0;
        aircraft.pz = -4500.0; // 4.5 km out
        aircraft.py = 235.0; // ~770 ft
        aircraft.yaw = 0.0;
        aircraft.pitch = 0.0;
        aircraft.roll = 0.0;
        aircraft.vx = 0.0;
        aircraft.vy = -1.75; // glide path sink rate
        aircraft.vz = 33.4; // 65 knots
        aircraft.throttle = 0.55;
        aircraft.brakes = false;
        aircraft.onGround = false;
        aircraft.flaps = 10.0;
        aircraft.flapsTarget = 10.0;
        aircraft.apActive = false;
    } else if (id == 3) {
        // Cruise
        initAircraft();
        aircraft.px = 0.0;
        aircraft.py = 914.4; // 3000 ft
        aircraft.pz = 2000.0;
        aircraft.yaw = 0.0;
        aircraft.pitch = 0.0;
        aircraft.roll = 0.0;
        aircraft.vx = 0.0;
        aircraft.vy = 0.0;
        aircraft.vz = 51.4; // 100 knots
        aircraft.throttle = 0.75;
        aircraft.brakes = false;
        aircraft.onGround = false;
        aircraft.flaps = 0.0;
        aircraft.flapsTarget = 0.0;
        aircraft.apActive = false;
    }
    
    // Reset camera smoothing
    camX = aircraft.px;
    camY = aircraft.py + 6.0;
    camZ = aircraft.pz - 18.0;
}

// ---- Keyboard Callbacks ----

void keyboardDown(unsigned char key, int /*x*/, int /*y*/) {
    keyStates[key] = true;

    // Check for shift/ctrl modifiers
    int mods = glutGetModifiers();
    if (mods & GLUT_ACTIVE_SHIFT) keyStates[1] = true;
    if (mods & GLUT_ACTIVE_CTRL)  keyStates[2] = true;

    // Toggle keys (press once)
    switch (key) {
        case 'v': case 'V':
            config.cameraMode = (config.cameraMode + 1) % 2;
            break;
        case 'p': case 'P':
            config.paused = !config.paused;
            break;
        case 'h': case 'H':
            config.hudVisible = !config.hudVisible;
            break;
        case 'i': case 'I':
            config.instructorPanelVisible = !config.instructorPanelVisible;
            break;
        case 'b': case 'B':
            aircraft.brakes = !aircraft.brakes;
            break;
        case 'r': case 'R':
            initAircraft();
            camX = 0; camY = 5; camZ = -15;  // reset camera
            break;
        case 'o': case 'O':
            environment.fogEnabled = !environment.fogEnabled;
            break;
        case '1':
            environment.windSpeed = std::max(0.0, environment.windSpeed - 1.0);
            environment.windX = environment.windSpeed * sin(environment.windDirection * DEG2RAD);
            environment.windZ = environment.windSpeed * cos(environment.windDirection * DEG2RAD);
            break;
        case '2':
            environment.windSpeed = std::min(15.0, environment.windSpeed + 1.0);
            environment.windX = environment.windSpeed * sin(environment.windDirection * DEG2RAD);
            environment.windZ = environment.windSpeed * cos(environment.windDirection * DEG2RAD);
            break;
        case '3':
            environment.engineFailure = !environment.engineFailure;
            break;
        case '4':
            environment.turbulenceLevel = (environment.turbulenceLevel + 1) % 3;
            break;
        case '[':
            aircraft.flapsTarget = std::max(0.0, aircraft.flapsTarget - 10.0);
            break;
        case ']':
            aircraft.flapsTarget = std::min(30.0, aircraft.flapsTarget + 10.0);
            break;
        case '/':
            environment.fogDensity += 0.001;
            if (environment.fogDensity > 0.01) environment.fogDensity = 0.001;
            break;
        case 't': case 'T':
            aircraft.apActive = !aircraft.apActive;
            if (aircraft.apActive) {
                aircraft.apTargetHdg = aircraft.yaw * RAD2DEG;
                aircraft.apTargetAlt = aircraft.py * 3.28084;
                aircraft.apHdgHold = true;
                aircraft.apAltHold = true;
            }
            break;
        case 'y': case 'Y':
            aircraft.apHdgHold = !aircraft.apHdgHold;
            if (aircraft.apHdgHold) {
                aircraft.apTargetHdg = aircraft.yaw * RAD2DEG;
            }
            break;
        case 'u': case 'U':
            aircraft.apAltHold = !aircraft.apAltHold;
            if (aircraft.apAltHold) {
                aircraft.apTargetAlt = aircraft.py * 3.28084;
            }
            break;
        case '5':
            triggerScenario(1);
            break;
        case '6':
            triggerScenario(2);
            break;
        case '7':
            triggerScenario(3);
            break;
        case 27: // ESC
            exit(0);
            break;
    }
}

void keyboardUp(unsigned char key, int /*x*/, int /*y*/) {
    keyStates[key] = false;

    // Clear modifier flags on any key up
    int mods = glutGetModifiers();
    if (!(mods & GLUT_ACTIVE_SHIFT)) keyStates[1] = false;
    if (!(mods & GLUT_ACTIVE_CTRL))  keyStates[2] = false;
}

void specialDown(int key, int /*x*/, int /*y*/) {
    if (key >= 0 && key < 256) specialKeys[key] = true;

    // Autopilot Target Adjustments
    switch (key) {
        case GLUT_KEY_F5:
            aircraft.apTargetHdg -= 1.0;
            if (aircraft.apTargetHdg < 0.0) aircraft.apTargetHdg += 360.0;
            break;
        case GLUT_KEY_F6:
            aircraft.apTargetHdg += 1.0;
            if (aircraft.apTargetHdg >= 360.0) aircraft.apTargetHdg -= 360.0;
            break;
        case GLUT_KEY_F7:
            aircraft.apTargetAlt = std::max(0.0, aircraft.apTargetAlt - 100.0);
            break;
        case GLUT_KEY_F8:
            aircraft.apTargetAlt += 100.0;
            break;
    }
}

void specialUp(int key, int /*x*/, int /*y*/) {
    if (key >= 0 && key < 256) specialKeys[key] = false;
}

// ---- Window Reshape ----
void reshape(int w, int h) {
    windowWidth = w;
    windowHeight = h;
    if (h == 0) h = 1;
    glViewport(0, 0, w, h);
}

// ---- Main Render Loop ----
void display() {
    // ---- Timing ----
    double currentTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
    double frameTime = currentTime - lastTime;
    lastTime = currentTime;

    // Cap frame time to prevent spiral of death
    if (frameTime > 0.1) frameTime = 0.1;

    // FPS calculation
    config.frameCount++;
    config.fpsTimer += frameTime;
    if (config.fpsTimer >= 1.0) {
        config.fps = config.frameCount;
        config.frameCount = 0;
        config.fpsTimer -= 1.0;
    }

    // ---- Physics update (fixed timestep) ----
    accumulator += frameTime;
    while (accumulator >= PHYSICS_DT) {
        updatePhysics(PHYSICS_DT);
        accumulator -= PHYSICS_DT;
    }

    // ---- Render ----
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Sky background (before perspective setup)
    renderSky();

    // Setup 3D camera
    setupCamera();

    // Fog
    setupFog();

    // 3D Scene
    renderGround();
    renderRunway();
    renderWindsock();

    // Aircraft model (only in chase view)
    if (config.cameraMode == 1) {
        renderAircraftModel();
    }

    // 2D HUD overlay
    renderHUD();

    // Instructor panel
    renderInstructorPanel();

    // Controls help (fades out)
    renderControlsHelp();

    // Paused overlay
    if (config.paused) {
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, windowWidth, 0, windowHeight, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glColor4f(0.0f, 0.0f, 0.0f, 0.4f);
        drawRect(0, 0, (float)windowWidth, (float)windowHeight);
        glColor3f(1.0f, 1.0f, 0.0f);
        drawText2D((float)windowWidth / 2.0f - 30, (float)windowHeight / 2.0f, "PAUSED", GLUT_BITMAP_TIMES_ROMAN_24);
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();
    }

    glutSwapBuffers();
    glutPostRedisplay();
}

// ---- Entry Point ----
int main(int argc, char** argv) {
    srand((unsigned int)time(nullptr));

    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(windowWidth, windowHeight);
    glutInitWindowPosition(100, 50);
    glutCreateWindow("Cessna 172 Flight Simulator — Educational Trainer");

    // Initialize GLEW
    GLenum glewErr = glewInit();
    if (glewErr != GLEW_OK) {
        fprintf(stderr, "GLEW Error: %s\n", glewGetErrorString(glewErr));
        return 1;
    }

    printf("OpenGL Version: %s\n", glGetString(GL_VERSION));
    printf("GLEW Version:   %s\n", glewGetString(GLEW_VERSION));
    printf("\n--- Cessna 172 Flight Simulator ---\n");
    printf("Controls: W/S=Pitch, A/D=Roll, Q/E=Rudder, +/-=Throttle\n");
    printf("          V=Camera, H=HUD, I=Instructor, P=Pause, R=Reset\n\n");

    // OpenGL setup
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Initialize simulation state
    initAircraft();
    initEnvironment();
    initConfig();

    lastTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0;

    // Register callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboardDown);
    glutKeyboardUpFunc(keyboardUp);
    glutSpecialFunc(specialDown);
    glutSpecialUpFunc(specialUp);
    glutIgnoreKeyRepeat(1);  // prevent auto-repeat

    // Enter main loop
    glutMainLoop();

    return 0;
}
