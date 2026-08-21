#include "SimHexapodAPI.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <thread>

// Simulator tuning constants. These approximate the real hardware behaviour
// described in the SmarPod Programmer's Guide; they are not exact figures.
constexpr double PI = 3.14159265358979323846;
constexpr double STAGE_RADIUS = 0.05;           // m, maps rotation to an equivalent arc length
constexpr double FREQ_STEP_SIZE = 1e-7;         // m per Hz when speed-control is disabled
constexpr double DEFAULT_NO_SC_SPEED = 0.001;   // m/s fallback velocity
constexpr double SENSOR_POWERUP_DELAY = 0.005;  // s, power-save sensor wake time
constexpr double WORKSPACE_LINEAR = 0.1;        // m, per-axis half-range
constexpr double WORKSPACE_ANGULAR = 20.0;      // deg, per-axis half-range
constexpr double JOG_LINEAR = 0.001;            // m, per-axis jog during referencing/calibration
constexpr double JOG_ANGULAR = 0.5;             // deg, per-axis jog during referencing/calibration
constexpr unsigned int HOLDTIME_INFINITE = 60000;

Pose interpolate(const Pose& a, const Pose& b, double f) {
    Pose p;
    p.x = a.x + (b.x - a.x) * f;
    p.y = a.y + (b.y - a.y) * f;
    p.z = a.z + (b.z - a.z) * f;
    p.rx = a.rx + (b.rx - a.rx) * f;
    p.ry = a.ry + (b.ry - a.ry) * f;
    p.rz = a.rz + (b.rz - a.rz) * f;
    return p;
}

void SimHexapod::SetMaxFrequency(unsigned int frequency) { this->maxFrequency = frequency; }

unsigned int SimHexapod::GetMaxFrequency() { return this->maxFrequency; }

void SimHexapod::SetSpeed(double speed, bool enableSpeedControl) {
    this->speed = speed;
    this->speedControlEnabled = enableSpeedControl;
}

std::tuple<double, bool> SimHexapod::GetSpeed() {
    return std::make_tuple(this->speed, this->speedControlEnabled);
}

void SimHexapod::SetAcceleration(double acceleration, bool enableAccelControl) {
    this->acceleration = acceleration;
    this->acclControlEnabled = enableAccelControl;
}
std::tuple<double, bool> SimHexapod::GetAcceleration() {
    return std::make_tuple(this->acceleration, this->acclControlEnabled);
}

void SimHexapod::FindReferenceMarks() {
    this->jogAllAxes(MoveStatus::REFERENCING);
    this->referenced = true;
}
void SimHexapod::Calibrate() {
    this->jogAllAxes(MoveStatus::CALIBRATING);
    this->calibrated = true;
}
bool SimHexapod::IsReferenced() { return this->referenced; }

void SimHexapod::SetSensorMode(SensorMode mode) { this->sensorMode = mode; }

SensorMode SimHexapod::GetSensorMode() { return this->sensorMode; }

void SimHexapod::SetPivot(double x, double y, double z) {
    this->pivotX = x;
    this->pivotY = y;
    this->pivotZ = z;
}

std::tuple<double, double, double> SimHexapod::GetPivot() {
    return std::make_tuple(this->pivotX, this->pivotY, this->pivotZ);
}

bool SimHexapod::IsPoseReachable(const Pose& pose) { return this->isWithinWorkspace(pose); }

Pose SimHexapod::GetPose() {
    // The pose is only known once the positioners have been referenced.
    if (!this->referenced) {
        throw std::runtime_error("Cannot get pose: SmarPod is not referenced");
    }
    this->advance();
    if (this->moveStatus.load() == MoveStatus::MOVING) {
        auto now = std::chrono::steady_clock::now();
        double total =
            std::chrono::duration<double>(this->moveEndTime - this->moveStartTime).count();
        double elapsed = std::chrono::duration<double>(now - this->moveStartTime).count();
        double f = total > 0.0 ? std::min(1.0, elapsed / total) : 1.0;
        return interpolate(this->moveStartPose, this->moveTargetPose, f);
    }
    return this->currentPose;
}

void SimHexapod::Move(const Pose& pose, unsigned int holdTime, bool waitForCompletion) {
    if (!this->referenced) {
        throw std::runtime_error("Cannot move: SmarPod is not referenced");
    }
    if (this->sensorMode == SensorMode::DISABLED) {
        throw std::runtime_error("Cannot move: SmarPod sensors are disabled");
    }
    if (!this->isWithinWorkspace(pose)) {
        throw std::runtime_error("Cannot move: target pose is unreachable");
    }

    double duration = this->computeMoveDuration(this->currentPose, pose);
    auto now = std::chrono::steady_clock::now();
    this->moveStartPose = this->currentPose;
    this->moveTargetPose = pose;
    this->moveStartTime = now;
    this->moveEndTime = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(duration));
    this->holdInfinite = (holdTime >= HOLDTIME_INFINITE);
    this->holdEndTime =
        this->moveEndTime + std::chrono::milliseconds(this->holdInfinite ? 0 : holdTime);
    this->moveStatus = MoveStatus::MOVING;

    if (waitForCompletion) {
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        this->currentPose = pose;
        this->advance();
    }
}

void SimHexapod::Stop() {
    this->currentPose = this->GetPose();
    this->moveStatus = MoveStatus::STOPPED;
}

void SimHexapod::StopAndHold(unsigned int holdTime) {
    this->currentPose = this->GetPose();
    auto now = std::chrono::steady_clock::now();
    this->holdInfinite = (holdTime >= HOLDTIME_INFINITE);
    this->holdEndTime = now + std::chrono::milliseconds(this->holdInfinite ? 0 : holdTime);
    this->moveStatus = MoveStatus::HOLDING;
}

void SimHexapod::Standby() {
    this->currentPose = this->GetPose();
    this->moveStatus = MoveStatus::STANDBY;
}

bool SimHexapod::isWithinWorkspace(const Pose& pose) const {
    // The reachable pose range shifts with the coordinate system and axes
    // orientation, so test the pose expressed in the default coordinate system.
    double px = this->coordSystem.x + pose.x;
    double py = this->coordSystem.y + pose.y;
    double pz = this->coordSystem.z + pose.z;
    double prx = this->coordSystem.rx + this->axisRx + pose.rx;
    double pry = this->coordSystem.ry + this->axisRy + pose.ry;
    double prz = this->coordSystem.rz + this->axisRz + pose.rz;
    return std::abs(px) <= WORKSPACE_LINEAR && std::abs(py) <= WORKSPACE_LINEAR &&
           std::abs(pz) <= WORKSPACE_LINEAR && std::abs(prx) <= WORKSPACE_ANGULAR &&
           std::abs(pry) <= WORKSPACE_ANGULAR && std::abs(prz) <= WORKSPACE_ANGULAR;
}

double SimHexapod::computeMoveDuration(const Pose& from, const Pose& to) const {
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double dz = to.z - from.z;
    double linearDist = std::sqrt(dx * dx + dy * dy + dz * dz);

    double drx = to.rx - from.rx;
    double dry = to.ry - from.ry;
    double drz = to.rz - from.rz;
    double angularDist = std::sqrt(drx * drx + dry * dry + drz * drz);
    double angularArc = (angularDist * PI / 180.0) * STAGE_RADIUS;

    double dist = linearDist + angularArc;
    if (dist <= 0.0) return 0.0;

    double velocity;
    if (this->speedControlEnabled && this->speed > 0.0) {
        velocity = this->speed;
    } else if (this->maxFrequency > 0) {
        velocity = this->maxFrequency * FREQ_STEP_SIZE;
    } else {
        velocity = DEFAULT_NO_SC_SPEED;
    }

    double t = dist / velocity;

    // Acceleration-control only has an effect when speed-control is enabled.
    if (this->acclControlEnabled && this->speedControlEnabled && this->acceleration > 0.0) {
        double rampDist = (velocity * velocity) / this->acceleration;
        if (dist >= rampDist) {
            t += velocity / this->acceleration;  // trapezoidal profile overhead
        } else {
            t = 2.0 * std::sqrt(dist / this->acceleration);  // triangular profile
        }
    }

    // Power-save mode wakes the sensors before moving, delaying the start.
    if (this->sensorMode == SensorMode::POWERSAVE) {
        t += SENSOR_POWERUP_DELAY;
    }
    return t;
}

void SimHexapod::advance() {
    auto now = std::chrono::steady_clock::now();
    MoveStatus status = this->moveStatus.load();
    if (status == MoveStatus::MOVING && now >= this->moveEndTime) {
        this->currentPose = this->moveTargetPose;
        if (this->holdInfinite || now < this->holdEndTime) {
            this->moveStatus = MoveStatus::HOLDING;
        } else {
            this->moveStatus = MoveStatus::STOPPED;
        }
    } else if (status == MoveStatus::HOLDING && !this->holdInfinite && now >= this->holdEndTime) {
        this->moveStatus = MoveStatus::STOPPED;
    }
}

void SimHexapod::jogAllAxes(MoveStatus activeStatus) {
    // Referencing and calibration jog every positioner a little with a fixed
    // frequency (find-ref-and-calibration frequency, else the max frequency),
    // not speed control. The axes return to their starting pose afterwards.
    double velocity;
    if (this->frefAndCalibFreq > 0.0) {
        velocity = this->frefAndCalibFreq * FREQ_STEP_SIZE;
    } else if (this->maxFrequency > 0) {
        velocity = this->maxFrequency * FREQ_STEP_SIZE;
    } else {
        velocity = DEFAULT_NO_SC_SPEED;
    }

    // Each of the three linear and three rotary axes jogs out and back.
    double linearJog = 3.0 * (2.0 * JOG_LINEAR);
    double angularJog = 3.0 * ((2.0 * JOG_ANGULAR) * PI / 180.0) * STAGE_RADIUS;
    double dist = linearJog + angularJog;
    double duration = velocity > 0.0 ? dist / velocity : 0.0;

    auto now = std::chrono::steady_clock::now();
    this->moveStartPose = this->currentPose;
    this->moveTargetPose = this->currentPose;  // net movement is zero
    this->moveStartTime = now;
    this->moveEndTime = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(duration));
    this->holdInfinite = false;
    this->holdEndTime = this->moveEndTime;
    this->moveStatus = activeStatus;

    std::this_thread::sleep_for(std::chrono::duration<double>(duration));
    this->moveStatus = MoveStatus::STOPPED;
}

void SimHexapod::SetCoordinateSystem(const Pose& pose) { this->coordSystem = pose; }

Pose SimHexapod::GetCoordinateSystem() { return this->coordSystem; }

void SimHexapod::SetCurrentPoseAsZero() { this->currentPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; }

void SimHexapod::SetAxesOrientation(double rx, double ry, double rz) {
    this->axisRx = rx;
    this->axisRy = ry;
    this->axisRz = rz;
}

std::tuple<double, double, double> SimHexapod::GetAxesOrientation() {
    return std::make_tuple(this->axisRx, this->axisRy, this->axisRz);
}

std::string SimHexapod::GetLocator() { return std::string(this->locator); }

void SimHexapod::ConfigureSystem() { /*No Op*/ }

void SimHexapod::SetFindRefDirection(Axis ax, FrefDirection direction) {
    switch (ax) {
        case Axis::X:
            this->xFrefDir = direction;
            break;
        case Axis::Y:
            this->yFrefDir = direction;
            break;
        case Axis::Z:
            this->zFrefDir = direction;
            break;
    }
}

FrefDirection SimHexapod::GetFindRefDirection(Axis ax) {
    switch (ax) {
        case Axis::X:
            return this->xFrefDir;
        case Axis::Y:
            return this->yFrefDir;
        case Axis::Z:
            return this->zFrefDir;
    }
    return FrefDirection::DEFAULT;
}
void SimHexapod::SetPivotMode(PivotMode mode) { this->pivotMode = mode; }
PivotMode SimHexapod::GetPivotMode() { return this->pivotMode; }
void SimHexapod::SetFindRefMethod(FrefMethod frefMethod) { this->frefMethod = frefMethod; }
FrefMethod SimHexapod::GetFindRefMethod() { return this->frefMethod; }
void SimHexapod::SetFindRefAndCalibFreq(double frefAndCalibFreq) {
    this->frefAndCalibFreq = frefAndCalibFreq;
}
double SimHexapod::GetFindRefAndCalibFreq() { return this->frefAndCalibFreq; }
MoveStatus SimHexapod::GetMoveStatus() {
    this->advance();
    return this->moveStatus.load();
}

std::tuple<int, int, int> SimHexapodAPI::GetVersion() { return std::make_tuple(1, 0, 0); }

std::vector<unsigned int> SimHexapodAPI::GetSupportedModels() {
    return std::vector<unsigned int>{0};
}

std::string SimHexapodAPI::GetModelName(unsigned int model) { return "Simulated Hexapod"; }

std::string SimHexapodAPI::GetStatusMessage(StatusCode status) {
    if (status == StatusCode::OK) return "OK";
    return "SimHexapod error, status code " + std::to_string(static_cast<int>(status));
}

std::unique_ptr<Hexapod> SimHexapodAPI::Open(unsigned int model, std::string locator) {
    return std::make_unique<SimHexapod>(this->nextId++);
}

void SimHexapodAPI::Close(const std::unique_ptr<Hexapod>& hexapod) {
    // No persistent resources to release in the simulator.
}

std::vector<std::string> SimHexapodAPI::FindSystems() {
    return std::vector<std::string>{std::string(this->locator)};
}

void SimHexapodAPI::ConfigureController(unsigned int model, std::string locator) {
    // No-op for the simulator.
}
