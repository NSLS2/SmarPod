#include "SmarPodAPI.hpp"

Smarpod_Pose toSmarpodPose(const Pose& pose) {
    Smarpod_Pose sp;
    sp.positionX = pose.x;
    sp.positionY = pose.y;
    sp.positionZ = pose.z;
    sp.rotationX = pose.rx;
    sp.rotationY = pose.ry;
    sp.rotationZ = pose.rz;
    return sp;
}

Pose fromSmarpodPose(const Smarpod_Pose& sp) {
    Pose pose;
    pose.x = sp.positionX;
    pose.y = sp.positionY;
    pose.z = sp.positionZ;
    pose.rx = sp.rotationX;
    pose.ry = sp.rotationY;
    pose.rz = sp.rotationZ;
    return pose;
}

void SmarPodHexapod::SetMaxFrequency(unsigned int frequency) {
    THROW_IF_NOT_OK(Smarpod_SetMaxFrequency(this->GetID(), frequency));
}

unsigned int SmarPodHexapod::GetMaxFrequency() {
    unsigned int maxFreq;
    THROW_IF_NOT_OK(Smarpod_GetMaxFrequency(this->GetID(), &maxFreq));
    return maxFreq;
}

void SmarPodHexapod::SetSpeed(double speed, bool enableSpeedControl) {
    THROW_IF_NOT_OK(Smarpod_SetSpeed(this->GetID(), enableSpeedControl ? 1 : 0, speed));
}

std::tuple<double, bool> SmarPodHexapod::GetSpeed() {
    double speed;
    int speedCtrlEnabled;
    THROW_IF_NOT_OK(Smarpod_GetSpeed(this->GetID(), &speedCtrlEnabled, &speed));
    return std::make_tuple(speed, speedCtrlEnabled == 1);
}

void SmarPodHexapod::SetAcceleration(double acceleration, bool enableAccelControl) {
    THROW_IF_NOT_OK(
        Smarpod_SetAcceleration(this->GetID(), enableAccelControl ? 1 : 0, acceleration));
}

std::tuple<double, bool> SmarPodHexapod::GetAcceleration() {
    double acceleration;
    int accelCtrlEnabled;
    THROW_IF_NOT_OK(Smarpod_GetAcceleration(this->GetID(), &accelCtrlEnabled, &acceleration));
    return std::make_tuple(acceleration, accelCtrlEnabled == 1);
}

void SmarPodHexapod::FindReferenceMarks() {
    THROW_IF_NOT_OK(Smarpod_FindReferenceMarks(this->GetID()));
}
void SmarPodHexapod::Calibrate() { THROW_IF_NOT_OK(Smarpod_Calibrate(this->GetID())); }
bool SmarPodHexapod::IsReferenced() {
    int referenced;
    THROW_IF_NOT_OK(Smarpod_IsReferenced(this->GetID(), &referenced));
    return referenced == 1;
}

void SmarPodHexapod::SetSensorMode(SensorMode mode) {
    THROW_IF_NOT_OK(Smarpod_SetSensorMode(this->GetID(), static_cast<unsigned int>(mode)));
}

SensorMode SmarPodHexapod::GetSensorMode() {
    unsigned int mode;
    THROW_IF_NOT_OK(Smarpod_GetSensorMode(this->GetID(), &mode));
    return static_cast<SensorMode>(mode);
}

void SmarPodHexapod::SetPivot(double x, double y, double z) {
    double pivot[3] = {x, y, z};
    THROW_IF_NOT_OK(Smarpod_SetPivot(this->GetID(), pivot));
}

std::tuple<double, double, double> SmarPodHexapod::GetPivot() {
    double pivot[3];
    THROW_IF_NOT_OK(Smarpod_GetPivot(this->GetID(), pivot));
    return std::make_tuple(pivot[0], pivot[1], pivot[2]);
}

bool SmarPodHexapod::IsPoseReachable(const Pose& pose) {
    Smarpod_Pose sp = toSmarpodPose(pose);
    int reachable;
    THROW_IF_NOT_OK(Smarpod_IsPoseReachable(this->GetID(), &sp, &reachable));
    return reachable == 1;
}

Pose SmarPodHexapod::GetPose() {
    Smarpod_Pose sp;
    THROW_IF_NOT_OK(Smarpod_GetPose(this->GetID(), &sp));
    return fromSmarpodPose(sp);
}

void SmarPodHexapod::Move(const Pose& pose, unsigned int holdTime, bool waitForCompletion) {
    Smarpod_Pose sp = toSmarpodPose(pose);
    THROW_IF_NOT_OK(Smarpod_Move(this->GetID(), &sp, holdTime, waitForCompletion ? 1 : 0));
}

void SmarPodHexapod::Stop() { THROW_IF_NOT_OK(Smarpod_Stop(this->GetID())); }

void SmarPodHexapod::StopAndHold(unsigned int holdTime) {
    THROW_IF_NOT_OK(Smarpod_StopAndHold(this->GetID(), holdTime));
}

void SmarPodHexapod::Standby() { THROW_IF_NOT_OK(Smarpod_Standby(this->GetID())); }

void SmarPodHexapod::SetCoordinateSystem(const Pose& pose) {
    Smarpod_Pose sp = toSmarpodPose(pose);
    THROW_IF_NOT_OK(Smarpod_SetCoordinateSystem(this->GetID(), &sp));
}

Pose SmarPodHexapod::GetCoordinateSystem() {
    Smarpod_Pose sp;
    THROW_IF_NOT_OK(Smarpod_GetCoordinateSystem(this->GetID(), &sp));
    return fromSmarpodPose(sp);
}

void SmarPodHexapod::SetCurrentPoseAsZero() {
    THROW_IF_NOT_OK(Smarpod_SetCurrentPoseAsZero(this->GetID()));
}

void SmarPodHexapod::SetAxesOrientation(double rx, double ry, double rz) {
    THROW_IF_NOT_OK(Smarpod_SetAxesOrientation(this->GetID(), rx, ry, rz));
}

std::tuple<double, double, double> SmarPodHexapod::GetAxesOrientation() {
    double rx, ry, rz;
    THROW_IF_NOT_OK(Smarpod_GetAxesOrientation(this->GetID(), &rx, &ry, &rz));
    return std::make_tuple(rx, ry, rz);
}

std::string SmarPodHexapod::GetLocator() {
    unsigned int bufferSize = 1024;
    std::string locator(bufferSize, '\0');
    THROW_IF_NOT_OK(Smarpod_GetSystemLocator(this->GetID(), &locator[0], &bufferSize));
    locator.resize(bufferSize);
    return locator;
}

void SmarPodHexapod::ConfigureSystem() { THROW_IF_NOT_OK(Smarpod_ConfigureSystem(this->GetID())); }

void SmarPodHexapod::SetFindRefDirection(Axis ax, FrefDirection direction) {
    unsigned int property = SMARPOD_FREF_ZDIRECTION;
    switch (ax) {
        case Axis::X:
            property = SMARPOD_FREF_XDIRECTION;
            break;
        case Axis::Y:
            property = SMARPOD_FREF_YDIRECTION;
            break;
        case Axis::Z:
            property = SMARPOD_FREF_ZDIRECTION;
            break;
    }
    THROW_IF_NOT_OK(Smarpod_Set_ui(this->GetID(), property, static_cast<unsigned int>(direction)));
}

FrefDirection SmarPodHexapod::GetFindRefDirection(Axis ax) {
    unsigned int property = SMARPOD_FREF_ZDIRECTION;
    switch (ax) {
        case Axis::X:
            property = SMARPOD_FREF_XDIRECTION;
            break;
        case Axis::Y:
            property = SMARPOD_FREF_YDIRECTION;
            break;
        case Axis::Z:
            property = SMARPOD_FREF_ZDIRECTION;
            break;
    }
    unsigned int direction;
    THROW_IF_NOT_OK(Smarpod_Get_ui(this->GetID(), property, &direction));
    return static_cast<FrefDirection>(direction);
}
void SmarPodHexapod::SetPivotMode(PivotMode mode) {
    THROW_IF_NOT_OK(
        Smarpod_Set_ui(this->GetID(), SMARPOD_PIVOT_MODE, static_cast<unsigned int>(mode)));
}
PivotMode SmarPodHexapod::GetPivotMode() {
    unsigned int mode;
    THROW_IF_NOT_OK(Smarpod_Get_ui(this->GetID(), SMARPOD_PIVOT_MODE, &mode));
    return static_cast<PivotMode>(mode);
}
void SmarPodHexapod::SetFindRefMethod(FrefMethod frefMethod) {
    THROW_IF_NOT_OK(
        Smarpod_Set_ui(this->GetID(), SMARPOD_FREF_METHOD, static_cast<unsigned int>(frefMethod)));
}
FrefMethod SmarPodHexapod::GetFindRefMethod() {
    unsigned int method;
    THROW_IF_NOT_OK(Smarpod_Get_ui(this->GetID(), SMARPOD_FREF_METHOD, &method));
    return static_cast<FrefMethod>(method);
}
void SmarPodHexapod::SetFindRefAndCalibFreq(double frefAndCalibFreq) {
    THROW_IF_NOT_OK(Smarpod_Set_ui(this->GetID(), SMARPOD_FREF_AND_CAL_FREQUENCY,
                                   static_cast<unsigned int>(frefAndCalibFreq)));
}
double SmarPodHexapod::GetFindRefAndCalibFreq() {
    unsigned int freq;
    THROW_IF_NOT_OK(Smarpod_Get_ui(this->GetID(), SMARPOD_FREF_AND_CAL_FREQUENCY, &freq));
    return static_cast<double>(freq);
}
MoveStatus SmarPodHexapod::GetMoveStatus() {
    unsigned int status;
    THROW_IF_NOT_OK(Smarpod_GetMoveStatus(this->GetID(), &status));
    return static_cast<MoveStatus>(status);
}

std::string smarpodStatusMessage(StatusCode status) {
    const char* info = nullptr;
    if (Smarpod_GetStatusInfo(static_cast<Smarpod_Status>(status), &info) == SMARPOD_OK &&
        info != nullptr) {
        return std::string(info);
    }
    return "unknown status code " + std::to_string(static_cast<int>(status));
}

std::tuple<int, int, int> SmarPodAPI::GetVersion() {
    unsigned int major, minor, update;
    THROW_IF_NOT_OK(Smarpod_GetDLLVersion(&major, &minor, &update));
    return std::make_tuple(static_cast<int>(major), static_cast<int>(minor),
                           static_cast<int>(update));
}

std::vector<unsigned int> SmarPodAPI::GetSupportedModels() {
    unsigned int count = 128;
    std::vector<unsigned int> models(count);
    THROW_IF_NOT_OK(Smarpod_GetModels(models.data(), &count));
    models.resize(count);
    return models;
}

std::string SmarPodAPI::GetModelName(unsigned int model) {
    const char* name = nullptr;
    THROW_IF_NOT_OK(Smarpod_GetModelName(model, &name));
    return name != nullptr ? std::string(name) : std::string();
}

std::string SmarPodAPI::GetStatusMessage(StatusCode status) { return smarpodStatusMessage(status); }

std::unique_ptr<Hexapod> SmarPodAPI::Open(unsigned int model, std::string locator) {
    unsigned int id = 0;
    THROW_IF_NOT_OK(Smarpod_Open(&id, model, locator.c_str(), ""));
    return std::unique_ptr<Hexapod>(new SmarPodHexapod(id));
}

void SmarPodAPI::Close(const std::unique_ptr<Hexapod>& hexapod) {
    if (hexapod) {
        THROW_IF_NOT_OK(Smarpod_Close(hexapod->GetID()));
    }
}

std::vector<std::string> SmarPodAPI::FindSystems() {
    unsigned int bufferSize = 4096;
    std::string buffer(bufferSize, '\0');
    THROW_IF_NOT_OK(Smarpod_FindSystems("", &buffer[0], &bufferSize));
    buffer.resize(bufferSize);

    // Results are returned as a newline-separated list of locators.
    std::vector<std::string> systems;
    std::string::size_type start = 0;
    while (start < buffer.size()) {
        std::string::size_type end = buffer.find('\n', start);
        if (end == std::string::npos) end = buffer.size();
        if (end > start) systems.push_back(buffer.substr(start, end - start));
        start = end + 1;
    }
    return systems;
}

void SmarPodAPI::ConfigureController(unsigned int model, std::string locator) {
    THROW_IF_NOT_OK(Smarpod_ConfigureController(model, locator.c_str(), ""));
}
