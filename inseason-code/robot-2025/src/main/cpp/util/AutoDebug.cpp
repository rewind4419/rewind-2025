#include "util/AutoDebug.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>
#include <iostream>

namespace {
    int acquireDetected = 0, acquireTimeout = 0;
    int releaseDetected = 0, releaseTimeout = 0;
    int jiggleUsed = 0;
    bool twoPieceSafe = false;

    double lastAcquireDuration = 0, lastAcquirePeakA = 0;
    double lastReleaseDuration = 0, lastReleaseBaselineA = 0, lastReleaseMinA = 0;

    wpi::log::StringLogEntry evLog;
    bool initLog = false;

    void ensureLog() {
        if (initLog) return;
        evLog = wpi::log::StringLogEntry(frc::DataLogManager::GetLog(), "Auto/Events");
        initLog = true;
    }
    void log(const std::string& s) {
        ensureLog();
        evLog.Append(s);
        std::cout << "[AUTO] " << s << std::endl;
    }
}

void AutoDebug::Reset() {
    acquireDetected = acquireTimeout = 0;
    releaseDetected = releaseTimeout = 0;
    jiggleUsed = 0;
    twoPieceSafe = false;
    lastAcquireDuration = lastAcquirePeakA = 0;
    lastReleaseDuration = lastReleaseBaselineA = lastReleaseMinA = 0;
    frc::SmartDashboard::PutBoolean("Auto/TwoPieceSafe", twoPieceSafe);
    Publish();
    log("Reset");
}

void AutoDebug::RecordTwoPieceSafe(bool enabled) {
    twoPieceSafe = enabled;
    frc::SmartDashboard::PutBoolean("Auto/TwoPieceSafe", twoPieceSafe);
    log(std::string("TwoPieceSafe=") + (enabled ? "true" : "false"));
}

void AutoDebug::RecordAcquireDetected(double durationS, double peakCurrentA) {
    acquireDetected++; lastAcquireDuration = durationS; lastAcquirePeakA = peakCurrentA;
    frc::SmartDashboard::PutBoolean("Auto/LastAcquireDetected", true);
    log("AcquireDetected dur=" + std::to_string(durationS) + "s peakA=" + std::to_string(peakCurrentA));
    Publish();
}

void AutoDebug::RecordAcquireTimeout(double durationS, double peakCurrentA) {
    acquireTimeout++; lastAcquireDuration = durationS; lastAcquirePeakA = peakCurrentA;
    frc::SmartDashboard::PutBoolean("Auto/LastAcquireDetected", false);
    log("AcquireTimeout dur=" + std::to_string(durationS) + "s peakA=" + std::to_string(peakCurrentA));
    Publish();
}

void AutoDebug::RecordReleaseDetected(double durationS, double baselineA, double minA) {
    releaseDetected++; lastReleaseDuration = durationS; lastReleaseBaselineA = baselineA; lastReleaseMinA = minA;
    frc::SmartDashboard::PutBoolean("Auto/LastReleaseDetected", true);
    log("ReleaseDetected dur=" + std::to_string(durationS) + "s baselineA=" + std::to_string(baselineA) + " minA=" + std::to_string(minA));
    Publish();
}

void AutoDebug::RecordReleaseTimeout(double durationS, double baselineA, double minA) {
    releaseTimeout++; lastReleaseDuration = durationS; lastReleaseBaselineA = baselineA; lastReleaseMinA = minA;
    frc::SmartDashboard::PutBoolean("Auto/LastReleaseDetected", false);
    log("ReleaseTimeout dur=" + std::to_string(durationS) + "s baselineA=" + std::to_string(baselineA) + " minA=" + std::to_string(minA));
    Publish();
}

void AutoDebug::RecordJiggleUsed() {
    jiggleUsed++;
    frc::SmartDashboard::PutBoolean("Auto/JiggleUsedLast", true);
    log("JiggleUsed");
    Publish();
}

void AutoDebug::Publish() {
    frc::SmartDashboard::PutNumber("Auto/CntAcquireDetected", acquireDetected);
    frc::SmartDashboard::PutNumber("Auto/CntAcquireTimeout", acquireTimeout);
    frc::SmartDashboard::PutNumber("Auto/CntReleaseDetected", releaseDetected);
    frc::SmartDashboard::PutNumber("Auto/CntReleaseTimeout", releaseTimeout);
    frc::SmartDashboard::PutNumber("Auto/CntJiggleUsed", jiggleUsed);

    frc::SmartDashboard::PutNumber("Auto/LastAcquireDurationS", lastAcquireDuration);
    frc::SmartDashboard::PutNumber("Auto/LastAcquirePeakA", lastAcquirePeakA);
    frc::SmartDashboard::PutNumber("Auto/LastReleaseDurationS", lastReleaseDuration);
    frc::SmartDashboard::PutNumber("Auto/LastReleaseBaselineA", lastReleaseBaselineA);
    frc::SmartDashboard::PutNumber("Auto/LastReleaseMinA", lastReleaseMinA);
}

void AutoDebug::PublishSummary() {
    log("Summary: AcqDet=" + std::to_string(acquireDetected) + " AcqTO=" + std::to_string(acquireTimeout) +
        " RelDet=" + std::to_string(releaseDetected) + " RelTO=" + std::to_string(releaseTimeout) +
        " Jiggle=" + std::to_string(jiggleUsed) + " TwoPieceSafe=" + std::string(twoPieceSafe ? "true" : "false"));
    Publish();
}


