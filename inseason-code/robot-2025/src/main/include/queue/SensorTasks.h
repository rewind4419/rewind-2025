#pragma once

#include "queue/Queue.h"
#include "util/MotorController.h"
#include "util/AutoDebug.h"
#include <frc/Timer.h>

class WaitForIntakeAcquireTask : public Task {
public:
    MotorController* intake;
    double currentThreshold;
    double holdTime;
    double timeout;

    double start = 0.0;
    double since = 0.0;
    double peakA = 0.0;

    WaitForIntakeAcquireTask(MotorController* in, double currentA, double holdS, double timeoutS)
        : intake(in), currentThreshold(currentA), holdTime(holdS), timeout(timeoutS) {}

    void Start() override {
        start = frc::Timer::GetFPGATimestamp().value();
        since = start;
        peakA = 0.0;
    }
    bool Loop() override {
        double now = frc::Timer::GetFPGATimestamp().value();
        double i = intake->motor.GetStatorCurrent().GetValue().value();
        if (i > peakA) peakA = i;
        if (i >= currentThreshold) {
            if (since + holdTime <= now) {
                AutoDebug::RecordAcquireDetected(now - start, peakA);
                return true;
            }
        } else {
            since = now;
        }
        if ((now - start) > timeout) {
            AutoDebug::RecordAcquireTimeout(now - start, peakA);
            return true; // succeed on timeout to avoid stall
        }
        return false;
    }
};

class WaitForOuttakeReleaseTask : public Task {
public:
    MotorController* intake;
    double dropA;
    double holdTime;
    double timeout;
    double presample;

    double start = 0.0;
    double sinceDrop = 0.0;
    double baselineA = 0.0;
    double minA = 0.0;

    WaitForOuttakeReleaseTask(MotorController* in, double dropA_, double holdS, double timeoutS, double presampleS = 0.0)
        : intake(in), dropA(dropA_), holdTime(holdS), timeout(timeoutS), presample(presampleS) {}

    void Start() override {
        start = frc::Timer::GetFPGATimestamp().value();
        sinceDrop = start;
        baselineA = intake->motor.GetStatorCurrent().GetValue().value();
        minA = baselineA;
    }
    bool Loop() override {
        double now = frc::Timer::GetFPGATimestamp().value();
        // presample window: ignore detection during presample to stabilize baseline
        if ((now - start) < presample) {
            // refresh baseline during presample window
            double bi = intake->motor.GetStatorCurrent().GetValue().value();
            baselineA = 0.7 * baselineA + 0.3 * bi; // simple smoothing to avoid spikes
            if (bi < minA) minA = bi;
            sinceDrop = now;
            return false;
        }
        double i = intake->motor.GetStatorCurrent().GetValue().value();
        if (i < minA) minA = i;
        bool dropped = (baselineA - i) >= dropA;
        if (dropped) {
            if (sinceDrop + holdTime <= now) {
                AutoDebug::RecordReleaseDetected(now - start, baselineA, minA);
                return true;
            }
        } else {
            sinceDrop = now;
        }
        if ((now - start) > timeout) {
            AutoDebug::RecordReleaseTimeout(now - start, baselineA, minA);
            return true; // succeed on timeout to move on
        }
        return false;
    }
};


