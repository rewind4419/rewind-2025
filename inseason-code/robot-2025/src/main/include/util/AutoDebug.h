#pragma once

#include <string>

namespace AutoDebug {
    void Reset();
    void RecordTwoPieceSafe(bool enabled);

    void RecordAcquireDetected(double durationS, double peakCurrentA);
    void RecordAcquireTimeout(double durationS, double peakCurrentA);

    void RecordReleaseDetected(double durationS, double baselineA, double minA);
    void RecordReleaseTimeout(double durationS, double baselineA, double minA);

    void RecordJiggleUsed();

    void Publish();
    void PublishSummary();
}


