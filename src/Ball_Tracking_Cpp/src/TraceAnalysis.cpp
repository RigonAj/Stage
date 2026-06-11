#include "TraceAnalysis.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <limits>
#include <utility>

#include "RegressionAccumulator.hpp"

bool AcceptTracePolarity(bool polarity, int polarityMode) {
    if (polarityMode == 1) {
        return polarity;
    }
    if (polarityMode == 2) {
        return !polarity;
    }
    return true;
}

const char *TracePolarityModeName(int polarityMode) {
    if (polarityMode == 1) {
        return "positive";
    }
    if (polarityMode == 2) {
        return "negative";
    }
    return "all";
}

std::vector<TracePoint> BuildTracePointsFromEvents(const dv::EventStore &events, int polarityMode) {
    std::vector<TracePoint> points;
    points.reserve(events.size());

    for (const auto &e : events) {
        const bool polarity = e.polarity();
        if (!AcceptTracePolarity(polarity, polarityMode)) {
            continue;
        }

        const float x = static_cast<float>(e.x());
        const float y = static_cast<float>(e.y());
        if (x >= 0.0f && x < 640.0f && y >= 0.0f && y < 480.0f) {
            points.push_back({{x, y}, e.timestamp(), polarity});
        }
    }

    return points;
}

std::vector<TracePoint> BuildTracePointsFromFloatSource(
    const std::vector<cv::Point2f> &pointsSource,
    const std::vector<int64_t> &timestamps,
    const std::vector<bool> *polarities,
    int polarityMode) {

    std::vector<TracePoint> points;
    const std::size_t count = std::min(pointsSource.size(), timestamps.size());
    const bool hasPolarities = polarities != nullptr && polarities->size() >= count;
    points.reserve(count);

    for (std::size_t i = 0; i < count; ++i) {
        const bool polarity = hasPolarities ? (*polarities)[i] : true;
        if (hasPolarities && !AcceptTracePolarity(polarity, polarityMode)) {
            continue;
        }

        const float x = pointsSource[i].x;
        const float y = pointsSource[i].y;
        if (std::isfinite(x)
            && std::isfinite(y)
            && x >= 0.0f
            && x < 640.0f
            && y >= 0.0f
            && y < 480.0f) {
            points.push_back({{x, y}, timestamps[i], polarity});
        }
    }

    return points;
}

TracePointSourceResult BuildTracePointSource(
    const std::vector<cv::Point2f> &accumulatedPoints,
    const std::vector<int64_t> &accumulatedTimestamps,
    const std::vector<bool> &accumulatedPolarities,
    const std::vector<cv::Point2f> *rawPoints,
    const std::vector<int64_t> *rawTimestamps,
    const std::vector<bool> *rawPolarities,
    const std::vector<cv::Point2f> *floatPoints,
    const std::vector<int64_t> *floatTimestamps,
    const std::vector<bool> *floatPolarities,
    const dv::EventStore &events,
    bool useRawInput,
    bool useRadiusGate,
    bool motionWindowValid,
    int polarityMode) {

    TracePointSourceResult result;
    result.label = useRawInput
        ? "raw events in moving ball window"
        : "undistorted events in moving ball window";
    result.label += useRadiusGate ? " + radius gate" : " without radius gate";
    result.label += std::string(" | pol=") + TracePolarityModeName(polarityMode);
    result.color = motionWindowValid ? DARKGREEN : MAROON;

    if (!accumulatedPoints.empty()
        && accumulatedPoints.size() == accumulatedTimestamps.size()
        && accumulatedPoints.size() == accumulatedPolarities.size()) {
        result.points = BuildTracePointsFromFloatSource(
            accumulatedPoints,
            accumulatedTimestamps,
            &accumulatedPolarities,
            polarityMode);
    }

    if (result.points.empty()
        && !useRadiusGate
        && useRawInput
        && rawPoints != nullptr
        && rawTimestamps != nullptr
        && !rawPoints->empty()
        && rawPoints->size() == rawTimestamps->size()) {
        result.points = BuildTracePointsFromFloatSource(
            *rawPoints,
            *rawTimestamps,
            rawPolarities,
            polarityMode);
        result.label = std::string("raw current window fallback | pol=")
            + TracePolarityModeName(polarityMode);
        result.color = MAROON;
    }

    if (result.points.empty()
        && !useRadiusGate
        && floatPoints != nullptr
        && floatTimestamps != nullptr
        && !floatPoints->empty()
        && floatPoints->size() == floatTimestamps->size()) {
        result.points = BuildTracePointsFromFloatSource(
            *floatPoints,
            *floatTimestamps,
            floatPolarities,
            polarityMode);
        result.label = std::string("undistorted current window fallback | pol=")
            + TracePolarityModeName(polarityMode);
        result.color = MAROON;
    }

    if (result.points.empty() && !useRadiusGate) {
        result.points = BuildTracePointsFromEvents(events, polarityMode);
        result.label = std::string("rounded current window fallback | pol=")
            + TracePolarityModeName(polarityMode);
        result.color = MAROON;
    }

    return result;
}

float Quantile(std::vector<float> values, float q) {
    if (values.empty()) {
        return 0.0f;
    }

    q = std::clamp(q, 0.0f, 1.0f);
    const std::size_t index = std::min(
        values.size() - 1,
        static_cast<std::size_t>(std::floor(q * static_cast<float>(values.size() - 1)))
    );

    std::nth_element(
        values.begin(),
        values.begin() + static_cast<std::ptrdiff_t>(index),
        values.end()
    );

    return values[index];
}

TraceEdgeEstimate EstimateSupportedEdges(
    const std::vector<float> &values,
    std::size_t minSupportCount,
    const TraceSupportEdgeSettings &settings) {

    TraceEdgeEstimate estimate;
    std::vector<float> finiteValues;
    finiteValues.reserve(values.size());
    for (float value : values) {
        if (std::isfinite(value)) {
            finiteValues.push_back(value);
        }
    }

    if (finiteValues.size() < std::max<std::size_t>(minSupportCount, 8)) {
        return estimate;
    }

    std::sort(finiteValues.begin(), finiteValues.end());

    const std::size_t feasibleSupport = std::max<std::size_t>(1, finiteValues.size());
    const std::size_t minLocalSupport = std::min(
        std::max<std::size_t>(settings.minLocalSupport, 1),
        feasibleSupport
    );
    const std::size_t maxLocalSupport = std::min(
        std::max(settings.maxLocalSupport, minLocalSupport),
        feasibleSupport
    );
    const float supportDivisor = std::max(settings.supportDivisor, 1.0f);
    const std::size_t localSupport = std::clamp<std::size_t>(
        static_cast<std::size_t>(std::floor(static_cast<float>(finiteValues.size()) / supportDivisor)),
        minLocalSupport,
        maxLocalSupport
    );
    const float supportRadius = std::max(settings.supportRadiusPx, 0.25f);

    auto supportedLow = [&]() -> float {
        std::size_t right = 0;
        for (std::size_t left = 0; left < finiteValues.size(); ++left) {
            while (right < finiteValues.size()
                && finiteValues[right] - finiteValues[left] <= supportRadius) {
                ++right;
            }
            if (right - left >= localSupport) {
                return finiteValues[left];
            }
        }
        return finiteValues.front();
    };

    auto supportedHigh = [&]() -> float {
        for (std::size_t reverseIndex = finiteValues.size(); reverseIndex > 0; --reverseIndex) {
            const std::size_t right = reverseIndex - 1;
            const auto begin = finiteValues.begin();
            const auto end = begin + static_cast<std::ptrdiff_t>(right + 1);
            const auto left = std::lower_bound(begin, end, finiteValues[right] - supportRadius);
            if (static_cast<std::size_t>(std::distance(left, end)) >= localSupport) {
                return finiteValues[right];
            }
        }
        return finiteValues.back();
    };

    float low = supportedLow();
    float high = supportedHigh();
    if (!std::isfinite(low) || !std::isfinite(high) || high <= low) {
        return estimate;
    }

    const float rawWidth = high - low;
    if (rawWidth < 2.0f) {
        return estimate;
    }

    const float pixelCenterToBorderPx = std::clamp(rawWidth * settings.borderRatio, 0.0f, 1.5f);
    low -= pixelCenterToBorderPx;
    high += pixelCenterToBorderPx;

    int supportCount = 0;
    for (float value : finiteValues) {
        if (value >= low && value <= high) {
            ++supportCount;
        }
    }

    if (supportCount < static_cast<int>(minSupportCount)) {
        return estimate;
    }

    estimate.valid = true;
    estimate.low = low;
    estimate.high = high;
    estimate.middle = 0.5f * (low + high);
    estimate.width = high - low;
    estimate.supportCount = supportCount;
    return estimate;
}

float MedianValue(std::vector<float> values) {
    return values.empty() ? 0.0f : Quantile(std::move(values), 0.50f);
}

float MedianAbsoluteDeviation(std::vector<float> values, float median) {
    if (values.empty()) {
        return 0.0f;
    }

    for (float &value : values) {
        value = std::fabs(value - median);
    }

    return MedianValue(std::move(values));
}

bool LocalLinearPrediction(
    const std::vector<float> &xValues,
    const std::vector<float> &yValues,
    std::size_t index,
    int radius,
    float &prediction,
    float &mad) {

    if (xValues.size() != yValues.size() || index >= xValues.size()) {
        return false;
    }

    const int count = static_cast<int>(xValues.size());
    const int begin = std::max(0, static_cast<int>(index) - radius);
    const int end = std::min(count - 1, static_cast<int>(index) + radius);

    int used = 0;
    double meanX = 0.0;
    double meanY = 0.0;
    for (int i = begin; i <= end; ++i) {
        if (i == static_cast<int>(index)) {
            continue;
        }

        meanX += xValues[static_cast<std::size_t>(i)];
        meanY += yValues[static_cast<std::size_t>(i)];
        ++used;
    }

    if (used < 4) {
        return false;
    }

    meanX /= static_cast<double>(used);
    meanY /= static_cast<double>(used);

    double numerator = 0.0;
    double denominator = 0.0;
    for (int i = begin; i <= end; ++i) {
        if (i == static_cast<int>(index)) {
            continue;
        }

        const double dx = static_cast<double>(xValues[static_cast<std::size_t>(i)]) - meanX;
        const double dy = static_cast<double>(yValues[static_cast<std::size_t>(i)]) - meanY;
        numerator += dx * dy;
        denominator += dx * dx;
    }

    const double slope = denominator > 1.0e-9 ? numerator / denominator : 0.0;
    const double intercept = meanY - slope * meanX;
    prediction = static_cast<float>(slope * static_cast<double>(xValues[index]) + intercept);

    std::vector<float> residuals;
    residuals.reserve(static_cast<std::size_t>(used));
    for (int i = begin; i <= end; ++i) {
        if (i == static_cast<int>(index)) {
            continue;
        }

        const double expected = slope * static_cast<double>(xValues[static_cast<std::size_t>(i)]) + intercept;
        residuals.push_back(static_cast<float>(std::fabs(static_cast<double>(yValues[static_cast<std::size_t>(i)]) - expected)));
    }

    mad = MedianValue(std::move(residuals));
    return std::isfinite(prediction);
}

void FilterCoherentRibbonSamples(
    std::vector<PolynomialSample> &middleSamples,
    std::vector<PolynomialSample> &lowSamples,
    std::vector<PolynomialSample> &highSamples,
    std::vector<float> &localWidths) {

    const std::size_t count = middleSamples.size();
    if (count < 7
        || lowSamples.size() != count
        || highSamples.size() != count
        || localWidths.size() != count) {
        return;
    }

    std::vector<float> sValues;
    std::vector<float> middleValues;
    std::vector<float> lowValues;
    std::vector<float> highValues;
    sValues.reserve(count);
    middleValues.reserve(count);
    lowValues.reserve(count);
    highValues.reserve(count);

    for (std::size_t i = 0; i < count; ++i) {
        sValues.push_back(static_cast<float>(middleSamples[i].x));
        middleValues.push_back(static_cast<float>(middleSamples[i].y));
        lowValues.push_back(static_cast<float>(lowSamples[i].y));
        highValues.push_back(static_cast<float>(highSamples[i].y));
    }

    const float medianWidth = MedianValue(localWidths);
    const float widthMad = MedianAbsoluteDeviation(localWidths, medianWidth);
    const float minEdgeLimit = std::max(2.8f, medianWidth * 0.18f);
    const float minWidthLimit = std::max(3.2f, medianWidth * 0.22f);

    std::vector<char> keep(count, 1);
    for (std::size_t i = 0; i < count; ++i) {
        float predictedHigh = 0.0f;
        float predictedLow = 0.0f;
        float predictedMiddle = 0.0f;
        float predictedWidth = 0.0f;
        float highMad = 0.0f;
        float lowMad = 0.0f;
        float middleMad = 0.0f;
        float localWidthMad = 0.0f;

        const bool hasHighPrediction = LocalLinearPrediction(sValues, highValues, i, 4, predictedHigh, highMad);
        const bool hasLowPrediction = LocalLinearPrediction(sValues, lowValues, i, 4, predictedLow, lowMad);
        const bool hasMiddlePrediction = LocalLinearPrediction(sValues, middleValues, i, 4, predictedMiddle, middleMad);
        const bool hasWidthPrediction = LocalLinearPrediction(sValues, localWidths, i, 4, predictedWidth, localWidthMad);
        if (!hasHighPrediction || !hasLowPrediction || !hasMiddlePrediction || !hasWidthPrediction) {
            continue;
        }

        const float highLimit = std::max(minEdgeLimit, highMad * 5.0f + 1.0f);
        const float lowLimit = std::max(minEdgeLimit, lowMad * 5.0f + 1.0f);
        const float middleLimit = std::max(minEdgeLimit, middleMad * 5.0f + 1.0f);
        const float widthLimit = std::max(minWidthLimit, std::max(widthMad, localWidthMad) * 5.0f + 1.0f);

        const bool highSpike = std::fabs(highValues[i] - predictedHigh) > highLimit;
        const bool lowSpike = std::fabs(lowValues[i] - predictedLow) > lowLimit;
        const bool middleSpike = std::fabs(middleValues[i] - predictedMiddle) > middleLimit;
        const bool widthSpike = std::fabs(localWidths[i] - predictedWidth) > widthLimit;

        if (widthSpike || highSpike || lowSpike || middleSpike) {
            keep[i] = 0;
        }
    }

    std::vector<PolynomialSample> filteredMiddle;
    std::vector<PolynomialSample> filteredLow;
    std::vector<PolynomialSample> filteredHigh;
    std::vector<float> filteredWidths;
    filteredMiddle.reserve(count);
    filteredLow.reserve(count);
    filteredHigh.reserve(count);
    filteredWidths.reserve(count);

    for (std::size_t i = 0; i < count; ++i) {
        if (!keep[i]) {
            continue;
        }

        filteredMiddle.push_back(middleSamples[i]);
        filteredLow.push_back(lowSamples[i]);
        filteredHigh.push_back(highSamples[i]);
        filteredWidths.push_back(localWidths[i]);
    }

    if (filteredMiddle.size() >= std::max<std::size_t>(7, count / 2)) {
        middleSamples = std::move(filteredMiddle);
        lowSamples = std::move(filteredLow);
        highSamples = std::move(filteredHigh);
        localWidths = std::move(filteredWidths);
    }
}

void FilterTraceWidthSpikes(std::vector<TraceWidthEstimate> &estimates) {
    const std::size_t count = estimates.size();
    if (count < 7) {
        return;
    }

    std::vector<float> sValues;
    std::vector<float> widths;
    sValues.reserve(count);
    widths.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        sValues.push_back(static_cast<float>(i));
        widths.push_back(estimates[i].widthPx);
    }

    const float medianWidth = MedianValue(widths);
    const float widthMad = MedianAbsoluteDeviation(widths, medianWidth);
    const float minWidthLimit = std::max(3.0f, medianWidth * 0.22f);

    std::vector<char> keep(count, 1);
    for (std::size_t i = 0; i < count; ++i) {
        float predictedWidth = 0.0f;
        float localWidthMad = 0.0f;
        if (!LocalLinearPrediction(sValues, widths, i, 3, predictedWidth, localWidthMad)) {
            continue;
        }

        const float widthLimit = std::max(minWidthLimit, std::max(widthMad, localWidthMad) * 5.0f + 1.0f);
        if (std::fabs(widths[i] - predictedWidth) > widthLimit) {
            keep[i] = 0;
        }
    }

    std::vector<TraceWidthEstimate> filtered;
    filtered.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        if (keep[i]) {
            filtered.push_back(estimates[i]);
        }
    }

    if (filtered.size() >= std::max<std::size_t>(5, count / 2)) {
        estimates = std::move(filtered);
    }
}

bool SolveWeightedPolynomialFit(
    const std::vector<double> &xs,
    const std::vector<double> &ys,
    const std::vector<double> &weights,
    int degree,
    std::array<double, 3> &coeffs) {

    const int terms = std::clamp(degree, 1, 2) + 1;
    double normal[3][4] = {};

    for (std::size_t i = 0; i < xs.size(); ++i) {
        const double w = weights[i];
        if (w <= 0.0) {
            continue;
        }
        const double basis[3] = {1.0, xs[i], xs[i] * xs[i]};
        for (int row = 0; row < terms; ++row) {
            for (int col = 0; col < terms; ++col) {
                normal[row][col] += w * basis[row] * basis[col];
            }
            normal[row][terms] += w * basis[row] * ys[i];
        }
    }

    for (int col = 0; col < terms; ++col) {
        int pivot = col;
        for (int row = col + 1; row < terms; ++row) {
            if (std::fabs(normal[row][col]) > std::fabs(normal[pivot][col])) {
                pivot = row;
            }
        }
        if (std::fabs(normal[pivot][col]) < 1.0e-12) {
            return false;
        }
        if (pivot != col) {
            for (int c = col; c <= terms; ++c) {
                std::swap(normal[col][c], normal[pivot][c]);
            }
        }
        for (int row = col + 1; row < terms; ++row) {
            const double factor = normal[row][col] / normal[col][col];
            for (int c = col; c <= terms; ++c) {
                normal[row][c] -= factor * normal[col][c];
            }
        }
    }

    coeffs = {0.0, 0.0, 0.0};
    for (int row = terms - 1; row >= 0; --row) {
        double sum = normal[row][terms];
        for (int col = row + 1; col < terms; ++col) {
            sum -= normal[row][col] * coeffs[static_cast<std::size_t>(col)];
        }
        coeffs[static_cast<std::size_t>(row)] = sum / normal[row][row];
    }
    return true;
}

// The ball depth changes smoothly along a throw, so the apparent width must be
// a smooth function of time. Individual slice widths carry pixel-level noise
// that converts directly into depth noise; this replaces each width with the
// value of a robust polynomial width(t) fitted across the whole ribbon.
void SmoothTraceWidthProfile(std::vector<TraceWidthEstimate> &estimates) {
    std::vector<std::size_t> validIndices;
    validIndices.reserve(estimates.size());
    for (std::size_t i = 0; i < estimates.size(); ++i) {
        if (estimates[i].valid && estimates[i].widthPx >= 1.0f) {
            validIndices.push_back(i);
        }
    }
    if (validIndices.size() < 6) {
        return;
    }

    double tMin = std::numeric_limits<double>::infinity();
    double tMax = -std::numeric_limits<double>::infinity();
    for (const std::size_t idx : validIndices) {
        tMin = std::min(tMin, estimates[idx].timeSeconds);
        tMax = std::max(tMax, estimates[idx].timeSeconds);
    }
    const double tRange = tMax - tMin;

    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> baseWeights;
    xs.reserve(validIndices.size());
    ys.reserve(validIndices.size());
    baseWeights.reserve(validIndices.size());
    for (std::size_t k = 0; k < validIndices.size(); ++k) {
        const TraceWidthEstimate &estimate = estimates[validIndices[k]];
        const double x = tRange > 1.0e-9
            ? 2.0 * (estimate.timeSeconds - tMin) / tRange - 1.0
            : 2.0 * static_cast<double>(k) / static_cast<double>(validIndices.size() - 1) - 1.0;
        xs.push_back(x);
        ys.push_back(static_cast<double>(estimate.widthPx));
        baseWeights.push_back(1.0 + static_cast<double>(std::min(estimate.localEventCount, 48)) / 12.0);
    }

    const int degree = validIndices.size() >= 10 ? 2 : 1;
    std::array<double, 3> coeffs{};
    std::vector<double> weights = baseWeights;
    if (!SolveWeightedPolynomialFit(xs, ys, weights, degree, coeffs)) {
        return;
    }

    auto evalPoly = [&coeffs](double x) {
        return coeffs[0] + coeffs[1] * x + coeffs[2] * x * x;
    };

    for (int iteration = 0; iteration < 2; ++iteration) {
        std::vector<float> absResiduals;
        absResiduals.reserve(xs.size());
        for (std::size_t i = 0; i < xs.size(); ++i) {
            absResiduals.push_back(static_cast<float>(std::fabs(ys[i] - evalPoly(xs[i]))));
        }
        const float scale = std::max(0.25f, MedianValue(absResiduals) * 1.4826f);
        for (std::size_t i = 0; i < xs.size(); ++i) {
            const double ratio = (ys[i] - evalPoly(xs[i])) / static_cast<double>(scale);
            weights[i] = baseWeights[i] / (1.0 + ratio * ratio);
        }
        if (!SolveWeightedPolynomialFit(xs, ys, weights, degree, coeffs)) {
            return;
        }
    }

    std::vector<float> rawWidths;
    rawWidths.reserve(ys.size());
    for (const double width : ys) {
        rawWidths.push_back(static_cast<float>(width));
    }
    const float medianWidth = MedianValue(rawWidths);
    const float lowLimit = std::max(2.0f, medianWidth * 0.55f);
    const float highLimit = medianWidth * 1.7f;

    for (std::size_t k = 0; k < validIndices.size(); ++k) {
        TraceWidthEstimate &estimate = estimates[validIndices[k]];
        const float fitted = std::clamp(static_cast<float>(evalPoly(xs[k])), lowLimit, highLimit);

        const Vector2 segment{
            estimate.upper.x - estimate.lower.x,
            estimate.upper.y - estimate.lower.y
        };
        const float segmentLength = std::sqrt(segment.x * segment.x + segment.y * segment.y);
        const Vector2 axis = segmentLength > 1.0e-4f
            ? Vector2{segment.x / segmentLength, segment.y / segmentLength}
            : estimate.normal;
        const Vector2 segmentMiddle{
            (estimate.upper.x + estimate.lower.x) * 0.5f,
            (estimate.upper.y + estimate.lower.y) * 0.5f
        };
        const float half = fitted * 0.5f;
        estimate.upper = {segmentMiddle.x + axis.x * half, segmentMiddle.y + axis.y * half};
        estimate.lower = {segmentMiddle.x - axis.x * half, segmentMiddle.y - axis.y * half};
        estimate.widthPx = fitted;
    }
}

float Distance3D(const Vector3 &a, const Vector3 &b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool IsFinite3D(const Vector3 &point) {
    return std::isfinite(point.x)
        && std::isfinite(point.y)
        && std::isfinite(point.z);
}

int FilterTraceWorldOutliers(std::vector<Vector3> &points, std::vector<float> &times) {
    const std::size_t count = points.size();
    if (count < 5 || times.size() != count) {
        return 0;
    }

    std::vector<char> keep(count, 1);
    auto keptCount = [&]() {
        return static_cast<std::size_t>(std::count(keep.begin(), keep.end(), 1));
    };

    auto dropTemporalJumps = [&]() {
        if (count < 3) {
            return 0;
        }

        std::vector<float> stepDistances;
        stepDistances.reserve(count - 1);
        for (std::size_t i = 1; i < count; ++i) {
            if (IsFinite3D(points[i - 1]) && IsFinite3D(points[i])) {
                stepDistances.push_back(Distance3D(points[i - 1], points[i]));
            }
        }

        if (stepDistances.empty()) {
            return 0;
        }

        const float medianStep = std::max(1.0e-4f, MedianValue(stepDistances));
        const float jumpLimit = std::max(0.16f, medianStep * 7.0f);
        const float bridgeLimit = std::max(0.12f, medianStep * 3.5f);
        int removed = 0;

        auto markDrop = [&](std::size_t index) {
            if (keep[index] && keptCount() > std::max<std::size_t>(4, count / 2)) {
                keep[index] = 0;
                ++removed;
            }
        };

        if (Distance3D(points[0], points[1]) > jumpLimit
            && Distance3D(points[1], points[2]) <= bridgeLimit) {
            markDrop(0);
        }

        if (Distance3D(points[count - 1], points[count - 2]) > jumpLimit
            && Distance3D(points[count - 2], points[count - 3]) <= bridgeLimit) {
            markDrop(count - 1);
        }

        for (std::size_t i = 1; i + 1 < count; ++i) {
            if (!keep[i]) {
                continue;
            }

            const float prevDistance = Distance3D(points[i], points[i - 1]);
            const float nextDistance = Distance3D(points[i], points[i + 1]);
            const float bridgeDistance = Distance3D(points[i - 1], points[i + 1]);
            if (prevDistance > jumpLimit
                && nextDistance > jumpLimit
                && bridgeDistance <= bridgeLimit) {
                markDrop(i);
            }
        }

        return removed;
    };

    auto dropFitResiduals = [&]() {
        if (keptCount() < 5) {
            return 0;
        }

        LinearRegression xReg;
        LinearRegression yReg;
        QuadraticRegression zReg;
        for (std::size_t i = 0; i < count; ++i) {
            if (!keep[i] || !IsFinite3D(points[i]) || !std::isfinite(times[i])) {
                continue;
            }

            xReg.add(times[i], points[i].x);
            yReg.add(times[i], points[i].y);
            zReg.add(times[i], points[i].z);
        }

        if (xReg.size() < 5 || yReg.size() < 5 || zReg.size() < 5) {
            return 0;
        }

        const coef xFit = xReg.fit();
        const coef yFit = yReg.fit();
        const coef2 zFit = zReg.fit();

        std::vector<float> residuals;
        residuals.reserve(keptCount());
        for (std::size_t i = 0; i < count; ++i) {
            if (!keep[i]) {
                continue;
            }

            const float t = times[i];
            const Vector3 predicted{
                xFit.a * t + xFit.b,
                yFit.a * t + yFit.b,
                zFit.a * t * t + zFit.b * t + zFit.c
            };
            residuals.push_back(Distance3D(points[i], predicted));
        }

        if (residuals.size() < 5) {
            return 0;
        }

        const float medianResidual = MedianValue(residuals);
        const float residualMad = MedianAbsoluteDeviation(residuals, medianResidual);
        const float threshold = std::max(
            0.10f,
            medianResidual + std::max(0.015f, residualMad) * 6.0f
        );

        std::vector<std::size_t> rejected;
        rejected.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            if (!keep[i]) {
                continue;
            }

            const float t = times[i];
            const Vector3 predicted{
                xFit.a * t + xFit.b,
                yFit.a * t + yFit.b,
                zFit.a * t * t + zFit.b * t + zFit.c
            };
            if (Distance3D(points[i], predicted) > threshold) {
                rejected.push_back(i);
            }
        }

        if (rejected.empty()
            || keptCount() - rejected.size() < std::max<std::size_t>(4, count / 2)) {
            return 0;
        }

        for (std::size_t index : rejected) {
            keep[index] = 0;
        }
        return static_cast<int>(rejected.size());
    };

    int removed = 0;
    for (int iteration = 0; iteration < 3; ++iteration) {
        const int before = removed;
        removed += dropTemporalJumps();
        removed += dropFitResiduals();
        if (removed == before) {
            break;
        }
    }

    if (removed == 0) {
        return 0;
    }

    std::vector<Vector3> filteredPoints;
    std::vector<float> filteredTimes;
    filteredPoints.reserve(keptCount());
    filteredTimes.reserve(keptCount());
    for (std::size_t i = 0; i < count; ++i) {
        if (!keep[i]) {
            continue;
        }
        filteredPoints.push_back(points[i]);
        filteredTimes.push_back(times[i]);
    }

    points = std::move(filteredPoints);
    times = std::move(filteredTimes);
    return removed;
}

Vector2 PointOnRibbon(const TraceRibbonFit &fit, float s, float h) {
    return {
        fit.center.x + fit.direction.x * s + fit.normal.x * h,
        fit.center.y + fit.direction.y * s + fit.normal.y * h
    };
}

float Dot2(const Vector2 &a, const Vector2 &b) {
    return a.x * b.x + a.y * b.y;
}

float Norm2(const Vector2 &v) {
    return std::sqrt(Dot2(v, v));
}

Vector2 NormalizeOr(const Vector2 &v, const Vector2 &fallback) {
    const float norm = Norm2(v);
    if (norm <= 1.0e-6f) {
        return fallback;
    }

    return {v.x / norm, v.y / norm};
}

float TraceFollowLateralWindowPx(float followWindowPx, float circleRadiusPx) {
    return std::clamp(
        std::max({18.0f, circleRadiusPx * 2.6f, followWindowPx * 0.35f}),
        18.0f,
        110.0f
    );
}

TracePcaResult FitTracePca(
    const std::vector<const TracePoint *> &points,
    const Vector2 &fallbackDirection) {

    TracePcaResult result;
    if (points.size() < 2) {
        return result;
    }

    double meanX = 0.0;
    double meanY = 0.0;
    int used = 0;
    for (const TracePoint *point : points) {
        if (point == nullptr
            || !std::isfinite(point->point.x)
            || !std::isfinite(point->point.y)) {
            continue;
        }

        meanX += static_cast<double>(point->point.x);
        meanY += static_cast<double>(point->point.y);
        ++used;
    }

    if (used < 2) {
        return result;
    }

    meanX /= static_cast<double>(used);
    meanY /= static_cast<double>(used);

    double cxx = 0.0;
    double cxy = 0.0;
    double cyy = 0.0;
    for (const TracePoint *point : points) {
        if (point == nullptr
            || !std::isfinite(point->point.x)
            || !std::isfinite(point->point.y)) {
            continue;
        }

        const double dx = static_cast<double>(point->point.x) - meanX;
        const double dy = static_cast<double>(point->point.y) - meanY;
        cxx += dx * dx;
        cxy += dx * dy;
        cyy += dy * dy;
    }

    cxx /= static_cast<double>(used);
    cxy /= static_cast<double>(used);
    cyy /= static_cast<double>(used);

    const double trace = cxx + cyy;
    if (!std::isfinite(trace) || trace <= 1.0e-9) {
        return result;
    }

    const double delta = std::sqrt(std::max(0.0, (cxx - cyy) * (cxx - cyy) + 4.0 * cxy * cxy));
    const double lambda1 = 0.5 * (trace + delta);
    const double lambda2 = 0.5 * (trace - delta);

    Vector2 direction{
        static_cast<float>(cxy),
        static_cast<float>(lambda1 - cxx)
    };

    if (std::fabs(direction.x) + std::fabs(direction.y) < 1.0e-6f) {
        direction = fallbackDirection;
    }

    direction = NormalizeOr(direction, NormalizeOr(fallbackDirection, {1.0f, 0.0f}));
    if (Dot2(direction, fallbackDirection) < 0.0f) {
        direction.x *= -1.0f;
        direction.y *= -1.0f;
    }

    result.valid = true;
    result.center = {static_cast<float>(meanX), static_cast<float>(meanY)};
    result.direction = direction;
    result.lambda1 = lambda1;
    result.lambda2 = lambda2;
    result.eventCount = used;
    return result;
}

std::vector<TracePcaSlice> BuildTemporalPcaSlices(
    const std::vector<TracePoint> &points,
    float requestedSlicePeriodMs,
    int64_t timeOriginUs,
    const Vector2 &fallbackDirection) {

    std::vector<TracePcaSlice> slices;
    if (points.size() < 2) {
        return slices;
    }

    int64_t tMin = points.front().timestampUs;
    int64_t tMax = points.front().timestampUs;
    for (const TracePoint &point : points) {
        tMin = std::min(tMin, point.timestampUs);
        tMax = std::max(tMax, point.timestampUs);
    }

    if (tMax <= tMin) {
        return slices;
    }

    constexpr int kMaxTimedPcaSlices = 160;
    const double requestedPeriodUs =
        static_cast<double>(std::clamp(requestedSlicePeriodMs, 2.0f, 80.0f)) * 1000.0;
    const double durationUs = static_cast<double>(tMax - tMin);
    const int sliceCount = std::clamp(
        static_cast<int>(std::ceil(durationUs / std::max(1.0, requestedPeriodUs))),
        1,
        kMaxTimedPcaSlices
    );
    slices.reserve(static_cast<std::size_t>(sliceCount));

    std::vector<std::vector<const TracePoint *>> slicePoints(static_cast<std::size_t>(sliceCount));
    for (const TracePoint &point : points) {
        double position = static_cast<double>(point.timestampUs - tMin) / durationUs;
        int slice = static_cast<int>(std::floor(position * static_cast<double>(sliceCount)));
        slice = std::clamp(slice, 0, sliceCount - 1);
        slicePoints[static_cast<std::size_t>(slice)].push_back(&point);
    }

    Vector2 previousDirection = NormalizeOr(fallbackDirection, {1.0f, 0.0f});
    for (int slice = 0; slice < sliceCount; ++slice) {
        const auto &currentPoints = slicePoints[static_cast<std::size_t>(slice)];
        const std::size_t minSliceSupport = std::max<std::size_t>(
            12,
            points.size() / static_cast<std::size_t>(sliceCount * 12)
        );
        if (currentPoints.size() < minSliceSupport) {
            continue;
        }

        TracePcaResult pca = FitTracePca(currentPoints, previousDirection);
        if (!pca.valid) {
            continue;
        }

        if (Dot2(pca.direction, previousDirection) < 0.0f) {
            pca.direction.x *= -1.0f;
            pca.direction.y *= -1.0f;
        }

        const int64_t sliceStartUs = tMin + static_cast<int64_t>(
            std::floor(static_cast<double>(slice) * durationUs / static_cast<double>(sliceCount))
        );
        const int64_t sliceEndUs = tMin + static_cast<int64_t>(
            std::ceil(static_cast<double>(slice + 1) * durationUs / static_cast<double>(sliceCount))
        );
        TracePcaSlice pcaSlice;
        pcaSlice.valid = true;
        pcaSlice.center = pca.center;
        pcaSlice.tangent = pca.direction;
        pcaSlice.normal = {-pca.direction.y, pca.direction.x};
        pcaSlice.timeStartSeconds = static_cast<double>(sliceStartUs - timeOriginUs) * 1.0e-6;
        pcaSlice.timeEndSeconds = static_cast<double>(sliceEndUs - timeOriginUs) * 1.0e-6;
        pcaSlice.eventCount = pca.eventCount;
        slices.push_back(pcaSlice);
        previousDirection = pca.direction;
    }

    return slices;
}

Vector2 CombineTemporalPcaDirection(
    const std::vector<TracePcaSlice> &slices,
    const Vector2 &globalDirection) {

    if (slices.size() < 2) {
        return globalDirection;
    }

    Vector2 weightedDirection{0.0f, 0.0f};
    const TracePcaSlice *first = nullptr;
    const TracePcaSlice *last = nullptr;
    for (const TracePcaSlice &slice : slices) {
        if (!slice.valid) {
            continue;
        }

        Vector2 tangent = slice.tangent;
        if (Dot2(tangent, globalDirection) < 0.0f) {
            tangent.x *= -1.0f;
            tangent.y *= -1.0f;
        }

        const float weight = static_cast<float>(std::max(1, slice.eventCount));
        weightedDirection.x += tangent.x * weight;
        weightedDirection.y += tangent.y * weight;

        if (first == nullptr) {
            first = &slice;
        }
        last = &slice;
    }

    if (first == nullptr || last == nullptr || first == last) {
        return globalDirection;
    }

    weightedDirection = NormalizeOr(weightedDirection, globalDirection);
    if (Dot2(weightedDirection, globalDirection) < 0.0f) {
        weightedDirection.x *= -1.0f;
        weightedDirection.y *= -1.0f;
    }

    Vector2 temporalDirection{
        last->center.x - first->center.x,
        last->center.y - first->center.y
    };
    temporalDirection = NormalizeOr(temporalDirection, weightedDirection);
    if (Dot2(temporalDirection, globalDirection) < 0.0f) {
        temporalDirection.x *= -1.0f;
        temporalDirection.y *= -1.0f;
    }

    return NormalizeOr(
        {
            globalDirection.x * 0.45f + weightedDirection.x * 0.35f + temporalDirection.x * 0.20f,
            globalDirection.y * 0.45f + weightedDirection.y * 0.35f + temporalDirection.y * 0.20f
        },
        globalDirection
    );
}

bool SolveSmallLinearSystem(
    std::array<std::array<double, 5>, 5> matrix,
    std::array<double, 5> rhs,
    int size,
    std::array<double, 5> &solution) {

    for (int pivot = 0; pivot < size; ++pivot) {
        int bestRow = pivot;
        double bestValue = std::fabs(matrix[pivot][pivot]);

        for (int row = pivot + 1; row < size; ++row) {
            const double value = std::fabs(matrix[row][pivot]);
            if (value > bestValue) {
                bestValue = value;
                bestRow = row;
            }
        }

        if (bestValue < 1.0e-12) {
            return false;
        }

        if (bestRow != pivot) {
            std::swap(matrix[pivot], matrix[bestRow]);
            std::swap(rhs[pivot], rhs[bestRow]);
        }

        const double divisor = matrix[pivot][pivot];
        for (int col = pivot; col < size; ++col) {
            matrix[pivot][col] /= divisor;
        }
        rhs[pivot] /= divisor;

        for (int row = 0; row < size; ++row) {
            if (row == pivot) {
                continue;
            }

            const double factor = matrix[row][pivot];
            if (std::fabs(factor) < 1.0e-18) {
                continue;
            }

            for (int col = pivot; col < size; ++col) {
                matrix[row][col] -= factor * matrix[pivot][col];
            }
            rhs[row] -= factor * rhs[pivot];
        }
    }

    solution = {};
    for (int i = 0; i < size; ++i) {
        solution[i] = rhs[i];
    }

    return true;
}

LocalQuadraticModel MakeLocalQuadraticModel(
    std::vector<PolynomialSample> samples,
    double bandwidth,
    int order) {

    LocalQuadraticModel model;
    model.order = std::clamp(order, 1, 2);
    model.valid = samples.size() >= static_cast<std::size_t>(model.order + 1);
    model.bandwidth = std::max(1.0, bandwidth);
    model.samples = std::move(samples);
    return model;
}

LocalQuadraticFit FitLocalQuadraticAt(const LocalQuadraticModel &model, float s) {
    LocalQuadraticFit fit;
    const int degree = std::clamp(model.order, 1, 2);
    const int fitSize = degree + 1;
    if (!model.valid || model.samples.size() < static_cast<std::size_t>(fitSize)) {
        return fit;
    }

    fit.degree = degree;
    fit.scale = std::max(1.0, model.bandwidth);

    auto solveWeighted = [&](bool localOnly, LocalQuadraticFit &output) {
        std::array<std::array<double, 5>, 5> normal{};
        std::array<double, 5> rhs{};
        int used = 0;
        double supportMin = std::numeric_limits<double>::max();
        double supportMax = std::numeric_limits<double>::lowest();

        for (const PolynomialSample &sample : model.samples) {
            const double x = (sample.x - static_cast<double>(s)) / fit.scale;
            const double distance = std::fabs(x);
            double weight = 0.0;

            if (distance <= 1.0) {
                const double oneMinusD3 = 1.0 - distance * distance * distance;
                weight = oneMinusD3 * oneMinusD3 * oneMinusD3;
            }
            else if (!localOnly) {
                weight = std::exp(-0.5 * distance * distance);
            }

            if (weight <= 1.0e-9) {
                continue;
            }

            const double x2 = x * x;
            const std::array<double, 5> powers{1.0, x, x2, x2 * x, x2 * x2};
            for (int row = 0; row < fitSize; ++row) {
                rhs[row] += weight * sample.y * powers[row];
                for (int col = 0; col < fitSize; ++col) {
                    normal[row][col] += weight * powers[row + col];
                }
            }
            supportMin = std::min(supportMin, sample.x);
            supportMax = std::max(supportMax, sample.x);
            ++used;
        }

        if (used < fitSize) {
            return false;
        }

        for (int i = 0; i < fitSize; ++i) {
            normal[i][i] += 1.0e-8;
        }

        std::array<double, 5> solution{};
        if (!SolveSmallLinearSystem(normal, rhs, fitSize, solution)) {
            return false;
        }

        output.valid = true;
        output.fallback = !localOnly;
        output.degree = degree;
        output.supportCount = used;
        output.supportSpanPx = static_cast<float>(std::max(0.0, supportMax - supportMin));
        output.sparse = output.fallback
            || output.supportCount <= degree + 1
            || output.supportSpanPx < static_cast<float>(fit.scale * 0.35);
        output.scale = fit.scale;
        output.c = solution[0];
        output.b = solution[1];
        output.a = degree >= 2 ? solution[2] : 0.0;
        return true;
    };

    if (solveWeighted(true, fit)) {
        return fit;
    }

    solveWeighted(false, fit);
    return fit;
}

float RibbonCurveH(const LocalQuadraticModel &curve, float s) {
    const LocalQuadraticFit fit = FitLocalQuadraticAt(curve, s);
    return fit.valid ? static_cast<float>(fit.c) : 0.0f;
}

float RibbonMiddleH(const TraceRibbonFit &fit, float s) {
    return RibbonCurveH(fit.middleModel, s);
}

double EvaluateGlobalPolynomial(const GlobalPolynomialModel &model, double x) {
    return model.c + model.b * x + model.a * x * x;
}

bool FitGlobalPolynomial(
    const std::vector<PolynomialSample> &samples,
    const std::vector<char> &mask,
    int degree,
    GlobalPolynomialModel &model) {

    degree = std::clamp(degree, 1, 2);
    const int fitSize = degree + 1;

    std::array<std::array<double, 5>, 5> normal{};
    std::array<double, 5> rhs{};
    int used = 0;

    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (!mask.empty() && !mask[i]) {
            continue;
        }

        const double x = samples[i].x;
        const double y = samples[i].y;
        const double x2 = x * x;
        const std::array<double, 5> powers{1.0, x, x2, x2 * x, x2 * x2};

        for (int row = 0; row < fitSize; ++row) {
            rhs[row] += y * powers[row];
            for (int col = 0; col < fitSize; ++col) {
                normal[row][col] += powers[row + col];
            }
        }
        ++used;
    }

    if (used < fitSize) {
        return false;
    }

    for (int i = 0; i < fitSize; ++i) {
        normal[i][i] += 1.0e-8;
    }

    std::array<double, 5> solution{};
    if (!SolveSmallLinearSystem(normal, rhs, fitSize, solution)) {
        return false;
    }

    model.valid = true;
    model.degree = degree;
    model.c = solution[0];
    model.b = solution[1];
    model.a = degree >= 2 ? solution[2] : 0.0;
    return true;
}

int FilterRibbonSamplesRansac(
    std::vector<PolynomialSample> &middleSamples,
    std::vector<PolynomialSample> &lowSamples,
    std::vector<PolynomialSample> &highSamples,
    std::vector<float> &localWidths) {

    const std::size_t count = middleSamples.size();
    if (count < 10
        || lowSamples.size() != count
        || highSamples.size() != count
        || localWidths.size() != count) {
        return 0;
    }

    const int degree = count >= 12 ? 2 : 1;
    const int fitSize = degree + 1;
    const float medianWidth = MedianValue(localWidths);
    const float widthMad = MedianAbsoluteDeviation(localWidths, medianWidth);
    const float residualThreshold = std::max(4.0f, medianWidth * 0.34f);

    GlobalPolynomialModel bestModel;
    int bestInlierCount = 0;
    double bestError = std::numeric_limits<double>::max();

    auto evaluateCandidate = [&](const std::vector<char> &candidateMask) {
        GlobalPolynomialModel candidate;
        if (!FitGlobalPolynomial(middleSamples, candidateMask, degree, candidate)) {
            return;
        }

        int inlierCount = 0;
        double error = 0.0;
        for (const PolynomialSample &sample : middleSamples) {
            const double residual = std::fabs(sample.y - EvaluateGlobalPolynomial(candidate, sample.x));
            if (residual <= residualThreshold) {
                ++inlierCount;
                error += residual;
            }
            else {
                error += residualThreshold * 2.0;
            }
        }

        const bool better =
            inlierCount > bestInlierCount
            || (inlierCount == bestInlierCount && error < bestError);
        if (better) {
            bestModel = candidate;
            bestInlierCount = inlierCount;
            bestError = error;
        }
    };

    std::vector<char> allMask(count, 1);
    evaluateCandidate(allMask);

    std::uint32_t state = 2166136261u ^ static_cast<std::uint32_t>(count * 16777619u);
    auto nextIndex = [&]() {
        state = state * 1664525u + 1013904223u;
        return static_cast<std::size_t>(state % static_cast<std::uint32_t>(count));
    };

    constexpr int kIterations = 96;
    for (int iteration = 0; iteration < kIterations; ++iteration) {
        std::vector<char> candidateMask(count, 0);
        int selected = 0;
        int guard = 0;
        while (selected < fitSize && guard < 32) {
            const std::size_t index = nextIndex();
            if (!candidateMask[index]) {
                candidateMask[index] = 1;
                ++selected;
            }
            ++guard;
        }

        if (selected == fitSize) {
            evaluateCandidate(candidateMask);
        }
    }

    if (!bestModel.valid || bestInlierCount < std::max<int>(7, static_cast<int>(count * 0.55f))) {
        return 0;
    }

    std::vector<float> inlierResiduals;
    inlierResiduals.reserve(count);
    for (const PolynomialSample &sample : middleSamples) {
        const float residual = static_cast<float>(
            std::fabs(sample.y - EvaluateGlobalPolynomial(bestModel, sample.x))
        );
        if (residual <= residualThreshold) {
            inlierResiduals.push_back(residual);
        }
    }

    const float residualMedian = MedianValue(inlierResiduals);
    const float residualMad = MedianAbsoluteDeviation(inlierResiduals, residualMedian);
    const float finalResidualThreshold = std::max(
        residualThreshold,
        residualMedian + residualMad * 5.0f + 1.0f
    );
    const float widthThreshold = std::max(
        medianWidth * 0.65f,
        widthMad * 6.0f + 2.0f
    );

    std::vector<char> finalMask(count, 0);
    for (std::size_t i = 0; i < count; ++i) {
        const float residual = static_cast<float>(
            std::fabs(middleSamples[i].y - EvaluateGlobalPolynomial(bestModel, middleSamples[i].x))
        );
        const float widthResidual = std::fabs(localWidths[i] - medianWidth);
        if (residual <= finalResidualThreshold && widthResidual <= widthThreshold) {
            finalMask[i] = 1;
        }
    }

    GlobalPolynomialModel refinedModel;
    if (FitGlobalPolynomial(middleSamples, finalMask, degree, refinedModel)) {
        bestModel = refinedModel;
        for (std::size_t i = 0; i < count; ++i) {
            const float residual = static_cast<float>(
                std::fabs(middleSamples[i].y - EvaluateGlobalPolynomial(bestModel, middleSamples[i].x))
            );
            const float widthResidual = std::fabs(localWidths[i] - medianWidth);
            finalMask[i] = (residual <= finalResidualThreshold && widthResidual <= widthThreshold) ? 1 : 0;
        }
    }

    std::size_t keptCount = 0;
    for (char keep : finalMask) {
        if (keep) {
            ++keptCount;
        }
    }

    if (keptCount < std::max<std::size_t>(7, count / 2)) {
        return 0;
    }

    std::vector<PolynomialSample> filteredMiddle;
    std::vector<PolynomialSample> filteredLow;
    std::vector<PolynomialSample> filteredHigh;
    std::vector<float> filteredWidths;
    filteredMiddle.reserve(keptCount);
    filteredLow.reserve(keptCount);
    filteredHigh.reserve(keptCount);
    filteredWidths.reserve(keptCount);

    for (std::size_t i = 0; i < count; ++i) {
        if (!finalMask[i]) {
            continue;
        }
        filteredMiddle.push_back(middleSamples[i]);
        filteredLow.push_back(lowSamples[i]);
        filteredHigh.push_back(highSamples[i]);
        filteredWidths.push_back(localWidths[i]);
    }

    const int removed = static_cast<int>(count - keptCount);
    if (removed > 0) {
        middleSamples = std::move(filteredMiddle);
        lowSamples = std::move(filteredLow);
        highSamples = std::move(filteredHigh);
        localWidths = std::move(filteredWidths);
    }

    return removed;
}

Vector2 RibbonCoordinates(const TraceRibbonFit &fit, const Vector2 &point) {
    const Vector2 delta{
        point.x - fit.center.x,
        point.y - fit.center.y
    };

    return {
        Dot2(delta, fit.direction),
        Dot2(delta, fit.normal)
    };
}

Vector2 RibbonCurvePoint(const TraceRibbonFit &fit, const LocalQuadraticModel &curve, float s) {
    return PointOnRibbon(fit, s, RibbonCurveH(curve, s));
}

Vector2 RibbonTangent(const TraceRibbonFit &fit, float s) {
    const LocalQuadraticFit localFit = FitLocalQuadraticAt(fit.middleModel, s);
    const float dhds = localFit.valid ? static_cast<float>(localFit.b / localFit.scale) : 0.0f;
    return NormalizeOr(
        {
            fit.direction.x + fit.normal.x * dhds,
            fit.direction.y + fit.normal.y * dhds
        },
        fit.direction
    );
}

bool CurveIntersectionOnNormal(
    const TraceRibbonFit &fit,
    const LocalQuadraticModel &curve,
    float referenceS,
    const Vector2 &middle,
    const Vector2 &lineTangent,
    Vector2 &intersection) {

    if (!curve.valid) {
        return false;
    }

    auto lineValue = [&](float s) {
        const Vector2 point = RibbonCurvePoint(fit, curve, s);
        return Dot2({point.x - middle.x, point.y - middle.y}, lineTangent);
    };

    constexpr int kSearchSteps = 64;
    const float searchRadius = std::max(12.0f, fit.widthPx * 4.0f);
    const float s0 = std::max(fit.sMin, referenceS - searchRadius);
    const float s1 = std::min(fit.sMax, referenceS + searchRadius);

    if (s1 <= s0) {
        return false;
    }

    float bestS = referenceS;
    float bestLineError = std::numeric_limits<float>::max();
    bool found = false;

    auto rememberCandidate = [&](float s, float value) {
        const float lineError = std::fabs(value);
        const float referenceBias = 0.05f * std::fabs(s - referenceS);
        const float score = lineError + referenceBias;
        if (score < bestLineError) {
            bestLineError = score;
            bestS = s;
            found = true;
        }
    };

    float previousS = s0;
    float previousValue = lineValue(previousS);
    rememberCandidate(previousS, previousValue);

    for (int i = 1; i <= kSearchSteps; ++i) {
        const float alpha = static_cast<float>(i) / static_cast<float>(kSearchSteps);
        float currentS = s0 + alpha * (s1 - s0);
        float currentValue = lineValue(currentS);
        rememberCandidate(currentS, currentValue);

        if (std::isfinite(previousValue)
            && std::isfinite(currentValue)
            && previousValue * currentValue <= 0.0f) {

            float lo = previousS;
            float hi = currentS;
            float loValue = previousValue;

            for (int iter = 0; iter < 18; ++iter) {
                const float mid = 0.5f * (lo + hi);
                const float midValue = lineValue(mid);

                if (loValue * midValue <= 0.0f) {
                    hi = mid;
                }
                else {
                    lo = mid;
                    loValue = midValue;
                }
            }

            const float root = 0.5f * (lo + hi);
            rememberCandidate(root, lineValue(root));
        }

        previousS = currentS;
        previousValue = currentValue;
    }

    if (!found) {
        return false;
    }

    intersection = RibbonCurvePoint(fit, curve, bestS);
    return true;
}

Vector3 TraceImagePointToWorldMeters(
    const Vector2 &imagePoint,
    float widthPx,
    const Vector2 &widthDirection,
    const CalibrationData &calibration,
    float ballRadiusMm) {

    if (!calibration.ready || widthPx <= 1.0e-3f || ballRadiusMm <= 0.0f) {
        return {0.0f, 0.0f, 0.0f};
    }

    const float fx = static_cast<float>(calibration.fx());
    const float fy = static_cast<float>(calibration.fy());
    const float cx = static_cast<float>(calibration.cx());
    const float cy = static_cast<float>(calibration.cy());

    if (std::fabs(fx) <= 1.0e-6f || std::fabs(fy) <= 1.0e-6f) {
        return {0.0f, 0.0f, 0.0f};
    }

    const float widthNorm = std::sqrt(widthDirection.x * widthDirection.x + widthDirection.y * widthDirection.y);
    if (widthNorm <= 1.0e-6f) {
        return {0.0f, 0.0f, 0.0f};
    }

    const float nx = widthDirection.x / widthNorm;
    const float ny = widthDirection.y / widthNorm;
    const float fEff = std::sqrt((fx * nx) * (fx * nx) + (fy * ny) * (fy * ny));
    const float diameterMm = 2.0f * ballRadiusMm;
    const float depthMm = fEff * diameterMm / widthPx;

    if (!std::isfinite(depthMm) || depthMm <= 0.0f) {
        return {0.0f, 0.0f, 0.0f};
    }

    const float xMm = ((imagePoint.x - cx) / fx) * depthMm;
    const float yMm = ((imagePoint.y - cy) / fy) * depthMm;

    const cv::Point3f cameraPositionMm{xMm, yMm, depthMm};
    return {
        cameraPositionMm.x * 1.0e-3f,
        cameraPositionMm.z * 1.0e-3f,
        -cameraPositionMm.y * 1.0e-3f
    };
}

std::vector<Vector2> BuildRibbonCurve(
    const TraceRibbonFit &fit,
    const LocalQuadraticModel &curve,
    int segments,
    std::vector<Vector2> *sparseFitPoints) {

    std::vector<Vector2> output;
    output.reserve(static_cast<std::size_t>(segments + 1));

    for (int i = 0; i <= segments; ++i) {
        const float alpha = static_cast<float>(i) / static_cast<float>(segments);
        const float s = fit.sMin + alpha * (fit.sMax - fit.sMin);
        const LocalQuadraticFit localFit = FitLocalQuadraticAt(curve, s);
        const float h = localFit.valid ? static_cast<float>(localFit.c) : 0.0f;
        const Vector2 point = PointOnRibbon(fit, s, h);
        output.push_back(point);

        if (sparseFitPoints != nullptr && (!localFit.valid || localFit.sparse)) {
            sparseFitPoints->push_back(point);
        }
    }

    return output;
}

LocalTraceSection EstimateLocalFlowSection(
    const TraceRibbonFit &fit,
    const std::vector<const TraceProjection *> &timeLocal,
    float fallbackS,
    double sampleTime,
    double timeStep,
    float halfTangentWindow) {

    LocalTraceSection section;
    if (timeLocal.size() < 20) {
        return section;
    }

    std::vector<float> xs;
    std::vector<float> ys;
    std::vector<float> ss;
    xs.reserve(timeLocal.size());
    ys.reserve(timeLocal.size());
    ss.reserve(timeLocal.size());

    double meanTime = 0.0;
    double meanX = 0.0;
    double meanY = 0.0;

    for (const TraceProjection *projection : timeLocal) {
        xs.push_back(projection->point.x);
        ys.push_back(projection->point.y);
        ss.push_back(projection->s);
        meanTime += projection->timeSeconds;
        meanX += projection->point.x;
        meanY += projection->point.y;
    }

    const double invCount = 1.0 / static_cast<double>(timeLocal.size());
    meanTime *= invCount;
    meanX *= invCount;
    meanY *= invCount;

    const Vector2 robustCenter{
        Quantile(xs, 0.50f),
        Quantile(ys, 0.50f)
    };
    const float medianS = std::clamp(Quantile(ss, 0.50f), fit.sMin, fit.sMax);
    const float referenceS = std::clamp(
        std::isfinite(medianS) ? medianS : fallbackS,
        fit.sMin,
        fit.sMax
    );

    double timeVariance = 0.0;
    double covXTime = 0.0;
    double covYTime = 0.0;
    double cxx = 0.0;
    double cxy = 0.0;
    double cyy = 0.0;

    for (const TraceProjection *projection : timeLocal) {
        const double dt = projection->timeSeconds - meanTime;
        const double dx = static_cast<double>(projection->point.x) - meanX;
        const double dy = static_cast<double>(projection->point.y) - meanY;

        timeVariance += dt * dt;
        covXTime += dt * dx;
        covYTime += dt * dy;
        cxx += dx * dx;
        cxy += dx * dy;
        cyy += dy * dy;
    }

    Vector2 geometricTangent = fit.direction;
    if (fit.middleModel.valid) {
        geometricTangent = RibbonTangent(fit, referenceS);
    }

    Vector2 tangent = geometricTangent;
    bool hasFlowTangent = false;

    if (timeVariance > 1.0e-10) {
        Vector2 flowDirection{
            static_cast<float>(covXTime / timeVariance),
            static_cast<float>(covYTime / timeVariance)
        };
        const float flowSpeed = Norm2(flowDirection);

        if (flowSpeed > 1.0e-3f) {
            flowDirection = NormalizeOr(flowDirection, geometricTangent);
            if (Dot2(flowDirection, geometricTangent) < 0.0f) {
                flowDirection.x *= -1.0f;
                flowDirection.y *= -1.0f;
            }

            const float flowDisplacement = flowSpeed * static_cast<float>(std::max(timeStep, 1.0e-6));
            const float flowWeight = std::clamp(
                flowDisplacement / std::max(3.0f, fit.widthPx),
                0.0f,
                0.90f
            );

            tangent = NormalizeOr(
                {
                    geometricTangent.x * (1.0f - flowWeight) + flowDirection.x * flowWeight,
                    geometricTangent.y * (1.0f - flowWeight) + flowDirection.y * flowWeight
                },
                flowDirection
            );
            hasFlowTangent = true;
        }
    }

    if (!hasFlowTangent) {
        const double trace = cxx + cyy;
        const double delta = std::sqrt(std::max(0.0, (cxx - cyy) * (cxx - cyy) + 4.0 * cxy * cxy));
        const double lambda1 = 0.5 * (trace + delta);

        Vector2 pcaTangent{
            static_cast<float>(cxy),
            static_cast<float>(lambda1 - cxx)
        };

        pcaTangent = NormalizeOr(pcaTangent, geometricTangent);
        if (Dot2(pcaTangent, geometricTangent) < 0.0f) {
            pcaTangent.x *= -1.0f;
            pcaTangent.y *= -1.0f;
        }

        tangent = NormalizeOr(
            {
                geometricTangent.x * 0.35f + pcaTangent.x * 0.65f,
                geometricTangent.y * 0.35f + pcaTangent.y * 0.65f
            },
            geometricTangent
        );
    }

    Vector2 normal{-tangent.y, tangent.x};
    if (Dot2(normal, fit.normal) < 0.0f) {
        normal.x *= -1.0f;
        normal.y *= -1.0f;
    }

    const Vector2 sectionCenter = fit.middleModel.valid
        ? PointOnRibbon(fit, referenceS, RibbonMiddleH(fit, referenceS))
        : robustCenter;

    std::vector<float> normalOffsets;
    normalOffsets.reserve(timeLocal.size());

    for (const TraceProjection *projection : timeLocal) {
        const Vector2 delta{
            projection->point.x - sectionCenter.x,
            projection->point.y - sectionCenter.y
        };
        const float tangentOffset = Dot2(delta, tangent);
        if (std::fabs(tangentOffset) > halfTangentWindow) {
            continue;
        }

        normalOffsets.push_back(Dot2(delta, normal));
    }

    if (normalOffsets.size() < 12) {
        return section;
    }

    const float medianOffset = Quantile(normalOffsets, 0.50f);
    std::vector<float> absoluteDeviations;
    absoluteDeviations.reserve(normalOffsets.size());
    for (float offset : normalOffsets) {
        absoluteDeviations.push_back(std::fabs(offset - medianOffset));
    }

    const float robustSigma = 1.4826f * Quantile(absoluteDeviations, 0.50f);
    const float trimLimit = std::max(6.0f, std::max(fit.widthPx * 0.95f, robustSigma * 3.0f));

    std::vector<float> trimmedOffsets;
    trimmedOffsets.reserve(normalOffsets.size());
    for (float offset : normalOffsets) {
        if (std::fabs(offset - medianOffset) <= trimLimit) {
            trimmedOffsets.push_back(offset);
        }
    }

    if (trimmedOffsets.size() < 12) {
        trimmedOffsets = std::move(normalOffsets);
    }

    const TraceEdgeEstimate edgeEstimate = EstimateSupportedEdges(
        trimmedOffsets,
        12,
        fit.supportEdge
    );
    if (!edgeEstimate.valid) {
        return section;
    }

    const float eventLow = edgeEstimate.low;
    const float eventHigh = edgeEstimate.high;
    const float widthPx = eventHigh - eventLow;
    if (!std::isfinite(widthPx) || widthPx < 1.0f) {
        return section;
    }

    const float middleOffset = 0.5f * (eventLow + eventHigh);
    const Vector2 middle{
        sectionCenter.x + normal.x * middleOffset,
        sectionCenter.y + normal.y * middleOffset
    };
    Vector2 endA{
        sectionCenter.x + normal.x * eventLow,
        sectionCenter.y + normal.y * eventLow
    };
    Vector2 endB{
        sectionCenter.x + normal.x * eventHigh,
        sectionCenter.y + normal.y * eventHigh
    };

    if (endA.y < endB.y) {
        section.upper = endA;
        section.lower = endB;
    }
    else {
        section.upper = endB;
        section.lower = endA;
    }

    const Vector2 ribbonMiddle = RibbonCoordinates(fit, middle);
    section.valid = true;
    section.middle = middle;
    section.tangent = tangent;
    section.normal = NormalizeOr(
        {
            section.upper.x - section.lower.x,
            section.upper.y - section.lower.y
        },
        normal
    );
    section.widthPx = widthPx;
    section.s = std::clamp(ribbonMiddle.x, fit.sMin, fit.sMax);
    section.timeSeconds = sampleTime;
    section.localEventCount = edgeEstimate.supportCount;
    return section;
}

std::vector<TraceWidthEstimate> EstimateTraceWidths(const TraceRibbonFit &fit, int sampleCount) {
    std::vector<TraceWidthEstimate> estimates;
    if (!fit.valid || fit.projections.empty() || sampleCount < 1) {
        return estimates;
    }

    double tMin = std::numeric_limits<double>::max();
    double tMax = std::numeric_limits<double>::lowest();
    for (const TraceProjection &projection : fit.projections) {
        tMin = std::min(tMin, projection.timeSeconds);
        tMax = std::max(tMax, projection.timeSeconds);
    }

    if (!std::isfinite(tMin) || !std::isfinite(tMax) || tMax <= tMin) {
        return estimates;
    }

    const double timeStep = (tMax - tMin) / static_cast<double>(sampleCount);
    const double halfTimeWindow = std::max(timeStep * 0.75, 1.0e-4);
    const float halfTangentWindow = std::max(
        8.0f,
        std::max(fit.widthPx * 1.35f, fit.lengthPx / static_cast<float>(sampleCount * 2))
    );
    const float roughHalfWidthLimit = std::max(12.0f, fit.widthPx * 1.8f);

    estimates.reserve(static_cast<std::size_t>(sampleCount));

    for (int sample = 0; sample < sampleCount; ++sample) {
        const double sampleTime = tMin + (static_cast<double>(sample) + 0.5) * timeStep;

        std::vector<const TraceProjection *> timeLocal;
        timeLocal.reserve(fit.projections.size() / static_cast<std::size_t>(sampleCount) + 16);

        for (const TraceProjection &projection : fit.projections) {
            if (std::fabs(projection.timeSeconds - sampleTime) > halfTimeWindow) {
                continue;
            }

            const float middleH = RibbonMiddleH(fit, projection.s);
            if (std::fabs(projection.h - middleH) <= roughHalfWidthLimit) {
                timeLocal.push_back(&projection);
            }
        }

        if (timeLocal.size() < 20) {
            continue;
        }

        std::vector<float> localS;
        localS.reserve(timeLocal.size());
        for (const TraceProjection *projection : timeLocal) {
            localS.push_back(projection->s);
        }

        const float sampleS = std::clamp(Quantile(localS, 0.50f), fit.sMin, fit.sMax);

        const LocalTraceSection localSection = EstimateLocalFlowSection(
            fit,
            timeLocal,
            sampleS,
            sampleTime,
            timeStep,
            halfTangentWindow
        );

        const Vector2 sectionMiddle = PointOnRibbon(fit, sampleS, RibbonMiddleH(fit, sampleS));
        const Vector2 geometricTangent = RibbonTangent(fit, sampleS);
        Vector2 geometricNormal{-geometricTangent.y, geometricTangent.x};
        if (Dot2(geometricNormal, fit.normal) < 0.0f) {
            geometricNormal.x *= -1.0f;
            geometricNormal.y *= -1.0f;
        }

        std::vector<float> normalOffsets;
        normalOffsets.reserve(timeLocal.size());

        for (const TraceProjection *projection : timeLocal) {
            const Vector2 delta{
                projection->point.x - sectionMiddle.x,
                projection->point.y - sectionMiddle.y
            };

            const float tangentOffset = Dot2(delta, geometricTangent);
            if (std::fabs(tangentOffset) > halfTangentWindow) {
                continue;
            }

            normalOffsets.push_back(Dot2(delta, geometricNormal));
        }

        if (normalOffsets.size() < 12) {
            continue;
        }

        const TraceEdgeEstimate edgeEstimate = EstimateSupportedEdges(
            normalOffsets,
            12,
            fit.supportEdge
        );
        if (!edgeEstimate.valid) {
            continue;
        }

        const float eventLow = edgeEstimate.low;
        const float eventHigh = edgeEstimate.high;
        const float eventWidth = eventHigh - eventLow;

        if (!std::isfinite(eventWidth) || eventWidth < 2.0f) {
            continue;
        }

        Vector2 endA{
            sectionMiddle.x + geometricNormal.x * eventLow,
            sectionMiddle.y + geometricNormal.y * eventLow
        };
        Vector2 endB{
            sectionMiddle.x + geometricNormal.x * eventHigh,
            sectionMiddle.y + geometricNormal.y * eventHigh
        };
        float endAOffset = eventLow;
        float endBOffset = eventHigh;

        Vector2 upperIntersection{};
        Vector2 lowerIntersection{};
        const bool hasUpperIntersection = CurveIntersectionOnNormal(
            fit,
            fit.upperModel,
            sampleS,
            sectionMiddle,
            geometricTangent,
            upperIntersection
        );
        const bool hasLowerIntersection = CurveIntersectionOnNormal(
            fit,
            fit.lowerModel,
            sampleS,
            sectionMiddle,
            geometricTangent,
            lowerIntersection
        );

        float widthPx = eventWidth;
        if (hasUpperIntersection && hasLowerIntersection) {
            const Vector2 upperDelta{
                upperIntersection.x - sectionMiddle.x,
                upperIntersection.y - sectionMiddle.y
            };
            const Vector2 lowerDelta{
                lowerIntersection.x - sectionMiddle.x,
                lowerIntersection.y - sectionMiddle.y
            };
            const float upperOffset = Dot2(upperDelta, geometricNormal);
            const float lowerOffset = Dot2(lowerDelta, geometricNormal);
            const float envelopeWidth = std::fabs(upperOffset - lowerOffset);
            const float widthRatio = envelopeWidth / eventWidth;

            if (std::isfinite(envelopeWidth)
                && envelopeWidth >= 2.0f
                && widthRatio >= 0.55f
                && widthRatio <= 1.70f) {

                widthPx = envelopeWidth;
                endAOffset = upperOffset;
                endBOffset = lowerOffset;
                endA = {
                    sectionMiddle.x + geometricNormal.x * endAOffset,
                    sectionMiddle.y + geometricNormal.y * endAOffset
                };
                endB = {
                    sectionMiddle.x + geometricNormal.x * endBOffset,
                    sectionMiddle.y + geometricNormal.y * endBOffset
                };
            }
        }

        if (!std::isfinite(widthPx) || widthPx < 1.0f) {
            continue;
        }

        TraceWidthEstimate estimate;
        estimate.valid = true;
        estimate.middle = sectionMiddle;
        estimate.widthPx = widthPx;
        estimate.timeSeconds = sampleTime;
        estimate.localEventCount = localSection.valid ? localSection.localEventCount : edgeEstimate.supportCount;
        estimate.tangent = geometricTangent;

        if (endA.y < endB.y) {
            estimate.upper = endA;
            estimate.lower = endB;
        }
        else {
            estimate.upper = endB;
            estimate.lower = endA;
        }
        estimate.normal = geometricNormal;

        estimates.push_back(estimate);
    }

    FilterTraceWidthSpikes(estimates);
    return estimates;
}

TraceRibbonFit FitTraceRibbon(
    const std::vector<TracePoint> &points,
    float lineBinWidthPx,
    float localWindowPx,
    int localOrder,
    float pcaPeriodMs,
    const TraceSupportEdgeSettings &supportEdge,
    bool refineEdges) {

    TraceRibbonFit fit;

    constexpr std::size_t kMinRawEvents = 500;
    constexpr std::size_t kMinInlierEvents = 250;
    constexpr int kMinValidBins = 7;
    constexpr int kSparseDiagnosticSegments = 90;
    const float requestedLineBinWidth = std::clamp(lineBinWidthPx, 4.0f, 48.0f);
    const float localWindow = std::clamp(localWindowPx, 8.0f, 240.0f);
    const int fitOrder = std::clamp(localOrder, 1, 2);
    const float timedPcaPeriodMs = std::clamp(pcaPeriodMs, 2.0f, 80.0f);
    TraceSupportEdgeSettings edgeSettings = supportEdge;
    edgeSettings.supportDivisor = std::clamp(edgeSettings.supportDivisor, 8.0f, 60.0f);
    edgeSettings.minLocalSupport = std::clamp<std::size_t>(edgeSettings.minLocalSupport, 1, 20);
    edgeSettings.maxLocalSupport = std::clamp<std::size_t>(
        std::max(edgeSettings.maxLocalSupport, edgeSettings.minLocalSupport),
        edgeSettings.minLocalSupport,
        30
    );
    edgeSettings.supportRadiusPx = std::clamp(edgeSettings.supportRadiusPx, 0.5f, 4.0f);
    edgeSettings.borderRatio = std::clamp(edgeSettings.borderRatio, 0.0f, 0.10f);

    if (points.size() < kMinRawEvents) {
        return fit;
    }

    int64_t timeOriginUs = points.front().timestampUs;
    for (const TracePoint &p : points) {
        timeOriginUs = std::min(timeOriginUs, p.timestampUs);
    }

    std::vector<float> xs;
    std::vector<float> ys;
    xs.reserve(points.size());
    ys.reserve(points.size());

    for (const TracePoint &p : points) {
        xs.push_back(p.point.x);
        ys.push_back(p.point.y);
    }

    const Vector2 robustCenter{
        Quantile(xs, 0.50f),
        Quantile(ys, 0.50f)
    };

    std::vector<float> squaredDistances;
    squaredDistances.reserve(points.size());
    for (const TracePoint &p : points) {
        const float dx = p.point.x - robustCenter.x;
        const float dy = p.point.y - robustCenter.y;
        squaredDistances.push_back(dx * dx + dy * dy);
    }

    const float radialLimit = Quantile(squaredDistances, 0.97f);

    std::vector<TracePoint> inliers;
    inliers.reserve(points.size());
    for (const TracePoint &p : points) {
        const float dx = p.point.x - robustCenter.x;
        const float dy = p.point.y - robustCenter.y;
        if (dx * dx + dy * dy <= radialLimit) {
            inliers.push_back(p);
        }
    }

    if (inliers.size() < kMinInlierEvents) {
        return fit;
    }

    std::vector<const TracePoint *> inlierRefs;
    inlierRefs.reserve(inliers.size());
    for (const TracePoint &p : inliers) {
        inlierRefs.push_back(&p);
    }

    const TracePcaResult globalPca = FitTracePca(inlierRefs, {1.0f, 0.0f});
    if (!globalPca.valid) {
        return fit;
    }

    std::vector<TracePcaSlice> temporalPcaSlices = BuildTemporalPcaSlices(
        inliers,
        timedPcaPeriodMs,
        timeOriginUs,
        globalPca.direction
    );
    Vector2 direction = temporalPcaSlices.size() > 1
        ? CombineTemporalPcaDirection(temporalPcaSlices, globalPca.direction)
        : globalPca.direction;
    direction = NormalizeOr(direction, globalPca.direction);

    const Vector2 center = globalPca.center;
    const Vector2 normal{-direction.y, direction.x};

    for (TracePcaSlice &slice : temporalPcaSlices) {
        if (Dot2(slice.tangent, direction) < 0.0f) {
            slice.tangent.x *= -1.0f;
            slice.tangent.y *= -1.0f;
        }
        slice.normal = {-slice.tangent.y, slice.tangent.x};
    }

    std::vector<float> sValues;
    std::vector<float> hValues;
    sValues.reserve(inliers.size());
    hValues.reserve(inliers.size());

    for (const TracePoint &p : inliers) {
        const float dx = p.point.x - center.x;
        const float dy = p.point.y - center.y;
        sValues.push_back(direction.x * dx + direction.y * dy);
        hValues.push_back(normal.x * dx + normal.y * dy);
    }

    fit.center = center;
    fit.direction = direction;
    fit.normal = normal;
    fit.supportEdge = edgeSettings;
    fit.pcaPeriodMs = timedPcaPeriodMs;
    fit.temporalPcaSlices = std::move(temporalPcaSlices);
    fit.sMin = Quantile(sValues, 0.03f);
    fit.sMax = Quantile(sValues, 0.97f);
    fit.lengthPx = fit.sMax - fit.sMin;
    fit.inlierCount = inliers.size();

    if (fit.lengthPx < 35.0f) {
        return fit;
    }

    std::vector<TraceProjection> projections;
    projections.reserve(inliers.size());

    for (std::size_t i = 0; i < sValues.size(); ++i) {
        if (sValues[i] >= fit.sMin && sValues[i] <= fit.sMax) {
            projections.push_back({
                sValues[i],
                hValues[i],
                inliers[i].point,
                static_cast<double>(inliers[i].timestampUs - timeOriginUs) * 1.0e-6
            });
        }
    }

    if (projections.size() < kMinInlierEvents) {
        return fit;
    }

    const int binCount = std::clamp(
        static_cast<int>(std::ceil(fit.lengthPx / requestedLineBinWidth)),
        8,
        260
    );
    fit.lineBinCount = binCount;
    fit.lineBinWidthPx = fit.lengthPx / static_cast<float>(binCount);

    std::vector<std::vector<float>> binS(static_cast<std::size_t>(binCount));
    std::vector<std::vector<float>> binH(static_cast<std::size_t>(binCount));

    const float binWidth = fit.lengthPx / static_cast<float>(binCount);
    if (binWidth <= 1.0e-6f) {
        return fit;
    }

    for (const TraceProjection &projection : projections) {
        int bin = static_cast<int>((projection.s - fit.sMin) / binWidth);
        bin = std::clamp(bin, 0, binCount - 1);
        binS[static_cast<std::size_t>(bin)].push_back(projection.s);
        binH[static_cast<std::size_t>(bin)].push_back(projection.h);
    }

    const std::size_t minEventsPerBin = std::max<std::size_t>(
        10,
        projections.size() / static_cast<std::size_t>(binCount * 10)
    );

    std::vector<PolynomialSample> middleSamples;
    std::vector<PolynomialSample> lowSamples;
    std::vector<PolynomialSample> highSamples;
    std::vector<float> localWidths;

    int validBins = 0;
    for (int bin = 0; bin < binCount; ++bin) {
        std::vector<float> &sBin = binS[static_cast<std::size_t>(bin)];
        std::vector<float> &hBin = binH[static_cast<std::size_t>(bin)];

        if (hBin.size() < minEventsPerBin) {
            continue;
        }

        const float s = Quantile(sBin, 0.50f);
        const TraceEdgeEstimate edgeEstimate = EstimateSupportedEdges(
            hBin,
            minEventsPerBin,
            edgeSettings
        );
        if (!edgeEstimate.valid) {
            continue;
        }

        const float hLow = edgeEstimate.low;
        const float hMiddle = edgeEstimate.middle;
        const float hHigh = edgeEstimate.high;
        const float localWidth = hHigh - hLow;

        if (!std::isfinite(localWidth) || localWidth < 2.0f) {
            continue;
        }

        middleSamples.push_back({s, hMiddle});
        lowSamples.push_back({s, hLow});
        highSamples.push_back({s, hHigh});
        localWidths.push_back(localWidth);
        ++validBins;
    }

    fit.ransacRejectedSamples = FilterRibbonSamplesRansac(
        middleSamples,
        lowSamples,
        highSamples,
        localWidths
    );
    FilterCoherentRibbonSamples(middleSamples, lowSamples, highSamples, localWidths);
    validBins = static_cast<int>(middleSamples.size());

    if (validBins < kMinValidBins
        || middleSamples.size() < static_cast<std::size_t>(fitOrder + 1)
        || lowSamples.size() < static_cast<std::size_t>(fitOrder + 1)
        || highSamples.size() < static_cast<std::size_t>(fitOrder + 1)) {
        return fit;
    }

    float sampleSMin = static_cast<float>(middleSamples.front().x);
    float sampleSMax = sampleSMin;
    for (const PolynomialSample &sample : middleSamples) {
        sampleSMin = std::min(sampleSMin, static_cast<float>(sample.x));
        sampleSMax = std::max(sampleSMax, static_cast<float>(sample.x));
    }

    if (sampleSMax - sampleSMin >= 35.0f) {
        fit.sMin = sampleSMin;
        fit.sMax = sampleSMax;
        fit.lengthPx = fit.sMax - fit.sMin;

        std::vector<TraceProjection> clippedProjections;
        clippedProjections.reserve(projections.size());
        for (const TraceProjection &projection : projections) {
            if (projection.s >= fit.sMin && projection.s <= fit.sMax) {
                clippedProjections.push_back(projection);
            }
        }
        projections = std::move(clippedProjections);
    }

    if (projections.size() < kMinInlierEvents) {
        return fit;
    }

    fit.widthPx = Quantile(localWidths, 0.50f);
    if (fit.widthPx < 1.0f) {
        return fit;
    }

    const int visualCurveSegments = std::clamp(
        static_cast<int>(std::ceil(std::max(1.0f, fit.lengthPx))),
        kSparseDiagnosticSegments,
        1400
    );

    const double localBandwidth = static_cast<double>(localWindow);
    LocalQuadraticModel middleCurve = MakeLocalQuadraticModel(middleSamples, localBandwidth, fitOrder);
    LocalQuadraticModel lowCurve = MakeLocalQuadraticModel(lowSamples, localBandwidth, fitOrder);
    LocalQuadraticModel highCurve = MakeLocalQuadraticModel(highSamples, localBandwidth, fitOrder);
    if (!middleCurve.valid || !lowCurve.valid || !highCurve.valid) {
        return fit;
    }

    if (refineEdges) {
        // Second pass: re-detect the edges of every bin inside a narrow band
        // around the fitted curves, so stray events outside the ribbon cannot
        // pull the supported extremes, then refit the three curves on the
        // refined samples.
        std::vector<PolynomialSample> refinedMiddle;
        std::vector<PolynomialSample> refinedLow;
        std::vector<PolynomialSample> refinedHigh;
        std::vector<float> refinedWidths;
        std::vector<float> bandValues;

        for (int bin = 0; bin < binCount; ++bin) {
            const std::vector<float> &sBin = binS[static_cast<std::size_t>(bin)];
            const std::vector<float> &hBin = binH[static_cast<std::size_t>(bin)];
            if (hBin.size() < minEventsPerBin) {
                continue;
            }

            const float s = Quantile(sBin, 0.50f);
            if (s < fit.sMin || s > fit.sMax) {
                continue;
            }

            const float predictedLow = RibbonCurveH(lowCurve, s);
            const float predictedHigh = RibbonCurveH(highCurve, s);
            if (!std::isfinite(predictedLow)
                || !std::isfinite(predictedHigh)
                || predictedHigh - predictedLow < 2.0f) {
                continue;
            }

            const float margin = std::clamp((predictedHigh - predictedLow) * 0.30f, 1.5f, 5.0f);
            bandValues.clear();
            for (const float h : hBin) {
                if (h >= predictedLow - margin && h <= predictedHigh + margin) {
                    bandValues.push_back(h);
                }
            }
            if (bandValues.size() < minEventsPerBin) {
                continue;
            }

            const TraceEdgeEstimate refined = EstimateSupportedEdges(
                bandValues,
                minEventsPerBin,
                edgeSettings
            );
            if (!refined.valid) {
                continue;
            }

            const float refinedWidth = refined.high - refined.low;
            if (!std::isfinite(refinedWidth) || refinedWidth < 2.0f) {
                continue;
            }

            refinedMiddle.push_back({s, refined.middle});
            refinedLow.push_back({s, refined.low});
            refinedHigh.push_back({s, refined.high});
            refinedWidths.push_back(refinedWidth);
        }

        const std::size_t minRefinedSamples =
            static_cast<std::size_t>(std::max(kMinValidBins, fitOrder + 1));
        if (refinedMiddle.size() >= minRefinedSamples) {
            FilterCoherentRibbonSamples(refinedMiddle, refinedLow, refinedHigh, refinedWidths);
            if (refinedMiddle.size() >= minRefinedSamples) {
                const LocalQuadraticModel refinedMiddleCurve =
                    MakeLocalQuadraticModel(refinedMiddle, localBandwidth, fitOrder);
                const LocalQuadraticModel refinedLowCurve =
                    MakeLocalQuadraticModel(refinedLow, localBandwidth, fitOrder);
                const LocalQuadraticModel refinedHighCurve =
                    MakeLocalQuadraticModel(refinedHigh, localBandwidth, fitOrder);
                if (refinedMiddleCurve.valid && refinedLowCurve.valid && refinedHighCurve.valid) {
                    middleCurve = refinedMiddleCurve;
                    lowCurve = refinedLowCurve;
                    highCurve = refinedHighCurve;
                    localWidths = std::move(refinedWidths);
                    fit.widthPx = Quantile(localWidths, 0.50f);
                }
            }
        }
    }

    std::vector<Vector2> sparseLowFitPoints;
    std::vector<Vector2> sparseHighFitPoints;
    (void)BuildRibbonCurve(
        fit,
        lowCurve,
        kSparseDiagnosticSegments,
        &sparseLowFitPoints
    );
    (void)BuildRibbonCurve(
        fit,
        highCurve,
        kSparseDiagnosticSegments,
        &sparseHighFitPoints
    );

    std::vector<Vector2> lowImageCurve = BuildRibbonCurve(
        fit,
        lowCurve,
        visualCurveSegments
    );
    std::vector<Vector2> highImageCurve = BuildRibbonCurve(
        fit,
        highCurve,
        visualCurveSegments
    );

    float lowMeanY = 0.0f;
    float highMeanY = 0.0f;
    for (const Vector2 &point : lowImageCurve) {
        lowMeanY += point.y;
    }
    for (const Vector2 &point : highImageCurve) {
        highMeanY += point.y;
    }
    lowMeanY /= static_cast<float>(lowImageCurve.size());
    highMeanY /= static_cast<float>(highImageCurve.size());

    fit.middleModel = middleCurve;
    fit.projections = std::move(projections);
    fit.middleCurve = BuildRibbonCurve(fit, fit.middleModel, visualCurveSegments);
    (void)BuildRibbonCurve(fit, fit.middleModel, kSparseDiagnosticSegments, &fit.sparseMiddleFitPoints);
    if (lowMeanY < highMeanY) {
        fit.upperModel = lowCurve;
        fit.lowerModel = highCurve;
        fit.upperCurve = std::move(lowImageCurve);
        fit.lowerCurve = std::move(highImageCurve);
        fit.sparseUpperFitPoints = std::move(sparseLowFitPoints);
        fit.sparseLowerFitPoints = std::move(sparseHighFitPoints);
    }
    else {
        fit.upperModel = highCurve;
        fit.lowerModel = lowCurve;
        fit.upperCurve = std::move(highImageCurve);
        fit.lowerCurve = std::move(lowImageCurve);
        fit.sparseUpperFitPoints = std::move(sparseHighFitPoints);
        fit.sparseLowerFitPoints = std::move(sparseLowFitPoints);
    }

    fit.valid = true;
    return fit;
}

int64_t TraceTimeOriginUs(const std::vector<TracePoint> &tracePoints) {
    int64_t originUs = 0;
    if (!tracePoints.empty()) {
        originUs = tracePoints.front().timestampUs;
        for (const TracePoint &tracePoint : tracePoints) {
            originUs = std::min(originUs, tracePoint.timestampUs);
        }
    }
    return originUs;
}

Trace3DAnalysis AnalyzeTrace3D(
    const TraceRibbonFit &fit,
    const CalibrationData &calibration,
    float ballRadiusMm,
    float widthStepPx,
    bool smoothWidthProfile,
    int64_t traceTimeOriginUs,
    const TraceGroundTruthLookup &lookupGroundTruth) {

    Trace3DAnalysis analysis;
    if (!fit.valid) {
        return analysis;
    }

    analysis.widthSampleCount = std::clamp(
        static_cast<int>(std::ceil(fit.lengthPx / std::max(widthStepPx, 1.0f))),
        3,
        80
    );
    analysis.widthEstimates = EstimateTraceWidths(fit, analysis.widthSampleCount);
    if (smoothWidthProfile) {
        SmoothTraceWidthProfile(analysis.widthEstimates);
    }
    analysis.widthSamples.reserve(analysis.widthEstimates.size());

    for (const TraceWidthEstimate &estimate : analysis.widthEstimates) {
        if (!estimate.valid || estimate.widthPx < 1.0f) {
            continue;
        }

        analysis.widthSamples.push_back(estimate.widthPx);

        if (!calibration.ready) {
            continue;
        }

        const Vector2 widthVector{
            estimate.upper.x - estimate.lower.x,
            estimate.upper.y - estimate.lower.y
        };
        const Vector3 worldPoint = TraceImagePointToWorldMeters(
            estimate.middle,
            estimate.widthPx,
            widthVector,
            calibration,
            ballRadiusMm
        );

        if (std::isfinite(worldPoint.x)
            && std::isfinite(worldPoint.y)
            && std::isfinite(worldPoint.z)
            && (std::fabs(worldPoint.x) + std::fabs(worldPoint.y) + std::fabs(worldPoint.z)) > 1.0e-6f) {
            analysis.worldPoints.push_back(worldPoint);
            analysis.times.push_back(static_cast<float>(estimate.timeSeconds));
        }
    }

    analysis.rejectedWorldOutliers = FilterTraceWorldOutliers(analysis.worldPoints, analysis.times);

    if (analysis.worldPoints.size() == analysis.times.size()) {
        for (std::size_t i = 0; i < analysis.worldPoints.size(); ++i) {
            const float absoluteTimeSeconds =
                static_cast<float>(traceTimeOriginUs) * 1.0e-6f + analysis.times[i];
            Vector3 groundTruthPoint{};
            if (lookupGroundTruth(absoluteTimeSeconds, groundTruthPoint)) {
                analysis.groundTruthPoints.push_back(groundTruthPoint);
                analysis.groundTruthEstimatePoints.push_back(analysis.worldPoints[i]);
            }
        }
    }

    if (analysis.worldPoints.size() >= 3 && !analysis.widthSamples.empty()) {
        analysis.valid = true;
        analysis.currentWorld = analysis.worldPoints[analysis.worldPoints.size() / 2];
        const float medianWidth = Quantile(analysis.widthSamples, 0.50f);
        analysis.poseText = std::format(
            "Trace 3D position (m): X={:.3f}  Y={:.3f}  Z={:.3f} | W={:.1f}px",
            analysis.currentWorld.x,
            analysis.currentWorld.y,
            analysis.currentWorld.z,
            medianWidth
        );
    }

    return analysis;
}

