#pragma once

// Trace ribbon algorithm extracted from Gui.cpp: pure computation, no UI.
//
// Pipeline: accumulated trace events -> point source selection (polarity,
// raw/undistorted) -> robust PCA orientation -> s/h ribbon frame -> supported
// edges per bin -> upper/middle/lower curves -> local width estimates ->
// optional smoothed width profile -> 3D positions -> outlier filtering.

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <dv-processing/core/core.hpp>
#include <opencv2/core.hpp>
#include <raylib.h>

#include "Camera.hpp"

using TraceGroundTruthLookup = std::function<bool(float, Vector3 &)>;

struct TracePoint {
    Vector2 point{0.0f, 0.0f};
    int64_t timestampUs = 0;
    bool polarity = true;
};

struct TraceProjection {
    float s = 0.0f;
    float h = 0.0f;
    Vector2 point{0.0f, 0.0f};
    double timeSeconds = 0.0;
};

struct TraceWidthEstimate {
    bool valid = false;
    Vector2 middle{0.0f, 0.0f};
    Vector2 upper{0.0f, 0.0f};
    Vector2 lower{0.0f, 0.0f};
    Vector2 normal{0.0f, 1.0f};
    Vector2 tangent{1.0f, 0.0f};
    float widthPx = 0.0f;
    double timeSeconds = 0.0;
    int localEventCount = 0;
};

struct LocalTraceSection {
    bool valid = false;
    Vector2 middle{0.0f, 0.0f};
    Vector2 upper{0.0f, 0.0f};
    Vector2 lower{0.0f, 0.0f};
    Vector2 tangent{1.0f, 0.0f};
    Vector2 normal{0.0f, 1.0f};
    float widthPx = 0.0f;
    float s = 0.0f;
    double timeSeconds = 0.0;
    int localEventCount = 0;
};

struct PolynomialSample {
    double x = 0.0;
    double y = 0.0;
};

struct LocalQuadraticFit {
    bool valid = false;
    bool fallback = false;
    bool sparse = false;
    int degree = 2;
    int supportCount = 0;
    float supportSpanPx = 0.0f;
    double scale = 1.0;
    double c = 0.0;
    double b = 0.0;
    double a = 0.0;
};

struct LocalQuadraticModel {
    bool valid = false;
    int order = 2;
    double bandwidth = 24.0;
    std::vector<PolynomialSample> samples;
};

struct TracePcaResult {
    bool valid = false;
    Vector2 center{0.0f, 0.0f};
    Vector2 direction{1.0f, 0.0f};
    double lambda1 = 0.0;
    double lambda2 = 0.0;
    int eventCount = 0;
};

struct TracePcaSlice {
    bool valid = false;
    Vector2 center{0.0f, 0.0f};
    Vector2 tangent{1.0f, 0.0f};
    Vector2 normal{0.0f, 1.0f};
    double timeStartSeconds = 0.0;
    double timeEndSeconds = 0.0;
    int eventCount = 0;
};

struct TraceSupportEdgeSettings {
    float supportDivisor = 28.0f;
    std::size_t minLocalSupport = 3;
    std::size_t maxLocalSupport = 9;
    float supportRadiusPx = 1.75f;
    float borderRatio = 0.035f;
};

struct TraceRibbonFit {
    bool valid = false;
    Vector2 center{0.0f, 0.0f};
    Vector2 direction{1.0f, 0.0f};
    Vector2 normal{0.0f, 1.0f};
    LocalQuadraticModel middleModel{};
    LocalQuadraticModel upperModel{};
    LocalQuadraticModel lowerModel{};
    float sMin = 0.0f;
    float sMax = 0.0f;
    float lengthPx = 0.0f;
    float widthPx = 0.0f;
    std::size_t inlierCount = 0;
    int lineBinCount = 0;
    float lineBinWidthPx = 0.0f;
    TraceSupportEdgeSettings supportEdge{};
    float pcaPeriodMs = 8.0f;
    int ransacRejectedSamples = 0;
    std::vector<TraceProjection> projections;
    std::vector<Vector2> middleCurve;
    std::vector<Vector2> upperCurve;
    std::vector<Vector2> lowerCurve;
    std::vector<Vector2> sparseMiddleFitPoints;
    std::vector<Vector2> sparseUpperFitPoints;
    std::vector<Vector2> sparseLowerFitPoints;
    std::vector<TracePcaSlice> temporalPcaSlices;
};

bool AcceptTracePolarity(bool polarity, int polarityMode);

const char *TracePolarityModeName(int polarityMode);

std::vector<TracePoint> BuildTracePointsFromEvents(const dv::EventStore &events, int polarityMode);

std::vector<TracePoint> BuildTracePointsFromFloatSource(
    const std::vector<cv::Point2f> &pointsSource,
    const std::vector<int64_t> &timestamps,
    const std::vector<bool> *polarities,
    int polarityMode);

struct TracePointSourceResult {
    std::vector<TracePoint> points;
    std::string label;
    Color color = MAROON;
};

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
    int polarityMode);

float Quantile(std::vector<float> values, float q);

struct TraceEdgeEstimate {
    bool valid = false;
    float low = 0.0f;
    float middle = 0.0f;
    float high = 0.0f;
    float width = 0.0f;
    int supportCount = 0;
};

TraceEdgeEstimate EstimateSupportedEdges(
    const std::vector<float> &values,
    std::size_t minSupportCount,
    const TraceSupportEdgeSettings &settings);

float MedianValue(std::vector<float> values);

float MedianAbsoluteDeviation(std::vector<float> values, float median);

bool LocalLinearPrediction(
    const std::vector<float> &xValues,
    const std::vector<float> &yValues,
    std::size_t index,
    int radius,
    float &prediction,
    float &mad);

void FilterCoherentRibbonSamples(
    std::vector<PolynomialSample> &middleSamples,
    std::vector<PolynomialSample> &lowSamples,
    std::vector<PolynomialSample> &highSamples,
    std::vector<float> &localWidths);

void FilterTraceWidthSpikes(std::vector<TraceWidthEstimate> &estimates);

bool SolveWeightedPolynomialFit(
    const std::vector<double> &xs,
    const std::vector<double> &ys,
    const std::vector<double> &weights,
    int degree,
    std::array<double, 3> &coeffs);

// The ball depth changes smoothly along a throw, so the apparent width must be
// a smooth function of time. Individual slice widths carry pixel-level noise
// that converts directly into depth noise; this replaces each width with the
// value of a robust polynomial width(t) fitted across the whole ribbon.
void SmoothTraceWidthProfile(std::vector<TraceWidthEstimate> &estimates);

float Distance3D(const Vector3 &a, const Vector3 &b);

bool IsFinite3D(const Vector3 &point);

int FilterTraceWorldOutliers(std::vector<Vector3> &points, std::vector<float> &times);

Vector2 PointOnRibbon(const TraceRibbonFit &fit, float s, float h);

float Dot2(const Vector2 &a, const Vector2 &b);

float Norm2(const Vector2 &v);

Vector2 NormalizeOr(const Vector2 &v, const Vector2 &fallback);

float TraceFollowLateralWindowPx(float followWindowPx, float circleRadiusPx);

TracePcaResult FitTracePca(
    const std::vector<const TracePoint *> &points,
    const Vector2 &fallbackDirection);

std::vector<TracePcaSlice> BuildTemporalPcaSlices(
    const std::vector<TracePoint> &points,
    float requestedSlicePeriodMs,
    int64_t timeOriginUs,
    const Vector2 &fallbackDirection);

Vector2 CombineTemporalPcaDirection(
    const std::vector<TracePcaSlice> &slices,
    const Vector2 &globalDirection);

bool SolveSmallLinearSystem(
    std::array<std::array<double, 5>, 5> matrix,
    std::array<double, 5> rhs,
    int size,
    std::array<double, 5> &solution);

LocalQuadraticModel MakeLocalQuadraticModel(
    std::vector<PolynomialSample> samples,
    double bandwidth,
    int order);

LocalQuadraticFit FitLocalQuadraticAt(const LocalQuadraticModel &model, float s);

float RibbonCurveH(const LocalQuadraticModel &curve, float s);

float RibbonMiddleH(const TraceRibbonFit &fit, float s);

struct GlobalPolynomialModel {
    bool valid = false;
    int degree = 2;
    double c = 0.0;
    double b = 0.0;
    double a = 0.0;
};

double EvaluateGlobalPolynomial(const GlobalPolynomialModel &model, double x);

bool FitGlobalPolynomial(
    const std::vector<PolynomialSample> &samples,
    const std::vector<char> &mask,
    int degree,
    GlobalPolynomialModel &model);

int FilterRibbonSamplesRansac(
    std::vector<PolynomialSample> &middleSamples,
    std::vector<PolynomialSample> &lowSamples,
    std::vector<PolynomialSample> &highSamples,
    std::vector<float> &localWidths);

Vector2 RibbonCoordinates(const TraceRibbonFit &fit, const Vector2 &point);

Vector2 RibbonCurvePoint(const TraceRibbonFit &fit, const LocalQuadraticModel &curve, float s);

Vector2 RibbonTangent(const TraceRibbonFit &fit, float s);

bool CurveIntersectionOnNormal(
    const TraceRibbonFit &fit,
    const LocalQuadraticModel &curve,
    float referenceS,
    const Vector2 &middle,
    const Vector2 &lineTangent,
    Vector2 &intersection);

Vector3 TraceImagePointToWorldMeters(
    const Vector2 &imagePoint,
    float widthPx,
    const Vector2 &widthDirection,
    const CalibrationData &calibration,
    float ballRadiusMm);

std::vector<Vector2> BuildRibbonCurve(
    const TraceRibbonFit &fit,
    const LocalQuadraticModel &curve,
    int segments,
    std::vector<Vector2> *sparseFitPoints = nullptr);

LocalTraceSection EstimateLocalFlowSection(
    const TraceRibbonFit &fit,
    const std::vector<const TraceProjection *> &timeLocal,
    float fallbackS,
    double sampleTime,
    double timeStep,
    float halfTangentWindow);

std::vector<TraceWidthEstimate> EstimateTraceWidths(const TraceRibbonFit &fit, int sampleCount);

TraceRibbonFit FitTraceRibbon(
    const std::vector<TracePoint> &points,
    float lineBinWidthPx,
    float localWindowPx,
    int localOrder,
    float pcaPeriodMs,
    const TraceSupportEdgeSettings &supportEdge);

int64_t TraceTimeOriginUs(const std::vector<TracePoint> &tracePoints);

struct Trace3DAnalysis {
    bool valid = false;
    Vector3 currentWorld{0.0f, 0.0f, 0.0f};
    std::vector<Vector3> worldPoints;
    std::vector<float> times;
    std::vector<Vector3> groundTruthPoints;
    std::vector<Vector3> groundTruthEstimatePoints;
    std::vector<TraceWidthEstimate> widthEstimates;
    std::vector<float> widthSamples;
    int widthSampleCount = 0;
    int rejectedWorldOutliers = 0;
    std::string poseText = "Trace pose: unavailable";
};

Trace3DAnalysis AnalyzeTrace3D(
    const TraceRibbonFit &fit,
    const CalibrationData &calibration,
    float ballRadiusMm,
    float widthStepPx,
    bool smoothWidthProfile,
    int64_t traceTimeOriginUs,
    const TraceGroundTruthLookup &lookupGroundTruth);

