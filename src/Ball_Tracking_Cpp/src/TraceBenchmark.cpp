#include "TraceBenchmark.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace {

struct Point2 {
    double x = 0.0;
    double y = 0.0;
};

std::string ReadTextFile(const std::string &path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Cannot open file: " + path);
    }
    std::ostringstream out;
    out << file.rdbuf();
    return out.str();
}

double JsonNumber(const std::string &text, const std::string &key, double fallback) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)");
    std::smatch match;
    if (std::regex_search(text, match, pattern)) {
        return std::stod(match[1].str());
    }
    return fallback;
}

std::string JsonString(const std::string &text, const std::string &key, const std::string &fallback) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
    std::smatch match;
    if (std::regex_search(text, match, pattern)) {
        return match[1].str();
    }
    return fallback;
}

std::string Trim(std::string value) {
    auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
    value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
    return value;
}

std::string Unquote(std::string value) {
    value = Trim(value);
    if (value.size() >= 2
        && ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\''))) {
        return value.substr(1, value.size() - 2);
    }
    return value;
}

std::string Lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

void ParseYamlScalarLine(const std::string &line, std::string &key, std::string &value) {
    const std::size_t comment = line.find('#');
    const std::string clean = Trim(line.substr(0, comment));
    const std::size_t colon = clean.find(':');
    if (colon == std::string::npos) {
        key.clear();
        value.clear();
        return;
    }
    key = Trim(clean.substr(0, colon));
    value = Trim(clean.substr(colon + 1));
}

double ToDouble(const std::string &value, double fallback) {
    try {
        return std::stod(Unquote(value));
    }
    catch (...) {
        return fallback;
    }
}

int ToInt(const std::string &value, int fallback) {
    try {
        return std::stoi(Unquote(value));
    }
    catch (...) {
        return fallback;
    }
}

bool ToBool(const std::string &value, bool fallback) {
    const std::string lower = Lower(Unquote(value));
    if (lower == "true" || lower == "yes" || lower == "on" || lower == "1") {
        return true;
    }
    if (lower == "false" || lower == "no" || lower == "off" || lower == "0") {
        return false;
    }
    return fallback;
}

std::vector<std::string> SplitCsvLine(const std::string &line) {
    std::vector<std::string> fields;
    std::string field;
    std::stringstream stream(line);
    while (std::getline(stream, field, ',')) {
        fields.push_back(Trim(field));
    }
    return fields;
}

int CsvColumnIndex(const std::vector<std::string> &header, const std::string &name) {
    for (std::size_t i = 0; i < header.size(); ++i) {
        if (header[i] == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

double Quantile(std::vector<double> values, double q) {
    if (values.empty()) {
        return 0.0;
    }
    q = std::clamp(q, 0.0, 1.0);
    const double index = q * static_cast<double>(values.size() - 1);
    const auto lo = static_cast<std::size_t>(std::floor(index));
    const auto hi = static_cast<std::size_t>(std::ceil(index));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(lo), values.end());
    const double loValue = values[lo];
    if (hi == lo) {
        return loValue;
    }
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(hi), values.end());
    const double hiValue = values[hi];
    return loValue + (hiValue - loValue) * (index - static_cast<double>(lo));
}

std::string CsvEscape(const std::string &value) {
    if (value.find_first_of(",\"\n") == std::string::npos) {
        return value;
    }
    std::string escaped = "\"";
    for (const char c : value) {
        if (c == '"') {
            escaped += "\"\"";
        }
        else {
            escaped += c;
        }
    }
    escaped += '"';
    return escaped;
}

std::string JsonEscape(const std::string &value) {
    std::string escaped;
    for (const char c : value) {
        if (c == '\\' || c == '"') {
            escaped.push_back('\\');
        }
        escaped.push_back(c);
    }
    return escaped;
}

TraceDetection EstimateTraceAtTimestamp(
    const dv::EventStore &events,
    int64_t timestampUs,
    int64_t windowStartUs,
    int64_t windowEndUs,
    const TraceBenchmarkIntrinsics &intrinsics,
    const TraceBenchmarkMetadata &metadata,
    const TraceBenchmarkSettings &settings) {

    using clock = std::chrono::steady_clock;
    const auto start = clock::now();
    TraceDetection detection;
    detection.timestampUs = timestampUs;
    detection.windowStartUs = windowStartUs;
    detection.windowEndUs = windowEndUs;
    detection.edgeMode = Lower(settings.edgeMode);
    detection.polarityMode = Lower(settings.polarityMode);

    std::vector<Point2> points;
    points.reserve(events.size());

    const std::string polarityMode = Lower(settings.polarityMode);
    for (const auto &event : events) {
        const bool polarity = event.polarity();
        if (polarityMode == "negative" && polarity) {
            continue;
        }
        if (polarityMode == "positive" && !polarity) {
            continue;
        }
        const double x = static_cast<double>(event.x());
        const double y = static_cast<double>(event.y());
        if (x >= 0.0 && x < intrinsics.width && y >= 0.0 && y < intrinsics.height) {
            points.push_back({x, y});
        }
    }

    if (settings.maxEvents > 0 && static_cast<int>(points.size()) > settings.maxEvents) {
        std::vector<Point2> sampled;
        sampled.reserve(static_cast<std::size_t>(settings.maxEvents));
        const double step = static_cast<double>(points.size()) / static_cast<double>(settings.maxEvents);
        for (int i = 0; i < settings.maxEvents; ++i) {
            sampled.push_back(points[static_cast<std::size_t>(std::floor(static_cast<double>(i) * step))]);
        }
        points = std::move(sampled);
    }

    detection.numEvents = static_cast<int>(points.size());
    if (static_cast<int>(points.size()) < settings.minEvents) {
        detection.failureReason = "insufficient_events";
        detection.runtimeMs = std::chrono::duration<double, std::milli>(clock::now() - start).count();
        return detection;
    }

    double meanX = 0.0;
    double meanY = 0.0;
    for (const Point2 &point : points) {
        meanX += point.x;
        meanY += point.y;
    }
    meanX /= static_cast<double>(points.size());
    meanY /= static_cast<double>(points.size());

    double cxx = 0.0;
    double cxy = 0.0;
    double cyy = 0.0;
    for (const Point2 &point : points) {
        const double dx = point.x - meanX;
        const double dy = point.y - meanY;
        cxx += dx * dx;
        cxy += dx * dy;
        cyy += dy * dy;
    }
    cxx /= static_cast<double>(points.size());
    cxy /= static_cast<double>(points.size());
    cyy /= static_cast<double>(points.size());

    const double angle = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
    const Point2 tangent{std::cos(angle), std::sin(angle)};
    const Point2 normal{-tangent.y, tangent.x};

    std::vector<double> sValues;
    std::vector<double> hValues;
    sValues.reserve(points.size());
    hValues.reserve(points.size());
    for (const Point2 &point : points) {
        const double dx = point.x - meanX;
        const double dy = point.y - meanY;
        sValues.push_back(dx * tangent.x + dy * tangent.y);
        hValues.push_back(dx * normal.x + dy * normal.y);
    }

    const std::string edgeMode = Lower(settings.edgeMode);
    const double trim = edgeMode == "support" ? 0.02 : 0.05;
    const double low = Quantile(hValues, trim);
    const double high = Quantile(hValues, 1.0 - trim);
    const double width = high - low;
    if (!std::isfinite(width) || width <= 1.0) {
        detection.failureReason = "invalid_trace_width";
        detection.runtimeMs = std::chrono::duration<double, std::milli>(clock::now() - start).count();
        return detection;
    }

    const double middleH = 0.5 * (low + high);
    const double middleS = Quantile(sValues, 0.5);
    const double u = meanX + tangent.x * middleS + normal.x * middleH;
    const double v = meanY + tangent.y * middleS + normal.y * middleH;

    const double fEff = std::sqrt(
        (intrinsics.fx * normal.x) * (intrinsics.fx * normal.x)
        + (intrinsics.fy * normal.y) * (intrinsics.fy * normal.y));
    const double depthM = fEff * (2.0 * metadata.ballRadiusM) / width;
    if (!std::isfinite(depthM) || depthM <= 0.0) {
        detection.failureReason = "invalid_depth";
        detection.runtimeMs = std::chrono::duration<double, std::milli>(clock::now() - start).count();
        return detection;
    }

    int supportCount = 0;
    double residual2 = 0.0;
    for (const double h : hValues) {
        if (h >= low && h <= high) {
            ++supportCount;
            const double centered = h - middleH;
            residual2 += centered * centered;
        }
    }

    detection.detected = true;
    detection.centerUPx = u;
    detection.centerVPx = v;
    detection.traceWidthPx = width;
    detection.traceAngleDeg = angle * 180.0 / M_PI;
    detection.depthEstM = depthM;
    detection.xEstM = ((u - intrinsics.cx) / intrinsics.fx) * depthM;
    detection.yEstM = ((v - intrinsics.cy) / intrinsics.fy) * depthM;
    detection.zEstM = depthM;
    detection.traceSupportCount = supportCount;
    detection.lineFitErrorPx = supportCount > 0 ? std::sqrt(residual2 / static_cast<double>(supportCount)) : 0.0;
    detection.failureReason.clear();
    detection.runtimeMs = std::chrono::duration<double, std::milli>(clock::now() - start).count();
    return detection;
}

}  // namespace

TraceBenchmarkIntrinsics LoadTraceIntrinsicsJson(const std::string &path) {
    const std::string text = ReadTextFile(path);
    TraceBenchmarkIntrinsics intrinsics;
    intrinsics.width = static_cast<int>(std::lround(JsonNumber(text, "width", intrinsics.width)));
    intrinsics.height = static_cast<int>(std::lround(JsonNumber(text, "height", intrinsics.height)));
    intrinsics.fx = JsonNumber(text, "fx", intrinsics.fx);
    intrinsics.fy = JsonNumber(text, "fy", intrinsics.fy);
    intrinsics.cx = JsonNumber(text, "cx", intrinsics.cx);
    intrinsics.cy = JsonNumber(text, "cy", intrinsics.cy);
    intrinsics.sourcePath = path;
    if (intrinsics.width <= 0 || intrinsics.height <= 0 || intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0) {
        throw std::runtime_error("Invalid intrinsics JSON: " + path);
    }
    return intrinsics;
}

TraceBenchmarkMetadata LoadTraceMetadataJson(const std::string &path) {
    const std::string text = ReadTextFile(path);
    TraceBenchmarkMetadata metadata;
    metadata.ballRadiusM = JsonNumber(text, "radius_m", metadata.ballRadiusM);
    metadata.sequenceName = JsonString(text, "sequence_name", "");
    metadata.sourcePath = path;
    if (metadata.ballRadiusM <= 0.0) {
        metadata.ballRadiusM = 0.02;
    }
    return metadata;
}

TraceGroundTruthSummary LoadTraceGroundTruthCsv(const std::string &path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Cannot open ground truth CSV: " + path);
    }
    TraceGroundTruthSummary summary;
    summary.sourcePath = path;
    std::string line;
    if (!std::getline(file, line)) {
        return summary;
    }
    const std::vector<std::string> header = SplitCsvLine(line);
    const int timeColumn = CsvColumnIndex(header, "timestamp_s");
    const int visibleColumn = CsvColumnIndex(header, "visible");
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        const std::vector<std::string> fields = SplitCsvLine(line);
        if (timeColumn >= 0 && static_cast<int>(fields.size()) > timeColumn) {
            const double timestamp = std::stod(fields[static_cast<std::size_t>(timeColumn)]);
            if (summary.rowCount == 0) {
                summary.firstTimestampS = timestamp;
            }
            summary.lastTimestampS = timestamp;
        }
        if (visibleColumn >= 0 && static_cast<int>(fields.size()) > visibleColumn) {
            summary.visibleCount += fields[static_cast<std::size_t>(visibleColumn)] == "1" ? 1 : 0;
        }
        ++summary.rowCount;
    }
    return summary;
}

TraceBenchmarkSettings LoadTraceSettingsYaml(const std::string &path, TraceBenchmarkSettings defaults) {
    if (path.empty()) {
        return defaults;
    }
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Cannot open tracker config YAML: " + path);
    }
    TraceBenchmarkSettings settings = defaults;
    std::string line;
    while (std::getline(file, line)) {
        std::string key;
        std::string value;
        ParseYamlScalarLine(line, key, value);
        if (key.empty() || value.empty()) {
            continue;
        }
        if (key == "trace_memory_ms") {
            settings.traceMemoryMs = ToDouble(value, settings.traceMemoryMs);
        }
        else if (key == "output_period_ms") {
            settings.outputPeriodMs = ToDouble(value, settings.outputPeriodMs);
        }
        else if (key == "min_events_per_cluster") {
            settings.minEvents = ToInt(value, settings.minEvents);
        }
        else if (key == "max_events") {
            settings.maxEvents = ToInt(value, settings.maxEvents);
        }
        else if (key == "edge_mode") {
            settings.edgeMode = Lower(Unquote(value));
        }
        else if (key == "polarity_mode") {
            settings.polarityMode = Lower(Unquote(value));
        }
        else if (key == "fit_input") {
            settings.fitInput = Lower(Unquote(value));
        }
        else if (key == "line_order") {
            settings.lineOrder = Lower(Unquote(value));
        }
        else if (key == "radius_gate") {
            settings.radiusGate = ToBool(value, settings.radiusGate);
        }
    }
    settings.outputPeriodMs = std::max(0.1, settings.outputPeriodMs);
    settings.traceMemoryMs = std::max(1.0, settings.traceMemoryMs);
    settings.minEvents = std::max(1, settings.minEvents);
    settings.maxEvents = std::max(settings.minEvents, settings.maxEvents);
    return settings;
}

std::vector<TraceDetection> RunTraceBenchmark(
    EventReader &reader,
    const TraceBenchmarkIntrinsics &intrinsics,
    const TraceBenchmarkMetadata &metadata,
    const TraceBenchmarkSettings &settings,
    TraceBenchmarkRuntime &runtime) {

    using clock = std::chrono::steady_clock;
    const auto start = clock::now();
    std::vector<TraceDetection> detections;
    if (reader.empty() || reader.durationUs() <= 0) {
        runtime.totalRuntimeMs = 0.0;
        return detections;
    }

    const int64_t firstUs = reader.startTimestampUs();
    const int64_t lastUs = reader.endTimestampUs();
    const int64_t periodUs = std::max<int64_t>(1, static_cast<int64_t>(std::llround(settings.outputPeriodMs * 1000.0)));
    const double memorySeconds = settings.traceMemoryMs * 1.0e-3;

    detections.reserve(static_cast<std::size_t>((lastUs - firstUs) / periodUs + 1));
    for (int64_t timestampUs = firstUs; timestampUs <= lastUs; timestampUs += periodUs) {
        dv::EventStore window;
        const double endSeconds = static_cast<double>(timestampUs - firstUs) * 1.0e-6;
        reader.readWindowEndingAt(window, endSeconds, memorySeconds);
        const int64_t memoryUs = static_cast<int64_t>(std::llround(memorySeconds * 1.0e6));
        detections.push_back(EstimateTraceAtTimestamp(
            window,
            timestampUs,
            std::max(firstUs, timestampUs - memoryUs),
            timestampUs,
            intrinsics,
            metadata,
            settings));
    }

    runtime.outputRows = static_cast<int>(detections.size());
    runtime.detectedRows = static_cast<int>(std::count_if(detections.begin(), detections.end(), [](const TraceDetection &d) {
        return d.detected;
    }));
    runtime.totalRuntimeMs = std::chrono::duration<double, std::milli>(clock::now() - start).count();
    return detections;
}

void WriteTraceDetectionsCsv(const std::string &path, const std::vector<TraceDetection> &detections) {
    std::ofstream file(path);
    if (!file) {
        throw std::runtime_error("Cannot write detections CSV: " + path);
    }
    file << "timestamp_s,timestamp_us,detected,center_u_px,center_v_px,trace_width_px,trace_angle_deg,"
            "depth_est_m,x_est_m,y_est_m,z_est_m,num_events,runtime_ms,edge_mode,polarity_mode,"
            "failure_reason,line_fit_error_px,trace_support_count,outlier_rejected,window_start_us,window_end_us\n";
    file << std::fixed << std::setprecision(9);
    for (const TraceDetection &detection : detections) {
        file << static_cast<double>(detection.timestampUs) * 1.0e-6 << ','
             << detection.timestampUs << ','
             << (detection.detected ? 1 : 0) << ',';
        if (detection.detected) {
            file << detection.centerUPx << ','
                 << detection.centerVPx << ','
                 << detection.traceWidthPx << ','
                 << detection.traceAngleDeg << ','
                 << detection.depthEstM << ','
                 << detection.xEstM << ','
                 << detection.yEstM << ','
                 << detection.zEstM << ',';
        }
        else {
            file << ",,,,,,,,";
        }
        file << detection.numEvents << ','
             << detection.runtimeMs << ','
             << CsvEscape(detection.edgeMode) << ','
             << CsvEscape(detection.polarityMode) << ','
             << CsvEscape(detection.failureReason) << ',';
        if (detection.detected) {
            file << detection.lineFitErrorPx << ','
                 << detection.traceSupportCount << ','
                 << (detection.outlierRejected ? 1 : 0) << ',';
        }
        else {
            file << ",,,";
        }
        file << detection.windowStartUs << ','
             << detection.windowEndUs << '\n';
    }
}

void WriteTraceRuntimeJson(const std::string &path, const TraceBenchmarkRuntime &runtime) {
    std::ofstream file(path);
    if (!file) {
        throw std::runtime_error("Cannot write runtime JSON: " + path);
    }
    file << std::fixed << std::setprecision(6);
    file << "{\n";
    file << "  \"events_h5\": \"" << JsonEscape(runtime.eventsPath) << "\",\n";
    file << "  \"camera\": \"" << JsonEscape(runtime.cameraPath) << "\",\n";
    file << "  \"ground_truth\": \"" << JsonEscape(runtime.groundTruthPath) << "\",\n";
    file << "  \"metadata\": \"" << JsonEscape(runtime.metadataPath) << "\",\n";
    file << "  \"input_event_count\": " << runtime.inputEventCount << ",\n";
    file << "  \"first_timestamp_us\": " << runtime.firstTimestampUs << ",\n";
    file << "  \"last_timestamp_us\": " << runtime.lastTimestampUs << ",\n";
    file << "  \"output_rows\": " << runtime.outputRows << ",\n";
    file << "  \"detected_rows\": " << runtime.detectedRows << ",\n";
    file << "  \"total_runtime_ms\": " << runtime.totalRuntimeMs << ",\n";
    file << "  \"trace_settings\": {\n";
    file << "    \"edge_mode\": \"" << JsonEscape(runtime.settings.edgeMode) << "\",\n";
    file << "    \"polarity_mode\": \"" << JsonEscape(runtime.settings.polarityMode) << "\",\n";
    file << "    \"radius_gate\": " << (runtime.settings.radiusGate ? "true" : "false") << ",\n";
    file << "    \"fit_input\": \"" << JsonEscape(runtime.settings.fitInput) << "\",\n";
    file << "    \"line_order\": \"" << JsonEscape(runtime.settings.lineOrder) << "\",\n";
    file << "    \"trace_memory_ms\": " << runtime.settings.traceMemoryMs << ",\n";
    file << "    \"output_period_ms\": " << runtime.settings.outputPeriodMs << "\n";
    file << "  },\n";
    file << "  \"metadata_summary\": {\n";
    file << "    \"sequence_name\": \"" << JsonEscape(runtime.metadata.sequenceName) << "\",\n";
    file << "    \"ball_radius_m\": " << runtime.metadata.ballRadiusM << "\n";
    file << "  },\n";
    file << "  \"ground_truth_summary\": {\n";
    file << "    \"rows\": " << runtime.groundTruth.rowCount << ",\n";
    file << "    \"visible_rows\": " << runtime.groundTruth.visibleCount << ",\n";
    file << "    \"first_timestamp_s\": " << runtime.groundTruth.firstTimestampS << ",\n";
    file << "    \"last_timestamp_s\": " << runtime.groundTruth.lastTimestampS << "\n";
    file << "  }\n";
    file << "}\n";
}
