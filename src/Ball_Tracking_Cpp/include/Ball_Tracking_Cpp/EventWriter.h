#pragma once

#include <cstdint>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <dv-processing/core/core.hpp>

namespace H5 {
class CompType;
class DataSet;
class H5File;
}

#pragma pack(push, 1)
struct PackedEvent {
    int16_t x;
    int16_t y;
    uint8_t polarity;
    int64_t timestamp;
};
#pragma pack(pop)

static_assert(sizeof(PackedEvent) == 13, "PackedEvent must be 13 bytes");

struct EventFileHeader {
    char magic[4] = {'E', 'V', 'T', '1'};
    uint32_t version = 1;
    uint64_t event_count = 0;
    int64_t t_start = 0;
    int64_t t_end = 0;
};

class EventWriter {
public:
    explicit EventWriter(const std::string &path);
    ~EventWriter();

    void Open(const std::string &path);
    void writeStore(const dv::EventStore &store);
    void writeEvent(const dv::Event &event);
    void close();

    uint64_t count() const { return count_; }

private:
    void openBinary(const std::string &path);
    void openH5(const std::string &path);
    void appendH5Events(const PackedEvent *events, std::size_t size);
    void writeH5Header();

    std::ofstream file_;
    std::unique_ptr<H5::H5File> h5_file_;
    std::unique_ptr<H5::DataSet> h5_events_;
    std::unique_ptr<H5::CompType> h5_event_type_;
    EventFileHeader header_;
    uint64_t count_ = 0;
    bool h5_mode_ = false;
};

class EventReader {
public:
    explicit EventReader(const std::string &path);
    ~EventReader();

    void Open(const std::string &path);
    void readStore(dv::EventStore &event, float start_coef, float slice_coef);
    void readWindow(dv::EventStore &event, double start_seconds, double duration_seconds);
    void readWindowEndingAt(dv::EventStore &event, double end_seconds, double duration_seconds);
    void close();

    uint64_t count() const { return count_; }
    bool empty() const { return all_events_.empty(); }
    int64_t startTimestampUs() const { return header_.t_start; }
    int64_t endTimestampUs() const { return header_.t_end; }
    int64_t durationUs() const { return header_.t_end - header_.t_start; }
    double durationSeconds() const { return static_cast<double>(durationUs()) * 1.0e-6; }

private:
    void loadEventsFromOpenFile();
    void loadEventsFromH5File(const std::string &path);

    std::ifstream file_;
    EventFileHeader header_;
    uint64_t count_ = 0;
    std::vector<PackedEvent> all_events_;
};
