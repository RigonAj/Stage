#pragma once

#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <dv-processing/core/core.hpp>

// On-disk layout: 13 bytes per event, no padding
#pragma pack(push, 1)
struct PackedEvent {
    int16_t x;
    int16_t y;
    uint8_t polarity;
    int64_t timestamp;   // microseconds, same as dv::Event
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
    std::ofstream file_;
    EventFileHeader header_;
    uint64_t count_ = 0;
};

class EventReader {
public:
    explicit EventReader(const std::string &path);
    ~EventReader();

    void Open(const std::string &path) {
        if (file_.is_open()) {
            file_.close();
        }

        file_.open(path, std::ios::binary);
        if (!file_) {
            throw std::runtime_error("EventReader: cannot open " + path);
        }

        file_.read(reinterpret_cast<char*>(&header_), sizeof(EventFileHeader));
    }

    void readStore(dv::EventStore &event, float start_coef, float slice_coef);
    void close();

    uint64_t count() const { return count_; }

private:
    std::ifstream file_;
    EventFileHeader header_;
    uint64_t count_ = 0;
    std::vector<PackedEvent> all_events_;
};
