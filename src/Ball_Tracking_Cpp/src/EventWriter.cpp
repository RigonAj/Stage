#include "EventWriter.h"

#include <algorithm>
#include <iostream>
#include <limits>

EventWriter::EventWriter(const std::string& path) {
    file_.open(path, std::ios::binary | std::ios::trunc);
    if (!file_) throw std::runtime_error("EventWriter: cannot open " + path);
    file_.write(reinterpret_cast<const char*>(&header_), sizeof(EventFileHeader));
}

void EventWriter::Open(const std::string& path) {
    if (file_.is_open()) close();
    file_.open(path, std::ios::binary | std::ios::trunc);
    if (!file_) throw std::runtime_error("EventWriter: cannot open " + path);
    header_ = EventFileHeader();
    count_ = 0;
    file_.write(reinterpret_cast<const char*>(&header_), sizeof(EventFileHeader));
}
void EventWriter::writeEvent(const dv::Event& e) {
    PackedEvent pe{
        e.x(),
        e.y(),
        static_cast<uint8_t>(e.polarity()),
        e.timestamp()
    };

    if (count_ == 0) header_.t_start = e.timestamp();
    header_.t_end = e.timestamp();

    file_.write(reinterpret_cast<const char*>(&pe), sizeof(PackedEvent));
    ++count_;
}

void EventWriter::writeStore(const dv::EventStore& store) {
    if (store.isEmpty()) return;

    std::vector<PackedEvent> buf;
    buf.reserve(store.size());

    int64_t last_ts = (count_ == 0) ? std::numeric_limits<int64_t>::min()
                                    : header_.t_end;

    for (const auto& e : store) {
        if (e.timestamp() < last_ts) continue;  // drop out-of-order
        last_ts = e.timestamp();
        buf.push_back({e.x(), e.y(),
                       static_cast<uint8_t>(e.polarity()),
                       e.timestamp()});
    }

    if (buf.empty()) return;
    if (count_ == 0) header_.t_start = buf.front().timestamp;
    header_.t_end = buf.back().timestamp;
    count_ += buf.size();

    file_.write(reinterpret_cast<const char*>(buf.data()),
                static_cast<std::streamsize>(buf.size() * sizeof(PackedEvent)));
}

void EventWriter::close() {
    if (!file_.is_open()) return;
    header_.event_count = count_;
    file_.seekp(0);
    file_.write(reinterpret_cast<const char*>(&header_), sizeof(EventFileHeader));
    file_.close();
}

EventWriter::~EventWriter() { close(); }

EventReader::EventReader(const std::string& path) {
    std::cerr << "Opening file: " << path << "\n";
    file_.open(path, std::ios::binary);
    if (!file_) {
        std::cerr << "FAILED to open file\n";
        throw std::runtime_error("EventReader: cannot open " + path);
    }
    std::cerr << "File opened OK\n";

    file_.read(reinterpret_cast<char*>(&header_), sizeof(EventFileHeader));
    std::cerr << "Header read, event_count=" << header_.event_count 
              << " magic=" << header_.magic[0] << header_.magic[1] 
              << header_.magic[2] << header_.magic[3] << "\n";

    PackedEvent pe;
    while (file_.read(reinterpret_cast<char*>(&pe), sizeof(PackedEvent))) {
        all_events_.push_back(pe);
    }
    std::cerr << "Events loaded: " << all_events_.size() << "\n";
}

void EventReader::readStore(dv::EventStore& store, float start_coef, float slice_coef) {

    if (all_events_.empty() ) return;

    const int64_t duration    = header_.t_end - header_.t_start;
    const int64_t slice_start = header_.t_start + static_cast<int64_t>(start_coef * duration );
    const int64_t slice_end   = slice_start + static_cast<int64_t>(slice_coef * duration);


    if (duration <= 0) return;
    if (all_events_.empty()) {
            return;
    }
    // Binary search for the window — O(log N) instead of O(N)
    auto lo = std::lower_bound(all_events_.begin(), all_events_.end(), slice_start,
                   [](const PackedEvent& e, int64_t t) { return e.timestamp < t; });
    auto hi = std::lower_bound(lo, all_events_.end(), slice_end,
                   [](const PackedEvent& e, int64_t t) { return e.timestamp < t; });

    store = dv::EventStore();
    for (auto it = lo; it != hi; ++it) {
        store.emplace_back(it->timestamp,
                           static_cast<int16_t>(it->x),
                           static_cast<int16_t>(it->y),
                           static_cast<bool>(it->polarity));
    }
}

void EventReader::close() {
    if (file_.is_open()) file_.close();
}

EventReader::~EventReader() { close(); }