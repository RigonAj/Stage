#include "EventWriter.h"

#include <H5Cpp.h>
#include <hdf5.h>

#include <algorithm>
#include <cstddef>
#include <cctype>
#include <cstring>
#include <iostream>
#include <limits>
#include <string_view>

namespace {

bool hasH5Extension(const std::string &path) {
    constexpr std::string_view h5 = ".h5";
    constexpr std::string_view hdf5 = ".hdf5";

    if (path.size() >= h5.size()
        && std::equal(h5.rbegin(), h5.rend(), path.rbegin(), [](char a, char b) {
               return std::tolower(static_cast<unsigned char>(a))
                   == std::tolower(static_cast<unsigned char>(b));
           })) {
        return true;
    }

    return path.size() >= hdf5.size()
        && std::equal(hdf5.rbegin(), hdf5.rend(), path.rbegin(), [](char a, char b) {
               return std::tolower(static_cast<unsigned char>(a))
                   == std::tolower(static_cast<unsigned char>(b));
           });
}

H5::CompType makePackedEventType() {
    H5::CompType type(sizeof(PackedEvent));
    type.insertMember("x", HOFFSET(PackedEvent, x), H5::PredType::NATIVE_INT16);
    type.insertMember("y", HOFFSET(PackedEvent, y), H5::PredType::NATIVE_INT16);
    type.insertMember("polarity", HOFFSET(PackedEvent, polarity), H5::PredType::NATIVE_UINT8);
    type.insertMember("timestamp", HOFFSET(PackedEvent, timestamp), H5::PredType::NATIVE_INT64);
    return type;
}

template <typename T>
void writeScalarAttribute(H5::H5File &file, const char *name, const H5::DataType &type, const T &value) {
    if (H5Aexists(file.getId(), name) > 0) {
        H5Adelete(file.getId(), name);
    }

    H5::DataSpace scalar(H5S_SCALAR);
    H5::Attribute attr = file.createAttribute(name, type, scalar);
    attr.write(type, &value);
}

template <typename T>
bool readScalarAttribute(H5::H5File &file, const char *name, const H5::DataType &type, T &value) {
    if (H5Aexists(file.getId(), name) <= 0) {
        return false;
    }

    H5::Attribute attr = file.openAttribute(name);
    attr.read(type, &value);
    return true;
}

void writeMagicAttribute(H5::H5File &file, const char *magic) {
    if (H5Aexists(file.getId(), "magic") > 0) {
        H5Adelete(file.getId(), "magic");
    }

    H5::DataSpace scalar(H5S_SCALAR);
    H5::StrType magicType(H5::PredType::C_S1, 4);
    H5::Attribute attr = file.createAttribute("magic", magicType, scalar);
    attr.write(magicType, magic);
}

bool readMagicAttribute(H5::H5File &file, char *magic) {
    if (H5Aexists(file.getId(), "magic") <= 0) {
        return false;
    }

    H5::StrType magicType(H5::PredType::C_S1, 4);
    H5::Attribute attr = file.openAttribute("magic");
    attr.read(magicType, magic);
    return true;
}

}

EventWriter::EventWriter(const std::string& path) {
    Open(path);
}

void EventWriter::openBinary(const std::string &path) {
    file_.open(path, std::ios::binary | std::ios::trunc);
    if (!file_) throw std::runtime_error("EventWriter: cannot open " + path);
    file_.write(reinterpret_cast<const char*>(&header_), sizeof(EventFileHeader));
}

void EventWriter::openH5(const std::string &path) {
    H5::Exception::dontPrint();

    h5_file_ = std::make_unique<H5::H5File>(path, H5F_ACC_TRUNC);
    h5_event_type_ = std::make_unique<H5::CompType>(makePackedEventType());

    const hsize_t initialDims[1] = {0};
    const hsize_t maxDims[1] = {H5S_UNLIMITED};
    H5::DataSpace fileSpace(1, initialDims, maxDims);

    H5::DSetCreatPropList props;
    const hsize_t chunkDims[1] = {65536};
    props.setChunk(1, chunkDims);

    h5_events_ = std::make_unique<H5::DataSet>(
        h5_file_->createDataSet("events", *h5_event_type_, fileSpace, props));
    writeH5Header();
}

void EventWriter::Open(const std::string& path) {
    if (file_.is_open()) close();
    h5_events_.reset();
    h5_event_type_.reset();
    h5_file_.reset();

    header_ = EventFileHeader();
    count_ = 0;
    h5_mode_ = hasH5Extension(path);

    if (h5_mode_) {
        openH5(path);
    }
    else {
        openBinary(path);
    }
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

    if (h5_mode_) {
        appendH5Events(&pe, 1);
    }
    else {
        file_.write(reinterpret_cast<const char*>(&pe), sizeof(PackedEvent));
    }
    ++count_;
}

void EventWriter::writeStore(const dv::EventStore& store) {
    if (store.isEmpty()) return;

    std::vector<PackedEvent> buf;
    buf.reserve(store.size());

    int64_t last_ts = (count_ == 0) ? std::numeric_limits<int64_t>::min()
                                    : header_.t_end;

    for (const auto& e : store) {
        if (e.timestamp() < last_ts) continue;
        last_ts = e.timestamp();
        buf.push_back({e.x(), e.y(),
                       static_cast<uint8_t>(e.polarity()),
                       e.timestamp()});
    }

    if (buf.empty()) return;
    if (count_ == 0) header_.t_start = buf.front().timestamp;
    header_.t_end = buf.back().timestamp;

    if (h5_mode_) {
        appendH5Events(buf.data(), buf.size());
    }
    else {
        file_.write(reinterpret_cast<const char*>(buf.data()),
                    static_cast<std::streamsize>(buf.size() * sizeof(PackedEvent)));
    }
    count_ += buf.size();
}

void EventWriter::appendH5Events(const PackedEvent *events, std::size_t size) {
    if (size == 0 || h5_events_ == nullptr || h5_event_type_ == nullptr) return;

    const hsize_t oldSize = count_;
    const hsize_t newDims[1] = {oldSize + static_cast<hsize_t>(size)};
    h5_events_->extend(newDims);

    H5::DataSpace fileSpace = h5_events_->getSpace();
    const hsize_t offset[1] = {oldSize};
    const hsize_t count[1] = {static_cast<hsize_t>(size)};
    fileSpace.selectHyperslab(H5S_SELECT_SET, count, offset);

    H5::DataSpace memSpace(1, count);
    h5_events_->write(events, *h5_event_type_, memSpace, fileSpace);
}

void EventWriter::writeH5Header() {
    if (h5_file_ == nullptr) return;

    header_.event_count = count_;
    writeMagicAttribute(*h5_file_, header_.magic);
    writeScalarAttribute(*h5_file_, "version", H5::PredType::NATIVE_UINT32, header_.version);
    writeScalarAttribute(*h5_file_, "event_count", H5::PredType::NATIVE_UINT64, header_.event_count);
    writeScalarAttribute(*h5_file_, "t_start", H5::PredType::NATIVE_INT64, header_.t_start);
    writeScalarAttribute(*h5_file_, "t_end", H5::PredType::NATIVE_INT64, header_.t_end);
}

void EventWriter::close() {
    header_.event_count = count_;

    if (h5_mode_) {
        writeH5Header();
        h5_events_.reset();
        h5_event_type_.reset();
        h5_file_.reset();
        h5_mode_ = false;
        return;
    }

    if (!file_.is_open()) return;
    file_.seekp(0);
    file_.write(reinterpret_cast<const char*>(&header_), sizeof(EventFileHeader));
    file_.close();
}

EventWriter::~EventWriter() { close(); }

EventReader::EventReader(const std::string& path) {
    Open(path);
}

void EventReader::Open(const std::string& path) {
    if (file_.is_open()) {
        file_.close();
    }

    all_events_.clear();
    header_ = EventFileHeader();
    count_ = 0;

    std::cerr << "Opening file: " << path << "\n";
    if (hasH5Extension(path)) {
        loadEventsFromH5File(path);
        return;
    }

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

    loadEventsFromOpenFile();
}

void EventReader::loadEventsFromH5File(const std::string &path) {
    H5::Exception::dontPrint();

    try {
        H5::H5File h5File(path, H5F_ACC_RDONLY);
        readMagicAttribute(h5File, header_.magic);
        readScalarAttribute(h5File, "version", H5::PredType::NATIVE_UINT32, header_.version);
        readScalarAttribute(h5File, "event_count", H5::PredType::NATIVE_UINT64, header_.event_count);
        readScalarAttribute(h5File, "t_start", H5::PredType::NATIVE_INT64, header_.t_start);
        readScalarAttribute(h5File, "t_end", H5::PredType::NATIVE_INT64, header_.t_end);

        H5::DataSet dataset = h5File.openDataSet("events");
        H5::DataSpace space = dataset.getSpace();
        hsize_t dims[1] = {0};
        space.getSimpleExtentDims(dims);

        all_events_.resize(static_cast<std::size_t>(dims[0]));
        if (!all_events_.empty()) {
            H5::CompType eventType = makePackedEventType();
            dataset.read(all_events_.data(), eventType);
        }

        count_ = static_cast<uint64_t>(all_events_.size());
        header_.event_count = count_;
        if (!all_events_.empty()) {
            header_.t_start = all_events_.front().timestamp;
            header_.t_end = all_events_.back().timestamp;
        }

        std::cerr << "H5 events loaded: " << all_events_.size() << "\n";
    }
    catch (const H5::Exception &e) {
        throw std::runtime_error("EventReader: cannot read H5 file " + path + ": " + e.getDetailMsg());
    }
}

void EventReader::loadEventsFromOpenFile() {
    PackedEvent pe;
    while (file_.read(reinterpret_cast<char*>(&pe), sizeof(PackedEvent))) {
        all_events_.push_back(pe);
    }
    count_ = static_cast<uint64_t>(all_events_.size());
    std::cerr << "Events loaded: " << all_events_.size() << "\n";
}

void EventReader::readStore(dv::EventStore& store, float start_coef, float slice_coef) {

    store = dv::EventStore();

    const int64_t duration    = header_.t_end - header_.t_start;
    if (duration <= 0 || all_events_.empty()) return;

    const double startSeconds = static_cast<double>(start_coef) * static_cast<double>(duration) * 1.0e-6;
    const double durationSeconds = static_cast<double>(slice_coef) * static_cast<double>(duration) * 1.0e-6;
    readWindow(store, startSeconds, durationSeconds);
}

void EventReader::readWindow(dv::EventStore& store, double start_seconds, double duration_seconds) {
    store = dv::EventStore();

    const int64_t duration = header_.t_end - header_.t_start;
    if (duration <= 0 || all_events_.empty()) return;

    if (duration_seconds <= 0.0) {
        duration_seconds = 0.001;
    }

    start_seconds = std::clamp(start_seconds, 0.0, durationSeconds());

    const int64_t slice_start = header_.t_start + static_cast<int64_t>(start_seconds * 1.0e6);
    const int64_t slice_end = std::min(
        header_.t_end + 1,
        slice_start + static_cast<int64_t>(duration_seconds * 1.0e6));

    if (slice_end <= slice_start) return;

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

void EventReader::readWindowEndingAt(dv::EventStore& store, double end_seconds, double duration_seconds) {
    store = dv::EventStore();

    const double fileDurationSeconds = durationSeconds();
    if (fileDurationSeconds <= 0.0 || all_events_.empty()) return;

    if (duration_seconds <= 0.0) {
        duration_seconds = 0.001;
    }

    end_seconds = std::clamp(end_seconds, 0.0, fileDurationSeconds);
    const double start_seconds = std::max(0.0, end_seconds - duration_seconds);

    readWindow(store, start_seconds, end_seconds - start_seconds);
}

void EventReader::close() {
    if (file_.is_open()) file_.close();
}

EventReader::~EventReader() { close(); }
