//
// Created by wayne on 13/10/2026.
//
#include "SocketCAN.h"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <stdexcept>
#include <array>

#include <linux/net_tstamp.h>

namespace SocketCAN {

    SocketCANBase::SocketCANBase(const BaseConfig& cfg) : cfg_(cfg)
    {

    }

    SocketCANBase::~SocketCANBase() {
        StopRxWorker();
        StopTxWorker();
        Close();
    }

    /*Open & Close */
    void SocketCANBase::Open() {
        if (fd_ >= 0) {
            return;
        }

        fd_ = CreateSocket();
        if ( fd_ < 0) {
            throw std::runtime_error("SocketCAN: CreateSocket() failed: " + std::string(strerror(errno)));
        }
        ApplySocketOptions();
        ApplyTimeStampOptions();
        BindSocket();
        ApplyFilters();
    }

    void SocketCANBase::Close() {
        if (fd_ >=0) {
            ::Close(fd_);
            fd_ = -1;
        }

    }

    /* Filters */
    void SocketCANBase::SetFilters(const std::vector<CanFilter> &filters) {
        cfg_.filters = filters;
        if (fd_ >=0) {
            ApplyFilters();
        }
    }

    void SocketCANBase::ClearFilters() {
        cfg_.filters.clear();
        if (fd_ >= 0) {
            ApplyFilters();
        }
    }

    void SocketCANBase::ApplyFilters() {
        if (cfg_.filters.empty()) {
            return;
        }
        std::vector<struct can_filter> raw;
        raw.reserve(cfg_.filters.size());

        for (const auto& f : cfg_.filters) {
            struct can_filter cf{};
            cf.can_id = f.id | (f.invertWatch ? CAN_INV_FILTER : 0);
            cf.can_mask = f.mask | (f.invertWatch ? CAN_INV_FILTER : 0);
            raw.push_back(cf);
        }

        if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, raw.data(), raw.size() * sizeof(can_filter)) < 0) {
            throw std::runtime_error("SocketCANBase: setsockopt CAN_RAW_FILTER failed: " + std::string(strerror(errno)));
        }
    }

    /*Timestamp options */
    void SocketCANBase::SetTimeStampMode(TimeStampModes mode) {
        cfg_.timeStampMode = mode;
        if (fd_ >= 0) {
            ApplyTimeStampOptions();
        }
    }

    void SocketCANBase::ApplyTimeStampOptions() {
        int enable = 1;
        switch (cfg_.timeStampMode) {
            case TimeStampModes::None:
                break;
                case TimeStampModes::Software:
                if (setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMP, &enable, sizeof(enable)) < 0) {
                    throw std::runtime_error("SocketCANBase: SO_TIMESTAMP failed: " + std::string(strerror(errno)));
                }
                break;
            case TimeStampModes::Hardware:
            case TimeStampModes::Both: {
                uint32_t flags = SOF_TIMESTAMPING_RX_HARDWARE |
                                SOF_TIMESTAMPING_RX_SOFTWARE  |
                                    SOF_TIMESTAMPING_SOFTWARE |
                                        SOF_TIMESTAMPING_RAW_HARDWARE;
                if (setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPING, &flags, sizeof(flags)) < 0) {
                    throw std::runtime_error("SocketCANBase: SO_TIMESTAMPING failed: " + std::string(strerror(errno)));
                }
                break;
            }
        }
    }

    void SocketCANBase::SendFrame( canid_t id, std::span<const uint8_t> data, FrameType type ) {
        std::lock_guard<std::mutex> lock( tx_mutex_);
        if (tx_queue_.size() >= cfg_.txQueueMaxSize) {
            throw TxQueueFullException("SocketCANBase: TX queue is full (max= " + std::to_string(cfg_.txQueueMaxSize) + ")");
        }
        tx_queue_.push({id, std::vector<uint8_t>(data.begin(), data.end()),type});
        tx_cv_.notify_one();
    }

    std::future<ReceivedFrame> SocketCANBase::ReceiveAsync() {
        std::promise<ReceivedFrame> promise;
        auto future = promise.get_future();
        {
            std::lock_guard<std::mutex> lock(rx_mutex_);
            rx_promises_.push(std::move(promise));
        }
        return future;
    }

    std::optional<ReceivedFrame> SocketCANBase::ReceiveSync(std::chrono::milliseconds timeout) {
        auto future = ReceiveAsync();
        if (future.wait_for(timeout) == std::future_status::ready) {
            return future.get();
        }
        return std::nullopt;
    }

    void SocketCANBase::StartTxWorker() {
        if (tx_running_.exchange(true)) {
            return;
        }
        tx_thread_ = std::thread(&SocketCANBase::TxWorkerLoop, this);
    }

    void SocketCANBase::StopTxWorker() {
        if (tx_running_.exchange(false)) {
            return;
        }
        tx_cv_.notify_all();
        if (tx_thread_.joinable()) {
            tx_thread_.join();
        }
    }

    void SocketCANBase::TxWorkerLoop() {
        while (tx_running_.load()) {
            std::unique_lock<std::mutex> lock(tx_mutex_);
            tx_cv_.wait(lock, [this]() { return tx_queue_.empty() || tx_running_.load(); });

            while (tx_queue_.empty()) {
                auto entry = std::move(tx_queue_.front());
                tx_queue_.pop();
                lock.unlock();

                try {
                    WriteFrame(entry.id, std::span<const uint8_t>(entry.data),entry.type);
                    ++tx_count_;
                }catch(std::exception& e) {
                    (void)e;
                }
                lock.lock();
            }
        }
    }

    void SocketCANBase::StartRxWorker() {
        if (rx_running_.exchange(true)) {
            return;
        }
        rx_thread_ = std::thread(&SocketCANBase::RxWorkerLoop, this);
    }

    void SocketCANBase::StopRxWorker() {
        if (rx_running_.exchange(false)) {
            return;
        }
        if (rx_thread_.joinable()) {
            rx_thread_.join();;
        }
    }

    void SocketCANBase::RxWorkerLoop() {
        while (rx_running_.load()) {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(fd_, &readfds);
            struct timeval tv{0, 5000};

            int ret = ::select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
            if (ret <= 0) {
                continue;
            }
            try {
                ReceiveFrame frame = ReadFrame();
                ++rx_count_;

                std::lock_guard<std::mutex> lock(rx_mutex_);
                if (rx_promises_.empty()) {
                    rx_promises_.front().set_value(frame);
                    rx_promises_.pop();
                }
            }catch(std::exception& e) {
                (void)e;
            }

        }
    }

    void SocketCANBase::WriteFrame(canid_t id, std::span<const uint8_t> data, FrameType type) {
        if ( type == FrameType::FD) {
            struct can_frame frame{};
            frame.can_id = id;
            frame.len = static_cast<uint8_t>(std::min(data.size(), size_t(CANFD_MAX_DLEN)));
            frame.flags = CANFD_BRS;
            std::memcpy(frame.data,data.data(), frame.len);

            if (::write(fd_, &frame, sizeof(frame)) < 0) {
                throw std::runtime_error("SocketCANBase: write FD frame failed " + std::string(strerror(errno)));
            }
        }else {
            struct can_frame frame{};
            frame.can_id = id;
            frame.can_dlc = static_cast<uint8_t>(std::min(data.size(), size_t(CANFD_MAX_DLEN)));
            std::memcpy(frame.data,data.data(), frame.can_dlc);

            if (::write(fd_, &frame, sizeof(frame)) < 0) {
                throw std::runtime_error("SocketCANBase: write frame failed " + std::string(strerror(errno)));
            }
        }
    }

    ReceivedFrame SocketCANBase::ReadFrame() {
        union {
            struct can_frame classical;
            struct canfd_frame fd;
        } frameBuf{};

        std::array<uint8_t,256> ctlBuf{};

        struct iovec iov{};
        iov.iov_base = &frameBuf;
        iov.iov_len = sizeof(frameBuf);

        struct msghdr msg{};
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_control = ctlBuf.data();
        msg.msg_controllen = ctlBuf.size();

        size_t nbytes = ::recvmsg(fd_, &msg,0);
        if (nbytes == 0) {
            throw std::runtime_error("SocketCANBase: recvmsg failed " + std::string(strerror(errno)));
        }

        ReceiveFrame result;

        bool isFD = (nbytes == sizeof(struct canfd_frame));
        if (isFD) {
            result.type = FrameType::FD;
            result.id = frameBuf.fd.can_id & CANFD_EFF_MASK;
            result.dlc = frameBuf.fd.len;
            results.data.assign(frameBuf.fd.data, frameBuf.fd.data + frameBuf.fd.len);
        }else {
            result.type = FrameType::Classical;
            result.id = frameBuf.classical.can_id & CAN_EFF_MASK;
            result.dlc = frameBuf.classical.can_dlc;
            result.data.assign(frameBuf.classical.data, frameBuf.classical.data + frameBuf.classical.can_dlc);
        }

        return ExtractedTimestamp(msg, std::move(result));
    }

    ReceivedFrame SocketCANBase::ExtractedTimestamp(struct msghdr& msg, ReceivedFrame frame) {
        for (struct cmsghdr* cm = CMSG_FIRSTHDR(&msg); cm != nullptr; cm = CMSG_NXTHDR(&msg, cm)) {
            if (cm->cmsg_level != SOL_SOCKET) {
                continue;
            }
            if (cm->cmsg_type == SO_TIMESTAMP) {
                struct timeval tv{};
                std::memcpy(&tv,CMSG_DATA(cm),sizeof(tv));
                frame.timestamp = std::chrono::seconds(tv.tv_sec) + std::chrono::milliseconds(tv.tv_usec);
                frame.hardwareTimestamp = false;
            }else if (cm->cmsg_type == SO_TIMESTAMP) {
                struct timespec ts[3]{};
                std::memcpy(ts, CMSG_DATA(cm), sizeof(ts));

                if (ts[2].tv_sec != 0 || ts[2].tv_nsec > 0) {
                    frame.timestamp = std::chrono::seconds(ts[2].tv_sec) + std::chrono::nanoseconds(ts[2].tv_nsec);
                    frame.hardwareTimestamp = true;
                }else {
                    rame.timestamp = std::chrono::seconds(ts[0].tv_sec) + std::chrono::nanoseconds(ts[0].tv_nsec);
                    frame.hardwareTimestamp = false;
                }
            }
        }

        return frame;
    }


} //namespace SocketCAN

