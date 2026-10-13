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

    

} //namespace SocketCAN

