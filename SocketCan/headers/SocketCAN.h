//
// Created by wayne on 13/10/2026.
//

#ifndef ETOOLS_SOCKETCAN_H
#define ETOOLS_SOCKETCAN_H

#include <string>
#include <vector>
#include <chrono>
#include <span>
#include <future>
#include <thread>
#include <atomic>
#include <stdexcept>
#include <optional>

#include <cstdint>
#include <future>
#include <queue>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/bcm.h>
#include <linux/can/isotp.h>
#include <linux/can/j1939.h>
#include <net/if.h>

namespace SocketCAN {

    /* Timestamp configuration*/
    enum class TimeStampModes {
        None,
        Software,
        Hardware,
        Both
    };

    /*CAN frame type */
    enum class FrameType {
        Classical,
        FD
    };

    /*Timestamped received frame */
    struct ReceivedFrame {
        canid_t id{0};
        uint8_t dlc{0};
        FrameType type{FrameType::Classical};
        std::vector<uint8_t> data;
        std::chrono::nanoseconds timestamp{0};
        bool hardwareTimestamp{false};
    };

    /*TX queue overflow exception*/
    class TxQueueFullException : public std::runtime_error {
        public:
        explicit TxQueueFullException(const std::string& what)
        : std::runtime_error(what) {}
    };

    /*Filter entry (id & mask)*/
    struct CanFilter {
        canid_t id{0};
        canid_t mask{CAN_SFF_MASK};
    };

    /*Base socket configuration*/
    struct BaseConfig {
        std::string interface{"can0"};
        FrameType frameType{FrameType::Classical};
        TimeStampModes timeStampMode{TimeStampModes::Software};
        size_t txQueueMaxSize{256};
        std::vector<CanFilter> filters;
    };

    /*SocketCANBase - base class*/
    class SocketCANBase {
    public:
        explicit SocketCANBase(const BaseConfig& cfg);
        virtual ~SocketCANBase();

        //Non copyable, movable
        SocketCANBase(const SocketCANBase&) = delete;
        SocketCANBase& operator=(const SocketCANBase&) = delete;
        SocketCANBase( SocketCANBase&&) = default;
        SocketCANBase& operator=(SocketCANBase&&) = default;

        //Connection controls
        virtual void Open();
        virtual void Close();
        bool IsOpen() const noexcept{return fd_ >= 0; }

        //Filtering
        void SetFilters(const std::vector<CanFilter>& filters);
        void ClearFilters();

        //Timestamps
        void SetTimeStampMode(TimeStampModes mode);
        TimeStampModes TimeStampMode() const noexcept{return cfg_.timeStampMode;}

        //Synchronus TX
        void SendFrame(canid_t id, std::span<const uint8_t> data, FrameType type = FrameType::Classical);

        //Asynchronus RX
        std::future<ReceivedFrame> ReceiveAsync();

        //Synchronus RX
        std::optional<ReceivedFrame> ReceiveSync(
            std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));

        //TX Worker control
        void StartTxWorker();
        void StopTxWorker();

        //RX worker control
        void StartRxWorker();
        void StopRxWorker();

        //stats
        uint64_t TxFrameCount() const noexcept{return tx_count_.load();}
        uint64_t RxFrameCount() const noexcept{return rx_count_.load();}

    protected:
        virtual int CreateSocket() = 0;
        virtual void BindSocket() = 0;
        virtual void ApplySocketOptions() = 0;

        virtual void WriteFrame(canid_t id, std::span<const uint8_t> data, FrameType type);
        virtual ReceivedFrame ReadFrame();

        void ApplyTimeStampOptions();
        void ApplyFilters();
        ReceivedFrame ExtractTimestamp(struct mshdr& msg, ReceivedFrame& frame);
        int fd_{-1};
        BaseConfig cfg_;

    private:
        //TX Queue
        struct TxEntry {
            canid_t id;
            std::vector<uint8_t> data;
            FrameType type;
        };

        std::queue<TxEntry> tx_queue_;
        std::mutex tx_mutex_;
        std::condition_variable tx_cv_;
        std::thread tx_thread_;
        std::atomic<bool> tx_running_{false};

        //RX promise queue
        std::queue<std::promise<ReceivedFrame>> rx_promises_;
        std::mutex rx_mutex_;
        std::thread rx_thread_;
        std::atomic<bool> rx_running_{false};


        std::atomic<uint64_t> tx_count_{0};
        std::atomic<uint64_t> rx_count_{0};

        void TxWorkerLoop();
        void RxWorkerLoop();
    };
}//namespace SocketCAN


#endif //ETOOLS_SOCKETCAN_H
