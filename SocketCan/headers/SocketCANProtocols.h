//
// Created by wayne on 31/10/2026.
//

#ifndef ETOOL_BMS_SOCKETCANPROTOCOLS_H
#define ETOOL_BMS_SOCKETCANPROTOCOLS_H

#include "SocketCAN.h"
#include <linux/can/bcm.h>

namespace SocketCAN {
    struct BcmConfig : BaseConfig {
        // No extra config required at construction; cyclic jobs are added via API
    };

    // Describes one cyclic TX job
    struct BcmTxJob {
        canid_t              id{0};
        std::vector<uint8_t> data;
        FrameType            type{FrameType::Classical};
        uint32_t             intervalMs{100};   // transmission interval
        uint32_t             count{0};          // 0 = repeat forever
    };

    // Describes one RX content-change subscription
    struct BcmRxJob {
        canid_t              id{0};
        canid_t              mask{0xFFFFFFFF};  // which bytes trigger a change notification
        uint32_t             timeoutMs{0};      // 0 = no timeout monitoring
        uint32_t             throttleMs{0};     // minimum interval between notifications
    };

    class SocketCANBcm : public SocketCANBase {
    public:
        explicit SocketCANBcm( const BcmConfig& cfg);

        void AddTxJob(const BcmTxJob& job);
        void RemoveTxJob( canid_t id);

        void AddRxJob(const BcmRxJob& job);
        void RemoveRxJob( canid_t id);

    protected:
        int CreateSocket() override;
        void BindSocket() override;
        void ApplySocketOptions() override;

        void WriteFrame(canid_t id, std::span<const uint8_t> data, FrameType type) override;

    private:
        BcmConfig config_;
        void WriteBcmMsg(struct bcm_msg_head& head, struct can_frame* frames, size_t n_frames);
    };

    struct IsoTpConfig : BaseConfig {
        canid_t tx_id{0x18DAF3F1};
        canid_t rx_id{0x18DAF1F3};
        bool extended_addressing{false};
        uint8_t tx_ext_address{0x00};
        uint8_t rx_ext_address{0x00};
        uint32_t tx_padding{0};
        uint32_t rx_block_size{0};
        uint32_t st_min_ns{0};
        bool wait_tx_done{true};
    };

    class SocketCANIsoTp : public SocketCANBase {
    public:
        explicit SocketCANIsoTp( const IsoTpConfig& cfg);

        void SendPayload(std::span<const uint8_t> payload);

        std::future<std::vector<uint8_t>> ReceivePayloadAsync();
        std::optional<std::vector<uint8_t>> ReceivePayloadSync( std::chrono::milliseconds timeout = std::chrono::milliseconds(100));

    protected:
        int CreateSocket() override;
        void BindSocket() override;
        void ApplySocketOptions() override;

    private:
        IsoTpConfig config_;

    };

    struct J1939Config : BaseConfig {
        uint64_t src_name{J1939_NO_NAME};
        uint32_t pgn{J1939_NO_PGN};
        uint8_t src_addr{J1939_NO_ADDR};
        uint8_t dest_addr{J1939_NO_ADDR};
        bool promiscuous{false};
        uint8_t send_priority{ 6 };
    };

    class SocketCANJ1939 : public SocketCANBase {
        public:
        explicit SocketCANJ1939( const J1939Config& cfg);

        void SendMessage(uint32_t pgn, uint8_t dest_addr, std::span< const uint8_t> data);

        std::future<ReceivedFrame> ReceiveAsync();

    protected:
        int CreateSocket() override;
        void BindSocket() override;
        void ApplySocketOptions() override;

        void WriteFrame(canid_t id, std::span<const uint8_t> data, FrameType type) override;
        private:
        J1939Config config_;
    };

}



#endif //ETOOL_BMS_SOCKETCANPROTOCOLS_H
