//
// Created by wayne on 31/10/2026.
//

#include "SocketCANProtocols.h"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <stdexcept>
#include <vector>

namespace SocketCAN {

    SocketCANBcm::SocketCANBcm(const BcmConfig& cfg) : SocketCANBase(cfg), config_(cfg) {

    }

    int SocketCANBcm::CreateSocket() {
        return ::socket(PF_CAN, SOCK_DGRAM, CAN_BCM);
    }

    void SocketCANBcm::BindSocket() {
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, config_.interface.c_str(), IFNAMSIZ - 1);
        if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
            throw std::runtime_error("SocketCAN BCM: ioctl failed: " + std::string(strerror(errno)));
        }

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (::connect(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            throw std::runtime_error("SocketCAN BCM: connect failed: " + std::string(strerror(errno)));
        }
    }

    void SocketCANBcm::ApplySocketOptions() {
        //TBD
    }
    void SocketCANBcm::AddTxJob( const BcmTxJob& job ) {
        struct can_frame frame{};
        frame.can_id = job.id;
        frame.can_dlc = static_cast<uint8_t>(std::min(job.data.size(), size_t(CAN_MAX_DLEN)));
        std::memcpy(frame.data, job.data.data(), frame.can_dlc);

        struct bcm_msg_head head{};
        head.opcode = TX_SETUP;
        head.flags = SETTIMER | STARTTIMER;
        head.count = job.count;
        head.ival2.tv_sec = job.intervalMs / 1000;
        head.ival2.tv_usec = job.intervalMs % 1000 * 1000;
        head.can_id = job.id;
        head.nframes = 1;

        WriteBcmMsg(head, &frame, 1);
    }

    void SocketCANBcm::RemoveTxJob(canid_t id) {
        struct bcm_msg_head head{};
        head.opcode = TX_DELETE;
        head.can_id = id;
        head.nframes = 1;
        WriteBcmMsg(head, nullptr, 0);
    }

    void SocketCANBcm::AddRxJob( const BcmRxJob& job ) {
        struct can_frame frame{};
        frame.can_id = job.mask;
        frame.can_dlc = CAN_MAX_DLEN;
        std::memset(frame.data, 0xFF, CAN_MAX_DLEN);

        struct bcm_msg_head head{};
        head.opcode = RX_SETUP;
        head.flags = (job.timeoutMs > 0 ? SETTIMER : 0) | (job.throttleMs > 0 ? RX_FILTER_ID : 0);
        head.ival1.tv_sec = job.timeoutMs / 1000;
        head.ival1.tv_usec = job.timeoutMs % 1000 * 1000;
        head.ival2.tv_sec = job.throttleMs / 1000;
        head.ival2.tv_usec = job.throttleMs % 1000 * 1000;
        head.can_id = job.id;
        head.nframes = 1;
        WriteBcmMsg(head, &frame, 1);
    }

    void SocketCANBcm::RemoveRxJob(canid_t id) {
        struct bcm_msg_head head{};
        head.opcode = RX_DELETE;
        head.can_id = id;
        head.nframes = 1;

        WriteBcmMsg(head, nullptr, 0);
    }

    void SocketCANBcm::WriteBcmMsg(struct bcm_msg_head &head, struct can_frame *frames, size_t n_frames) {
        size_t msg_len = sizeof(struct bcm_msg_head) + n_frames * sizeof(struct can_frame);
        std::vector<uint8_t> buf(msg_len, 0);

        std::memcpy(buf.data(), &head, sizeof(head));
        if (frames && n_frames > 0) {
            std::memcpy(buf.data() + sizeof(head), frames, n_frames * sizeof(struct can_frame));
        }

        if (::write(fd_, buf.data(), msg_len) < 0) {
            throw std::runtime_error("SocketCAN BCM: write failed: " + std::string(strerror(errno)));
        }

    }

    void SocketCANBcm::WriteFrame(canid_t id, std::span<const uint8_t> data, FrameType type) {
        BcmTxJob job;
        job.id = id;
        job.data = std::vector<uint8_t>(data.begin(), data.end());
        job.intervalMs = 0;
        job.count = 1;
        AddTxJob(job);
    }

    //ISO-TP

    SocketCANIsoTp::SocketCANIsoTp(const IsoTpConfig &cfg) : SocketCANBase(cfg), config_(cfg) {

    }

    int SocketCANIsoTp::CreateSocket() {
        return ::socket(PF_CAN, SOCK_DGRAM, CAN_ISOTP);
    }

    void SocketCANIsoTp::BindSocket() {
        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, config_.interface.c_str(), IFNAMSIZ - 1);
        if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
            throw std::runtime_error("SocketCAN ISOTP: ioctl failed: " + std::string(strerror(errno)));
        }

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        addr.can_addr.tp.tx_id = config_.tx_id;
        addr.can_addr.tp.rx_id = config_.rx_id;

        if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            throw std::runtime_error("SocketCAN ISOTP: bind failed: " + std::string(strerror(errno)));
        }
    }

    void SocketCANIsoTp::ApplySocketOptions() {
        struct can_isotp_options opts{};

        if (config_.extended_addressing) {
            opts.flags    |= CAN_ISOTP_EXTEND_ADDR;
            opts.ext_address = config_.tx_ext_address;
            opts.rx_ext_address = config_.rx_ext_address;
        }

        if (config_.tx_padding) {
            opts.flags       |= CAN_ISOTP_TX_PADDING;
            opts.txpad_content = static_cast<uint8_t>( config_.tx_padding);
        }

        if (config_.wait_tx_done) {
            opts.flags       |= CAN_ISOTP_WAIT_TX_DONE;
        }

        if (setsockopt(fd_, SOL_CAN_ISOTP, CAN_ISOTP_OPTS, &opts, sizeof(opts)) < 0) {
            throw std::runtime_error("SocketCAN ISOTP: CAN_ISOTP_OPTS failed: " + std::string(strerror(errno)));
        }

        if (config_.rx_block_size || config_.st_min_ns) {
            struct can_isotp_fc_options fc{};
            fc.bs    = static_cast<uint8_t>( config_.rx_block_size);
            fc.stmin = 0; // kernel handles stmin from stMinNs
            fc.wftmax = 0;
            if (setsockopt(fd_, SOL_CAN_ISOTP, CAN_ISOTP_RECV_FC, &fc, sizeof(fc)) < 0) {
                throw std::runtime_error("SocketCAN ISOTP: CAN_ISOTP_RECV_FC failed: " + std::string(strerror(errno)));
            }
        }
    }

    void SocketCANIsoTp::SendPayload(std::span<const uint8_t> payload) {
        if (::write( fd_, payload.data(), payload.size()) < 0) {
            throw std::runtime_error("SocketCAN ISOTP: write failed: " + std::string(strerror(errno)));
        }
    }

    std::future<std::vector<uint8_t> > SocketCANIsoTp::ReceivePayloadAsync() {
        return std::async(std::launch::async, [this]() -> std::vector<uint8_t> {
            std::vector<uint8_t> buf(4096);
            ssize_t n = ::read(fd_, buf.data(), buf.size());
            if (n < 0) {
                throw std::runtime_error("SocketCAN ISOTP: read failed: " + std::string(strerror(errno)));
            }
            buf.resize(static_cast<size_t>(n));
        return buf;
        });
    }
    std::optional<std::vector<uint8_t>> SocketCANIsoTp::ReceivePayloadSync(std::chrono::milliseconds timeout)
    {
        // Use select() directly rather than going through the promise queue,
        // which can accumulate stale unfulfilled promises and miss frames.
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET( fd_, &readfds);

        struct timeval tv{};
        tv.tv_sec  = timeout.count() / 1000;
        tv.tv_usec = (timeout.count() % 1000) * 1000;

        int ret = ::select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
        if (ret <= 0)
            return std::nullopt; // timeout or error

        std::vector<uint8_t> buf(4096);
        ssize_t n = ::read( fd_, buf.data(), buf.size());
        if (n <= 0)
            return std::nullopt;

        buf.resize(static_cast<size_t>(n));
        return buf;
    }

    // J1939

    SocketCANJ1939::SocketCANJ1939(const J1939Config &cfg) : SocketCANBase(cfg), config_(cfg) {

    }

    int SocketCANJ1939::CreateSocket(){
        return ::socket(PF_CAN, SOCK_DGRAM, CAN_J1939);
    }

    void SocketCANJ1939::BindSocket() {
        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, config_.interface.c_str(), IFNAMSIZ - 1);
        if (::ioctl( fd_, SIOCGIFINDEX, &ifr) < 0)
            throw std::runtime_error("SocketCAN J1939: ioctl failed: " + std::string(strerror(errno)));

        struct sockaddr_can addr{};
        addr.can_family              = AF_CAN;
        addr.can_ifindex             = ifr.ifr_ifindex;
        addr.can_addr.j1939.name     = config_.src_name;
        addr.can_addr.j1939.pgn      = config_.pgn;
        addr.can_addr.j1939.addr     = config_.src_addr;

        if (::bind( fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            throw std::runtime_error("SocketCAN J1939: bind failed: " + std::string(strerror(errno)));
        }
    }

    void SocketCANJ1939::ApplySocketOptions() {
        if ( config_.promiscuous) {
            int v = 1;
            if (setsockopt(fd_, SOL_CAN_J1939, SO_J1939_PROMISC, &v, sizeof(v)) < 0) {
                throw std::runtime_error("SocketCAN J1939: SO_J1939_PROMISC failed: " + std::string(strerror(errno)));
            }
        }

        // Set send priority
        {
            int prio = static_cast<int>( config_.send_priority);
            if (setsockopt(fd_, SOL_CAN_J1939, SO_J1939_SEND_PRIO, &prio, sizeof(prio)) < 0) {
                throw std::runtime_error("SocketCAN J1939: SO_J1939_SEND_PRIO failed: " + std::string(strerror(errno)));
            }
        }
    }

    void SocketCANJ1939::SendMessage(uint32_t pgn, uint8_t dest_addr, std::span<const uint8_t> data) {
        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, config_.interface.c_str(), IFNAMSIZ - 1);
        ::ioctl(fd_, SIOCGIFINDEX, &ifr);

        struct sockaddr_can dest{};
        dest.can_family              = AF_CAN;
        dest.can_ifindex             = ifr.ifr_ifindex;
        dest.can_addr.j1939.pgn      = pgn;
        dest.can_addr.j1939.addr     = dest_addr;
        dest.can_addr.j1939.name     = J1939_NO_NAME;

        ssize_t n = ::sendto(fd_, data.data(), data.size(), 0,reinterpret_cast<struct sockaddr*>(&dest),sizeof(dest));
        if (n < 0) {
            throw std::runtime_error("SocketCAN J1939: sendto failed: " + std::string(strerror(errno)));
        }

    }

    void SocketCANJ1939::WriteFrame(canid_t id, std::span<const uint8_t> data, FrameType type) {
        // For J1939 the TX queue worker falls through here;
        // use sendMessage() directly for full PGN control.
        if (::write( fd_, data.data(), data.size()) < 0) {
            throw std::runtime_error("SocketCAN J1939: write failed: " + std::string(strerror(errno)));
        }
    }

    std::future<ReceivedFrame> SocketCANJ1939::ReceiveAsync() {
        return SocketCANBase::ReceiveAsync();
    }
}
