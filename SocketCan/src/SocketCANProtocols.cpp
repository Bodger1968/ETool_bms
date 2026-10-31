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


    }

}
