//
// Created by wayne on 15/10/2026.
//
#include "../headers/SocketCANRaw.h"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <stdexcept>

namespace SocketCAN {

    SocketCANRaw::SocketCANRaw(const RawConfig& cfg) : SocketCANBase(cfg), raw_config_(cfg) {

    }

    int SocketCANRaw::CreateSocket() {
        int domain = PF_CAN;
        int type = SOCK_RAW;
        int proto =  CAN_RAW;
        return ::socket(domain, type, proto);
    }

    void SocketCANRaw::BindSocket() {
        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, cfg_.interface.c_str(), IFNAMSIZ - 1);
        if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0 ) {
            throw std::runtime_error("SocketCAN RAW: ioctl SIOCGIFINDEX failed for '" +
                cfg_.interface + "': " + strerror(errno));
        }
        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            throw std::runtime_error("SocketCAN RAW: bind failed: " +
                                     std::string(strerror(errno)));
        }
    }

    void SocketCANRaw::ApplySocketOptions() {
        if (cfg_.frameType == FrameType::FD) {
            int enable = 1;
            if ( setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0) {
                throw std::runtime_error("SocketCAN RAW: CAN_RAW_FD_FRAMES failed: " + std::string(strerror(errno)));
            }
        }
        // ── Loopback ─────────────────────────────────────────────
        {
            int v = raw_config_.loopback ? 1 : 0;
            if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK,
                           &v, sizeof(v)) < 0)
                throw std::runtime_error("SocketCAN RAW: CAN_RAW_LOOPBACK failed: " +
                                         std::string(strerror(errno)));
        }

        // ── Receive own messages ──────────────────────────────────
        {
            int v = raw_config_.recvOwnMessage ? 1 : 0;
            if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
                           &v, sizeof(v)) < 0)
                throw std::runtime_error("SocketCAN RAW: CAN_RAW_RECV_OWN_MSGS failed: " +
                                         std::string(strerror(errno)));
        }

        // ── Error frames ─────────────────────────────────────────
        if (raw_config_.errorFrames) {
            can_err_mask_t errMask = CAN_ERR_MASK;
            if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
                           &errMask, sizeof(errMask)) < 0)
                throw std::runtime_error("SocketCAN RAW: CAN_RAW_ERR_FILTER failed: " +
                                         std::string(strerror(errno)));
        }

        // ── Join filters (OR semantics) ───────────────────────────
        if (raw_config_.joinFilters) {
            int v = 1;
            if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS,
                           &v, sizeof(v)) < 0)
                throw std::runtime_error("SocketCAN RAW: CAN_RAW_JOIN_FILTERS failed: " +
                                         std::string(strerror(errno)));
        }

    }
}
