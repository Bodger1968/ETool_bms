//
// Created by wayne on 15/10/2026.
//
#ifndef ETOOL_BMS_SOCKETCANRAW_H
#define ETOOL_BMS_SOCKETCANRAW_H

#include "SocketCAN.h"


namespace SocketCAN {
    struct RawConfig : BaseConfig {
        bool loopback{true};
        bool recvOwnMessage{false};
        bool joinFilters{false};
        bool errorFrames{false};
    };

    class SocketCANRaw : public SocketCANBase {
    public:
        explicit SocketCANRaw(const RawConfig& cfg);

    protected:
        int CreateSocket() override;
        void BindSocket() override;
        void ApplySocketOptions() override;

    private:
        RawConfig raw_config_;
    };
} //Namespace SocketCANRaw



#endif //ETOOL_BMS_SOCKETCANRAW_H
