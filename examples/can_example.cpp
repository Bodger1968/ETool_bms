//
// Created by wayne on 13/10/2026.
//
#include "../SocketCan/headers/SocketCAN.h"
#include "../SocketCan/headers/SocketCANRaw.h"
#include "../SocketCan/headers/SocketCANProtocols.h"
#include <iostream>
#include <iomanip>
#include <numeric>
#include <thread>
#include <chrono>

using namespace SocketCAN;
using namespace std::chrono_literals;

void printFrame(const ReceivedFrame& f) {
    std::cout << "[" << (f.hardwareTimestamp ? "HW" : "SW") << " "
              << f.timestamp.count() << "ns] "
              << "ID=0x" << std::hex << std::setw(3) << std::setfill('0') << f.id
              << " DLC=" << std::dec << static_cast<int>(f.dlc)
              << " DATA=";
    for (auto b : f.data)
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(b) << " ";
    std::cout << "\n";
}

void exampleRaw() {
    std::cout << "\n=== RAW Example ===\n";

    RawConfig cfg;
    cfg.interface      = "can0";
    cfg.frameType      = FrameType::Classical;
    cfg.timeStampMode  = TimeStampModes::Both;
    cfg.txQueueMaxSize = 128;
    cfg.loopback       = true;
    cfg.recvOwnMessage = true;

    // ID + mask filter: accept 0x100–0x1FF
    cfg.filters = {{ .id = 0x100, .mask = 0x700 }};

    SocketCANRaw sock(cfg);
    sock.Open();
    sock.StartTxWorker();
    sock.StartRxWorker();

    // Async RX: post a future before sending
    auto rxFuture = sock.ReceiveAsync();

    // Enqueue a frame
    std::vector<uint8_t> payload = {0x01, 0x02, 0x03, 0x04};
    sock.SendFrame(0x123, payload);

    // Wait for the frame we just sent (loopback)
    if (rxFuture.wait_for(500ms) == std::future_status::ready) {
        printFrame(rxFuture.get());
    } else {
        std::cout << "RX timeout\n";
    }

    // Synchronous RX (blocks up to 1s)
    auto result = sock.ReceiveSync(1000ms);
    if (result)
        printFrame(*result);

    sock.StopRxWorker();
    sock.StopTxWorker();
    sock.Close();
}

// ─────────────────────────────────────────────────────────────
//  Example 2: BCM — cyclic TX frame every 100 ms
// ─────────────────────────────────────────────────────────────
void exampleBcm() {
    std::cout << "\n=== BCM Example ===\n";

    BcmConfig cfg;
    cfg.interface     = "can0";
    cfg.timeStampMode = TimeStampModes::Software;

    SocketCANBcm sock(cfg);
    sock.Open();

    // Add a cyclic TX job: 0x200 every 100 ms, forever
    BcmTxJob job;
    job.id         = 0x18DAF3F1 | CAN_EFF_FLAG;
    job.data       = {0x03, 0x19, 0x01, 0xFF, 0x55, 0x55, 0x55, 0x55};
    job.intervalMs = 1000;
    job.count      = 0; // forever
    sock.AddTxJob(job);

    std::cout << "BCM TX job running for 500ms...\n";
    std::this_thread::sleep_for(5000ms);

    sock.RemoveTxJob(0x18DAF3F1);
    sock.Close();
}

// ─────────────────────────────────────────────────────────────
//  Example 3: ISO-TP multi-frame send/receive
// ─────────────────────────────────────────────────────────────
void exampleIsoTp() {
    std::cout << "\n=== ISO-TP Example ===\n";

    // Sender socket
    IsoTpConfig txCfg;
    txCfg.interface    = "can0";
    txCfg.tx_id         = 0x18DAF3F1;
    txCfg.rx_id         = 0x18DAF1F3;
    txCfg.wait_tx_done  = true;
    txCfg.timeStampMode = TimeStampModes::Software;

    // Receiver socket (inverted IDs)
    IsoTpConfig rxCfg  = txCfg;
    rxCfg.tx_id         = 0x18DAF3F1;
    rxCfg.rx_id         = 0x18DAF1F3;

    SocketCANIsoTp txSock(txCfg);
    SocketCANIsoTp rxSock(rxCfg);

    txSock.Open();
    rxSock.Open();

    // Post async receive BEFORE send
    auto rxFuture = rxSock.ReceivePayloadAsync();

    // Send a 50-byte payload (triggers multi-frame segmentation)
    std::vector<uint8_t> bigPayload(50);
    std::iota(bigPayload.begin(), bigPayload.end(), 0x00);
    txSock.SendPayload(bigPayload);

    if (rxFuture.wait_for(2s) == std::future_status::ready) {
        auto received = rxFuture.get();
        std::cout << "ISO-TP received " << received.size() << " bytes\n";
    } else {
        std::cout << "ISO-TP RX timeout\n";
    }

    txSock.Close();
    rxSock.Close();
}

void exampleReceiveUDS() {
    std::cout << "\n=== UDS Example ===\n";


}

// ─────────────────────────────────────────────────────────────
//  Example 4: TxQueueFullException
// ─────────────────────────────────────────────────────────────
void exampleQueueFull() {
    std::cout << "\n=== TX Queue Full Exception Example ===\n";

    RawConfig cfg;
    cfg.interface      = "can0";
    cfg.txQueueMaxSize = 4; // deliberately tiny

    SocketCANRaw sock(cfg);
    sock.Open();
    // Note: TX worker not started so the queue fills up

    try {
        for (int i = 0; i < 10; ++i) {
            std::vector<uint8_t> f = {0x01, 0x02};
            sock.SendFrame(0x100 + i, f);
        }
    } catch (const TxQueueFullException& e) {
        std::cout << "Caught expected exception: " << e.what() << "\n";
    }

    sock.Close();
}

// ─────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────
int main() {
    try {
        exampleRaw();
        exampleBcm();
        exampleIsoTp();
        exampleQueueFull();
    } catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
