// tcp_listen_host.cpp
#include <any>
#include <cstdint>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include "cThread.hpp"

using namespace std::chrono_literals;

#define DEFAULT_VFPGA_ID 0

enum class TcpRegs : uint32_t {
    LISTEN_PORT_SIGNAL = 0, // W1S: bit0=GO
    LISTEN_PORT        = 1, // WR : 16b port
    PORT_STATUS_SIGNAL = 2, // RO : resp ready (1이면 응답 래치)
    PORT_STATUS        = 3, // RO : [7:0] status code (마지막 값 유지)
    PORT_STATUS_READ   = 4, // W1S: bit0=CLEAR/ACK
    LISTEN_PORT_NUM    = 5  // RO : 누적 응답 카운트(리셋 전까지 누적)
};

inline void pulseW1S(std::unique_ptr<coyote::cThread<std::any>>& th,
                     TcpRegs reg, uint64_t mask = 0x1) {
    th->setCSR(mask, static_cast<uint32_t>(reg));
}

static std::string ipToStr(uint32_t ip_be) {
    // ip_be는 상수 표기 그대로(0x0A FD 4A 5C). 보통 상위 바이트가 첫 옥텟이라고 가정.
    uint8_t a = (ip_be >> 24) & 0xFF;
    uint8_t b = (ip_be >> 16) & 0xFF;
    uint8_t c = (ip_be >> 8)  & 0xFF;
    uint8_t d = (ip_be >> 0)  & 0xFF;
    return std::to_string(a) + "." + std::to_string(b) + "." +
           std::to_string(c) + "." + std::to_string(d);
}

int main(int argc, char** argv) {
    try {
        uint16_t port = 5101;

        std::unique_ptr<coyote::cThread<std::any>> th(
            new coyote::cThread<std::any>(DEFAULT_VFPGA_ID, getpid(), 0)
        );

        uint64_t init_ready  = th->getCSR((uint32_t)TcpRegs::PORT_STATUS_SIGNAL);
        uint64_t init_status = th->getCSR((uint32_t)TcpRegs::PORT_STATUS);
        uint64_t init_count  = th->getCSR((uint32_t)TcpRegs::LISTEN_PORT_NUM);

        std::cout << "[INIT] PORT_STATUS_SIGNAL(Reg2) = " << init_ready << "\n";
        std::cout << "[INIT] PORT_STATUS       (Reg3) = 0x"
                  << std::hex << (uint32_t)(init_status & 0xFF) << std::dec << "\n";
        std::cout << "[INIT] LISTEN_PORT_NUM   (Reg5) = " << init_count << "\n";

        // (B) 포트 쓰고 GO 펄스
        std::cout << "[STEP] Set listen port = " << port << "  & GO\n";
        th->setCSR((uint64_t)port, (uint32_t)TcpRegs::LISTEN_PORT);
        pulseW1S(th, TcpRegs::LISTEN_PORT_SIGNAL, 0x1);

        // (C) 폴링: 응답 올 때까지 Reg2/3/5를 지속 확인(변화 시에만 로그)
        const auto t_start = std::chrono::steady_clock::now();
        const auto timeout = 5s;

        uint64_t prev_ready = init_ready;
        uint64_t prev_status = init_status;
        uint64_t prev_count = init_count;

        while (true) {
            uint64_t ready  = th->getCSR((uint32_t)TcpRegs::PORT_STATUS_SIGNAL);
            uint64_t status = th->getCSR((uint32_t)TcpRegs::PORT_STATUS);
            uint64_t count  = th->getCSR((uint32_t)TcpRegs::LISTEN_PORT_NUM);

            bool changed = false;
            if (ready != prev_ready) {
                std::cout << "[POLL] Reg2 ready: " << prev_ready << " -> " << ready << "\n";
                prev_ready = ready; changed = true;
            }
            if ((status & 0xFF) != (prev_status & 0xFF)) {
                std::cout << "[POLL] Reg3 status: 0x" << std::hex
                          << (uint32_t)(prev_status & 0xFF) << " -> 0x"
                          << (uint32_t)(status & 0xFF) << std::dec << "\n";
                prev_status = status; changed = true;
            }
            if (count != prev_count) {
                std::cout << "[POLL] Reg5 count : " << prev_count << " -> " << count << "\n";
                prev_count = count; changed = true;
            }

            if (ready) {
                std::cout << "[OK] Response ready. status_code=0x"
                          << std::hex << (uint32_t)(status & 0xFF) << std::dec
                          << "  (accepted_count=" << count << ")\n";
                break;
            }

            if (std::chrono::steady_clock::now() - t_start > timeout) {
                std::cerr << "[ERR] Timeout while waiting response on port " << port << "\n";
                return EXIT_FAILURE;
            }

            // 변화 없을 땐 짧게 쉼
            if (!changed) std::this_thread::sleep_for(200us);
        }

        // (D) ACK(CLEAR) 후, 신호 클리어 확인(선택)
        pulseW1S(th, TcpRegs::PORT_STATUS_READ, 0x1);
        auto sig_after_ack = th->getCSR((uint32_t)TcpRegs::PORT_STATUS_SIGNAL);
        std::cout << "[STEP] ACK sent. Reg2 after ACK = " << sig_after_ack << "\n";

        std::cout << "[DONE]\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
}
