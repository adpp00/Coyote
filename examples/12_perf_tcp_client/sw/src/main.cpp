// tcp_perf_client_host.cpp (minimal options)
// Only: --help/-h, --ip (required), --sessions, --words, --perconn

#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>
#include <cstdint>
#include <algorithm>
#include <boost/program_options.hpp>
#include <unistd.h>   // getpid
#include "cThread.hpp"

#define DEFAULT_VFPGA_ID 0

enum class PerfRegs : uint32_t {
    RUN_TX            = 0, // RW: bit0=1(start), 0(stop)
    NUM_SESSIONS      = 1, // [15:0]
    PKG_WORD_COUNT    = 2, // [7:0]
    SERVER_IP_ADDR    = 3, // [31:0] BE: A.B.C.D -> 0xAA_BB_CC_DD
    TOTAL_PKG_PERCONN = 4  // [7:0]
};

// "A.B.C.D" -> 0xAA_BB_CC_DD (big-endian)
static uint32_t parseIpBE(const std::string& ip_str) {
    std::istringstream iss(ip_str);
    std::string tok; uint32_t b[4]; int i = 0;
    while (std::getline(iss, tok, '.')) {
        if (i >= 4) throw std::invalid_argument("IP has more than 4 octets");
        tok.erase(tok.begin(), std::find_if(tok.begin(), tok.end(),
                   [](unsigned char c){ return !std::isspace(c); }));
        tok.erase(std::find_if(tok.rbegin(), tok.rend(),
                   [](unsigned char c){ return !std::isspace(c); }).base(), tok.end());
        if (tok.empty()) throw std::invalid_argument("Empty IP octet");
        char* endp = nullptr;
        long v = std::strtol(tok.c_str(), &endp, 10);
        if (*endp != '\0' || v < 0 || v > 255)
            throw std::invalid_argument("Invalid IP octet: " + tok);
        b[i++] = static_cast<uint32_t>(v);
    }
    if (i != 4) throw std::invalid_argument("IP must have 4 octets");
    return (b[0] << 24) | (b[1] << 16) | (b[2] << 8) | (b[3] << 0);
}

static std::string ipToStr(uint32_t ip_be) {
    return std::to_string((ip_be>>24)&0xFF) + "." +
           std::to_string((ip_be>>16)&0xFF) + "." +
           std::to_string((ip_be>> 8)&0xFF) + "." +
           std::to_string((ip_be>> 0)&0xFF);
}

int main(int argc, char* argv[]) {
    namespace po = boost::program_options;

    // ---- CLI: only help/ip/sessions/words/perconn ----
    std::string ip_str;                // required (no default)
    unsigned int sessions = 1;         // 0..65535
    unsigned int words = 16;           // 0..255
    unsigned int perconn = 1;          // 0..255

    po::options_description desc("tcp_perf_client host options");
    desc.add_options()
        ("help,h", "Show help")
        ("ip,i",       po::value<std::string>(&ip_str)->required(), "Server IP A.B.C.D (required)")
        ("sessions,s", po::value<unsigned int>(&sessions)->default_value(1), "numSessions (0..65535)")
        ("words,w",    po::value<unsigned int>(&words)->default_value(16),   "pkgWordCount (0..255)")
        ("total,t",  po::value<unsigned int>(&perconn)->default_value(1),  "TotalPkgPerConn (0..255)");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if (vm.count("help")) { std::cout << desc << "\n"; return EXIT_SUCCESS; }
        po::notify(vm);  // triggers required() check for --ip
    } catch (const po::error& e) {
        std::cerr << "Argument error: " << e.what() << "\n\n" << desc << std::endl;
        return EXIT_FAILURE;
    }

    if (sessions > 65535) throw std::invalid_argument("--sessions out of range (0..65535)");
    if (words > 255)      throw std::invalid_argument("--words out of range (0..255)");
    if (perconn > 255)    throw std::invalid_argument("--perconn out of range (0..255)");

    uint32_t ip_be = parseIpBE(ip_str);
    uint64_t run_val = 1; // always start

    // ---- Coyote thread ----
    coyote::cThread coyote_thread(DEFAULT_VFPGA_ID, getpid());

    std::cout << "[CFG] sessions=" << sessions
                << " words=" << words
                << " perConn=" << perconn
                << " ip=" << ip_str << " (0x" << std::hex << ip_be << std::dec << ")"
                << " runTx=1\n";

    // ---- Write registers ----
    coyote_thread.setCSR(static_cast<uint64_t>(sessions), (uint32_t)PerfRegs::NUM_SESSIONS);
    coyote_thread.setCSR(static_cast<uint64_t>(words),    (uint32_t)PerfRegs::PKG_WORD_COUNT);
    coyote_thread.setCSR(static_cast<uint64_t>(ip_be),    (uint32_t)PerfRegs::SERVER_IP_ADDR);
    coyote_thread.setCSR(static_cast<uint64_t>(perconn),  (uint32_t)PerfRegs::TOTAL_PKG_PERCONN);
    coyote_thread.setCSR(static_cast<uint64_t>(run_val),  (uint32_t)PerfRegs::RUN_TX);

    std::cout << "[DONE]\n";
    return EXIT_SUCCESS;

}
