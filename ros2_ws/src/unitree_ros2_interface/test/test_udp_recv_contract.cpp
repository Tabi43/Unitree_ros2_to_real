/**
 * Contract test for UNITREE_LEGGED_SDK::UDP::Recv() in RecvEnum::blockTimeout mode.
 *
 * The whole liveness layer of legged-sdk-interface (freshness timestamps, the staleness
 * watchdog, the ENABLING_LOW/ENABLING_HIGH handshakes, the diagnostics counters) rests on
 * one property: Recv() reports whether a datagram ACTUALLY arrived. That property is not
 * documented in the SDK headers and the SDK ships as a precompiled static library, so a
 * different libunitree_legged_sdk.a could silently change it. These tests pin it down.
 *
 * The property does NOT hold in the SDK's polling mode, where Recv() re-validates the
 * previous buffer and cannot distinguish "nothing arrived" from "same frame again".
 * Which mode is in force turns out to depend on SetRecvTimeout() rather than on the
 * constructor's RecvEnum, which is easy to break by accident - deleting the
 * SetRecvTimeout() call below makes ReportsTimeoutWhenNoDataArrives and
 * BlocksForRoughlyTheConfiguredTimeout fail, which is exactly what they are here for.
 *
 * No robot required: everything runs against a loopback socket.
 */

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <vector>

#include "unitree_ros2_interface/legged-sdk-interface.hpp"

namespace {

// Loopback ports well away from the ones the real interface binds (8090/8091) so the
// test can run while a node is up. Each test uses its own pair: the SDK's UDP destructor
// does not free the bound port for an immediate rebind in the same process, so reusing
// one port across tests silently lands the socket somewhere else and any injected
// datagram is then never delivered.
constexpr char kLoopback[] = "127.0.0.1";
constexpr uint16_t kTimeoutLocalPort = 18091;
constexpr uint16_t kBlockingLocalPort = 18092;
constexpr uint16_t kMalformedLocalPort = 18093;
constexpr uint16_t kMalformedPeerPort = 19093;

/**
 * @brief A UDP socket bound to kPeerPort, standing in for the robot. The SDK connect()s
 * its socket to the peer address, so injected datagrams must originate from exactly this
 * address/port or the kernel drops them before Recv() ever sees them.
 */
class FakeRobot {
 public:
  FakeRobot(uint16_t peer_port, uint16_t sdk_port)
  : sdk_port_(sdk_port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(peer_port);
    addr.sin_addr.s_addr = inet_addr(kLoopback);
    bind_ok_ = fd_ >= 0 && ::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0;
  }

  ~FakeRobot() {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  bool ok() const { return bind_ok_; }

  /// Sends `bytes` to the SDK socket's local port.
  void send(const std::vector<uint8_t> & bytes) {
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(sdk_port_);
    dst.sin_addr.s_addr = inet_addr(kLoopback);
    ::sendto(fd_, bytes.data(), bytes.size(), 0,
             reinterpret_cast<sockaddr *>(&dst), sizeof(dst));
  }

 private:
  int fd_{-1};
  uint16_t sdk_port_{0};
  bool bind_ok_{false};
};

/// Builds the SDK socket exactly the way LeggedSDKInterface does, but on loopback.
UNITREE_LEGGED_SDK::UDP makeSdkSocket(uint16_t local_port, uint16_t peer_port) {
  UNITREE_LEGGED_SDK::UDP udp(
      local_port, kLoopback, peer_port,
      UNITREE_LEGGED_SDK::LOW_CMD_LENGTH, UNITREE_LEGGED_SDK::LOW_STATE_LENGTH,
      false, UNITREE_LEGGED_SDK::RecvEnum::blockTimeout);
  udp.SetRecvTimeout(kUdpRecvTimeoutMs);
  return udp;
}

}  // namespace

/**
 * The load-bearing assertion: with nothing on the wire, Recv() must report a distinct
 * "no data" code rather than success. If this ever returns UDP_RECV_OK, the receive paths
 * would refresh their freshness timestamps against a dead link and the staleness watchdog
 * would never fire — the exact failure this design replaced.
 */
TEST(UdpRecvContract, ReportsTimeoutWhenNoDataArrives) {
  auto udp = makeSdkSocket(kTimeoutLocalPort, kMalformedPeerPort);

  const int rc = udp.Recv();

  EXPECT_EQ(rc, UDP_RECV_TIMEOUT);
  EXPECT_NE(rc, UDP_RECV_OK);
}

/**
 * Recv() must also block for approximately the configured timeout rather than spinning:
 * that is what paces the receive loop and keeps it off the CPU between frames. A very
 * loose lower bound is used so the test does not become a flaky timing check.
 */
TEST(UdpRecvContract, BlocksForRoughlyTheConfiguredTimeout) {
  auto udp = makeSdkSocket(kBlockingLocalPort, kMalformedPeerPort);

  const auto start = std::chrono::steady_clock::now();
  udp.Recv();
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start).count();

  EXPECT_GE(elapsed_ms, kUdpRecvTimeoutMs / 2);
  EXPECT_LT(elapsed_ms, 500);  // must not block indefinitely
}

/**
 * A datagram that is not a valid robot frame must be rejected, not accepted as state.
 * This guards the head/CRC gate inside Recv(): without it, any stray traffic reaching
 * the bound port would be latched and published as robot state.
 */
TEST(UdpRecvContract, RejectsMalformedDatagram) {
  FakeRobot robot(kMalformedPeerPort, kMalformedLocalPort);
  ASSERT_TRUE(robot.ok()) << "could not bind the fake robot socket on " << kMalformedPeerPort;

  auto udp = makeSdkSocket(kMalformedLocalPort, kMalformedPeerPort);

  std::vector<uint8_t> garbage(UNITREE_LEGGED_SDK::LOW_STATE_LENGTH, 0xAB);
  robot.send(garbage);

  const int rc = udp.Recv();

  ASSERT_NE(rc, UDP_RECV_TIMEOUT)
      << "the injected datagram never reached the SDK socket; the test is not exercising "
         "the rejection path (check that both sockets bound their intended ports)";
  EXPECT_NE(rc, UDP_RECV_OK) << "malformed datagram was accepted as a robot frame";
  EXPECT_TRUE(rc == UDP_RECV_BAD_HEAD || rc == UDP_RECV_CRC_ERROR)
      << "unexpected rejection code: " << rc;
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
