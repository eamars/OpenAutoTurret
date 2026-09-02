// Unit tests for the yousee adapter framing codec (PHY-agnostic layer).
#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

#include "can/yousee_transport.hpp"

namespace {

using ota::can::RawFrame;
using ota::can::YouseeCodec;

struct Sink {
  std::vector<RawFrame> frames;
  // NOTE: bind via make_sink(), never pass `sink` by value — std::function
  // copies functors, and mutations would land on the copy.
};

auto make_sink(Sink& s) {
  return [&s](const RawFrame& f) { s.frames.push_back(f); };
}

std::vector<uint8_t> encoded(uint32_t id, const uint8_t d[8]) {
  std::vector<uint8_t> out;
  YouseeCodec::encode(id, d, 8, out);
  return out;
}

TEST(YouseeCodec, EncodeWireFormat) {
  uint8_t d[8] = {0x1E, 0x70, 0, 0, 0, 0, 0, 0};
  auto b = encoded(0x11000064, d);
  ASSERT_EQ(b.size(), 17u);
  EXPECT_EQ(b[0], 'A');
  EXPECT_EQ(b[1], 'T');
  // cid = (0x11000064 << 3) | 4 = 0x88000324
  EXPECT_EQ(b[2], 0x88);
  EXPECT_EQ(b[3], 0x00);
  EXPECT_EQ(b[4], 0x03);
  EXPECT_EQ(b[5], 0x24);
  EXPECT_EQ(b[6], 8);
  EXPECT_EQ(memcmp(&b[7], d, 8), 0);
  EXPECT_EQ(b[15], '\r');
  EXPECT_EQ(b[16], '\n');
}

TEST(YouseeCodec, RoundTrip) {
  Sink sink;
  YouseeCodec codec(make_sink(sink));
  uint8_t d[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  auto b = encoded(0x11006400, d);
  codec.feed(b.data(), b.size(), 123);
  ASSERT_EQ(sink.frames.size(), 1u);
  EXPECT_EQ(sink.frames[0].id, 0x11006400u);
  EXPECT_EQ(sink.frames[0].dlc, 8);
  EXPECT_EQ(sink.frames[0].rx_ns, 123u);
  EXPECT_EQ(memcmp(sink.frames[0].data, d, 8), 0);
  EXPECT_EQ(codec.resyncs(), 0u);
}

TEST(YouseeCodec, SplitAcrossChunks) {
  Sink sink;
  YouseeCodec codec(make_sink(sink));
  uint8_t d[8] = {9, 8, 7, 6, 5, 4, 3, 2};
  auto b = encoded(0x02806400, d);
  codec.feed(b.data(), 5, 1);          // mid-header split
  EXPECT_TRUE(sink.frames.empty());
  codec.feed(b.data() + 5, 6, 2);      // mid-payload split
  EXPECT_TRUE(sink.frames.empty());
  codec.feed(b.data() + 11, b.size() - 11, 3);
  ASSERT_EQ(sink.frames.size(), 1u);
  EXPECT_EQ(sink.frames[0].id, 0x02806400u);
}

TEST(YouseeCodec, GarbageResyncThenFrame) {
  Sink sink;
  YouseeCodec codec(make_sink(sink));
  uint8_t d[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22};
  auto b = encoded(0x18000065, d);
  std::vector<uint8_t> stream = {'X', 'Y', '\0', 'A', 'Q'};  // garbage
  stream.insert(stream.end(), b.begin(), b.end());
  codec.feed(stream.data(), stream.size(), 7);
  ASSERT_EQ(sink.frames.size(), 1u);
  EXPECT_EQ(sink.frames[0].id, 0x18000065u);
  EXPECT_GT(codec.resyncs(), 0u);  // garbage counted as corruption signal
}

TEST(YouseeCodec, AtTextLineConsumed) {
  Sink sink;
  YouseeCodec codec(make_sink(sink));
  const std::string s = "AT+CAN_BAUD:1000000\r\n";
  uint8_t d[8] = {0};
  auto b = encoded(0x00000000, d);
  std::vector<uint8_t> stream(s.begin(), s.end());
  stream.insert(stream.end(), b.begin(), b.end());
  codec.feed(stream.data(), stream.size(), 9);
  ASSERT_EQ(sink.frames.size(), 1u);
  EXPECT_EQ(codec.resyncs(), 0u);  // AT replies are not corruption
}

TEST(YouseeCodec, DlcOverEightResyncs) {
  Sink sink;
  YouseeCodec codec(make_sink(sink));
  std::vector<uint8_t> bogus = {'A', 'T', 0, 0, 0, 0, 9, 1, 2, '\r', '\n'};
  uint8_t d[8] = {5, 5, 5, 5, 5, 5, 5, 5};
  auto b = encoded(0x12000064, d);
  bogus.insert(bogus.end(), b.begin(), b.end());
  codec.feed(bogus.data(), bogus.size(), 1);
  ASSERT_EQ(sink.frames.size(), 1u);
  EXPECT_EQ(sink.frames[0].id, 0x12000064u);
  EXPECT_GT(codec.resyncs(), 0u);
}

TEST(YouseeCodec, PayloadLookingLikeAtDoesNotBreakFraming) {
  // A data payload containing 'A','T' must not confuse the parser once a
  // header is recognized (position-based consumption).
  Sink sink;
  YouseeCodec codec(make_sink(sink));
  uint8_t d[8] = {'A', 'T', '+', 'C', 'G', '\r', '\n', 0};
  auto b = encoded(0x02806500, d);
  codec.feed(b.data(), b.size(), 1);
  ASSERT_EQ(sink.frames.size(), 1u);
  EXPECT_EQ(sink.frames[0].id, 0x02806500u);
  EXPECT_EQ(memcmp(sink.frames[0].data, d, 8), 0);
}

}  // namespace
