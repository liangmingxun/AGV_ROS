#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "chassis_driver/fixed_frame_parser.hpp"

namespace {

std::vector<std::uint8_t> makeFrame(const std::vector<std::uint8_t>& payload) {
  std::vector<std::uint8_t> frame{'#', '$', '#'};
  frame.insert(frame.end(), payload.begin(), payload.end());
  frame.insert(frame.end(), {'!', '@', '!'});
  return frame;
}

std::vector<std::uint8_t> payloadWith(std::uint8_t value) {
  return std::vector<std::uint8_t>(
      chassis_driver::FixedFrameParser::kPayloadSize, value);
}

TEST(FixedFrameParser, ReassemblesAFragmentedFrame) {
  chassis_driver::FixedFrameParser parser;
  const auto expected = payloadWith(0x21);
  const auto frame = makeFrame(expected);
  std::vector<std::uint8_t> actual;

  parser.append(frame.data(), 2);
  EXPECT_FALSE(parser.popPayload(actual));
  parser.append(frame.data() + 2, frame.size() - 2);
  EXPECT_TRUE(parser.popPayload(actual));
  EXPECT_EQ(expected, actual);
}

TEST(FixedFrameParser, IgnoresDelimiterBytesInsideBinaryPayload) {
  chassis_driver::FixedFrameParser parser;
  auto expected = payloadWith(0x00);
  expected[5] = '!';
  expected[6] = '@';
  expected[7] = '!';
  expected[20] = '#';
  expected[21] = '$';
  expected[22] = '#';
  const auto frame = makeFrame(expected);
  std::vector<std::uint8_t> actual;

  parser.append(frame.data(), frame.size());
  EXPECT_TRUE(parser.popPayload(actual));
  EXPECT_EQ(expected, actual);
}

TEST(FixedFrameParser, ExtractsBackToBackFramesAfterNoise) {
  chassis_driver::FixedFrameParser parser;
  const auto first_payload = payloadWith(0x11);
  const auto second_payload = payloadWith(0x22);
  auto bytes = std::vector<std::uint8_t>{0x00, 0x01, 0x02};
  const auto first_frame = makeFrame(first_payload);
  const auto second_frame = makeFrame(second_payload);
  bytes.insert(bytes.end(), first_frame.begin(), first_frame.end());
  bytes.insert(bytes.end(), second_frame.begin(), second_frame.end());
  parser.append(bytes.data(), bytes.size());

  std::vector<std::uint8_t> actual;
  bool resynchronised = false;
  EXPECT_TRUE(parser.popPayload(actual, &resynchronised));
  EXPECT_TRUE(resynchronised);
  EXPECT_EQ(first_payload, actual);
  EXPECT_TRUE(parser.popPayload(actual));
  EXPECT_EQ(second_payload, actual);
  EXPECT_FALSE(parser.popPayload(actual));
}

TEST(FixedFrameParser, RejectsCorruptFrameAndFindsNextValidFrame) {
  chassis_driver::FixedFrameParser parser;
  auto corrupt = makeFrame(payloadWith(0x00));
  corrupt.back() = 0x00;
  const auto expected = payloadWith(0x42);
  const auto valid = makeFrame(expected);
  corrupt.insert(corrupt.end(), valid.begin(), valid.end());
  parser.append(corrupt.data(), corrupt.size());

  std::vector<std::uint8_t> actual;
  bool resynchronised = false;
  EXPECT_TRUE(parser.popPayload(actual, &resynchronised));
  EXPECT_TRUE(resynchronised);
  EXPECT_EQ(expected, actual);
}

}  // namespace
