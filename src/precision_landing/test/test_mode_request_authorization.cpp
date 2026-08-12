#include <chrono>

#include <gtest/gtest.h>

#include "precision_landing/mode_request_authorization.hpp"

namespace precision_landing {
namespace {

ModeRequestClock::time_point atMilliseconds(int milliseconds) {
  return ModeRequestClock::time_point(
      std::chrono::duration_cast<ModeRequestClock::duration>(
          std::chrono::milliseconds(milliseconds)));
}

ModeRequestAuthorization authorizedUntil(
    ModeRequestClock::time_point expiry) {
  ModeRequestAuthorization authorization;
  authorization.authorized = true;
  authorization.state_snapshot_fresh = true;
  authorization.state_connected = true;
  authorization.state_armed = true;
  authorization.state_offboard = true;
  authorization.generation = 7U;
  authorization.expiry = expiry;
  return authorization;
}

TEST(ModeRequestAuthorization, AllowsCachedAuthorizationBeforeExpiry) {
  const auto authorization = authorizedUntil(atMilliseconds(500));

  EXPECT_TRUE(authorization.validAt(7U, atMilliseconds(499)));
}

TEST(ModeRequestAuthorization,
     RejectsCachedAuthorizationExactlyAtAndAfterExpiry) {
  const auto authorization = authorizedUntil(atMilliseconds(500));

  EXPECT_FALSE(authorization.validAt(7U, atMilliseconds(500)));
  EXPECT_FALSE(authorization.validAt(7U, atMilliseconds(501)));
  EXPECT_TRUE(authorization.authorized);
}

}  // namespace
}  // namespace precision_landing
