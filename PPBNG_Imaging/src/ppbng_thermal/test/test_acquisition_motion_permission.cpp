#include "ppbng_thermal/acquisition_motion_permission.hpp"
#include <gtest/gtest.h>
using ppbng_thermal::AcquisitionMotionPermission;
TEST(AcquisitionMotionPermission, NeedsRecordingAndEveryFreshStream) {
  AcquisitionMotionPermission p({"fx", "gps"});
  EXPECT_FALSE(p.permitted(1,10));
  p.manager("s",false,false,1); p.sample("s","fx",1,1);
  p.manager("s",true,false,2); p.sample("s","gps",1,2);
  EXPECT_FALSE(p.permitted(2,10));
  p.sample("s","fx",1,2); EXPECT_TRUE(p.permitted(2,10));
}
TEST(AcquisitionMotionPermission, WarningLatchesUntilNewSession) {
  AcquisitionMotionPermission p({"fx"});
  p.manager("s",true,false,1); p.sample("s","fx",1,1);
  EXPECT_TRUE(p.permitted(1,10));
  p.fault("other","ignored"); EXPECT_TRUE(p.permitted(1,10));
  p.fault("s","overflow"); p.manager("s",true,false,2);
  p.sample("s","fx",2,2); EXPECT_FALSE(p.permitted(2,10));
  p.manager("new",true,false,3); p.sample("new","fx",1,3);
  EXPECT_TRUE(p.permitted(3,10));
}
TEST(AcquisitionMotionPermission, FrozenStreamAndDuplicateSamplesLatch) {
  AcquisitionMotionPermission p({"fx"});
  p.manager("s",true,false,1); p.sample("s","fx",1,1);
  EXPECT_TRUE(p.permitted(1,10));
  p.manager("s",true,false,12); p.sample("s","fx",1,12);
  EXPECT_FALSE(p.permitted(12,10));
  p.sample("s","fx",2,13); EXPECT_FALSE(p.permitted(13,10));
}
TEST(AcquisitionMotionPermission, ManagerSilenceAndShutdownDeny) {
  AcquisitionMotionPermission p({"fx"});
  p.manager("s",true,false,1); p.sample("s","fx",1,1);
  EXPECT_TRUE(p.permitted(1,10)); p.sample("s","fx",2,12);
  EXPECT_FALSE(p.permitted(12,10));
  p.manager("new",true,false,20); p.sample("new","fx",1,20);
  EXPECT_TRUE(p.permitted(20,10)); p.manager("new",false,false,21);
  EXPECT_FALSE(p.permitted(21,10));
}
TEST(AcquisitionMotionPermission, DarkSamplesAndOldSessionDoNotAuthorizeScene) {
  AcquisitionMotionPermission p({"fx"});
  p.manager("s",false,false,1); p.sample("s","fx",600,1);
  p.manager("s",true,false,2); p.sample("old","fx",999,2);
  EXPECT_FALSE(p.permitted(2,10)); p.sample("s","fx",1,2);
  EXPECT_TRUE(p.permitted(2,10));
}
TEST(AcquisitionMotionPermission, StartupFaultAndManagerWarningLatch) {
  AcquisitionMotionPermission p({"fx"});
  p.manager("s",false,false,1); p.fault("s","startup fault");
  p.manager("s",true,false,2); p.sample("s","fx",1,2);
  EXPECT_FALSE(p.permitted(2,10));
  p.manager("new",true,true,3); p.sample("new","fx",1,3);
  EXPECT_FALSE(p.permitted(3,10));
}
