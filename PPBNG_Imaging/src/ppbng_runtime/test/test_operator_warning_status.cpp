#include <gtest/gtest.h>
#include "ppbng_runtime/operator_warning_status.hpp"

TEST(OperatorWarningStatus, EmptyStatusIsExplicit)
{
  EXPECT_EQ(ppbng_runtime::operator_warning_status({}),
    ";operator_warning_count=0;operator_warnings_hex=;");
}

TEST(OperatorWarningStatus, DelimitersCannotInjectStatusFields)
{
  EXPECT_EQ(ppbng_runtime::operator_warning_status({"A;=\n", "B"}),
    ";operator_warning_count=2;operator_warnings_hex=413b3d0a0a42;");
}

TEST(OperatorWarningStatus, NonGlobalFaultIsNotLost)
{
  const auto value = ppbng_runtime::operator_warning_status({"fx10e: overflow"});
  EXPECT_NE(value.find("operator_warning_count=1;"), std::string::npos);
  EXPECT_NE(value.find("66783130653a206f766572666c6f77"), std::string::npos);
}
