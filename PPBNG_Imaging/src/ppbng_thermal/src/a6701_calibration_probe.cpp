#include "ppbng_thermal/spinnaker_a6701_backend.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: ppbng_a6701_calibration_probe <exact-device-id>\n";
    return 2;
  }
  using namespace ppbng_thermal;
  StopSource stop;
  SpinnakerA6701Backend backend({argv[1], "A6701"});
  auto fail = [&backend](const char * stage, const Status & status) {
    std::cerr << stage << " failed: " << status.detail << '\n';
    backend.stop(std::chrono::milliseconds(1000));
    return 1;
  };
  std::vector<std::string> identities;
  auto status = backend.discover(std::chrono::milliseconds(5000), stop.token(), identities);
  if (!status.ok()) return fail("discover", status);
  if (std::count(identities.begin(), identities.end(), argv[1]) != 1) {
    return fail("identity", {ErrorCode::not_found, "exact device was not discovered exactly once"});
  }
  status = backend.open(argv[1], std::chrono::milliseconds(5000), stop.token());
  if (!status.ok()) return fail("open", status);
  ThermalCalibrationSnapshot snapshot;
  status = backend.calibration_snapshot(std::chrono::milliseconds(5000), stop.token(), snapshot);
  if (!status.ok()) return fail("snapshot", status);
  for (const auto & value : snapshot.values) {
    std::cout << value.key << '=' << (value.available ? value.value : "UNAVAILABLE") << '\n';
  }
  const auto validation = validate_a6701_calibration_snapshot(snapshot);
  std::cout << "snapshot_hash=" << calibration_snapshot_hash(snapshot) << '\n';
  std::cout << "validation=" << (validation.ok() ? "PASS" : "FAIL") << '\n';
  std::cout << "validation_detail=" <<
    (validation.ok() ? "complete and internally consistent" : validation.detail) << '\n';
  backend.stop(std::chrono::milliseconds(1000));
  return validation.ok() ? 0 : 3;
}
