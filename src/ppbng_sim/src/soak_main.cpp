#include "ppbng_sim/integration_scenario.hpp"

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
std::uint64_t parse_duration(const int argc, char ** argv)
{
  if (argc == 1) {
    return 7200;
  }
  if (argc != 2) {
    throw std::invalid_argument("usage: ppbng_sim_soak [virtual_seconds]");
  }
  std::size_t consumed = 0;
  const std::string input{argv[1]};
  const auto seconds = std::stoull(input, &consumed, 10);
  if (consumed != input.size() || seconds < 3601) {
    throw std::invalid_argument("virtual_seconds must be an integer >= 3601");
  }
  return seconds;
}

bool expected_counts(const ppbng_sim::IntegrationScenario & scenario, const std::uint64_t seconds)
{
  using ppbng_sim::SimDevice;
  return scenario.trigger_count(SimDevice::fx10e) == seconds * 120U &&
         scenario.trigger_count(SimDevice::swir) == seconds * 80U &&
         scenario.trigger_count(SimDevice::rgb) == seconds * 2U &&
         scenario.trigger_count(SimDevice::thermal) == seconds * 2U &&
         scenario.produced_count(SimDevice::fx10e) == (seconds - 1U) * 120U &&
         scenario.produced_count(SimDevice::swir) == seconds * 80U &&
         scenario.produced_count(SimDevice::rgb) == seconds * 2U &&
         scenario.produced_count(SimDevice::thermal) == (seconds - 10U) * 2U;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto seconds = parse_duration(argc, argv);
    ppbng_sim::IntegrationScenario scenario;
    if (!scenario.start_and_preflight() || !scenario.collect_dark_and_wait_for_pps() ||
      !scenario.start_on_next_pps())
    {
      std::cerr << "simulation startup failed\n";
      return 2;
    }

    for (std::uint64_t second = 0; second < seconds; ++second) {
      if (second == 600) {
        scenario.set_thermal_disconnected(true);
      }
      if (second == 610 && !scenario.recover_thermal()) {
        std::cerr << "thermal recovery failed\n";
        return 3;
      }
      if (second == 1800) {
        scenario.disconnect_hsi_on_next_line(ppbng_hsi::CameraKind::fx10e);
      }
      if (second == 1801 && !scenario.recover_hsi(ppbng_hsi::CameraKind::fx10e)) {
        std::cerr << "FX10e recovery failed\n";
        return 4;
      }
      if (second == 3000) {
        scenario.set_rtk_fixed(false);
      }
      if (second == 3030) {
        scenario.set_rtk_fixed(true);
      }
      scenario.run_one_second();
    }

    const bool counts_ok = expected_counts(scenario, seconds);
    const bool finalized = scenario.normal_stop_and_finalize();
    const bool segments_ok =
      scenario.segment_index(ppbng_sim::SimDevice::fx10e).records().size() == 2U &&
      scenario.segment_index(ppbng_sim::SimDevice::swir).records().size() == 1U &&
      scenario.segment_index(ppbng_sim::SimDevice::rgb).records().size() == 1U &&
      scenario.segment_index(ppbng_sim::SimDevice::thermal).records().size() == 2U;

    std::cout << "{\"virtual_seconds\":" << seconds <<
      ",\"trigger_events\":" << scenario.observations().size() <<
      ",\"counts_ok\":" << (counts_ok ? "true" : "false") <<
      ",\"segments_ok\":" << (segments_ok ? "true" : "false") <<
      ",\"finalized\":" << (finalized ? "true" : "false") << "}\n";
    return counts_ok && segments_ok && finalized ? 0 : 5;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
