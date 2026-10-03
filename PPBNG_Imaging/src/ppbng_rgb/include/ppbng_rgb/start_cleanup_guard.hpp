#pragma once
#include <functional>
#include <utility>
namespace ppbng_rgb {class StartCleanupGuard{public:explicit StartCleanupGuard(std::function<void()>f):f_(std::move(f)){}~StartCleanupGuard(){if(armed_)f_();}StartCleanupGuard(const StartCleanupGuard&)=delete;StartCleanupGuard&operator=(const StartCleanupGuard&)=delete;void commit()noexcept{armed_=false;}private:std::function<void()>f_;bool armed_{true};};}
