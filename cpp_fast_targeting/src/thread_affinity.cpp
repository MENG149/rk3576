#include "rkfast/thread_affinity.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <pthread.h>
#include <sched.h>

#include <sstream>
#include <vector>

namespace rkfast {

namespace {

std::vector<int> ParseCpuSpec(const std::string& spec) {
  std::vector<int> cpus;
  std::stringstream ss(spec);
  std::string token;
  while (std::getline(ss, token, ',')) {
    token.erase(std::remove_if(token.begin(), token.end(),
                               [](unsigned char ch) { return std::isspace(ch) != 0; }),
                token.end());
    if (token.empty()) {
      continue;
    }
    const auto dash = token.find('-');
    if (dash == std::string::npos) {
      cpus.push_back(std::stoi(token));
      continue;
    }
    const int begin = std::stoi(token.substr(0, dash));
    const int end = std::stoi(token.substr(dash + 1));
    if (begin <= end) {
      for (int cpu = begin; cpu <= end; ++cpu) {
        cpus.push_back(cpu);
      }
    } else {
      for (int cpu = begin; cpu >= end; --cpu) {
        cpus.push_back(cpu);
      }
    }
  }
  return cpus;
}

}  // namespace

bool BindCurrentThread(const std::string& cpu_spec, std::string* error) {
  if (cpu_spec.empty()) {
    return true;
  }

  const std::vector<int> cpus = ParseCpuSpec(cpu_spec);
  if (cpus.empty()) {
    if (error != nullptr) {
      *error = "empty cpu affinity spec: " + cpu_spec;
    }
    return false;
  }

  cpu_set_t set;
  CPU_ZERO(&set);
  for (const int cpu : cpus) {
    if (cpu >= 0) {
      CPU_SET(cpu, &set);
    }
  }

#ifdef __linux__
  const int ret = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
  if (ret != 0 && error != nullptr) {
    *error = "pthread_setaffinity_np failed: " + std::string(std::strerror(ret));
  }
  return ret == 0;
#else
  (void)set;
  if (error != nullptr) {
    *error = "thread affinity is only supported on Linux";
  }
  return false;
#endif
}

}  // namespace rkfast
