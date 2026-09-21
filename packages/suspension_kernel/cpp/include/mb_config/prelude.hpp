#pragma once

// The standard-library includes the kernel translation units expect, and nothing
// else.
//
// The K2 transitional aggregate carried these alongside its declarations, so
// every unit got them.  When that header was deleted (K6, MODULES.md section 4)
// they needed a home that is not a layering device: this file has no kernel
// declarations and no kernel types, so including it says nothing about which
// module a unit may depend on.  A unit that needs one of these on its own is
// welcome to include it directly; this is the floor, not the rule.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <limits>
#include <numeric>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef _OPENMP
#include <omp.h>
#endif
