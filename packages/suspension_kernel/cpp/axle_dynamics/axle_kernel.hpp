#pragma once

#include <cstddef>
#include <cstdint>
#include "mb_model/enums.hpp"

#if defined(_WIN32)
#define AXLE_API __declspec(dllexport)
#else
#define AXLE_API __attribute__((visibility("default")))
#endif

// The input/output payloads live in a low level module so the solver layers do
// not have to include this public ABI header merely to name them.
#include "mb_input/types.hpp"

extern "C" {

AXLE_API int axle_kernel_abi_version();

AXLE_API int vehicle_kernel_abi_version();

// The contract boundary.  Both payloads are containers in the `mb_contract`
// wire format; the kernel parses, runs and returns a result container of the
// same format.  `result_length_in_out` carries the capacity in and the length
// out, and a return of 11 means the buffer was too small and the required size
// has been written there instead.
//
// This is the entry point new callers are meant to grow against: a new case
// family or element changes the documents, not this signature.
AXLE_API int32_t suspension_kernel_contract_version();

/// Serialise the kernel's capability declaration.
///
/// The authoring layer has to refuse a tire that requests a PAC2002 feature this
/// kernel does not implement, and it can only do that if the kernel tells it the
/// scope.  One canonical JSON document, `*written` bytes: the same protocol as
/// `suspension_kernel_run` (0 written, 11 for "grow the buffer",
/// `*written` always the size needed).
AXLE_API int32_t suspension_kernel_capabilities(
    char* buffer, size_t capacity, size_t* written);

AXLE_API int32_t suspension_kernel_run(
    const std::uint8_t* model_payload,
    std::size_t model_length,
    const std::uint8_t* case_payload,
    std::size_t case_length,
    std::uint8_t* result_out,
    std::size_t* result_length_in_out,
    char* error_buffer,
    std::size_t error_capacity);

}



