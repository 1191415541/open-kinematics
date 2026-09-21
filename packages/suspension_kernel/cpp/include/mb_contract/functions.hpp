#pragma once

/// The free functions of the `mb_contract` module.
///
/// This module owns the wire format between the Python authoring layer and the
/// kernel: the canonical JSON form, the container layout, and the
/// compile-time registries that let the solver talk to element / joint / tire
/// / case implementations without knowing them.

#include "mb_config/prelude.hpp"
#include "mb_contract/types.hpp"

namespace axle_kernel {

// --- canonical JSON -------------------------------------------------------
/// Parse ordinary JSON (whitespace tolerated) into `out`.
bool contract_parse_json(const std::string& text, JsonValue& out, std::string& error);

/// Serialise `value` back into the canonical form.
bool contract_write_canonical(const JsonValue& value, std::string& out,
                              std::string& error);

/// Parse then re-serialise; succeeds only when the input already was canonical.
bool contract_verify_canonical(const std::string& text, std::string& error);

// --- identity -------------------------------------------------------------
/// Lowercase hex SHA-256 of exactly the bytes given.
std::string contract_sha256_hex(const std::string& bytes);

// --- container ------------------------------------------------------------
/// Split a container payload into its document and blob section.
bool contract_parse_container(const std::uint8_t* payload, std::size_t size,
                              ContractPayload& out, std::string& error);

/// Build a container payload from a canonical document text and a blob.
std::string contract_build_container(const std::string& canonical_json,
                                     const std::string& blob);

/// The container magic the kernel accepts.
extern const char kContractMagic[4];
extern const std::uint32_t kContractVersion;

// --- registries -----------------------------------------------------------
/// Row count of a joint type, or -1 when the name is unknown.
int contract_joint_rows(const std::string& name);

/// Whether an element type name is known.
bool contract_element_known(const std::string& name);

/// Whether a tire model name is known.
bool contract_tire_known(const std::string& name);

/// Whether a case family name is known.
bool contract_case_family_known(const std::string& name);

/// Number of registered entries in each table (used by the self-test).
int contract_registry_size(int table);

}  // namespace axle_kernel