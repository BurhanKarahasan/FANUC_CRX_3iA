// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <optional>
#include <string>

namespace rmi
{

template <typename T>
std::string ToJSON(const T& data);

template <typename T>
std::optional<T> FromJSON(const std::string& json);

}  // namespace rmi
