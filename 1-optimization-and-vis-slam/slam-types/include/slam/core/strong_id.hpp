/**
 * @file strong_id.hpp
 * @brief Provides a strongly-typed ID wrapper using the Phantom Type parameter pattern.
 *
 * This header defines the `StrongId` template class, which is used to create
 * type-safe identifiers (e.g., PoseId, LandmarkId) that cannot be implicitly
 * mixed, preventing logic errors in complex graph operations.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>

namespace slam {
namespace core {

/**
 * @brief A strongly typed wrapper for identifier values.
 *
 * @tparam Tag An empty struct used solely to differentiate types at compile time.
 * @tparam T The underlying integral type (defaults to uint64_t).
 */
template <typename Tag, typename T = uint64_t>
class StrongId {
public:
    /// Expose the underlying integral type.
    using ValueType = T;

    /**
     * @brief Default constructor. Initializes the ID to an invalid state.
     */
    constexpr StrongId() noexcept : value_(invalid_value()) {}

    /**
     * @brief Explicit constructor from an underlying value.
     * @param value The raw identifier value.
     * @note Explicit keyword prevents accidental conversions from raw integers.
     */
    constexpr explicit StrongId(T value) noexcept : value_(value) {}

    // Trivial copy and move semantics
    constexpr StrongId(const StrongId&) noexcept = default;
    constexpr StrongId(StrongId&&) noexcept = default;
    constexpr StrongId& operator=(const StrongId&) noexcept = default;
    constexpr StrongId& operator=(StrongId&&) noexcept = default;

    /**
     * @brief Retrieves the underlying raw value.
     * @return The raw identifier value.
     */
    constexpr T value() const noexcept { return value_; }

    /**
     * @brief Checks if the ID represents a valid entity.
     * @return True if valid, false if uninitialized/invalid.
     */
    constexpr bool isValid() const noexcept { return value_ != invalid_value(); }

    /**
     * @brief Returns a canonical Invalid ID for this type.
     * @return A StrongId initialized to the invalid state.
     */
    static constexpr StrongId Invalid() noexcept { return StrongId(invalid_value()); }

    // --- Relational Operators ---

    constexpr bool operator==(const StrongId& other) const noexcept { return value_ == other.value_; }
    constexpr bool operator!=(const StrongId& other) const noexcept { return value_ != other.value_; }
    constexpr bool operator<(const StrongId& other) const noexcept  { return value_ < other.value_; }
    constexpr bool operator>(const StrongId& other) const noexcept  { return value_ > other.value_; }
    constexpr bool operator<=(const StrongId& other) const noexcept { return value_ <= other.value_; }
    constexpr bool operator>=(const StrongId& other) const noexcept { return value_ >= other.value_; }

    /**
     * @brief Stream output operator for logging and debugging.
     */
    friend std::ostream& operator<<(std::ostream& os, const StrongId& id) {
        if (id.isValid()) {
            os << id.value_;
        } else {
            os << "INVALID";
        }
        return os;
    }

private:
    T value_;

    /**
     * @brief Defines the sentinel value representing an invalid state.
     * @return The maximum possible value for type T.
     */
    static constexpr T invalid_value() noexcept {
        return std::numeric_limits<T>::max();
    }
};

} // namespace core
} // namespace slam

// --- Hash Specialization ---
namespace std {

/**
 * @brief Specialization of std::hash for slam::core::StrongId.
 *
 * Allows StrongId to be used seamlessly as keys in unordered containers.
 */
template <typename Tag, typename T>
struct hash<slam::core::StrongId<Tag, T>> {
    size_t operator()(const slam::core::StrongId<Tag, T>& id) const noexcept {
        return std::hash<T>{}(id.value());
    }
};

} // namespace std
