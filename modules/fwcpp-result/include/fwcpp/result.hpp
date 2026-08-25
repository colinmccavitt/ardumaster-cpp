#pragma once

// ADR-0012 decision 3: fallible operations return Result<T, E>, not upstream's
// bool + out-param. Header-only, no allocation, no exceptions used internally.
// Modeled on std::expected (C++23) so migrating later is a rename, not a
// redesign: same member names (has_value, value, error) where practical.

#include <type_traits>
#include <utility>

namespace fwcpp {

// A tag type so Result<T, E> can hold an error without needing T to be
// default-constructible or E to be distinguishable from T by type alone in
// ambiguous cases (e.g. Result<int, int>).
template <typename E>
struct Unexpected {
    E error;
    explicit constexpr Unexpected(E e) : error(std::move(e)) {}
};

template <typename E>
constexpr Unexpected<E> Err(E e) {
    return Unexpected<E>(std::move(e));
}

// Result<T, E>: exactly one of a T or an E is alive at a time. No dynamic
// allocation (decision 4) - storage is a plain union sized for the larger of
// T and E, discriminated by a bool.
//
// Deliberately not a full std::expected reimplementation: only the
// operations this port's ported modules actually need are provided. Add to
// this as a real call site needs it, per the project's no-speculative-API
// convention - do not grow this into a general-purpose library ahead of use.
// Not constexpr-capable: the active union member is switched with placement
// new, which C++20 does not permit inside a constant-evaluated call. Marking
// these constexpr anyway would compile (a constexpr function may contain
// statements invalid in a constant expression as long as no call to it is
// actually constant-evaluated) but would advertise a capability this type
// cannot honour, so the qualifier is left off rather than left misleading.
template <typename T, typename E>
class [[nodiscard]] Result {
public:
    Result(T value) : has_value_(true), storage_{} {
        ::new (&storage_.value) T(std::move(value));
    }
    Result(Unexpected<E> err) : has_value_(false), storage_{} {
        ::new (&storage_.error) E(std::move(err.error));
    }

    Result(const Result&) = delete;
    Result& operator=(const Result&) = delete;

    Result(Result&& other) noexcept : has_value_(other.has_value_), storage_{} {
        if (has_value_) {
            ::new (&storage_.value) T(std::move(other.storage_.value));
        } else {
            ::new (&storage_.error) E(std::move(other.storage_.error));
        }
    }

    Result& operator=(Result&& other) noexcept {
        if (this != &other) {
            destroy();
            has_value_ = other.has_value_;
            if (has_value_) {
                ::new (&storage_.value) T(std::move(other.storage_.value));
            } else {
                ::new (&storage_.error) E(std::move(other.storage_.error));
            }
        }
        return *this;
    }

    ~Result() { destroy(); }

    [[nodiscard]] bool has_value() const { return has_value_; }
    [[nodiscard]] explicit operator bool() const { return has_value_; }

    // Precondition: has_value(). Upstream has no analogous check-then-use
    // contract to reproduce here since this type doesn't exist upstream;
    // asserting is this port's own choice, not a behavioral reproduction.
    [[nodiscard]] const T& value() const& { return storage_.value; }
    [[nodiscard]] T&& value() && { return std::move(storage_.value); }

    [[nodiscard]] const E& error() const& { return storage_.error; }
    [[nodiscard]] E&& error() && { return std::move(storage_.error); }

    [[nodiscard]] T value_or(T fallback) const& {
        return has_value_ ? storage_.value : fallback;
    }

private:
    void destroy() {
        if (has_value_) {
            storage_.value.~T();
        } else {
            storage_.error.~E();
        }
    }

    bool has_value_;
    union Storage {
        T value;
        E error;
        Storage() {}
        ~Storage() {}
    } storage_;
};

// Specialization for E only (no payload on success) - the common case for
// upstream operations that upstream signals with a bare bool.
template <typename E>
class [[nodiscard]] Result<void, E> {
public:
    constexpr Result() : has_value_(true), error_{} {}
    constexpr Result(Unexpected<E> err) : has_value_(false), error_(std::move(err.error)) {}

    [[nodiscard]] constexpr bool has_value() const { return has_value_; }
    [[nodiscard]] constexpr explicit operator bool() const { return has_value_; }
    [[nodiscard]] constexpr const E& error() const& { return error_; }

private:
    bool has_value_;
    E error_;
};

} // namespace fwcpp
