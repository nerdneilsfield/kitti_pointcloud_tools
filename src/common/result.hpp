#pragma once

#include <stdexcept>
#include <utility>
#include <variant>

namespace kpt {

template <class T, class E> class [[nodiscard]] Result {
public:
  Result(const T &value) : storage_(std::in_place_index<0>, value) {}
  Result(T &&value) : storage_(std::in_place_index<0>, std::move(value)) {}
  Result(const E &error) : storage_(std::in_place_index<1>, error) {}
  Result(E &&error) : storage_(std::in_place_index<1>, std::move(error)) {}

  [[nodiscard]] bool hasValue() const noexcept { return storage_.index() == 0; }

  explicit operator bool() const noexcept { return hasValue(); }

  T &value() & {
    if (!hasValue())
      throw std::logic_error("Result has no value");
    return std::get<0>(storage_);
  }

  const T &value() const & {
    if (!hasValue())
      throw std::logic_error("Result has no value");
    return std::get<0>(storage_);
  }

  T &&value() && {
    if (!hasValue())
      throw std::logic_error("Result has no value");
    return std::get<0>(std::move(storage_));
  }

  E &error() & {
    if (hasValue())
      throw std::logic_error("Result has no error");
    return std::get<1>(storage_);
  }

  const E &error() const & {
    if (hasValue())
      throw std::logic_error("Result has no error");
    return std::get<1>(storage_);
  }

  E &&error() && {
    if (hasValue())
      throw std::logic_error("Result has no error");
    return std::get<1>(std::move(storage_));
  }

private:
  std::variant<T, E> storage_;
};

template <class E> class [[nodiscard]] Result<void, E> {
public:
  Result() = default;
  Result(const E &error) : storage_(std::in_place_index<1>, error) {}
  Result(E &&error) : storage_(std::in_place_index<1>, std::move(error)) {}

  [[nodiscard]] bool hasValue() const noexcept { return storage_.index() == 0; }

  explicit operator bool() const noexcept { return hasValue(); }

  void value() const {
    if (!hasValue())
      throw std::logic_error("Result has no value");
  }

  E &error() & {
    if (hasValue())
      throw std::logic_error("Result has no error");
    return std::get<1>(storage_);
  }

  const E &error() const & {
    if (hasValue())
      throw std::logic_error("Result has no error");
    return std::get<1>(storage_);
  }

  E &&error() && {
    if (hasValue())
      throw std::logic_error("Result has no error");
    return std::get<1>(std::move(storage_));
  }

private:
  std::variant<std::monostate, E> storage_;
};

} // namespace kpt
