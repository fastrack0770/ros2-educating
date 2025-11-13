#pragma once

#include <exception>

struct Nullopt_t
{
    constexpr explicit Nullopt_t(int)
    {
    }
};

inline constexpr Nullopt_t Nullopt{0};

class BadOptionalAccess : public std::exception
{
  public:
    const char *what() const noexcept override
    {
        return "Bad Optional Access";
    }
};

/**
 * Optional
 * Similar to the std::optional from c++17 standard
 */
template <typename T> class Optional
{
  public:
    constexpr Optional() : _has(false), _value()
    {
    }
    constexpr Optional(Nullopt_t) : _has(false), _value()
    {
    }
    constexpr Optional(const T &value) : _has(true), _value(value)
    {
    }
    constexpr Optional(T &&value) : _has(true), _value(std::move(value))
    {
    }
    constexpr Optional(const Optional<T> &value) : _has(value._has), _value(value._value)
    {
    }
    constexpr Optional(Optional<T> &&value) : _has(value._has), _value(std::move(value._value))
    {
        value._has = false;
    }

    constexpr T &operator*()
    {
        return _value;
    }

    constexpr const T &operator*() const
    {
        return _value;
    }

    constexpr T *operator->()
    {
        return &_value;
    }

    constexpr const T *operator->() const
    {
        return &_value;
    }

    constexpr bool has_value() const noexcept
    {
        return _has;
    }

    constexpr T &value()
    {
        if (not _has)
        {
            throw BadOptionalAccess();
        }

        return _value;
    }
    constexpr const T &value() const
    {
        if (not _has)
        {
            throw BadOptionalAccess();
        }

        return _value;
    }

    Optional &operator=(const T &rhv)
    {
        _value = rhv;
        _has = true;
        return *this;
    }

    constexpr Optional &operator=(const Optional &rhv)
    {
        _value = rhv._value;
        _has = rhv._has;
        return *this;
    }

    void reset()
    {
        if (not _has)
        {
            return;
        }

        _has = false;

        if constexpr (not std::is_trivially_destructible_v<T> and std::is_nothrow_destructible_v<T>)
        {
            _value.~T();
        }
    }

    friend std::ostream &operator<<(std::ostream &os, const Optional &rhv)
    {
        if (not rhv.has_value())
        {
            return os << "{ No data }";
        }

        return os << *rhv;
    }

  private:
    bool _has = false;
    T _value;
};
