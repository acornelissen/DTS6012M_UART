#pragma once

#include <cstdint>
#include <cstddef>
#include <chrono>
#include <thread>
#include <iostream>
#include <cstring>

using byte = uint8_t;

#ifndef HIGH
#define HIGH 0x1
#endif

#ifndef LOW
#define LOW 0x0
#endif

inline unsigned long millis()
{
  using clock = std::chrono::steady_clock;
  static const auto start = clock::now();
  return static_cast<unsigned long>(std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - start).count());
}

inline void delay(unsigned long ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

class Stream
{
public:
  virtual ~Stream() = default;
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
  virtual void flush() {}
  virtual size_t write(uint8_t) = 0;

  virtual size_t write(const uint8_t *buffer, size_t size)
  {
    size_t written = 0;
    for (size_t i = 0; i < size; ++i)
    {
      written += write(buffer[i]);
    }
    return written;
  }

  size_t print(const char *str)
  {
    std::cout << (str ? str : "");
    return str ? strlen(str) : 0;
  }

  size_t print(int value)
  {
    std::cout << value;
    return 0;
  }

  size_t print(unsigned long value)
  {
    std::cout << value;
    return 0;
  }

  size_t println()
  {
    std::cout << std::endl;
    return 1;
  }

  size_t println(const char *str)
  {
    print(str);
    std::cout << std::endl;
    return (str ? strlen(str) : 0) + 1;
  }

  size_t println(int value)
  {
    print(value);
    std::cout << std::endl;
    return 0;
  }
};

class SerialStub : public Stream
{
public:
  void begin(unsigned long) {}
  operator bool() const { return true; }

  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  size_t write(uint8_t b) override
  {
    std::cout << static_cast<char>(b);
    return 1;
  }
};

inline SerialStub Serial;
