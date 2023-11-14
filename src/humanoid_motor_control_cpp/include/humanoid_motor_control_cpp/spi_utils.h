#pragma once

#include <cstdint>
#include <string>

namespace pi
{
    int32_t spi_open(const std::string& spi_dev, uint32_t speed, uint16_t delay, uint8_t bits_per_word, uint8_t mode);
    void    spi_close(int32_t spi_fd);
    int32_t spi_send(int32_t spi_fd, uint8_t* send, uint8_t* recv, size_t size, uint32_t speed, uint16_t delay, uint8_t bits_per_word);
} // namespace pi
