#include "humanoid_motor_control_cpp/spi_utils.h"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace pi
{
    int32_t spi_open(const std::string& spi_dev, uint32_t speed, uint16_t delay, uint8_t bits_per_word, uint8_t mode)
    {
        //使用SPI接口
        int32_t spi_fd = open(spi_dev.c_str(), O_RDWR);
        if (spi_fd < 0)
        {
            perror("Error: Cann't open SPI Dev.\n");
            return -1;
        }

        int ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
        if (ret == -1)
        {
            perror("Error: SPI_IOC_WR_MODE fault.\n");
            return -1;
        }

        ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
        if (ret == -1)
        {
            perror("Error: SPI_IOC_WR_BITS fault.\n");
            return -1;
        }

        ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
        if (ret == -1)
        {
            perror("Error: SPI_IOC_WR_MAX_SPEED fault.\n");
            return -1;
        }

        return spi_fd;
    }

    void spi_close(int32_t spi_fd) { close(spi_fd); }

    int32_t spi_send(int32_t spi_fd, uint8_t* send, uint8_t* recv, size_t size, uint32_t speed, uint16_t delay, uint8_t bits_per_word)
    {
        struct spi_ioc_transfer spi {};
        
        spi.tx_buf           = (unsigned long)send;
        spi.rx_buf           = (unsigned long)recv;
        spi.len              = size;
        spi.delay_usecs      = delay;
        spi.speed_hz         = speed;
        spi.bits_per_word    = bits_per_word;
        // Send wr_addr
        int32_t ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi);
        return ret;
    }
} // namespace pi
