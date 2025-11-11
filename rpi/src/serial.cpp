#include "serial.h"

#include <cerrno>
#include <cstdint>
#include <cstddef>
#include <mutex>
#include <map>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

struct Serial
{
    Serial() : fd(-100500) {}

    std::mutex mtx;
    int fd;
};

static std::map<int, Serial> ports;

static bool configure_port_115200_8N1(int fd)
{
    termios tio{};
    if (tcgetattr(fd, &tio) != 0)
        return false;

    cfmakeraw(&tio);

    // 115200 baud
    if (cfsetispeed(&tio, B115200) != 0)
        return false;

    // 8N1, no flow control
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag |= CREAD | CLOCAL;

    // Blocking reads; return as soon as at least 1 byte is available
    tio.c_cc[VMIN]  = 1;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) return false;

    tcflush(fd, TCIOFLUSH);
    return true;
}

int Serial_Init(int dev, const char* const path)
{
    if (!path)
        return -10;

    ports[dev].fd = ::open(path, O_RDWR | O_NOCTTY);
    if (ports[dev].fd < 0)
        return -20;

    // Ensure blocking mode
    int flags = fcntl(ports[dev].fd, F_GETFL, 0);
    if (flags >= 0)
        fcntl(ports[dev].fd, F_SETFL, flags & ~O_NONBLOCK);

    if (!configure_port_115200_8N1(ports[dev].fd))
    {
        ::close(ports[dev].fd);
        return -30;
    }

    return 0;
}

size_t Serial_Read(int dev, uint8_t* buff, size_t count)
{
    if (ports[dev].fd < 0 || !buff || count == 0)
        return static_cast<size_t>(-1);

    size_t got = 0;
    while (got < count)
    {
        ssize_t n = ::read(ports[dev].fd, buff + got, count - got);
        if (n > 0)
        {
            got += static_cast<size_t>(n);
        }
        else if (n == 0)
        {
            // EOF on tty is unusual, but handle gracefully
            return 0;
        } else
        {
            if (errno == EINTR)
                continue; // retry
            return static_cast<size_t>(-1);
        }
    }

    return got;
}

size_t Serial_Write(int dev, const uint8_t* buff, size_t count)
{
    if (ports[dev].fd < 0 || !buff || count == 0)
        return static_cast<size_t>(-1);

    std::lock_guard<std::mutex> lk(ports[dev].mtx);

    size_t sent = 0;
    while (sent < count)
    {
        ssize_t n = ::write(ports[dev].fd, buff + sent, count - sent);
        if (n > 0)
        {
            sent += static_cast<size_t>(n);
        }
        else if (n < 0 && errno == EINTR)
        {
            continue; // interrupted, try again
        }
        else
        {
            return static_cast<size_t>(-1);
        }
    }

    return sent;
}