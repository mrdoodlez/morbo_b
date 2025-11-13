#include "tcp_server.h"

#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <iostream>

// POSIX networking
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

#include "logger.h"

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

static std::atomic<bool> g_running{false};

static int g_listen_fd = -1;
// Active connection FD, or -1 if none
static std::atomic<int> g_conn_fd{-1};

static std::mutex g_conn_mtx;
static std::condition_variable g_conn_cv;

static std::mutex g_write_mtx;

static std::thread g_accept_thread;

static void acceptLoop(uint16_t port)
{
    // Create listening socket
    g_listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (g_listen_fd < 0)
    {
        std::perror("P2pLink socket");
        g_running = false;
        return;
    }

    int opt = 1;
    if (setsockopt(g_listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
    {
        std::perror("P2pLink setsockopt");
        ::close(g_listen_fd);
        g_listen_fd = -1;
        g_running = false;
        return;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(g_listen_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        std::perror("P2pLink bind");
        ::close(g_listen_fd);
        g_listen_fd = -1;
        g_running = false;
        return;
    }

    if (listen(g_listen_fd, 1) < 0)
    {
        std::perror("P2pLink listen");
        ::close(g_listen_fd);
        g_listen_fd = -1;
        g_running = false;
        return;
    }

    vlog.text << "[P2P] listening on port " << port << "\n";

    while (g_running)
    {
        sockaddr_in cli_addr{};
        socklen_t cli_len = sizeof(cli_addr);

        int fd = ::accept(g_listen_fd, reinterpret_cast<sockaddr *>(&cli_addr), &cli_len);
        if (fd < 0)
        {
            if (!g_running)
                break;
            continue; // transient error
        }

        char ip_str[64];
        inet_ntop(AF_INET, &cli_addr.sin_addr, ip_str, sizeof(ip_str));
        vlog.text << "[P2P] new client " << ip_str
                    << ":" << ntohs(cli_addr.sin_port)
                    << " fd=" << fd << "\n";

        // Replace current connection (if any)
        int old_fd = g_conn_fd.exchange(fd);
        if (old_fd >= 0)
        {
            ::shutdown(old_fd, SHUT_RDWR);
            ::close(old_fd);
        }

        // Notify waiting readers that a connection exists
        {
            std::lock_guard<std::mutex> lk(g_conn_mtx);
            // nothing else to store; conn_fd already updated
        }
        g_conn_cv.notify_all();
    }

    // Cleanup listen socket
    if (g_listen_fd >= 0)
    {
        ::shutdown(g_listen_fd, SHUT_RDWR);
        ::close(g_listen_fd);
        g_listen_fd = -1;
    }

    vlog.text << "[P2P] accept thread exit\n";
}

int P2pLink_Init(uint16_t port)
{
    g_conn_fd.store(-1);

    try
    {
        g_accept_thread = std::thread(acceptLoop, port);
    }
    catch (...)
    {
        return -10;
    }

    g_running = true;

    return 0;
}

extern "C" size_t P2pLink_Write(int /*dev*/, const uint8_t* buf, size_t count)
{
    if (!g_running)
        return -1;

    std::lock_guard<std::mutex> lock(g_write_mtx);

    int fd = g_conn_fd.load();
    if (fd < 0)
        return -1; // no active connection

    const uint8_t *ptr = static_cast<const uint8_t *>(buf);
    size_t sent = 0;

    while (sent < count)
    {
        size_t n = ::send(fd, ptr + sent, count - sent, MSG_NOSIGNAL);
        if (n <= 0)
        {
            // error/broken connection
            std::perror("P2pLink send");
            int expected = fd;
            if (g_conn_fd.compare_exchange_strong(expected, -1))
            {
                ::shutdown(fd, SHUT_RDWR);
                ::close(fd);
            }
            return -1;
        }
        sent += static_cast<size_t>(n);
    }

    return static_cast<size_t>(sent);
}

extern "C" size_t P2pLink_Read(int /*dev*/, uint8_t* buf, size_t count)
{
    if (!g_running)
        return -1;

    uint8_t *ptr = static_cast<uint8_t *>(buf);

    while (g_running)
    {
        int fd = g_conn_fd.load();
        if (fd < 0)
        {
            // No connection yet: wait until we have one or we are shutting down
            std::unique_lock<std::mutex> lk(g_conn_mtx);
            g_conn_cv.wait(lk, []
                           { return !g_running || g_conn_fd.load() >= 0; });

            if (!g_running)
                return -1;

            continue; // loop, reload fd
        }

        // We have some active connection
        size_t n = ::recv(fd, ptr, count, 0);
        if (n > 0)
        {
            return n;
        }

        // Connection broken
        std::perror("P2pLink recv");
        int expected = fd;
        if (g_conn_fd.compare_exchange_strong(expected, -1))
        {
            ::shutdown(fd, SHUT_RDWR);
            ::close(fd);
        }

        // loop: will now either wait for new connection or use new one
    }

    return -1;
}
