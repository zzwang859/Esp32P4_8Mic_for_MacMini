#include "audio_diagnostics.h"
#include <sys/reent.h>
#include <sys/types.h>
#include <unistd.h>
#include "sdkconfig.h"
#if CONFIG_ESP_CONSOLE_UART
#include "driver/uart.h"
#endif

// Linker wrapping covers libc printf/puts/fwrite/write from SDK libraries as
// well as application code. Other file descriptors retain normal I/O behavior.
ssize_t __real__write_r(struct _reent *r, int fd, const void *data, size_t size);
ssize_t __wrap__write_r(struct _reent *r, int fd, const void *data, size_t size)
{
    if ((fd == STDOUT_FILENO || fd == STDERR_FILENO) &&
        (!audio_diagnostics_logs_enabled() || audio_diagnostics_uart_suspended())) return (ssize_t)size;
    return __real__write_r(r, fd, data, size);
}

void audio_diagnostics_write_kws_line(const char *line, size_t length)
{
    if (audio_diagnostics_uart_suspended()) return;
#if CONFIG_ESP_CONSOLE_UART
    // The UART VFS writes one byte at a time, taking driver semaphores for
    // every character. Submit each debug line in one driver call instead;
    // FIFO backpressure blocks this worker without consuming inference CPU.
    if (uart_is_driver_installed(CONFIG_ESP_CONSOLE_UART_NUM)) {
#if CONFIG_LIBC_STDOUT_LINE_ENDING_CRLF || CONFIG_LIBC_STDOUT_LINE_ENDING_CR
        // Match stdout's configured line endings without per-byte VFS I/O.
        char buffer[512];
        while (length) {
            size_t used = 0;
            while (length && used + 2 <= sizeof(buffer)) {
                const char ch = *line++;
                --length;
                if (ch == '\n') {
                    buffer[used++] = '\r';
#if CONFIG_LIBC_STDOUT_LINE_ENDING_CRLF
                    buffer[used++] = '\n';
#endif
                } else buffer[used++] = ch;
            }
            uart_write_bytes(CONFIG_ESP_CONSOLE_UART_NUM, buffer, used);
        }
#else
        uart_write_bytes(CONFIG_ESP_CONSOLE_UART_NUM, line, length);
#endif
        return;
    }
#endif
    // Avoid shared stdio buffers: a partial SDK printf must never piggyback
    // on an allowed KWS result/trace. Called by the UART worker or console/control
    // tasks for explicit command acknowledgments, never an audio task.
    size_t sent = 0;
    while (sent < length) {
        const ssize_t wrote = __real__write_r(__getreent(), STDOUT_FILENO,
                                               line + sent, length - sent);
        if (wrote <= 0) break;
        sent += (size_t)wrote;
    }
}
