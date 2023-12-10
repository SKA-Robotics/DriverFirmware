#include "output.h"
#include <errno.h>
#include <sys/unistd.h>

int _write(int file, char* data, int len) {
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    }
    HAL_StatusTypeDef status =
        HAL_UART_Transmit(&huart2, (uint8_t*)data, len, 1000);
    return (status == HAL_OK ? len : 0);
}