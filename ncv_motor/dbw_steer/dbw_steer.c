#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/sensors/qencoder.h>
#include <sys/ioctl.h>

#define QENCODER_PATH "/dev/qe0"

#define MEASURE_PERIOD_US 10000

#define TCP_PORT 5000

int main(int argc, char *argv[])
{
  int fd;

  fd = open(QENCODER_PATH, O_RDONLY);
  if (fd < 0)
    {
      printf("dbw_steer: open %s failed: %d\n", QENCODER_PATH, errno);
      return errno;
    }

  const int tcp_socket = socket(AF_INET, SOCK_STREAM, 0);

  struct sockaddr_in saddr;
  memset(&saddr, 0, sizeof(struct sockaddr_in));
  saddr.sin_family = AF_INET;
  saddr.sin_addr.s_addr = INADDR_ANY;
  saddr.sin_port = htons(TCP_PORT);

  bind(tcp_socket, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in));

  printf("Listenning for connections on port %d...", TCP_PORT);
  fflush(stdout);
  listen(tcp_socket, 1);
  int tcp_conn_socket = accept(tcp_socket, NULL, NULL);
  printf("connected\n");
  bool bool_true = 1;
  setsockopt(tcp_conn_socket, SOL_SOCKET, SO_KEEPALIVE, &bool_true, sizeof(bool_true));
  setsockopt(tcp_conn_socket, IPPROTO_TCP, TCP_NODELAY, &bool_true, sizeof(bool_true));

  while (1)
    {
      int32_t position;
      int ret;

      ret = ioctl(fd, QEIOC_POSITION, (unsigned long)((uintptr_t) &position));
      if (ret < 0)
        {
          printf("dbw_steer: ioctl position read failed: %d\n", errno);
          return errno;
        }

      ret = write(tcp_conn_socket, &position, sizeof(position));
      if (ret < 0)
        {
          printf("dbw_steer: send failed: %d\n", errno);
          return errno;
        }

      printf("Pos: %ld\n", position);
      usleep(MEASURE_PERIOD_US);
    }

  while(1)
    {
      int8_t incoming;
      errno = 0;
      int num_read = read(tcp_conn_socket, &incoming, sizeof(incoming));
      if (num_read <= 0)
        {
          if (num_read < 0)
            {
              printf("Error during reading, destroying\n");
            }
          else
            {
              printf("Connection ended, destroying\n");
            }
          break;
        }

      printf("incoming: %d\n", incoming);
      fflush(stdout);

    }

  close(tcp_conn_socket);
  close(tcp_socket);

  return 0;
}
