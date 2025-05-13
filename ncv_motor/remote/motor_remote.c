#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/tcp.h>

#define SERVER_IP   "10.0.0.2"
#define SERVER_PORT 5000

static void print_usage(char **argv)
{
  printf("Usage: %s <pwm duty>\n", argv[0]);
  printf("  <pwm duty> is duty percentage in ragne [-100, 100]\n");
  printf("  positive values result in CCW motion, negative in CW motion\n");
}

int main(int argc, char **argv)
{
  int sock;
  struct sockaddr_in server_addr;
  int8_t motor_command;
  int boolean_true = 1;
  int err;
  ssize_t sent;

  if (argc == 2)
    {
      long input;
      char *endchar;
      input = strtol(argv[1], &endchar, 10);
      if (*endchar == '\0') /* OK */
        {
          if (-100 > input || input > 100) /* check range */
            {
              printf("Out of range\n");
              print_usage(argv);
              return 1;
            }
          motor_command = input; /* OK */
        }
      else /* invalid */
        {
          printf("Wrong input\n");
          print_usage(argv);
          return 1;
        }
    }
  else if (argc == 1)
    {
      motor_command = 0;
    }
  else
    {
      printf("Wrong usage\n");
      print_usage(argv);
      return 1;
    }

  /* Create TCP socket */

  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0)
    {
      perror("Socket creation failed");
      return 1;
    }

  /* Disable Nagle's */

  err = setsockopt(sock,
                   IPPROTO_TCP, TCP_NODELAY,
                   (char *)&boolean_true, sizeof(int));
  if (err < 0)
    {
      perror("setsockopt(TCP_NODELAY) failed");
      close(sock);
      return 1;
    }

  /* Server address configuration */

  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(SERVER_PORT);
  err = inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);
  if (err <= 0)
    {
      perror("Invalid address or address not supported");
      close(sock);
      return 1;
    }

  /* Connect to server */

  printf("Connecting\n");
  err = connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if (err < 0)
    {
      perror("Connection failed");
      close(sock);
      return 1;
    }

  /* Send */

  do
    {
      char input[5];
      char *endchar;
      long inint;
      printf("  Sending %d\n", motor_command);
      sent = send(sock, &motor_command, sizeof(motor_command), 0);
      if (sent != sizeof(motor_command))
        {
          perror("Failed to send all bytes");
        }

      printf("  Sent command: %d\n", motor_command);
      printf("Enter command: ");
      fflush(stdout);

      scanf("%4s", input);
      if (input[0] == 'q')
        {
          break;
        }
      inint = strtol(input, &endchar, 10);
      if (*endchar == '\0') /* OK */
        {
          if (-100 > inint || inint > 100) /* check range */
            {
              printf("Out of range\n");
              continue;
            }
          motor_command = inint; /* OK */
        }
      else /* invalid */
        {
          printf("Wrong input\n");
          continue;
        }
    } while (true);

  sleep(1);

  close(sock);
  return 0;
}
