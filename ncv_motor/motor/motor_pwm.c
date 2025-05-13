#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/timers/pwm.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/sensors/qencoder.h>

#include <sys/ioctl.h>

#define PWM_F 8000

#define TCP_PORT 5000

#define MOTOR_IN1_PATH "/dev/gpio0"
#define MOTOR_IN2_PATH "/dev/gpio1"
#define MOTOR_PWM_PATH "/dev/pwm0"

struct remote_motor_s
{
  int in1fd;
  int in2fd;
  int pwmfd;
  struct pwm_info_s pwminfo;
};

static int motor_init(struct remote_motor_s *motor,
               char *in1_path, char *in2_path, char *pwm_path)
{
  motor->in1fd = open(in1_path, O_RDWR);
  motor->in2fd = open(in2_path, O_RDWR);
  motor->pwmfd = open(pwm_path, O_RDWR);
  if (!(motor->in1fd && motor->in2fd && motor->pwmfd))
    {
      return -1;
    }

  ioctl(motor->in1fd, GPIOC_SETPINTYPE, (unsigned long) GPIO_OUTPUT_PIN);
  ioctl(motor->in2fd, GPIOC_SETPINTYPE, (unsigned long) GPIO_OUTPUT_PIN);

  ioctl(motor->in1fd, GPIOC_WRITE, 0);
  ioctl(motor->in2fd, GPIOC_WRITE, 0);

  memset(&motor->pwminfo, 0, sizeof(struct pwm_info_s));
  motor->pwminfo.frequency = PWM_F;

  ioctl(motor->pwmfd,
        PWMIOC_SETCHARACTERISTICS,
        (unsigned long)((uintptr_t)&motor->pwminfo));

  ioctl(motor->pwmfd, PWMIOC_START, 0);

  return 0;
}

static void motor_destroy(struct remote_motor_s *motor)
{
  ioctl(motor->in1fd, GPIOC_WRITE, 0);
  ioctl(motor->in2fd, GPIOC_WRITE, 0);
  ioctl(motor->pwmfd, PWMIOC_STOP, 0);

  close(motor->in1fd);
  close(motor->in2fd);
  close(motor->pwmfd);
}

static int motor_set_speed(struct remote_motor_s* motor, int speed)
{
  if (speed < 0)
    {
      ioctl(motor->in1fd, GPIOC_WRITE, 1);
      ioctl(motor->in2fd, GPIOC_WRITE, 0);

      speed *= -1;
    }
  else
    {
      ioctl(motor->in1fd, GPIOC_WRITE, 0);
      ioctl(motor->in2fd, GPIOC_WRITE, 1);
    }
  motor->pwminfo.duty = speed ? b16divi(uitoub16((uint8_t)speed) - 1, 100) : 0;

  ioctl(motor->pwmfd,
        PWMIOC_SETCHARACTERISTICS,
        (unsigned long)((uintptr_t)&motor->pwminfo));

  return 0;
}

int main(int argc, char *argv[])
{
  struct remote_motor_s motor;
  if (motor_init(&motor, MOTOR_IN1_PATH, MOTOR_IN2_PATH, MOTOR_PWM_PATH) != 0)
    {
      printf("Error initializing motor\n");
      return 1;
    }

  const int tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
  const int bool_true = 1;
  setsockopt(tcp_socket, SOL_SOCKET, SO_REUSEADDR, &bool_true, sizeof(bool_true));

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
  setsockopt(tcp_conn_socket, SOL_SOCKET, SO_KEEPALIVE, &bool_true, sizeof(bool_true));
  setsockopt(tcp_conn_socket, IPPROTO_TCP, TCP_NODELAY, &bool_true, sizeof(bool_true));

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

      motor_set_speed(&motor, incoming);
    }

  close(tcp_conn_socket);
  close(tcp_socket);

  motor_destroy(&motor);
  return 0;
}
