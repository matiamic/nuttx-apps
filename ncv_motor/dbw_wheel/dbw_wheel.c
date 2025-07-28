#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>

#include <sched.h>

#include <nuttx/timers/pwm.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/sensors/qencoder.h>

#include <pthread.h>

#include <sys/ioctl.h>

#define PWM_F 8000

#define SERVER_IP "10.0.0.2"
#define TCP_PORT  5000

#define MOTOR_IN1_PATH "/dev/gpio0"
#define MOTOR_IN2_PATH "/dev/gpio1"
#define MOTOR_PWM_PATH "/dev/pwm0"
#define MOTOR_QEN_PATH "/dev/qe0"

#define CONTROL_LOOP_PERIOD_NS 10000000 // 10ms
#define P_FACTOR 5

struct motor_s
{
  int in1fd;
  int in2fd;
  int pwmfd;
  int qenfd;
  int32_t reference;
  struct pwm_info_s pwminfo;

  pthread_mutex_t mutex;
};

static struct motor_s g_motor = { 0 };

static int motor_init(struct motor_s *motor,
                      char *in1_path, char *in2_path,
                      char *pwm_path, char *qen_path)
{
  motor->in1fd = open(in1_path, O_RDWR);
  motor->in2fd = open(in2_path, O_RDWR);
  motor->pwmfd = open(pwm_path, O_RDWR);
  motor->qenfd = open(qen_path, O_RDONLY);
  if (!(motor->in1fd && motor->in2fd && motor->pwmfd && motor->qenfd))
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

  pthread_mutex_init(&motor->mutex, NULL);

  return 0;
}

static void motor_destroy(struct motor_s *motor)
{
  ioctl(motor->in1fd, GPIOC_WRITE, 0);
  ioctl(motor->in2fd, GPIOC_WRITE, 0);
  ioctl(motor->pwmfd, PWMIOC_STOP, 0);

  close(motor->in1fd);
  close(motor->in2fd);
  close(motor->pwmfd);
  close(motor->qenfd);

  pthread_mutex_destroy(&motor->mutex);
}

static int motor_set_speed(struct motor_s* motor, int speed)
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

  speed = (speed > 100) ? 100 : speed;

  motor->pwminfo.duty = speed ? b16divi(uitoub16((uint8_t)speed) - 1, 100) : 0;

  ioctl(motor->pwmfd,
        PWMIOC_SETCHARACTERISTICS,
        (unsigned long)((uintptr_t)&motor->pwminfo));

  return 0;
}

static void add_to_timespec(struct timespec *tp, long long increment_ns)
{
  int increment_sec = increment_ns / 1000000000;
  tp->tv_sec += increment_sec;
  increment_ns -= increment_sec * 1000000000;
  tp->tv_nsec += increment_ns;
  if (tp->tv_nsec >= 1000000000)
    {
      tp->tv_nsec -= 1000000000;
      tp->tv_sec += 1;
    }
}

static int reference_receiver_task(int argc, char *argv[])
{
  /* Create TCP socket */

  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0)
    {
      perror("Socket creation failed");
      return 1;
    }

  /* Disable Nagle's */

  bool boolean_true = 1;
  int err = setsockopt(sock,
                       IPPROTO_TCP, TCP_NODELAY,
                       (char *)&boolean_true, sizeof(int));
  if (err < 0)
    {
      perror("setsockopt(TCP_NODELAY) failed");
      close(sock);
      return 1;
    }

  /* Server address configuration */

  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(TCP_PORT);
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

  while (1)
    {
      int32_t reference;
      read(sock, &reference, sizeof(reference));
      pthread_mutex_lock(&g_motor.mutex);
      g_motor.reference = reference;
      pthread_mutex_unlock(&g_motor.mutex);
    }
}

int main(int argc, char *argv[])
{
  if (motor_init(&g_motor, MOTOR_IN1_PATH, MOTOR_IN2_PATH, MOTOR_PWM_PATH, MOTOR_QEN_PATH) != 0)
    {
      printf("Error initializing motor\n");
      return 1;
    }

  task_create("dbw-reference-receiver", 100, 16384, reference_receiver_task, NULL);

  struct timespec tp;
  clock_gettime(CLOCK_MONOTONIC, &tp);

  while (1)
    {
      int32_t position;
      int ret = ioctl(g_motor.qenfd, QEIOC_POSITION, (unsigned long)((uintptr_t) &position));
      if (ret < 0)
        {
          printf("dbw_wheel: ioctl position read failed: %d\n", errno);
          return errno;
        }

      pthread_mutex_lock(&g_motor.mutex);
      int32_t reference = g_motor.reference;
      pthread_mutex_unlock(&g_motor.mutex);

      int err = position - reference;
      int speed = -err * P_FACTOR;
      motor_set_speed(&g_motor, speed);

      add_to_timespec(&tp, CONTROL_LOOP_PERIOD_NS);
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tp, NULL);
    }

  motor_destroy(&g_motor);
  return 0;
}
