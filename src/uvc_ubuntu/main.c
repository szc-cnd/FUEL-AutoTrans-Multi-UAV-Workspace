#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

extern void  prepare_hik_sensor(void);
extern void  start_hik_sensor(void);
extern void  stop_hik_sensor(void);

static volatile int  bRun = 0;

static void sig_handler(int signo)
{
	signal(SIGTERM, SIG_IGN);
	bRun = 0;
}

int  main(int argc, char *argv[])
{
	/* init thermal sensor */
	prepare_hik_sensor();
	bRun = 1;
	/* some initiation here */
	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);

	/* start thermal capturing. */
	start_hik_sensor();
	while (bRun) {
		sleep(1);
	}

	/* quit thermal capturing */
	stop_hik_sensor();

	exit(0);
}


