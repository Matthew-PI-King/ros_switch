/*
 *  Copyright (c) 1999-2000 Vojtech Pavlik
 *  Copyright (c) 2009-2011 Red Hat, Inc
 */

/**
 * @file
 * Event device test program
 *
 * evtest prints the capabilities on the kernel devices in /dev/input/eventX
 * and their events. Its primary purpose is for kernel or X driver
 * debugging.
 *
 * See INSTALL for installation details or manually compile with
 * gcc -o evtest evtest.c
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Should you need to contact me, the author, you can do so either by
 * e-mail - mail your message to <vojtech@ucw.cz>, or by paper mail:
 * Vojtech Pavlik, Simunkova 1594, Prague 8, 182 00 Czech Republic
 */

#include "ros/ros.h"
#include <math.h> 
#include <stdio.h>
#include <stdint.h>

#if HAVE_CONFIG_H
#include <config.h>
#endif

#include <linux/version.h>
#include <linux/input.h>

#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <errno.h>
#include <getopt.h>
#include <ctype.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <geometry_msgs/Twist.h>

#define BITS_PER_LONG (sizeof(long) * 8)
#define NBITS(x) ((((x)-1)/BITS_PER_LONG)+1)
#define OFF(x)  ((x)%BITS_PER_LONG)
#define BIT(x)  (1UL<<OFF(x))
#define LONG(x) ((x)/BITS_PER_LONG)
#define test_bit(bit, array)	((array[LONG(bit)] >> OFF(bit)) & 1)

#define DEV_INPUT_EVENT "/dev/input"
#define EVENT_DEV_NAME "event"

#ifndef EV_SYN
#define EV_SYN 0
#endif
#ifndef SYN_MAX
#define SYN_MAX 3
#define SYN_CNT (SYN_MAX + 1)
#endif
#ifndef SYN_MT_REPORT
#define SYN_MT_REPORT 2
#endif
#ifndef SYN_DROPPED
#define SYN_DROPPED 3
#endif

#define NAME_ELEMENT(element) [element] = #element


ros::Publisher myPub;

/**
 * Filter for the AutoDevProbe scandir on /dev/input.
 *
 * @param dir The current directory entry provided by scandir.
 *
 * @return Non-zero if the given directory entry starts with "event", or zero
 * otherwise.
 */
static int is_event_device(const struct dirent *dir) {
	return strncmp(EVENT_DEV_NAME, dir->d_name, 5) == 0;
}

/**
 * Scans all /dev/input/event*, display them and ask the user which one to
 * open.
 *
 * @return The event device file name of the device file selected. This
 * string is allocated and must be freed by the caller.
 */
static char* scan_devices(void)
{
	struct dirent **namelist;
	int i, ndev, devnum, match;
	char *filename;
	int max_device = 0;

	ndev = scandir(DEV_INPUT_EVENT, &namelist, is_event_device, versionsort);
	if (ndev <= 0)
		return NULL;

	fprintf(stderr, "Available devices:\n");

	for (i = 0; i < ndev; i++)
	{
		char fname[64];
		int fd = -1;
		char name[256] = "???";

		snprintf(fname, sizeof(fname),
			 "%s/%s", DEV_INPUT_EVENT, namelist[i]->d_name);
		fd = open(fname, O_RDONLY);
		if (fd < 0)
			continue;
		ioctl(fd, EVIOCGNAME(sizeof(name)), name);

		fprintf(stderr, "%s:	%s\n", fname, name);
		close(fd);

		match = sscanf(namelist[i]->d_name, "event%d", &devnum);
		if (match >= 1 && devnum > max_device)
			max_device = devnum;

		free(namelist[i]);
	}

	fprintf(stderr, "Select the device event number [0-%d]: ", max_device);

	match = scanf("%d", &devnum);
	if (match < 1 || devnum > max_device || devnum < 0)
		return NULL;

	if (asprintf(&filename, "%s/%s%d",
		     DEV_INPUT_EVENT, EVENT_DEV_NAME,
		     devnum) < 0)
		return NULL;

	return filename;
}

static int version(void)
{
#ifndef PACKAGE_VERSION
#define PACKAGE_VERSION "<version undefined>"
#endif
	printf("%s %s\n", program_invocation_short_name, PACKAGE_VERSION);
	return EXIT_SUCCESS;
}


/**
 * Print usage information.
 */
static int usage(void)
{
	printf("USAGE:\n");
	printf(" Capture mode:\n");
	printf("   %s [--grab] /dev/input/eventX\n", program_invocation_short_name);
	printf("     --grab  grab the device for exclusive access\n");
	printf("\n");
	printf(" Query mode: (check exit code)\n");
	printf("   %s --query /dev/input/eventX <type> <value>\n",
		program_invocation_short_name);

	printf("\n");
	printf("<type> is one of: EV_KEY, EV_SW, EV_LED, EV_SND\n");
	printf("<value> can either be a numerical value, or the textual name of the\n");
	printf("key/switch/LED/sound being queried (e.g. SW_DOCK).\n");

	return EXIT_FAILURE;
}


static int get_state(int fd, unsigned int type, unsigned long *array, size_t size)
{
	int rc;

	switch(type) {
	case EV_LED:
		rc = ioctl(fd, EVIOCGLED(size), array);
		break;
	case EV_SND:
		rc = ioctl(fd, EVIOCGSND(size), array);
		break;
	case EV_SW:
		rc = ioctl(fd, EVIOCGSW(size), array);
		break;
	case EV_KEY:
		/* intentionally not printing the value for EV_KEY, let the
		 * repeat handle this */
	default:
		return 1;
	}
	if (rc == -1)
		return 1;

	return 0;
}

/**
 * Print static device information (no events). This information includes
 * version numbers, device name and all bits supported by this device.
 *
 * @param fd The file descriptor to the device.
 * @return 0 on success or 1 otherwise.
 */
static int print_device_info(int fd)
{
	unsigned int type, code;
	int version;
	unsigned short id[4];
	char name[256] = "Unknown";
	unsigned long bit[EV_MAX][NBITS(KEY_MAX)];
	unsigned long state[KEY_CNT] = {0};
#ifdef INPUT_PROP_SEMI_MT
	unsigned int prop;
	unsigned long propbits[INPUT_PROP_MAX];
#endif
	int stateval;
	int have_state;

	if (ioctl(fd, EVIOCGVERSION, &version)) {
		perror("evtest: can't get version");
		return 1;
	}

	printf("Input driver version is %d.%d.%d\n",
		version >> 16, (version >> 8) & 0xff, version & 0xff);

	ioctl(fd, EVIOCGID, id);
	printf("Input device ID: bus 0x%x vendor 0x%x product 0x%x version 0x%x\n",
		id[ID_BUS], id[ID_VENDOR], id[ID_PRODUCT], id[ID_VERSION]);

	ioctl(fd, EVIOCGNAME(sizeof(name)), name);
	printf("Input device name: \"%s\"\n", name);

	memset(bit, 0, sizeof(bit));
	ioctl(fd, EVIOCGBIT(0, EV_MAX), bit[0]);
	printf("Supported events:\n");

	for (type = 0; type < EV_MAX; type++) {
		if (test_bit(type, bit[0]) && type != EV_REP) {
			have_state = (get_state(fd, type, state, sizeof(state)) == 0);

			//printf("  Event type %d (%s)\n", type, typename(type));
			if (type == EV_SYN) continue;
			ioctl(fd, EVIOCGBIT(type, KEY_MAX), bit[type]);
			for (code = 0; code < KEY_MAX; code++)
				if (test_bit(code, bit[type])) {
					if (have_state) {
						stateval = test_bit(code, state);
					} else {
						//printf("    Event code %d (%s)\n", code, codename(type, code));
						printf("Broke 273");
					}
					if (type == EV_ABS)
						//print_absdata(fd, code);
						printf("Broke 276");
				}
		}
	}

	if (test_bit(EV_REP, bit[0])) {
		printf("Key repeat handling:\n");
		//printf("  Repeat type %d (%s)\n", EV_REP, events[EV_REP] ?  events[EV_REP] : "?");
		//print_repdata(fd);
	}

	return 0;
}
struct pos{int x; int y;};

/**
 * Print device events as they come in.
 *
 * @param fd The file descriptor to the device.
 * @return 0 on success or 1 otherwise.
 */
static int print_events(int fd)
{
	struct input_event ev[64];
	int i, rd;
	fd_set rdfs;
	pos p;
        bool d_pad=false;
	p.x = 0;
	p.y = 0;
	

	FD_ZERO(&rdfs);
	FD_SET(fd, &rdfs);

	while (ros::ok()) {
		select(fd + 1, &rdfs, NULL, NULL, NULL);
		rd = read(fd, ev, sizeof(ev));
		printf("Read\n");
		if (rd < (int) sizeof(struct input_event)) {
			printf("expected %d bytes, got %d\n", (int) sizeof(struct input_event), rd);
			perror("\nevtest: error reading");
			return 1;
		}
		printf("Will loop\n");
		for (i = 0; i < rd / sizeof(struct input_event); i++) {
			unsigned int type, code;
			int val;

			type = ev[i].type;
			code = ev[i].code;
			val = ev[i].value;

			printf("Event: time %ld.%06ld, ", ev[i].time.tv_sec, ev[i].time.tv_usec);

			if((type == 0) && (code == 0))
				printf("---------------------------------\n");
			else
			{
				//printf("woops");
				if (type == EV_MSC && (code == MSC_RAW || code == MSC_SCAN))
					printf("Type %d, Code %d, value %02x\n", type , code, val);
				else
					printf("Type %d, Code %d, value %d\n", type , code, val);
				if(type==3 && code == 1)  //X axis of left joystick
					p.x=-(val-128);
				else if (type == 3 && code ==0) //Y axis of left joystick
					p.y=-(val-128);
				else if (type == 3 && code ==17)
				{
					d_pad=true; //will use d_pad control variant.
					p.x=-(val*127);
				}
			
			}

			
		}
		geometry_msgs::Twist msg;
		if(d_pad)
		{
		msg.linear.x=p.x;
		d_pad=false;
		}
		else
		{
		msg.linear.x = p.x;
		msg.angular.z = atan2(p.y,p.x);
		}
		myPub.publish(msg);
		printf("Publishing!\n");

	}

	ioctl(fd, EVIOCGRAB, (void*)0);
	return EXIT_SUCCESS;
}

/**
 * Grab and immediately ungrab the device.
 *
 * @param fd The file descriptor to the device.
 * @return 0 if the grab was successful, or 1 otherwise.
 */
static int test_grab(int fd, int grab_flag)
{
	int rc;

	rc = ioctl(fd, EVIOCGRAB, (void*)1);

	if (rc == 0 && !grab_flag)
		ioctl(fd, EVIOCGRAB, (void*)0);

	return rc;
}

/**
 * Enter capture mode. The requested event device will be monitored, and any
 * captured events will be decoded and printed on the console.
 *
 * @param device The device to monitor, or NULL if the user should be prompted.
 * @return 0 on success, non-zero on error.
 */
static int do_capture(const char *device, int grab_flag)
{
	int fd;
	char *filename = NULL;

	if (!device) {
		fprintf(stderr, "No device specified, trying to scan all of %s/%s*\n",
			DEV_INPUT_EVENT, EVENT_DEV_NAME);

		if (getuid() != 0)
			fprintf(stderr, "Not running as root, no devices may be available.\n");

		filename = scan_devices();
		if (!filename)
			return usage();
	} else
		filename = strdup(device);

	if (!filename)
		return EXIT_FAILURE;

	if ((fd = open(filename, O_RDONLY)) < 0) {
		perror("evtest");
		if (errno == EACCES && getuid() != 0)
			fprintf(stderr, "You do not have access to %s. Try "
					"running as root instead.\n",
					filename);
		goto error;
	}

	if (!isatty(fileno(stdout)))
		setbuf(stdout, NULL);

	if (print_device_info(fd))
		goto error;

	printf("Testing ... (interrupt to exit)\n");

	if (test_grab(fd, grab_flag))
	{
		printf("***********************************************\n");
		printf("  This device is grabbed by another process.\n");
		printf("  No events are available to evtest while the\n"
		       "  other grab is active.\n");
		printf("  In most cases, this is caused by an X driver,\n"
		       "  try VT-switching and re-run evtest again.\n");
		printf("  Run the following command to see processes with\n"
		       "  an open fd on this device\n"
		       " \"fuser -v %s\"\n", filename);
		printf("***********************************************\n");
	}

	//signal(SIGINT, interrupt_handler);
	//signal(SIGTERM, interrupt_handler);

	free(filename);

	return print_events(fd);

error:
	free(filename);
	return EXIT_FAILURE;
}



int main (int argc, char **argv)
{       ros::init(argc, argv, "rosSwitchNode");
	ros::NodeHandle myRosNodeHandle;
	myPub = myRosNodeHandle.advertise<geometry_msgs::Twist>("slow_msgs", 50, true);
		//ROS Initialization

	const char *device = NULL;
	const char *keyname;
	const char *event_type;

		return do_capture(device, 1);

}
/* vim: set noexpandtab tabstop=8 shiftwidth=8: */
