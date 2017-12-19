/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>

#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#define readbuffsize 14
#define writebuffsize 14

using namespace std;
using namespace cv;

unsigned short CmdFromUart = 0xffff;
bool MainControl = true;
bool ReadEnd = false;
bool WriteEnd = false;

typedef struct passdatathread {
	int tty_filedescriptor;
}*Ppassdatathread;
int readqpeng(int p_ttyfd, void *dst, int size, int waitnum) {
	int mwaitnum = 0;
	int num = -1;
	do {
		num = read(p_ttyfd, dst, size);
		mwaitnum++;
	} while ((num != size) && (mwaitnum < waitnum));
	if (num == size) {
		return 1;
	} else {
		return -1;
	}
}
void *readfun(void *datafrommainthread) {
	int m_ttyfd = ((Ppassdatathread) datafrommainthread)->tty_filedescriptor;
	unsigned char buff[readbuffsize];
	while (MainControl) {
		int index = 0;
		int waitnum = 400;
		if (read(m_ttyfd, &buff[index], 1) == 1) {
			if (buff[index] == 0x55) {
				index++;
				if (readqpeng(m_ttyfd, &buff[index], 1, waitnum) == 1) {
					if (buff[index] == 0xaa) {
						index++;
						for (int i = 0; i < 4; i++) {
							if (readqpeng(m_ttyfd, &buff[index], 1, waitnum)
									== 1) {
								index++;
							} else {
								printf("read break index:%d\n", index);
								break;
							}
						}
						if (index == 6) {
							unsigned short datalength;
							memcpy(&datalength, &buff[4],
									sizeof(unsigned short));
							if ((datalength >= 0)
									&& (datalength < (readbuffsize - 9))) {
								for (int i = 0; i < (datalength + 3); i++) {
									if (readqpeng(m_ttyfd, &buff[index], 1,
											waitnum) == 1) {
										index++;
									} else {
										break;
									}
								}
								if (index == (6 + datalength + 3)) {
									unsigned int cyc = 0;
									unsigned char cl, ch;
									for (int i = 0; i < (index - 3); i++) {
										cyc = cyc + (unsigned int) (buff[i]);
									}
									cl = (cyc & 0x000000ff);
									ch = ((cyc >> 8) & 0x000000ff);
									if ((cl == buff[index - 3])
											&& (ch == buff[index - 2])) {
										unsigned short cmd;
										memcpy(&cmd, &buff[2],
												sizeof(unsigned short));
										CmdFromUart = cmd;
										printf("cmd is:%x lastchar:%x\n", cmd,
												buff[index - 1]);
									} else {
										printf("didt pass cyc\n");
									}
								} else {
									printf("read data area fialed\n");
								}
							} else {
								printf("datalenth overflow :%d\n", datalength);
							}
						} else {
							printf("read cmd 2byte failed :index:%d\n", index);
						}
					} else {
						printf("seconde byte not aa\n");
					}
				} else {
					printf("read seconde byte failed\n");
				}
			} else {
				printf("first byte not 55 :%X\n", buff[0]);
			}
		} else {
			//printf("read no data\n");
		}
	}
	printf("end read thread\n");
	ReadEnd = true;
}
void *writefun(void *datafrommainthread) {
	/*
	 detect_thresh = 0.5f;     //!<  detection confidence threshold
	 sigma=0.2f;               //!<  gaussian kernel bandwidth
	 lambda=0.0001f;           //!<  regularization
	 interp_factor=0.075f;     //!<  linear interpolation factor for adaptation
	 output_sigma_factor=1.0f / 16.0f;  //!<  spatial bandwidth (proportional to target)
	 resize=true;              //!<  activate the resize feature to improve the processing speed
	 max_patch_size=80*80;     //!<  threshold for the ROI size
	 split_coeff=true;         //!<  split the training coefficients into two matrices
	 wrap_kernel=false;        //!<  wrap around the kernel values
	 desc_npca = GRAY;         //!<  non-compressed descriptors of TrackerKCF::MODE
	 desc_pca = CN;            //!<  compressed descriptors of TrackerKCF::MODE

	 //feature compression
	 compress_feature=true;    //!<  activate the pca method to compress the features
	 compressed_size=2;        //!<  feature size after compression
	 pca_learning_rate=0.15f;  //!<  compression learning rate
	 *
	 */

	Ptr<Tracker> tracker;
	TrackerKCF::Params params;
	params.pca_learning_rate = 0.1f;
	params.detect_thresh = 0.25f;

	int m_ttyfd = ((Ppassdatathread) datafrommainthread)->tty_filedescriptor;

	VideoCapture cap(0);
	cap.set(CAP_PROP_FRAME_WIDTH,320);
	cap.set(CAP_PROP_FRAME_HEIGHT,240);
	if (!cap.isOpened()) {
		printf("cant open camera!\n");
	} else {
		Mat frame;
		Rect2d object_rect, init_rect, center_rect;
		int object_center_x, object_center_y;
		unsigned char xl, xh, yl, yh;
		cap >> frame;
		center_rect = Rect(frame.cols * 0.40, frame.rows * 0.45,frame.cols * 0.2, frame.rows * 0.1);
		init_rect = center_rect;
		namedWindow("FEIFANUAV", 0);
		int looseflag = 0;
		unsigned char trackstatus = 0;
		char keyboardcmd = 'c';
		bool intracking = false;
		while (MainControl) {
			cap >> frame;
			switch (CmdFromUart) {
			case 0x0002:
				init_rect.x--;
				if (init_rect.x <= 0) {
					init_rect.x++;
				}
				CmdFromUart = 0xffff;
				break;
			case 0x0003:
				init_rect.x++;
				if (init_rect.br().x >= frame.cols) {
					init_rect.x--;
				}
				CmdFromUart = 0xffff;
				break;
			case 0x0004:
				init_rect.y--;
				if (init_rect.y <= 0) {
					init_rect.y++;
				}
				CmdFromUart = 0xffff;
				break;
			case 0x0005:
				init_rect.y++;
				if (init_rect.br().y >= frame.rows) {
					init_rect.y--;
				}
				CmdFromUart = 0xffff;
				break;
			case 0x0006:
				init_rect.width++;
				if (init_rect.br().x >= frame.cols) {
					init_rect.width--;
				}
				CmdFromUart = 0xffff;
				break;
			case 0x0007:
				init_rect.width--;
				if (init_rect.width <= 0) {
					init_rect.width++;
				}
				CmdFromUart = 0xffff;
				break;
			case 0x0008:
				init_rect.height++;
				if (init_rect.br().y >= frame.rows) {
					init_rect.height--;
				}
				CmdFromUart = 0xffff;
				break;
			case 0x0009:
				init_rect.height--;
				if (init_rect.height <= 0) {
					init_rect.height++;
				}
				CmdFromUart = 0xffff;
				break;
			case 0x0010:
				intracking = false;
				CmdFromUart = 0xffff;
				break;
			default:
				break;
			}

			if ((keyboardcmd == 'a') || (CmdFromUart == 0x0001)) {
				printf("inti roi frmae w:%d h:%d\n", frame.cols, frame.rows);
				object_rect = init_rect;
				tracker = TrackerKCF::create(params);
				tracker->init(frame, object_rect);
				intracking = true;
				CmdFromUart = 0xffff;
			} else {
			}
			if (intracking) {
				if (tracker->update(frame, object_rect)) {
					switch (looseflag) {
					case 0:
						putText(frame, "tracking", Point(10, 10),
								FONT_HERSHEY_COMPLEX, 0.3, Scalar(0, 0, 255), 1,
								8);
						looseflag++;
						break;
					case 1:
						putText(frame, "tracking.", Point(10, 10),
								FONT_HERSHEY_COMPLEX, 0.3, Scalar(0, 0, 255), 1,
								8);
						looseflag++;
						break;
					case 2:
						putText(frame, "tracking..", Point(10, 10),
								FONT_HERSHEY_COMPLEX, 0.3, Scalar(0, 0, 255), 1,
								8);
						looseflag++;
						break;
					case 3:
						putText(frame, "tracking...", Point(10, 10),
								FONT_HERSHEY_COMPLEX, 0.3, Scalar(0, 0, 255), 1,
								8);
						looseflag++;
						break;
					case 4:
						putText(frame, "tracking....", Point(10, 10),
								FONT_HERSHEY_COMPLEX, 0.3, Scalar(0, 0, 255), 1,
								8);
						looseflag = 0;
						break;
					default:
						putText(frame, "tracking tracking!", Point(10, 10),
								FONT_HERSHEY_COMPLEX, 0.3, Scalar(0, 0, 255), 1,
								8);
						looseflag = 0;
						break;
					}
					rectangle(frame, object_rect, Scalar(0, 0, 255), 1, 1);
					object_center_x = object_rect.x + object_rect.width * 0.5;
					object_center_y = object_rect.y + object_rect.height * 0.5;
					trackstatus = 1;
				} else {
					putText(frame, "lose object press A retrack", Point(10, 10),
							FONT_HERSHEY_COMPLEX, 0.3, Scalar(0, 0, 255), 1, 8);
					object_center_x = frame.cols * 0.5;
					object_center_y = frame.rows * 0.5;
					trackstatus = 2;
				}
			} else {
				rectangle(frame, init_rect, Scalar(255, 0, 0), 1, 1);
				putText(frame, "press A to begin", Point(10, 10),
						FONT_HERSHEY_COMPLEX, 0.3, Scalar(0, 0, 255), 1, 8);
				object_center_x = frame.cols * 0.5;
				object_center_y = frame.rows * 0.5;
				trackstatus = 0;
			}
			imshow("FEIFANUAV", frame);
			keyboardcmd = (char) waitKey(1);
			if (keyboardcmd == 'b')
				break;

			xl = (object_center_x & 0x000000ff);
			xh = ((object_center_x >> 8) & 0x000000ff);
			yl = (object_center_y & 0x000000ff);
			yh = ((object_center_y >> 8) & 0x000000ff);
			unsigned char buff[writebuffsize] = { 0x55, 0xaa, 0x00, 0x00, 0x05,
					0x00, xl, xh, yl, yh, trackstatus, 0xff, 0xff, 0x16 };
			unsigned int cyc = 0;
			unsigned char cl, ch;
			for (int i = 0; i < (writebuffsize - 3); i++) {
				cyc = cyc + (unsigned int) (buff[i]);
			}
			cl = (cyc & 0x000000ff);
			ch = ((cyc >> 8) & 0x000000ff);
			buff[writebuffsize - 3] = cl;
			buff[writebuffsize - 2] = ch;
			for (int i = 0; i < writebuffsize; i++) {
				write(m_ttyfd, &buff[i], 1);
			}
		}
	}
	printf("end write thread\n");
	WriteEnd = true;
	MainControl = false;
}
int main(int argc, char *argv[]) {
	/*create log file to record*/
	FILE *logFile = fopen("./log.data", "w");
	if (logFile == NULL) {
		printf("fopen logfile error!%X", (int*) logFile);
		return -1;
	}

	fprintf(logFile, "begin\n");
	fflush(logFile);
	/*open tty to comunication with out device*/
	struct termios tio;
	memset(&tio, 0, sizeof(tio));
	tio.c_iflag = 0;
	tio.c_oflag = 0;
	tio.c_cflag = CS8 | CREAD | CLOCAL; // 8n1, see termios.h for more information
	tio.c_lflag = 0;
	tio.c_cc[VMIN] = 1;
	tio.c_cc[VTIME] = 0;
	const char ttyname[] = "/dev/ttyS0";
	int tty_fd = open(ttyname, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if (!isatty(tty_fd)) {
		fprintf(logFile,"filedescritor is not a tty device!\n");
		fflush(logFile);
		fclose(logFile);
		return -1;
	}
	cfsetospeed(&tio, B115200);              // 115200 baud
	cfsetispeed(&tio, B115200);              // 115200 baud
	tcsetattr(tty_fd, TCSANOW, &tio);
	fprintf(logFile, "finish open tty\n");
	fflush(logFile);


	/*create two thread to control read and write*/
	pthread_t thread_r, thread_w;
	long id_r, id_w;
	Ppassdatathread readthreaddata = (Ppassdatathread) malloc(
			1 * sizeof(passdatathread));
	Ppassdatathread writethreaddata = (Ppassdatathread) malloc(
			1 * sizeof(passdatathread));
	readthreaddata->tty_filedescriptor = tty_fd;
	writethreaddata->tty_filedescriptor = tty_fd;

	id_w = pthread_create(&thread_w, NULL, writefun, (void*) writethreaddata);
	fprintf(logFile, "finish create threadw\n");
	fflush(logFile);
	id_r = pthread_create(&thread_r, NULL, readfun, (void*) readthreaddata);
	fprintf(logFile, "finish create threadr\n");
	fflush(logFile);

	/*wait for two thread end then clean resource*/
	while ((!ReadEnd) || (!WriteEnd)) {
	};
	fprintf(logFile, "thread exit\n");
	close(tty_fd);
	fprintf(logFile, "close tty\n");
	fclose(logFile);
	return 0;
}
