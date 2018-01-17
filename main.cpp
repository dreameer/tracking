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
#include <sstream>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#define readbuffsize 32
#define databuffsize 4
#define writebuffsize 14

#define protocol_width  320
#define protocol_height 240

//#define RECORDVEDIO

using namespace std;
using namespace cv;

unsigned short CmdFromUart = 0xffff;
char databuff[databuffsize] = {0}; 
bool MainControl = true;
bool ReadEnd = false;
bool WriteEnd = false;

typedef struct passdatathread {
	int tty_filedescriptor;
}*Ppassdatathread;
 
namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}
std::string get_tegra_pipeline(int width, int height, int fps) {
    return "nvcamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)" + patch::to_string(width) + ", height=(int)" +
           patch::to_string(height) + ", format=(string)I420, framerate=(fraction)" + patch::to_string(fps) +
           "/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

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
string gettimestrwithavi(void){
    time_t current_time;
    char* c_time_string;
    current_time = time(NULL);
    c_time_string = ctime(&current_time);
    cout<<c_time_string<<endl;
    string timestr;
    timestr.append(&c_time_string[0],3);
    timestr.append(&c_time_string[4],3);
    timestr.append(&c_time_string[8],2);
    timestr.append(&c_time_string[11],2);
    timestr.append(&c_time_string[14],2);
    timestr.append(&c_time_string[17],2);
    timestr.append(&c_time_string[20],4);
    cout<<timestr<<endl;
    const string NAME = timestr + ".avi";   // Form the new name with container
    return NAME;
}
void drawcross(Mat frame,Rect2d init_rect,const Scalar& color){
	line(frame,Point((init_rect.x+init_rect.width*0.5),init_rect.y+init_rect.height*0.4),
	Point((init_rect.x+init_rect.width*0.5),init_rect.y+init_rect.height*0.6),color,1,1);
	line(frame,Point(init_rect.x+init_rect.width*0.4,init_rect.y+init_rect.height*0.5),
	Point(init_rect.x+init_rect.width*0.6,init_rect.y+init_rect.height*0.5),color,1,1);
}
bool issamerect(Rect2d rect1,Rect2d rect2){
	if((rect1.x==rect2.x)&&(rect1.y==rect2.y)&&(rect1.width==rect2.width)&&(rect1.height==rect2.height)){
		return true;
	}
	else{
		return false;
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
										memcpy(&cmd, &buff[2],sizeof(unsigned short));
										CmdFromUart = cmd;
										memcpy(databuff,&buff[6],datalength*sizeof(char));
										printf("cmd is:%x datalength:%d\n", cmd,datalength);
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
	params.detect_thresh = 0.4f;

	int m_ttyfd = ((Ppassdatathread) datafrommainthread)->tty_filedescriptor;

    int WIDTH = 1920;
    int HEIGHT = 1080;
    int FPS = 30;
 
    // Define the gstream pipeline
    std::string pipeline = get_tegra_pipeline(WIDTH, HEIGHT, FPS);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";
 
    // Create OpenCV capture object, ensure it works.
    cv::VideoCapture inputcamera(pipeline, cv::CAP_GSTREAMER);
    
	VideoWriter outputVideo;
	//inputcamera.set(CAP_PROP_FRAME_WIDTH,320);
	//inputcamera.set(CAP_PROP_FRAME_HEIGHT,240);
	if (!inputcamera.isOpened()) {
		printf("cant open camera!\n");
	} else {
		Mat frame;
		Rect2d object_rect, init_rect, center_rect;
		int object_center_x, object_center_y;
		unsigned char xl, xh, yl, yh;
		inputcamera >> frame;
		center_rect = Rect(frame.cols * 0.40, frame.rows * 0.45,frame.cols * 0.2, frame.rows * 0.1);
		init_rect = center_rect;
		object_rect = init_rect;
		namedWindow("FEIFANUAV", 1);
		//setWindowProperty("FEIFANUAV",CV_WND_PROP_FULLSCREEN,CV_WINDOW_FULLSCREEN);
		unsigned char track_status = 0;
		unsigned char track_turn   = 0;
		char keyboardcmd = 'c';
		bool intracking = false;
		unsigned char Lock = 0;
		short X_Move = 0;
		short Y_Move = 0;
		unsigned short Width = 0;
		unsigned short Height = 0;
		unsigned short temp = 0;
		unsigned short temp1 = 0;
		while (MainControl) {
			int64 start = cv::getTickCount();
			switch(keyboardcmd){
			case 'q':track_turn = 1;break;
			case 'w':intracking = false;
					init_rect.x = frame.cols*0.5-init_rect.width*0.5;
					init_rect.y = frame.rows*0.5-init_rect.height*0.5;
					break;
			case 'e':MainControl = false;break;
			default:break;
			}
			switch (CmdFromUart) {
			case 0x0001:
			    memcpy(&Lock,databuff,sizeof(unsigned char));
			    if(Lock==0){
					intracking = false;
					init_rect.x = frame.cols*0.5-init_rect.width*0.5;
					init_rect.y = frame.rows*0.5-init_rect.height*0.5;
				}else if(Lock==1){
					track_turn = 1;
				}
				CmdFromUart = 0xffff;
				break;
			case 0x0002:
			    memcpy(&X_Move,databuff,sizeof(short));
			    temp = init_rect.x;
				init_rect.x = (float)(X_Move+protocol_width*0.5)/protocol_width*frame.cols - init_rect.width*0.5;
				if ((init_rect.x <= 0) || (init_rect.br().x >= frame.cols) ) {
					init_rect.x = temp;
				}
				CmdFromUart = 0xffff;
				break;
			case 0x0003:
			    memcpy(&Y_Move,databuff,sizeof(short));
			    temp = init_rect.y;
				init_rect.y = (float)(Y_Move+protocol_height*0.5)/protocol_height*frame.rows - init_rect.height*0.5;
				if ( (init_rect.y <= 0) || (init_rect.br().y >= frame.rows)) {
					init_rect.y = temp;
				}
				CmdFromUart = 0xffff;
				break;
			case 0x0004:
			    memcpy(&Width,databuff,sizeof(unsigned short));
			    temp = init_rect.width;
			    temp1 = init_rect.x;
				init_rect.width = ((float)Width/protocol_width)*frame.cols;
				init_rect.x = init_rect.x - (init_rect.width-temp)*0.5;
				if((init_rect.x <= 0) || (init_rect.br().x >= frame.cols) ){
					init_rect.width = temp;
					init_rect.x = temp1;
				}
				CmdFromUart = 0xffff;
				break;
			case 0x0005:
		        memcpy(&Height,databuff,sizeof(unsigned short));
				temp = init_rect.height;
				temp1 = init_rect.y;
				init_rect.height = ((float)Height/protocol_height)*frame.rows;
				init_rect.y = init_rect.y - (init_rect.height-temp)*0.5;
				if ( (init_rect.y <= 0) || (init_rect.br().y >= frame.rows)) {
					init_rect.height = temp;
					init_rect.y = temp1;
				}
				CmdFromUart = 0xffff;
				break;
			default:
				break;
			}
			inputcamera >> frame;
			if (track_turn==1 || (!issamerect(object_rect,init_rect) && intracking)) {
				printf("inti roi frmae w:%d h:%d\n", frame.cols, frame.rows);
				object_rect = init_rect;
				tracker = TrackerKCF::create(params);
				tracker->init(frame, object_rect);
				
				#ifdef RECORDVEDIO
			    const string NAME = gettimestrwithavi();
			    Size S = Size((int) inputcamera.get(CAP_PROP_FRAME_WIDTH),
			                  (int) inputcamera.get(CAP_PROP_FRAME_HEIGHT));
			    outputVideo.open(NAME, CV_FOURCC('D','I','V','X'), inputcamera.get(CAP_PROP_FPS), S, true);
			    if (!outputVideo.isOpened())
			    {
			        cout  << "Could not open the output video for write: " << endl;
			        break;
			    }
			    #endif
				intracking = true;
				track_turn = 0;
			} else {
			}
			if (intracking) {
				if (tracker->update(frame, object_rect)) {
					init_rect = object_rect;
					putText(frame, "tracking", Point(10, 10),FONT_HERSHEY_COMPLEX, 0.3, Scalar(0, 0, 255), 1,8);
					rectangle(frame, object_rect, Scalar(0, 0, 255), 1, 1);
					drawcross(frame,init_rect,Scalar(0,0,255));
					object_center_x = object_rect.x + object_rect.width * 0.5;
					object_center_y = object_rect.y + object_rect.height * 0.5;
					track_status = 1;
				} else {
					putText(frame, "lose object press A retrack", Point(10, 10),
							FONT_HERSHEY_COMPLEX, 0.3, Scalar(0, 0, 255), 1, 8);
					object_center_x = 160;
					object_center_y = 120;
					track_status = 2;
				}
				#ifdef RECORDVEDIO
				outputVideo << frame;
				#endif

			} else {
				rectangle(frame, init_rect, Scalar(255, 0, 0), 1, 1);
				drawcross(frame,init_rect,Scalar(255,0,0));
				putText(frame, "press A to begin", Point(10, 10),
						FONT_HERSHEY_COMPLEX, 0.3, Scalar(0, 0, 255), 1, 8);
				object_center_x = 160;
				object_center_y = 120;
				track_status = 0;
			}
			imshow("FEIFANUAV", frame);
			keyboardcmd = (char) waitKey(1);

            short x_offset,y_offset;
            x_offset =   object_center_x - frame.cols*0.5;
            y_offset = - (object_center_y - frame.rows*0.5);
			xl = (x_offset & 0x000000ff);
			xh = ((x_offset >> 8) & 0x000000ff);
			yl = (y_offset & 0x000000ff);
			yh = ((y_offset >> 8) & 0x000000ff);
			unsigned char buff[writebuffsize] = { 0x55, 0xaa, 0x00, 0x00, 0x05,
					0x00, xl, xh, yl, yh, track_status, 0xff, 0xff, 0x16 };
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
			double fps = cv::getTickFrequency() / (cv::getTickCount()-start);
			cout<<frame.cols<<" "<<frame.rows<<" "<<fps<<endl;
		}
	}
	printf("end write thread\n");
	WriteEnd = true;
	MainControl = false;
}
int main(int argc, char *argv[]) {
	/*create log file to record*/
	FILE *logFile = fopen("log.data", "w");
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
