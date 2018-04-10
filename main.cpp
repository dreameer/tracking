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


#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>


#define readbuffsize 32
#define databuffsize 4
#define writebuffsize 14

#define protocol_width  1000
#define protocol_height 1000

#define camera_width 1280 
#define camera_height 720 
    
#define RECORDVEDIO
#define fixed_fps 40
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
    return "nvcamerasrc sensor-id= 0 ! video/x-raw(memory:NVMM), width=(int)" + patch::to_string(width) + ", height=(int)" +
           patch::to_string(height) + ", format=(string)I420, framerate=(fraction)" + patch::to_string(fps) +
           "/1 ! nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
           //"/1 ! nvvidconv flip-method=2 !  appsink";
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
	line(frame,Point((init_rect.x+init_rect.width*0.5),init_rect.y+init_rect.height*0.1),
	Point((init_rect.x+init_rect.width*0.5),init_rect.y+init_rect.height*0.95),color,1,1);
	line(frame,Point(init_rect.x+init_rect.width*0.1,init_rect.y+init_rect.height*0.5),
	Point(init_rect.x+init_rect.width*0.95,init_rect.y+init_rect.height*0.5),color,1,1);
}
bool issamerect(Rect2d rect1,Rect2d rect2){
	if((rect1.x==rect2.x)&&(rect1.y==rect2.y)&&(rect1.width==rect2.width)&&(rect1.height==rect2.height)){
		return true;
	}
	else{
		return false;
	}
}
bool isrectinmat(Rect2d rect,Mat frame){
	if((rect.x>=0)&&
	   (rect.y>=0)&&
	   ((rect.x+rect.width)<=frame.cols)&&
	   ((rect.y+rect.height)<=frame.rows)){
		   return true;
	   }
	else{
		   return false;
	   }
}
void putrectcenter(Rect2d &smallrect,Rect2d largerect){
	smallrect.x = largerect.width*0.5-smallrect.width*0.5;
	smallrect.y = largerect.height*0.5-smallrect.height*0.5;
}
Rect2d getcenterrect(Mat frame){
	return Rect(frame.cols * 0.45, frame.rows*0.5-frame.cols*0.05,frame.cols * 0.1, frame.cols * 0.1);
}
void scale_window(unsigned short scale,Rect2d &roi_rect,Mat raw){
	if(scale>50||scale<1){
		printf("error scale over flow\n");
	}else{
		roi_rect.width = raw.cols-(scale-1)*raw.cols*0.02;
		roi_rect.height = raw.rows - (scale-1)*raw.rows*0.02;
		roi_rect.x = (raw.cols - roi_rect.width)*0.5;
		roi_rect.y = (raw.rows - roi_rect.height)*0.5;
	}
}
		
void focal_length(unsigned short length){
	unsigned short _length = length;
	if(_length>993||length<0){
		printf("error:focal_length:%d over flow\n",_length);
	}else{
		int file;
		int adapter_nr = 2; //probably dynamically determined
		char filename[20];
		snprintf(filename,19,"/dev/i2c-%d",adapter_nr);
		file = open(filename,O_RDWR);
		if(file<0){
			printf("open file failed\n");
			exit(1);
		}
		int addr = 0x0c; //the i2c address
		if(ioctl(file,I2C_SLAVE,addr)<0){
			printf("ioctl error\n");
			exit(1);
		}
		__u8 reg = 0x04;
		__s32 res;
		char buff[10];
		buff[0] = reg;
		memcpy(&buff[1],((unsigned char*)&_length)+1,1);
		memcpy(&buff[2],((unsigned char*)&_length),1);
		res = write(file,buff,3);
		if(res!=3){
			printf("write failed:%d\n",res);
		}else{
			}
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
			//printf("*");
		}
	}
	printf("end read thread\n");
	ReadEnd = true;
}
void *writefun(void *datafrommainthread) {
	
	int m_ttyfd = ((Ppassdatathread) datafrommainthread)->tty_filedescriptor;
	
	/*media flow
    pointsInGrid=10;             //!<square root of number of keypoints used; increase it to trade accurateness for speed
    winSize = Size(3,3);         //!<window size parameter for Lucas-Kanade optical flow
    maxLevel = 5;                //!<maximal pyramid level number for Lucas-Kanade optical flow
    termCriteria = TermCriteria(TermCriteria::COUNT|TermCriteria::EPS,20,0.3); //!<termination criteria for Lucas-Kanade optical flow
    winSizeNCC = Size(30,30);     //!<window size around a point for normalized cross-correlation check
    maxMedianLengthOfDisplacementDifference = 10;    //!<criterion for loosing the tracked object
	*/
	
	/*kcf
	  detect_thresh = 0.5f;
      sigma=0.2f;
      lambda=0.0001f;
      interp_factor=0.075f;
      output_sigma_factor=1.0f / 16.0f;
      resize=true;
      max_patch_size=80*80;
      split_coeff=true;
      wrap_kernel=false;
      desc_npca = GRAY;
      desc_pca = CN;

      //feature compression
      compress_feature=true;
      compressed_size=2;
      pca_learning_rate=0.15f;
	 */
	Ptr<Tracker> tracker;
	TrackerKCF::Params params;
	params.detect_thresh = 0.5f;
	params.pca_learning_rate=0.15f;
	params.wrap_kernel = false;
	int FPS = 30;
	std::string pipeline = get_tegra_pipeline(camera_width, camera_height, FPS);
	VideoCapture inputcamera(pipeline, cv::CAP_GSTREAMER);
	VideoWriter outputVideo;
    
	if (!inputcamera.isOpened()) {
		printf("cant open camera!\n");
	} else {
		Mat raw,frame,roi,record_frame;
		
		
		Rect2d roi_rect,object_rect, init_rect, center_rect;
		
		
		int object_center_x, object_center_y;
		unsigned char xl, xh, yl, yh;
		
		
		
		
		inputcamera >> raw;
		roi_rect = Rect(0,0,raw.cols,raw.rows);
		roi = raw(roi_rect);
		frame = roi;
		center_rect = getcenterrect(frame);
		init_rect = center_rect;
		object_rect = init_rect;
		
		
		
		
		const char windowname[] = "FEIFANUAV";
		namedWindow(windowname,WINDOW_NORMAL );
		//moveWindow(windowname,200,100);
		//resizeWindow(windowname,camera_width,camera_height);
		setWindowProperty(windowname,CV_WND_PROP_FULLSCREEN,CV_WINDOW_FULLSCREEN);
		
		
		
		unsigned char track_status = 0;
		unsigned char track_turn   = 0;
		char keyboardcmd = '.';
		bool intracking = false;
		
		
		
		unsigned char Lock = 0;
		short X_Move = 0;
		short Y_Move = 0;
		unsigned short Width = 0;
		unsigned short Height = 0;
		unsigned short scale = 1;
		unsigned short temp = 0;
		unsigned short temp1 = 0;
		unsigned short length = 350;
		focal_length(length);
		
		while (MainControl) {
			int64 start = cv::getTickCount();
			
			
			
			
			
			switch(keyboardcmd){
			case 'q':track_turn = 1;break;
			case 'w':intracking = false;
					break;
			case 'f':length = length+10;
			         if((length<993)&&(length>0)){
						 focal_length(length);
					 }else{
						length = 0;
					 }
					break;
			case 'g':length = length-10;
			         if((length<993)&&(length>0)){
						 focal_length(length);
					 }else{
						length = 0;
					 }
					break;
			case 'v':length = length+1;
			         if((length<993)&&(length>0)){
						 focal_length(length);
					 }else{
						length = 0;
					 }
					break;
			case 'b':length = length-1;
			         if((length<993)&&(length>0)){
						 focal_length(length);
					 }else{
						length = 0;
					 }
					break;
			case 'j':roi_rect.x = roi_rect.x-roi_rect.width*0.1;
			         if(!isrectinmat(roi_rect,raw)){
						 roi_rect.x = roi_rect.x+roi_rect.width*0.1;
					 }
					 else{
						 putrectcenter(init_rect,roi_rect);
					 }
					 break;
			case 'l':roi_rect.x = roi_rect.x+roi_rect.width*0.1;
			         if(!isrectinmat(roi_rect,raw)){
						 roi_rect.x = roi_rect.x-roi_rect.width*0.1;
					 }
					 else{
						 putrectcenter(init_rect,roi_rect);
					}
					break;
			case 'i':roi_rect.y = roi_rect.y-roi_rect.height*0.1;
                     if(!isrectinmat(roi_rect,raw)){
						 roi_rect.y=roi_rect.y+roi_rect.height*0.1;
					 }
					 else{
						 putrectcenter(init_rect,roi_rect);
					}break;
			case 'k':roi_rect.y = roi_rect.y+roi_rect.height*0.1;
					 if(!isrectinmat(roi_rect,raw)){
						 roi_rect.y = roi_rect.y-roi_rect.height*0.1;
					 }
					 else{
						 putrectcenter(init_rect,roi_rect);
					}break;
			case 'u':scale = scale -1;
			         if(scale<1){
						 scale = 1;
					 }else{
						 scale_window(scale,roi_rect,raw);
						 center_rect = getcenterrect(raw(roi_rect));
						 if(!intracking){init_rect = center_rect;}
					 }
			         break;
			case 'o':scale = scale +1;
			         if(scale>50){
						 scale = 50;
					 }else{
						 scale_window(scale,roi_rect,raw);
						 center_rect = getcenterrect(raw(roi_rect));
						 if(!intracking){init_rect = center_rect;}
					 }
			         break;
			case 'p':roi_rect = Rect(0,0,raw.cols,raw.rows);
			         init_rect = center_rect;
			         break;
			case 'n':init_rect.x = init_rect.x - 0.5*(init_rect.width*0.1);
					 init_rect.width = init_rect.width*1.1;
			         init_rect.y = init_rect.y - 0.5*(init_rect.height*0.1);
			         init_rect.height = init_rect.height*1.1;
			         if(!isrectinmat(init_rect,frame)){
						 init_rect.width = init_rect.width/1.1;
						 init_rect.x = init_rect.x + 0.5*(init_rect.width*0.1);
						 init_rect.height = init_rect.height/1.1;
						 init_rect.y = init_rect.y + 0.5*(init_rect.height*0.1);
					 }else{
						 
					}break;
			case 'm':init_rect.x = init_rect.x + 0.5*(init_rect.width*0.1);
					 init_rect.width = init_rect.width*0.9;
			         init_rect.y = init_rect.y + 0.5*(init_rect.height*0.1);
			         init_rect.height = init_rect.height*0.9;
			         if(!isrectinmat(init_rect,frame)){
						 init_rect.width = init_rect.width/0.9;
						 init_rect.x = init_rect.x - 0.5*(init_rect.width*0.1);
						 init_rect.height = init_rect.height/0.9;
						 init_rect.y = init_rect.y - 0.5*(init_rect.height*0.1);
					 }else{
					}break;
			case 'c':imwrite("raw.jpg",raw);break;
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
			case 0x0006:
			    memcpy(&scale,databuff,sizeof(unsigned short));
			    scale_window(scale,roi_rect,raw);
				center_rect = getcenterrect(raw(roi_rect));
				if(!intracking){init_rect = center_rect;}
				CmdFromUart = 0xffff;
				break;
			case 0x0007:
				memcpy(&length,databuff,sizeof(unsigned short));
				focal_length(length);
				CmdFromUart = 0xffff;
				break;
			default:
				break;
			}
			
			
			
			
			inputcamera >> raw;
			roi = raw(roi_rect);
			frame = roi;
			if (track_turn==1 || (!issamerect(object_rect,init_rect) && intracking)) {
				object_rect.x = (int)(init_rect.x);
				object_rect.y = (int)(init_rect.y);
				object_rect.width = (int)(init_rect.width);
				object_rect.height = (int)(init_rect.height);
				tracker = TrackerKCF::create(params);
				bool initstatus = tracker->init(frame, object_rect);

				
				#ifdef RECORDVEDIO
			    const string NAME = gettimestrwithavi();
			    Size S = Size((int) (frame.cols+1)/2,
			                  (int) (frame.rows+1)/2);
			    outputVideo.open(NAME, CV_FOURCC('D','I','V','X'), (int)(1000.0/fixed_fps), S, true);
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
					rectangle(frame, object_rect, Scalar(0, 0, 255), 2, 1);
					object_center_x = (object_rect.x + object_rect.width * 0.5)*((float)protocol_width/(float)frame.cols);
					object_center_y = (object_rect.y + object_rect.height*0.5)*((float)protocol_height/(float)frame.rows);
					track_status = 1;
				} else {
					//rectangle(frame, object_rect, Scalar(0, 255, 0), 2, 1);
					object_center_x = protocol_width*0.5;
					object_center_y = protocol_height*0.5;
					track_status = 2;
				}

			} else {
				rectangle(frame, init_rect, Scalar(255, 0, 0), 2, 1);
				object_center_x = protocol_width*0.5;
				object_center_y = protocol_height*0.5;
				track_status = 0;
			}

            short x_offset,y_offset;
            x_offset =   object_center_x - protocol_width*0.5;
            y_offset = - (object_center_y - protocol_height*0.5);
			xl = (x_offset & 0x000000ff);
			xh = ((x_offset >> 8) & 0x000000ff);
			yl = (y_offset & 0x000000ff);
			yh = ((y_offset >> 8) & 0x000000ff);
			unsigned char buff[writebuffsize] = { 0x55, 0xaa, 0x00, 0x00, 0x05,
					0x00,xl, xh, yl, yh, track_status, 0xff, 0xff, 0x16 };
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
			double fps =  (double)(cv::getTickCount()-start)*1000 / cv::getTickFrequency();
			if((int)fps<fixed_fps){
				int control_fps = fixed_fps-(int)fps;
				usleep(control_fps*1000);
			}
			fps =  (double)(cv::getTickCount()-start)*1000 / cv::getTickFrequency();
			
			putText(frame, patch::to_string((int)roi_rect.width)+"X"+patch::to_string((int)roi_rect.height), Point(10, 40),FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 0), 2, 8);
			putText(frame, patch::to_string((int)fps)+"ms", Point(frame.cols-120, 40),FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 0), 2, 8);
			drawcross(frame,center_rect,Scalar(0,255,0));
			putText(frame, "x="+patch::to_string(x_offset)+",y="+ patch::to_string(y_offset), Point(frame.cols*0.5-20, frame.rows-40),FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 0), 2, 8);
			
			#ifdef RECORDVEDIO
			if(intracking){
				pyrDown(frame,record_frame,Size((frame.cols+1)/2,(frame.rows+1)/2));
				outputVideo << record_frame;
				}
			#endif
			imshow(windowname, frame);
			keyboardcmd = (char) waitKey(1);
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
	const char ttyname[] = "/dev/ttyTHS1";
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
	ReadEnd = true;
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
int main1(){
	int FPS = 30;
	std::string pipeline = get_tegra_pipeline(2592,1944, FPS);
	VideoCapture cap("nvcamerasrc sensor-id= 0 ! video/x-raw(memory:NVMM), width=(int)2592, height=(int)1944, format=(string)I420, framerate=(fraction)30/1 !nvvidconv flip-method=2 !  appsink", cv::CAP_GSTREAMER);
	if (!cap.isOpened()) {
		printf("cant open camera!\n");
	} else {
		Mat frame;
		char keyboardcmd;
		unsigned short length = 350;
		for(;;){
			cap>>frame;
			namedWindow("just",WINDOW_NORMAL);
			imshow("just",frame);
			keyboardcmd = (char) waitKey(1);
			if(keyboardcmd=='e')break;
			switch(keyboardcmd){
				case 'f':length = length+10;
			         if((length<993)&&(length>0)){
						 focal_length(length);
					 }else{
						length = 0;
					 }
					break;
				case 'g':length = length-10;
			         if((length<993)&&(length>0)){
						 focal_length(length);
					 }else{
						length = 0;
					 }
					break;
			}
			
		}
	}
	return 0;
}
