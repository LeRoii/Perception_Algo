#include <unistd.h>
#include <signal.h>
#include <thread>
#include <atomic>
#include <termios.h>
#include <fcntl.h>
#include <algorithm>
#include <iostream>
#include "sdireader.h"
#include "idetector.h"
#include "kcftracker.hpp"
#include "jetsonEncoder.h"


#include "multitracker.h"
#include "MessageQueueHandle.h"
#include "nvrender.h"
#include "pid.h"

using namespace std;


const int ORI_IMG_W = 1920;
const int ORI_IMG_H = 1080;

const int DISP_IMG_W = 960;
const int DISP_IMG_H = 540;


nvrenderCfg renderCfg{DISP_IMG_W,DISP_IMG_H,DISP_IMG_W,DISP_IMG_H,0,0,0};
nvrender *render;

static bool trackerInit = false;

std::vector<bbox_t> boxs;

struct stSysStats
{
    uint8_t u8TrackingOn; //跟踪，0-否，1-是
    uint8_t u8DetOn; //识别，0-否，1-是
    uint8_t u8FusionOn;	//融合，0-否，1-是  
    uint8_t u8RtspOn;	//推流，0-否，1-是
    uint8_t u8DrawCross;
    int32_t i32TrackingTargetID;
    uint8_t u8TrackingGateSize;
    int32_t i32ZoomScale;
    cv::Point stDrawCrossPos;
};

static stSysStats mStats;

void initMat(cv::Mat &mat, float(*p)[3]){
    for(int i = 0; i < mat.rows; i++){
        for(int j = 0; j < mat.cols; j++){
            mat.at<float>(i,j) = *(*(p+i)+j);
        }
    }
}

void load_homo(std::string txt_path, std::vector<cv::Mat> &homo_buffer){
    //读取数据，每行存9个float数据作为一个homo矩阵    
    std::ifstream myfile(txt_path);
    while(!(myfile.eof())){
        if (!myfile.is_open()){
            std::cout << "can not open this file" << std::endl;
        }
        float test[3][3];
        for(int i = 0; i < 3; i++){
           for(int j = 0; j < 3; j++){
                myfile >> test[i][j];
            }
        }

        std::vector<cv::Mat> h;
        cv::Mat test_m(3, 3, CV_32F);
        initMat(test_m, test);
        homo_buffer.push_back(test_m);
    }

    myfile.close();
}

cv::Mat fusion(cv::Mat &vis, cv::Mat &ir, float gamma){
  //load homo matrix
  std::string txt_path = "../homography.txt";
  std::vector<cv::Mat> homo_buffer;
  load_homo(txt_path, homo_buffer);
  cv::Mat H = homo_buffer[0];
    cv::Mat ret;
  cv::addWeighted(vis(cv::Rect(100,100,640,512)), 0.5, ir, 0.5, 0, ret);
  ret.copyTo(vis(cv::Rect(100,100,640,512)));
  return vis;

  cv::Mat mask = cv::Mat::ones(ir.rows, ir.cols, CV_8UC1);     //mask of fusion area
//   cv::cuda::GpuMat ir_cu(ir);
//   cv::cuda::GpuMat mask_cu(mask);
//   transform(H, ir_cu, vis);
//   transform(H, mask_cu, vis);  
//   mask_cu.download(mask);
    cv::warpPerspective(ir, vis, H, cv::Size(1920,1080));
    cv::warpPerspective(mask, vis, H, cv::Size(1920,1080));


//   transform(H, ir, vis);
//   transform(H, mask, vis);
  
//   cv::cuda::cvtColor(ir_cu, ir_cu, CV_BGR2GRAY);
  cv::cvtColor(ir, ir, CV_BGR2GRAY);
  
  cv::Mat y_channel;//ir_cu.download(ir);                 
  cv::extractChannel(vis, y_channel, 0);
  y_channel = y_channel - gamma*y_channel.mul(mask) + gamma*ir;
  cv::insertChannel(y_channel, vis, 0);
  cv::Mat bgr(vis.rows, vis.cols, CV_8UC3);
  // printf("111111\n");
  cv::cvtColor(vis, bgr, cv::COLOR_YUV2RGB);
  // printf("111111222\n");

  return bgr;
}


//-------------------------------------------PID-----------------------------------------------------------

float refX = 0;
float mesX = 0;
float errX = refX - mesX;
float preErrX = errX;
float Kp = 6;
float Kd = 0.5;
float Ki = 0.0;
float dt = 0.03; // 30ms
float inte = 1;
float outX = 0;

float pidProc(float errX)
{
    inte += errX;
    float out = Kp * errX + Ki * inte + Kd * (errX - preErrX);

    preErrX = errX;
    printf("pid:errx:%f pixel, out:%f pixel\n", errX, out);
    return out;
}

//------------------------------------------------------------------------------------------------


/************************ui start*********************************/

bool quit = false;
int nFrames = 0;
//手动绘制跟踪框时在画面显示框标志位
bool selecting_roi = false;
//手工跟踪开关标志位
bool manualTrackFlag = false;
//手动绘制跟踪坐标框变量
cv::Point2i start_point(-1, -1), end_point(-1, -1);
cv::Point2i start_point_temp(-1, -1), end_point_temp(-1, -1);
//鼠标移动标志位
bool moveFlag = false;
int trackQuit=0;
cv::Rect result;
//detect flag
bool detection_flag=false;

static float TrackerOffset[2] = {0,0};



static void signal_handle(int signum)
{
	quit = true;
	// _xdma_reader_ch0.xdma_close();
}

//鼠标回调函数
void mouse_callback(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        // manualTrackFlag = false;
        start_point_temp = cv::Point2i(x, y);
        end_point_temp = cv::Point2i(x, y);
        moveFlag = true;
    }
    else if (event == cv::EVENT_LBUTTONUP)
    {
        moveFlag = false;
        selecting_roi = false;
        end_point_temp = cv::Point2i(x, y);
        start_point.x=min(start_point_temp.x,end_point_temp.x);
        start_point.y=min(start_point_temp.y,end_point_temp.y);
        end_point.x=max(start_point_temp.x,end_point_temp.x);
        end_point.y=max(start_point_temp.y,end_point_temp.y);
        
        if(abs(end_point.x-start_point.x)>5 && abs(end_point.y-start_point.y)>5){
            manualTrackFlag = true;
            nFrames = 0;
        }  
    }
    else if (event == cv::EVENT_MOUSEMOVE && moveFlag)
    {
        end_point_temp = cv::Point2i(x, y);
        start_point.x=min(start_point_temp.x,end_point_temp.x);
        start_point.y=min(start_point_temp.y,end_point_temp.y);
        end_point.x=max(start_point_temp.x,end_point_temp.x);
        end_point.y=max(start_point_temp.y,end_point_temp.y);
        if(abs(end_point.x-start_point.x)>5 && abs(end_point.y-start_point.y)>5){
            selecting_roi = true;
            manualTrackFlag = false;
        }
        else{
            selecting_roi = false;
        }  
    }  
}

// 键盘事件处理函数
void process_keyboard_events()
{
    struct termios old_tio, new_tio;
    unsigned char c;
    int tty_fd;

    // 打开标准输入设备（键盘）设备文件
    tty_fd = open("/dev/tty", O_RDONLY | O_NONBLOCK);
    if (tty_fd == -1)
    {
        std::cerr << "无法打开标准输入设备文件" << std::endl;
        quit = 1;
        return;
    }

    // 保存并修改终端设置  
    tcgetattr(tty_fd, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(tty_fd, TCSANOW, &new_tio);

    while (!quit)
    {
        // 读取一个字符
        if (read(tty_fd, &c, 1) > 0)
        {
            std::cout <<c<< std::endl;
            // 处理按键事件
            switch (c)
            {
            case 'd': // 按下r键
                detection_flag=!detection_flag;
                // std::cout << "按键d已按下" << std::endl;
                break;
            case 't': // 按下q键
                mStats.u8TrackingOn = !mStats.u8TrackingOn;
                // trackerInit = !trackerInit;
                trackerInit = false;
                // std::cout << "按键t已按下" << std::endl;
                break;
            case 'v':
                mStats.u8FusionOn = 0;
                break;
            case 'i':
                mStats.u8FusionOn = 1;
                break;
            case 'f':
                mStats.u8FusionOn = 2;
                break;
            }
        }

        // 程序暂停一段时间，避免CPU占用率过高
        usleep(1000);
    }

    // 还原终端设置
    tcsetattr(tty_fd, TCSANOW, &old_tio);
    close(tty_fd);
}





void trackingg(cv::Mat& rgbFrame,KCFTracker* tracker){

	if (manualTrackFlag && !rgbFrame.empty())
	{
		if (nFrames == 0){
			if(end_point.x-start_point.x>5 && end_point.y-start_point.y>5){
				tracker->init(cv::Rect(start_point.x, start_point.y, end_point.x-start_point.x, end_point.y-start_point.y), rgbFrame);
			}
			else{
				manualTrackFlag = false;
			}
		}
			
		else
		{
			result = tracker->update(rgbFrame);
			// cv::rectangle(rgbFrame, cv::Point(result.x, result.y), cv::Point(result.x + result.width, result.y + result.height), cv::Scalar(48, 48, 255), 2, 8);
            drawCrosshair(rgbFrame, cv::Point(result.x, result.y));
		}
		nFrames++; 
	}
	//------------------------------------------------------------------------------------
	if(!rgbFrame.empty() && selecting_roi){
		cv::rectangle(rgbFrame, start_point, end_point, cv::Scalar(0, 0, 255), 2);
	}
}

void trackerProcess(cv::Mat& rgbFrame,KCFTracker* tracker){

    
    cv::Point centerPt = cv::Point(ORI_IMG_W/2, ORI_IMG_H/2);
    cv::Size gateSize[3] = {cv::Size(32,32), cv::Size(64,64), cv::Size(128,128)};
    cv::Point gateLtPt = cv::Point((ORI_IMG_W - gateSize[0].width)/2, (ORI_IMG_H - gateSize[0].height)/2);
    // cv::Rect trackerGate[3] = {cv::Rect()};

    cv::Rect trackerInitRect = cv::Rect(gateLtPt, gateSize[0]);
    static auto lastPos = result;

	if (!rgbFrame.empty())
	{
		if ( !trackerInit){
            if(1 == mStats.u8TrackingOn)
            {
                int idx = 0;
                if(mStats.u8TrackingGateSize >=0 || mStats.u8TrackingGateSize <= 2)
                    idx = mStats.u8TrackingGateSize;
                trackerInitRect = cv::Rect(gateLtPt, gateSize[idx]);
            }
            else if(2 == mStats.u8TrackingOn)
            {
                int trackId = mStats.i32TrackingTargetID;
                printf("tracking target ID:%d\n", trackId);
                for(auto &box:boxs)
                {
                    if(trackId == box.track_id)
                    {
                        printf("find target!!!!!\n");
                        trackerInitRect = cv::Rect(box.x, box.y, box.w, box.h);
                        break;
                    }
                }
            }
            tracker->init(trackerInitRect, rgbFrame);
            trackerInit = true;
            drawCrosshair(rgbFrame, cv::Point(trackerInitRect.x + trackerInitRect.width/2, trackerInitRect.y + trackerInitRect.height/2));

                // cv::rectangle(rgbFrame, cv::Rect(gateLtPt, gateSize[0]), cv::Scalar(255, 255, 255), 2, 8);
		}
			
		else
		{
			result = tracker->update(rgbFrame);
			drawCrosshair(rgbFrame, cv::Point(result.x + result.width/2, result.y + result.height/2));
            TrackerOffset[0] = result.x - ORI_IMG_W/2;
            TrackerOffset[1] = ORI_IMG_H/2 - result.y;
            // lastPos = result;
            printf("TrackerOffset[0]:%d, TrackerOffset[1]:%d\n", TrackerOffset[0], TrackerOffset[1]);
#ifdef DEBUG_INFO
            cv::rectangle(rgbFrame, cv::Point(result.x, result.y), cv::Point(result.x + result.width, result.y + result.height), cv::Scalar(255, 0, 255), 2, 8);
#endif
		}
		nFrames++; 
	}
	//------------------------------------------------------------------------------------
	if(!rgbFrame.empty() && selecting_roi){
		cv::rectangle(rgbFrame, start_point, end_point, cv::Scalar(255, 0, 255), 2);
	}
}

/************************ui end*********************************/



void stats2Msg(stSysStats &stats, IPSERVER::ST_MQ_ALGOSEND_MSG &msg)
{
    msg.st_MQ_Type = sizeof(IPSERVER::ST_MQ_ALGOSEND_MSG)-sizeof(msg.st_MQ_Type);
    msg.st_Resp_Key = 0;
    msg.st_Command = IPSERVER::enum_MQ_Command::e_MQ_SearchStatus;
    msg.st_Tracking_Status = stats.u8TrackingOn;
    msg.st_Identify_Status = stats.u8DetOn;
    msg.st_Fusion_Status = stats.u8FusionOn;
    msg.st_Plugflow_Status = stats.u8RtspOn;
    msg.st_Draw_Cross = stats.u8DrawCross;
    msg.st_Tracking_Gate_Size = stats.u8TrackingGateSize;
    msg.st_Tracking_Target_ID = stats.i32TrackingTargetID;
    msg.st_Objects_Num = 0;
    msg.st_MissX = 0;
    msg.st_MissY = 0;
}

void processComMsg(IPSERVER::ST_MQ_COMSEND_MSG &st_msg)
{
    trackerInit = false;
    if(2 == st_msg.st_Tracking_Status)
    {
        bool findTarget = false;
        for(auto &box:boxs)
        {
            if(st_msg.st_Tracking_Target_ID == box.track_id)
            {
                printf("find target!!!!!\n");
                findTarget = true;
                mStats.u8TrackingOn = st_msg.st_Tracking_Status;
                mStats.i32TrackingTargetID = st_msg.st_Tracking_Target_ID;
                break;
            }
        }
        if(!findTarget)
        {
            printf("canot find target, track center\n");
            mStats.u8TrackingOn = 1;
            mStats.i32TrackingTargetID = -1;
        }
    }
    else if(1 == st_msg.st_Tracking_Status)
    {
        mStats.u8TrackingOn = 1;
    }
}

void algoCallback(IPSERVER::ST_MQ_COMSEND_MSG &st_msg)
{
    printf("msg.st_MQ_Type:%ld, u8TrackingOn:%d, u8DetOn:%d, u8FusionOn:%d, u8RtspOn:%d,\
    st_Draw_Cross:%d\n",\
     st_msg.st_MQ_Type, st_msg.st_Tracking_Status, st_msg.st_Identify_Status, st_msg.st_Fusion_Status,
     st_msg.st_Plugflow_Status, st_msg.st_Draw_Cross);
    // switch(st_msg.st_Command){
    //     case e_MQ_Set_Identify_Mode:
    //         mStats.u8DetOn = st_msg.st_Identify_Status;
    //         break;
    //     case e_MQ_Set_Tracking_Mode:
    //         processComMsg(st_msg);
    //         break;
    //     case e_MQ_Set_Fusion_Mode:
    //         mStats.u8FusionOn = st_msg.st_Fusion_Status;
    //         break;
    //     case e_MQ_Set_Plugflow_Mode:
    //         mStats.u8RtspOn = st_msg.st_Plugflow_Status;
    //         break;
    //     case e_MQ_Set_Draw_Cros_Mode:
    //         mStats.u8DrawCross = st_msg.st_Draw_Cross;
    //         break;
    //     default:
    //         break;
    // }

    // IPSERVER::ST_MQ_ALGOSEND_MSG msg;
    // stats2Msg(mStats, msg);
    // msg.st_Command = st_msg.st_Command;
    
    // IPSERVER::MessageQueueHandle::GetInstance()->SendMsg(msg);

    // return;
    
    if(st_msg.st_Tracking_Status == 1)
        trackerInit = false;
    mStats.u8TrackingOn = st_msg.st_Tracking_Status;
    mStats.u8DetOn = st_msg.st_Identify_Status;
    mStats.u8FusionOn = st_msg.st_Fusion_Status;
    mStats.u8RtspOn = st_msg.st_Plugflow_Status;
    mStats.u8DrawCross = st_msg.st_Draw_Cross;
    mStats.u8TrackingGateSize = st_msg.st_Tracking_Gate_Size;
    // mStats.i32TrackingTargetID = st_msg.st_Tracking_Target_ID;
    mStats.stDrawCrossPos = cv::Point(st_msg.st_Draw_Cross_Point[0], st_msg.st_Draw_Cross_Point[1]);
    mStats.i32ZoomScale = 0;
    mStats.i32ZoomScale = st_msg.st_ZoomScale;
    if(2 == st_msg.st_Tracking_Status)
    {
        bool findTarget = false;
        for(auto &box:boxs)
        {
            if(st_msg.st_Tracking_Target_ID == box.track_id)
            {
                printf("find target!!!!!\n");
                findTarget = true;
                mStats.u8TrackingOn = st_msg.st_Tracking_Status;
                mStats.i32TrackingTargetID = st_msg.st_Tracking_Target_ID;
                break;
            }
        }
        if(!findTarget)
        {
            printf("canot find target, track center\n");
            mStats.u8TrackingOn = 1;
            mStats.i32TrackingTargetID = -1;
        }
    }
    // IPSERVER::ST_MQ_ALGOSEND_MSG msg;
    // stats2Msg(mStats, msg);
    // IPSERVER::MessageQueueHandle::GetInstance()->SendMsg(msg);

}



int main(int argc, char* argv[])
{
    // render = new nvrender(renderCfg);

    // stSysStats mStats; 
    IPSERVER::ST_MQ_ALGOSEND_MSG msg;
    msg.st_MQ_Type = sizeof(IPSERVER::ST_MQ_ALGOSEND_MSG)-sizeof(msg.st_MQ_Type);
    msg.st_Resp_Key = 0;
    msg.st_Command = IPSERVER::enum_MQ_Command::e_MQ_SearchStatus;
    msg.st_Tracking_Status = mStats.u8TrackingOn = 0;
    msg.st_Identify_Status = mStats.u8DetOn = 1;
    msg.st_Fusion_Status = mStats.u8FusionOn = 0;
    msg.st_Plugflow_Status = mStats.u8RtspOn = 0;
    msg.st_Draw_Cross = mStats.u8DrawCross = 1;
    msg.st_Tracking_Gate_Size = mStats.u8TrackingGateSize = 0;
    msg.st_Tracking_Target_ID = mStats.i32TrackingTargetID = 0;
    msg.st_MissX = 55;
    msg.st_MissY = 22;

    printf("len:%d\n", msg.st_MQ_Type);


    //启动消息队列，线程detached
    IPSERVER::MessageQueueHandle::GetInstance()->OnStart(algoCallback);

    // // while(1)
    // // {
    // //     usleep(10000); 
    // // }
    // IPSERVER::MessageQueueHandle::GetInstance()->SendMsg(msg);

    // // printf("send end\n");

    // return 0;
	//detector init
    printf("detector init...\n");
	idetector* vis_detector = new idetector("/home/nx/model/vp-cls2.engine");
    
    
	//跟踪相关设置
    bool HOG = true;
    bool FIXEDWINDOW = false;
    bool MULTISCALE = true;
    bool SILENT = true;
    bool LAB = false;
    printf("tracker init...\n");
    KCFTracker* tracker=new KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
    
    // keyboard event thread   
    // std::thread keyboard_event_thread(process_keyboard_events);

	struct sigaction sig_action;
	sig_action.sa_handler = signal_handle;
	sigemptyset(&sig_action.sa_mask);
	sig_action.sa_flags = 0;
	sigaction(SIGINT, &sig_action, NULL);

	cv::Mat frame0, frame1,detection_frame0;

    //rtsp 
    jetsonEncoder * temp = new jetsonEncoder(8554);

	//鼠标回调函数设置
    // cv::namedWindow("Frame");
    // cv::setMouseCallback("Frame", mouse_callback);

	Sdireader_Init("/etc/config/NXConfig.ini");

//*******************************multitracker init*************************
    size_t m_batchSize = 1;
    float m_fps = 25;
    const int minStaticTime = 5;

    cv::Mat tmp = cv::Mat(1080, 1920, CV_8UC3);
    FrameInfo frameInfo(m_batchSize);
	frameInfo.m_frames.resize(frameInfo.m_batchSize);
	frameInfo.m_frameInds.resize(frameInfo.m_batchSize);
    frameInfo.m_frames[0].GetMatBGRWrite() = tmp;
    cv::UMat umatFrame = frameInfo.m_frames[0].GetUMatBGR();

    std::unique_ptr<BaseTracker> m_tracker;
    TrackerSettings settings;
    genTrackerSettings(settings);
    m_tracker = BaseTracker::CreateTracker(settings);
    frameInfo.CleanRegions();
    frameInfo.CleanTracks();
//*******************************multitracker init end *************************
    // frameInfo.m_frames[0].GetMatBGRWrite() = cv::imread("/home/nx/code/1.jpg"); 
    // cv::Mat ii = cv::imread("/home/nx/code/123.PNG");
    // cv::imshow("1", ii);
    // cv::waitKey(0);

    //detection

    // cv::VideoCapture cap("/home/nx/data/1.mp4");

    // cv::VideoWriter writer;
    // writer.open("ret.avi", CV_FOURCC('M','J','P','G'),20, cv::Size(1280,720), true);

//*******************************fusion init start *************************
    cv::Mat fus;
    cv::Mat h[3];
    cv::Rect roi[3];
    //2
    std::vector<cv::Point2f> irPts{cv::Point2f(96,290), cv::Point2f(96,181), cv::Point2f(186,181), cv::Point2f(227,176), cv::Point2f(283,299), cv::Point2f(337,198), cv::Point2f(523,242)};
    std::vector<cv::Point2f> visPts{cv::Point2f(550,538), cv::Point2f(550,413),  cv::Point2f(694,413), cv::Point2f(760,405), cv::Point2f(852,547), cv::Point2f(941,432), cv::Point2f(1239,479)};
    h[0] = findHomography(irPts,visPts);
    roi[0] = cv::Rect(407,217,1003,561);
    // std::vector<cv::Point2f> visPts{cv::Point2f(96+20,290), cv::Point2f(96+20,181), cv::Point2f(523+20,242), cv::Point2f(587+20,223)};

    //3
    irPts = std::vector<cv::Point2f> {cv::Point2f(74,277), cv::Point2f(74,170), cv::Point2f(163,170), cv::Point2f(205,162), cv::Point2f(262,286), cv::Point2f(315,184), cv::Point2f(498,230)};
    visPts = std::vector<cv::Point2f> {cv::Point2f(303,513), cv::Point2f(303,325),  cv::Point2f(517,325), cv::Point2f(619,315), cv::Point2f(757,527), cv::Point2f(889,353), cv::Point2f(1333,427)};
    h[1] = findHomography(irPts,visPts);
    roi[1] = cv::Rect(131,43,1517,861);

    //4
    irPts = std::vector<cv::Point2f> {cv::Point2f(78,280), cv::Point2f(78,170), cv::Point2f(170,170), cv::Point2f(209,165), cv::Point2f(266,288), cv::Point2f(319,187), cv::Point2f(504,231)};
    visPts = std::vector<cv::Point2f> {cv::Point2f(103,503), cv::Point2f(103,257),  cv::Point2f(395,257), cv::Point2f(523,243), cv::Point2f(709,521), cv::Point2f(881,291), cv::Point2f(1473,387)};
    h[2] = findHomography(irPts,visPts);
    roi[2] = cv::Rect(23,27,1857,969);

    // h[0] = h[1] = h[2] = findHomography(irPts,visPts); 

    std::cout<<h<<std::endl;

    // roi[0] = roi[1] = roi[2] = cv::Rect(407,217,1003,561);
//*******************************fusion init end *************************

	sleep(2);

    cv::Mat frame, tempp;
    regions_t regions;
    cv::Mat disImg;
    cv::Mat visImg, irImg;

    cv::VideoCapture IRCamera("rtspsrc location=rtsp://192.168.2.119:554/stream1 latency=0 ! rtph264depay ! h264parse ! omxh264dec ! videoconvert ! appsink max-buffers=1 drop=true sync=false", cv::CAP_GSTREAMER);
     if (!IRCamera.isOpened())
    {
        std::cout << "**************IRCamera Connection failed \n";
        return -1;
    }
	while(!quit)
	{
		
		Sdireader_GetFrame(visImg, irImg);
        IRCamera >> irImg;
        // cv::imwrite("1iii.png", irImg);
        // return 0;
        if(mStats.u8FusionOn == 0)
            frame = visImg;
        else if(mStats.u8FusionOn == 1)
            frame = irImg;
        else if(mStats.u8FusionOn == 2)
        {
            printf("mStats.i32ZoomScale:%d\n", mStats.i32ZoomScale);
            int fusHidx = mStats.i32ZoomScale > 1 ?  mStats.i32ZoomScale -2 : 0;
            cv::resize(irImg, irImg, cv::Size(640,512));
            cv::warpPerspective(irImg, irImg, h[fusHidx], cv::Size(1920,1080));
            cv::addWeighted(visImg(roi[fusHidx]), 0.3, irImg(roi[fusHidx]), 0.7, 0, fus);
            // cv::imwrite("fus.png", fus); 
            fus.copyTo(visImg(roi[fusHidx]));
            frame = visImg;
            
        }

        // cv::resize(irImg, irImg, cv::Size(640,512));

        // // frame = fusion(visImg, irImg, 0.8);

        // cv::imwrite("1.png", irImg);
        // cv::imwrite("0.png", visImg);
        // return 0;
        

        // cap >> frame;
        
        if(frame.empty())
        {
            ptintf("empty frame\n");
            break;
        }

        disImg = frame.clone();

        // cv::resize(frame, frame, cv::Size(1280,720));  
        if(detection_flag || mStats.u8DetOn == 1){
        // if(detection_flag){
            frameInfo.m_frames[0].GetMatBGRWrite() = frame.clone();
            //detector
            vis_detector->process(frame,detection_frame0, boxs);
            // printf("det end\n");

            // msg.st_Objects_Num = boxs.size();
            // // msg.st_Objects_Content.clear();
            // for(int i=0;i<boxs.size();i++)
            // {
            //     msg.st_Objects_Content[i] = boxs[i];
            //     printf("x:%d,", boxs[i].x);
            //     // msg.st_Objects_Content.emplace_back(boxs[i]);
            // }

            // printf("\nlen:%d\n", sizeof(msg)-sizeof(msg.st_MQ_Type));

            // IPSERVER::MessageQueueHandle::GetInstance()->SendMsg(msg);

            // cv::resize(detection_frame0, detection_frame0, cv::Size(960,540));
            // cv::imshow("Frame", detection_frame0);
            // cv::waitKey(0);

            // continue;
            
            frameInfo.CleanRegions();

            printf("frameInfo.m_regions[0] size%d, boxs :%d\n", frameInfo.m_regions[0].size(), boxs.size());
            regions.clear();
            for(auto &box:boxs)
            {
                regions.emplace_back(cv::Rect(cvRound(1.0*box.x), cvRound(1.0*box.y), cvRound(1.0*box.w), cvRound(1.0*box.h)), (box.obj_id), box.prob);
            }

            // printf("frameInfo.m_regions[0] size%d, regions:%d\n", frameInfo.m_regions[0].size(), regions.size());
            frameInfo.m_regions[0] = regions;
            m_tracker->Update(frameInfo.m_regions[0], frameInfo.m_frames[0].GetUMatGray(), m_fps);
            // printf("track size:%d\n", frameInfo.m_tracks[0].size());
            m_tracker->GetTracks(frameInfo.m_tracks[0]);

            // printf("size:%d, id:%d\n", frameInfo.m_tracks[0].size(), frameInfo.m_tracks[0][0].m_ID);

            Tracks2Boxs(frameInfo.m_tracks[0], boxs);
            msg.st_Objects_Num = boxs.size();
            for(int i=0;i<boxs.size();i++)
            {
                msg.st_Objects_Content[i] = boxs[i];
                printf("x:%d,", boxs[i].x);
                // msg.st_Objects_Content.emplace_back(boxs[i]);
            }

            stats2Msg(mStats, msg);
            msg.st_Command = e_MQ_TargetDetectionData;
            IPSERVER::MessageQueueHandle::GetInstance()->SendMsg(msg);

            DrawData(frameInfo.m_frames[0].GetMatBGR(), frameInfo.m_tracks[0], frameInfo.m_frameInds[0], 0);

            disImg = frameInfo.m_frames[0].GetMatBGR().clone();
            // drawCrosshair(disImg, cv::Point(960,540));
            // cv::resize(disImg, disImg, cv::Size(DISP_IMG_W,DISP_IMG_H));
            // cv::imshow("Frame", disImg);

            // writer<<disImg;

            // cv::imshow("Frame", detection_frame0);
            // temp->process(detection_frame0);
        }
        // else{
        //     // trackingg(frame,tracker);
        //     disImg = frame.clone();
        //     drawCrosshair(disImg, cv::Point(960,540));
        //     cv::resize(disImg, disImg, cv::Size(960,540));
        //     cv::imshow("Frame", disImg);
        //     // temp->process(frame);
        // }
        else if(mStats.u8TrackingOn)
        {
            trackerProcess(frame,tracker);
            disImg = frame.clone();
            // drawCrosshair(disImg, cv::Point(960,540));
            // cv::resize(disImg, disImg, cv::Size(960,540));
            
            stats2Msg(mStats, msg);
            msg.st_MissX = pidProc(TrackerOffset[0]);
            msg.st_MissY = pidProc(TrackerOffset[1]);
            msg.st_Command = e_MQ_TargetDetectionData;
            IPSERVER::MessageQueueHandle::GetInstance()->SendMsg(msg);
        }
        // else
        {
            mStats.u8DrawCross = 1;
            // disImg =  frame.clone();
            if(!mStats.u8TrackingOn && mStats.u8DrawCross)
                drawCrosshair(disImg, cv::Point(ORI_IMG_W/2,ORI_IMG_H/2));
            cv::resize(disImg, disImg, cv::Size(DISP_IMG_W,DISP_IMG_H));
            // render->render(disImg);
            printf("111111111111111111111\n");
            temp->process(disImg);
            // cv::imshow("Frame", disImg);
		    // cv::waitKey(5);sss
		    // cv::waitKey(10);
        }
	}

    // writer.release();
    delete vis_detector;
	delete tracker;
	return 0;
}

