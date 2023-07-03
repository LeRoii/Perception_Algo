#ifndef _NVRENDER_H_
#define _NVRENDER_H_

#include <opencv2/opencv.hpp>
#include "NvEglRenderer.h"
#include "nvbuf_utils.h"
// #include "spdlog/spdlog.h"
// #include "stitcherglobal.h"

const int RENDER_EGL = 0;
const int RENDER_OCV = 1;

struct nvrenderCfg
{
    int bufferw;
    int bufferh;
    int renderw;
    int renderh;
    int renderx;
    int rendery;
    int mode;//0 for egl, 1 for opencv
};

static int offsetX, offsetY, h, w;
static double fitscale;
class nvrender
{
public:
    nvrender(nvrenderCfg cfg);
    ~nvrender();
    void render(unsigned char *data);
    void drawIndicator();
    void fit2final(cv::Mat &input, cv::Mat &output);
    void renderegl(cv::Mat &img);
    void renderocv(cv::Mat &img, cv::Mat &final);
    void render(cv::Mat &img);
    void render(cv::Mat &img, cv::Mat &final);
    void showImg(cv::Mat &img);
    void showImg();
    void renderimgs(cv::Mat &img, cv::Mat &inner, int x, int y);


private:
    NvEglRenderer *renderer;
    int nvbufferfd;
    int nvbufferWidth, nvbufferHeight;
    cv::Mat canvas;
    int m_mode;
    int maxHeight, maxWidth;
    int longStartX, uplongStartY, uplongEndY, upshortEndY;
    int downlongStartY, downlongEndY, downshortEndY;
    int indicatorStartX;
};

#endif
