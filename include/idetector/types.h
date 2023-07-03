#pragma once

#include "config.h"

struct YoloKernel {
  int width;
  int height;
  float anchors[kNumAnchor * 2];
};

struct alignas(float) Detection {
  float bbox[4];  // center_x center_y w h
  float conf;  // bbox_conf * cls_conf
  float class_id;
  float mask[32];
};

struct bbox_t {
    unsigned int x, y, w, h;       // (x,y) - top-left corner, (w, h) - width & height of bounded box
    float prob;                    // confidence - probability that the object was found correctly
    unsigned int obj_id;           // class of object - from range [0, classes-1]
    unsigned int track_id;         // tracking id for video (0 - untracked, 1 - inf - tracked object)
    unsigned int frames_counter;   // counter of frames on which the object was detected
    float x_3d, y_3d, z_3d;        // center of object (in Meters) if ZED 3D Camera is used
    bbox_t(unsigned int xx, unsigned int yy, unsigned int ww, unsigned int hh, unsigned int cls, unsigned int id, float conf):
    x(xx), y(yy), w(ww), h(hh), obj_id(cls), track_id(id), prob(conf){};
    bbox_t(){}
};

