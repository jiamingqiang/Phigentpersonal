//
// Created on 2022/4/19.
//
#pragma once
#include <cmath>
#include <memory>
#include <vector>

struct BBox_2d
{
public:
    BBox_2d() : x1(0), y1(0), x2(0), y2(0), score(0)
    {
        for (int i = 0; i < 32; ++i)
        {
            category[i] = 'b';
        }
    }
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
    char category[32]; //分类
};
struct Point2d
{
public:
    Point2d() : x(0), y(0), score(0), visible(false)
    {
        for (int i = 0; i < 32; ++i)
        {
            category[i] = 'p';
        }
    }
    float x;
    float y;
    float score;
    uint32_t visible;  // 是否可见
    char category[32]; // 分类
};
typedef short Vehicle2DType_Type;
struct Vehicle2D
{
public:
    Vehicle2D()
        : main_box(BBox_2d()),
          has_front_box(false),
          front_box(BBox_2d()),
          has_rear_box(false),
          rear_box(BBox_2d()),
          vehicle2d_point_num(0),
          occluded(0),
          truncated(0),
          cate(0),
          cate_score(0)
    {
        for (int i = 0; i < 8; ++i)
        {
            earch_points[i] = Point2d();
        }
    }
    BBox_2d main_box; // 车辆检测框
    uint32_t
        has_front_box; //取值之前先判断是否有车头框，目前算法给出的是互斥的，要么只有车头框，要么只有车尾框
    BBox_2d front_box; // 车头框
    uint32_t has_rear_box;
    BBox_2d rear_box; // 车尾框
    long vehicle2d_point_num;
    Point2d earch_points[8]; //接地点只有两个
    float occluded;          //遮挡比
    float truncated;         //截断比
    uint32_t cate;           // 5 tiny_car , 6 small_medium_car, 7 trucks , 8 bus, 9
                             // special_vehicle
    float cate_score;
};
struct FrameInfo2D
{
public:
    FrameInfo2D()
        : camera_id(0), frame_id(0), timestamp(0), in_use(0), vehicles_num(0)
    {
        for (int i = 0; i < 100; ++i)
        {
            vehicles[i] = Vehicle2D();
        }
    }
    long camera_id;
    uint64_t frame_id;  //  frame id 代表帧的计数
    uint64_t timestamp; // timestamp that frame received
    long in_use;
    long vehicles_num; // <=100
    Vehicle2D vehicles[100];
};
