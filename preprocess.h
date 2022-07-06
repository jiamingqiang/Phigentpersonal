#pragma once
#include <cmath>
#include <memory>
#include <vector>
#include "vehicle_data_parser.h"
#include "container.h"
//#include "include/vehicleattribute/unscented_kalman_filter.h"
//#include "opencv2/opencv.hpp"
namespace pg
{
    namespace vehicleprocess
    {
        struct Point2f
        {
        public:
            Point2f() : x(0), y(0) {}
            float x;
            float y;
        };
        struct FusedVehicleOutput
        {
        public:
            FusedVehicleOutput()
            {
                frameinfo2d = FrameInfo2D();
                for (int i = 0; i < 100; ++i)
                {
                    length_[i] = {0};
                    width_[i] = {0};
                    height_[i] = {0};
                    yaw_[i] = {0};
                }
            }

            FrameInfo2D frameinfo2d;
            float length_[100];
            float width_[100];
            float height_[100];
            float yaw_[100];
        };
        class TrackingCubeStatusFrame
        {
        public:
            TrackingCubeStatusFrame()
            {
                rect_width_ukf_ = 0.0f;     ///////////////////
                rect_width_ukf_dist_ = 50;  // RectUKF::Max_Dist;   ///
                rect_height_ukf_ = 0.0f;    /////////////////////
                rect_height_ukf_dist_ = 50; // RectUKF::Max_Dist;  ////
            }
            virtual ~TrackingCubeStatusFrame() {}

        public:
            float rect_width_ukf_;
            float rect_width_ukf_dist_;
            float rect_height_ukf_;
            float rect_height_ukf_dist_;
        };
        struct TrackingFrameInfos
        {
            std::vector<FrameInfo2D *> inputs_frame_;
            TrackingCubeStatusFrame *status_frame_;
        };
        class vehicleattribute
        {
        public:
            vehicleattribute();
            ~vehicleattribute();
            void Calcattribute(FrameInfo2D &image_msg);
            void SetRectStaByUKF(FrameInfo2D &image_msg);
            float CalIOU(BBox_2d &box1, BBox_2d &box2);
            Point2f CvtImageToVcsGnd(Point2f &pt);
            FrameInfo2D *GetPrevCurInputFrame(int input_idx);
            FrameInfo2D *GetPrevInputFrameByIdx(int idx);
            //  TrackingCubeStatusFrame* GetLastStatusFrame()   {
            //   if (frame_info_buf_) {
            //     return frame_info_buf_->GetWriteBuff()->status_frame_;
            //   }
            //   return nullptr;
            // }
            void GetVehWidthByType(Vehicle2D &Vehicle, float &veh_w_classify);
            void GetVehheightByType(Vehicle2D &Vehicle, float &veh_h_classify);
            float SetVehHeight(Vehicle2D &Vehicle, BBox_2d &Bbox);
            float SetVehWidth(Vehicle2D &Vehicle, BBox_2d &Bbox);
            void RunRectUKF(FrameInfo2D &image_msg);
            void GetVehheigthByType(Vehicle2D &Vehicle, float &veh_h_classify);
            void GetVehLeightRangeByType(Vehicle2D &Vehicle, float &min_value_,
                                         float &mid_value_, float &max_value_);
            bool InitVehLByWidth(Vehicle2D &Vehicle, float W, float &length);
            bool GetVehLenByBoardsidePoints(Vehicle2D &Vehicle, float &length,
                                            float &min_value_, float &mid_value_,
                                            float &max_value_);
            float ComputeBoardSideYaw(Vehicle2D &Vehicle);
            float ReGet3dInfoByClusteringConeForCase(Vehicle2D &Vehicle, float W,
                                                     float L);
            float SetVehLength(Vehicle2D &Vehicle, float W);
            void ComputeBoardSideYaw(FrameInfo2D *imagemssage);

        private:
            // RectUKF *Ukf_rect_;
            FusedVehicleOutput VehOutput_;
            RingBuffer<TrackingFrameInfos> *frame_info_buf_;
            TrackingCubeStatusFrame *status_frame[100];
        };
    } // namespace vehicleprocess
} // namespace pg