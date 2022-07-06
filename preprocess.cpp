//#include "krider/postfusion/algo/vehicle/include/vehicleattribute.h"
#include <iostream>
#include "preprocess.h"
#define TINY_VEHICLE_WIDTH 1.3f
#define TINY_VEHICLE_HEIGHT 1.5f
#define MINI_VEHICLE_WIDTH 1.6f
#define MINI_VEHICLE_HEIGHT 1.5f
#define VEHICLE_WIDTH 1.8f
#define VEHICLE_HEIGHT 1.50f
#define TWO_WIDTH 2.0f
#define BUS_WIDTH 2.3f
#define BUS_HEIGHT 3.0f
#define BIG_CAR_WIDTH 2.5f
#define BIG_CAR_HEIGHT 2.7f
#define TINY_VEHICLE_LENGTH 2.5f
#define MINI_VEHICLE_LENGTH 4.0f
#define VEHICLE_LENGTH 4.5f
#define BUS_LENGTH 10.0f
#define BIG_CAR_LENGTH 12.5f
#define M_PI 3.14159265358979323846
namespace pg
{
    namespace vehicleprocess
    {
        vehicleattribute::vehicleattribute()
        {
            // frame_info_buf_ = new RingBuffer<TrackingFrameInfo>(new TrackingFrameInfo);
            frame_info_buf_ = nullptr; // static_cast<pg::preprocess*>(nullptr);
            // Ukf_rect_ = nullptr;
            // VehOutput_.FusedVehicleOutput();
            for (int i = 0; i < 100; ++i)
            {
                status_frame[i] = new TrackingCubeStatusFrame;
            }
            // TrackingCubeVehObject_ptr_ = nullptr;
        }

        vehicleattribute::~vehicleattribute()
        {
            // if (Ukf_rect_ != NULL) {
            //   delete Ukf_rect_;
            //   Ukf_rect_ = NULL;
            // }
            if (frame_info_buf_ != NULL)
            {
                delete frame_info_buf_;
                frame_info_buf_ = NULL;
            }
            for (int i = 0; i < 100; ++i)
            {
                if (status_frame[i] != NULL)
                {
                    delete status_frame[i];
                    status_frame[i] = NULL;
                }
            }
        }
        FrameInfo2D *vehicleattribute::GetPrevCurInputFrame(int input_idx)
        {
            return GetPrevInputFrameByIdx(input_idx);
        }

        FrameInfo2D *vehicleattribute::GetPrevInputFrameByIdx(int idx)
        {
            // frame_info_buf_ = new RingBuffer<TrackingFrameInfos>(new
            // TrackingFrameInfos);
            if (idx < 100.f && frame_info_buf_ && frame_info_buf_->Pop())
            {
                return frame_info_buf_->Pop()->inputs_frame_[idx];
            }
            return nullptr;
        }
        void vehicleattribute::Calcattribute(FrameInfo2D &image_msg)
        {
            if (image_msg.in_use)
            {

                RunRectUKF(image_msg);
                SetRectStaByUKF(image_msg);

                // SetVehLength(&image_msg);
                // SetVehHeight(&image_msg);
                // SetVehWidth(&image_msg);
                // object_ptr_ = std::make_shared<pg::VehUKF>();
                // for (int i = 0; i < image_msg.vehicles_num; ++i) {
                //   VehOutput_.measure_[i].h_ = VehOutput_.height_[i];
                //   VehOutput_.measure_[i].w_ = VehOutput_.width_[i];
                //   VehOutput_.measure_[i].yaw_ = VehOutput_.yaw_[i];
                // object_ptr_->Run1(&VehOutput_.state_result_[i],
                // &VehOutput_.state_result_raw_[i], &VehOutput_.measure_[i]);
            }
            else
            {
                return;
            }
        }

        void vehicleattribute::RunRectUKF(FrameInfo2D &image_msg)
        {
            for (int i = 0; i < image_msg.vehicles_num; ++i)
            {
                if (image_msg.vehicles[i].main_box.score > 0.5)
                {
                    // TrackingCubeStatusFrame *status_frame =
                    //     static_cast<TrackingCubeStatusFrame *>(
                    //         GetLastStatusFrame());

                    uint64_t frame_id = image_msg.frame_id;
                    uint64_t timestamp = image_msg.timestamp;
                    bool valid_measure = true;
                    float im_width = 1920.0f;
                    float im_height = 1080.0f;
                    float border = 2.f;
                    float width = 0;
                    float height = 0;
                    if (image_msg.vehicles[i].has_rear_box)
                    {
                        width = image_msg.vehicles[i].rear_box.x2 -
                                image_msg.vehicles[i].rear_box.x1;
                        height = image_msg.vehicles[i].rear_box.y2 -
                                 image_msg.vehicles[i].rear_box.y1;
                        if (image_msg.vehicles[i].rear_box.x1 < border ||
                            image_msg.vehicles[i].rear_box.y1 < border ||
                            image_msg.vehicles[i].rear_box.x2 > im_width - border ||
                            image_msg.vehicles[i].rear_box.y2 > im_height - border ||
                            width < 1e-6f || height < 1e-6f)
                        {
                            valid_measure = false;
                        }
                    }
                    else if (image_msg.vehicles[i].has_front_box)
                    {
                        width = image_msg.vehicles[i].front_box.x2 -
                                image_msg.vehicles[i].front_box.x1;
                        height = image_msg.vehicles[i].front_box.y2 -
                                 image_msg.vehicles[i].front_box.y1;
                        if (image_msg.vehicles[i].front_box.x1 < border ||
                            image_msg.vehicles[i].front_box.y1 < border ||
                            image_msg.vehicles[i].front_box.x2 > im_width - border ||
                            image_msg.vehicles[i].front_box.y2 > im_height - border ||
                            width < 1e-6f || height < 1e-6f)
                        {
                            valid_measure = false;
                        }
                    }
                    else
                    {
                    }
                    float cover_score = image_msg.vehicles[i].occluded;

                    if (cover_score > 0.5f) //遮挡分数判断是不是有效测量
                    {
                        valid_measure = false;
                    }
                    // Ukf_rect_ = new RectUKF();

                    // if (valid_measure) {
                    //   Ukf_rect_ = new RectUKF();
                    //   Ukf_rect_->Init(width, height, timestamp);
                    // }

                    // Ukf_rect_->frame_id_ = frame_id;
                    // if (valid_measure) {  //对有效的框 做预测
                    //   Ukf_rect_->Predict(width, height, timestamp);
                    // } else {
                    //   Mat status = Ukf_rect_->GetState();
                    //   Ukf_rect_->Predict(status.ptr<float>(0)[0], status.ptr<float>(1)[0],
                    //                      timestamp);
                    // }

                    // if (valid_measure) {
                    //   Ukf_rect_->Correct(width, height);  // 矫正
                    // }

                    // if (Ukf_rect_) {  // 对结果赋值
                    //   Mat res = Ukf_rect_->GetState();
                    //   status_frame[i]->rect_width_ukf_ = res.ptr<float>(0)[0];
                    //   status_frame[i]->rect_height_ukf_ = res.ptr<float>(1)[0];
                    //   std::pair<float, float> w_h_dist =
                    //       Ukf_rect_->CalcDistByErrorCov(width, height);
                    //   status_frame[i]->rect_width_ukf_dist_ = w_h_dist.first;
                    //   status_frame[i]->rect_height_ukf_dist_ = w_h_dist.second;
                    // }
                }
            }
        }

        void vehicleattribute::SetRectStaByUKF(FrameInfo2D &image_msg)
        {
            BBox_2d trackbox[100];
            BBox_2d inputbox[100];
            for (int i = 0; i < image_msg.vehicles_num; ++i)
            {
                if (image_msg.vehicles[i].main_box.score > 0.5)
                {
                    // if (Ukf_rect_) {

                    // float half_w = status_frame[i]->rect_width_ukf_ * 0.5f;
                    // float half_h = status_frame[i]->rect_height_ukf_ * 0.5f;
                    //  float half_w,half_h;
                    //  if (image_msg.vehicles[i].has_rear_box ){
                    //      half_w = (image_msg.vehicles[i].rear_box.x2 -
                    //      image_msg.vehicles[i].rear_box.x1) * 0.5f; half_h =
                    //      (image_msg.vehicles[i].rear_box.y2 -
                    //      image_msg.vehicles[i].rear_box.y1) * 0.5f;
                    //  }
                    //  else{
                    //      half_w = (image_msg.vehicles[i].front_box.x2 -
                    //      image_msg.vehicles[i].front_box.x1) * 0.5f; half_h =
                    //      (image_msg.vehicles[i].front_box.y2 -
                    //      image_msg.vehicles[i].front_box.y1) * 0.5f;
                    //  }
                    float cx, cy, half_w, half_h;
                    if (image_msg.vehicles[i].has_rear_box)
                    {
                        std::cout << "has_rear_box = " << image_msg.vehicles[i].has_rear_box << std::endl;
                        cx = (image_msg.vehicles[i].rear_box.x1 +
                              image_msg.vehicles[i].rear_box.x2) /
                             2;
                        cy = (image_msg.vehicles[i].rear_box.y1 +
                              image_msg.vehicles[i].rear_box.y2) /
                             2;
                        std::cout << "cx=" << cx << std::endl;
                        half_w = (image_msg.vehicles[i].rear_box.x2 -
                                  image_msg.vehicles[i].rear_box.x1) *
                                 0.5;
                        half_h = (image_msg.vehicles[i].rear_box.y2 -
                                  image_msg.vehicles[i].rear_box.y1) *
                                 0.5;
                        if (cx > 1.f && cy > 1.f && half_w > 1.f && half_h > 1.f)
                        {
                            trackbox[i].x1 = cx - half_w;
                            trackbox[i].x2 = cx + half_w;
                            trackbox[i].y1 = cy - half_h;
                            trackbox[i].y2 = cy + half_h;
                            inputbox[i] = image_msg.vehicles[i].rear_box;
                        }
                    }
                    else if (image_msg.vehicles[i].has_front_box)
                    {
                        cx = (image_msg.vehicles[i].front_box.x1 +
                              image_msg.vehicles[i].front_box.x2) /
                             2;
                        cy = (image_msg.vehicles[i].front_box.y1 +
                              image_msg.vehicles[i].front_box.y2) /
                             2;
                        half_w = (image_msg.vehicles[i].rear_box.x2 -
                                  image_msg.vehicles[i].rear_box.x1) *
                                 0.5;
                        half_h = (image_msg.vehicles[i].rear_box.y2 -
                                  image_msg.vehicles[i].rear_box.y1) *
                                 0.5;
                        if (cx > 1.f && cy > 1.f && half_w > 1.f && half_h > 1.f)
                        {
                            trackbox[i].x1 = cx - half_w;
                            trackbox[i].x2 = cx + half_w;
                            trackbox[i].y1 = cy - half_h;
                            trackbox[i].y2 = cy + half_h;
                            inputbox[i] = image_msg.vehicles[i].front_box;
                        }
                    }
                    else
                    {
                    }
                    //}

                    float iou = CalIOU(trackbox[i], inputbox[i]);
                    if (iou > 0.9f)
                    {
                        status_frame[i]->rect_width_ukf_ = trackbox[i].x2 - trackbox[i].x1;
                        status_frame[i]->rect_height_ukf_ = trackbox[i].y2 - trackbox[i].y1;
                        inputbox[i] = trackbox[i];
                    }
                    else if (iou > 0.7f)
                    {
                        float r_ukf = iou;
                        float r_obs = 1.f - iou;
                        status_frame[i]->rect_width_ukf_ =
                            (trackbox[i].x2 - trackbox[i].x1) * r_ukf +
                            (inputbox[i].x2 - inputbox[i].x1) * r_obs;
                        status_frame[i]->rect_height_ukf_ =
                            (trackbox[i].y2 - trackbox[i].y1) * r_ukf +
                            (inputbox[i].y2 - inputbox[i].y1) * r_obs;
                        inputbox[i].x1 = trackbox[i].x1 * r_ukf + inputbox[i].x1 * r_obs;
                        std::cout << "inputbox[i].x1 = " << inputbox[i].x1 << std::endl;
                        inputbox[i].x2 = trackbox[i].x2 * r_ukf + inputbox[i].x2 * r_obs;
                        std::cout << " inputbox[i].x2 = " << inputbox[i].x2 << std::endl;
                        inputbox[i].y1 = trackbox[i].y1 * r_ukf + inputbox[i].y1 * r_obs;
                        std::cout << "inputbox[i].y1 = " << inputbox[i].y1 << std::endl;
                        inputbox[i].y2 = trackbox[i].y2 * r_ukf + inputbox[i].y2 * r_obs;
                        std::cout << "inputbox[i].y2 = " << inputbox[i].y2 << std::endl;
                    }
                    else
                    {
                        status_frame[i]->rect_width_ukf_ = inputbox[i].x2 - inputbox[i].x1;
                        status_frame[i]->rect_height_ukf_ = inputbox[i].y2 - inputbox[i].y1;
                    }
                    VehOutput_.width_[i] = SetVehWidth(image_msg.vehicles[i], inputbox[i]);
                    std::cout << "VehOutput_.width_[i] = " << VehOutput_.width_[i] << std::endl;
                    VehOutput_.height_[i] = SetVehHeight(image_msg.vehicles[i], inputbox[i]);
                    std::cout << "VehOutput_.height_[i] = " << VehOutput_.height_[i] << std::endl;
                    VehOutput_.length_[i] =
                        SetVehLength(image_msg.vehicles[i], VehOutput_.width_[i]);
                    std::cout << " VehOutput_.length_[i] = " << VehOutput_.length_[i] << std::endl;
                    VehOutput_.yaw_[i] = ReGet3dInfoByClusteringConeForCase(
                        image_msg.vehicles[i], VehOutput_.width_[i], VehOutput_.length_[i]);
                    std::cout << "   VehOutput_.yaw_[i] =  " << VehOutput_.yaw_[i] << std::endl;
                }
            }
        }

        float vehicleattribute::CalIOU(BBox_2d &box1, BBox_2d &box2)
        {
            // LOG(INFO) << "start to calc IOU "
            //<< "\n";
            float area1 = (box1.x2 - box1.x1) * (box1.y2 - box1.y1);
            float area2 = (box2.x2 - box2.x1) * (box2.y2 - box2.y1);
            float interarea = (box1.x2 - box2.x1) * (box1.y2 - box2.y1);
            float iou = interarea / (area1 + area2 - interarea);
            return iou;
        }
        Point2f vehicleattribute::CvtImageToVcsGnd(Point2f &pt)
        {
            Point2f pt2;
            float p[9] = {3719.81893613484, 0.f, 879.914688308125,
                          0.f, 3740.80441113661, 519.071597474487,
                          0.f, 0.f, 1.f};
            float t = p[6] * pt.x + p[7] * pt.y + p[8];
            pt2.x = (p[0] * pt.x + p[1] * pt.y + p[2]) / t;
            pt2.y = (p[3] * pt.x + p[4] * pt.y + p[5]) / t;
            return pt2;
        }
        void vehicleattribute::ComputeBoardSideYaw(FrameInfo2D *imagemssage)
        {
            if (imagemssage == nullptr)
            {
                return;
            }
            // float yaw[imagemssage->vehicles_num]={0};
            int vehicles_num = static_cast<int>(imagemssage->vehicles_num);
            for (int i = 0; i < vehicles_num; ++i)
            {
                if (imagemssage->vehicles[i].vehicle2d_point_num > 1.f)
                {
                    if ((imagemssage->vehicles[i].earch_points[0].visible &&
                         imagemssage->vehicles[i].earch_points[1].visible) &&
                        imagemssage->vehicles[i].earch_points[0].score > 0.5 &&
                        imagemssage->vehicles[i].earch_points[1].score > 0.5)
                    {
                        Point2f pt0, pt1;
                        pt0.x = imagemssage->vehicles[i].earch_points[0].x;
                        pt0.y = imagemssage->vehicles[i].earch_points[0].y;
                        pt1.x = imagemssage->vehicles[i].earch_points[1].x;
                        pt1.y = imagemssage->vehicles[i].earch_points[1].y;
                        pt0 = CvtImageToVcsGnd(pt0);
                        pt1 = CvtImageToVcsGnd(pt1);
                        VehOutput_.yaw_[i] = std::atan2(pt0.y - pt1.y, pt0.x - pt1.x);
                    }
                }
                else
                {
                    VehOutput_.yaw_[i] = 0.0001f;
                }
                // imagemssage->vehicles[i].yaw = yaw[i];
            }
        }
        float vehicleattribute::SetVehLength(Vehicle2D &Vehicle, float W)
        {
            // float length[imagemssage->vehicles_num]={0};
            float length = 0;
            if (Vehicle.main_box.score > 0.5)
            {
                BBox_2d pt = Vehicle.main_box;
                Point2f pt1, pt2;
                pt1.x = pt.x1;
                pt1.y = pt.y1;
                pt2.x = pt.x2;
                pt2.y = pt.y2;
                pt1 = CvtImageToVcsGnd(pt1);
                pt2 = CvtImageToVcsGnd(pt2);
                float length1 = pt2.x - pt1.x;
                float length2 = 0;
                InitVehLByWidth(Vehicle, W, length2);
                float min_value_, mid_value_, max_value_;
                float length3 = 0;
                GetVehLeightRangeByType(Vehicle, min_value_, mid_value_, max_value_);
                if (GetVehLenByBoardsidePoints(Vehicle, length, min_value_, mid_value_,
                                               max_value_))
                {
                    length3 = length;
                }
                else
                {
                    length3 = length1;
                }
                length = (length3 + length2) / 2;
            }
            else
            {
                length = 5.5;
            }
            // LOG(INFO) << "length[i]: " << length << "\n";
            //  imagemssage->vehicles[i].length = length[i];

            return length;
        }
        float vehicleattribute::SetVehWidth(Vehicle2D &Vehicle, BBox_2d &Bbox)
        {
            float width;
            // float width[imagemssage->vehicles_num]={0};
            // for (int i =0;i<imagemssage->vehicles_num;++i){
            //  if(imagemssage->vehicles[i].has_rear_box&&
            //  imagemssage->vehicles[i].rear_box.score>0.5){
            // BBox_2d pt = imagemssage->vehicles[i].rear_box;
            BBox_2d pt = Bbox;
            Point2f pt1, pt2;
            pt1.x = pt.x1;
            pt1.y = pt.y1;
            pt2.x = pt.x2;
            pt2.y = pt.y2;
            pt1 = CvtImageToVcsGnd(pt1);
            pt2 = CvtImageToVcsGnd(pt2);
            width = pt2.x - pt1.x;
            width = std::min(BIG_CAR_WIDTH + 0.5f, width);
            float veh_w_classify = VEHICLE_WIDTH;
            GetVehWidthByType(Vehicle, veh_w_classify);
            std::cout << "veh_w_classify=" << veh_w_classify << std::endl;
            if (Vehicle.rear_box.score > 0.8 || Vehicle.front_box.score > 0.8)
            {
                width = width;
            }
            else if (Vehicle.cate_score > 0.8)
            {
                width = veh_w_classify;
            }
            else
            {
                width =
                    width * (1 - Vehicle.cate_score) + veh_w_classify * Vehicle.cate_score;
            }
            // VehOutput_.width_[i]= pt.x2 - pt.x1;
            // VehOutput_.width_[i] = std::min(3+ 0.5f, VehOutput_.width_[i]);
            //}
            // else if (imagemssage->vehicles[i].has_front_box&&
            // imagemssage->vehicles[i].front_box.score>0.5){
            //   BBox_2d pt = imagemssage->vehicles[i].front_box;
            //   //float *p = reinterpret_cast<float *>(Qinv_.data);
            //   float p[9] = {-0.213887259,0.000447383267,-1.89844968e-06,
            //   0.667621017,-8.44449896e-05,-0.000671758549,
            //   0.654391408,0.000701600628,8.14549276e-06};
            //     float t1 = p[6] * pt.x1 + p[7] * pt.y2 + p[8];
            //     pt.x1 = (p[0] * pt.x1 + p[1] * pt.y2 + p[2]) / t1;
            //     float t2 = p[6] * pt.x2 + p[7] * pt.y1 + p[8];
            //     pt.x2 = (p[0] * pt.x2 + p[1] * pt.y1 + p[2]) / t2;
            //    VehOutput_.width_[i]= pt.x2 - pt.x1;
            //    VehOutput_.width_[i] = std::min(3 + 0.5f, VehOutput_.width_[i]);
            // }
            //   else{
            //     VehOutput_.width_[i] = 3;
            //   }
            // LOG(INFO) << "width[i]: "<< VehOutput_.width_[i]<<"\n";
            // imagemssage->vehicles[i].width = width[i];
            //}

            return width;
        }
        float vehicleattribute::SetVehHeight(Vehicle2D &Vehicle, BBox_2d &Bbox)
        {
            float height;
            // float height[imagemssage->vehicles_num]={0};
            //   for (int i =0;i<imagemssage->vehicles_num;++i){
            //      if(imagemssage->vehicles[i].has_rear_box&&
            //      imagemssage->vehicles[i].rear_box.score>0.5){
            // BBox_2d pt = imagemssage->vehicles[i].rear_box;
            BBox_2d pt = Bbox;
            Point2f pt1, pt2;
            pt1.x = pt.x1;
            pt1.y = pt.y1;
            pt2.x = pt.x2;
            pt2.y = pt.y2;
            pt1 = CvtImageToVcsGnd(pt1);
            pt2 = CvtImageToVcsGnd(pt2);
            height = pt2.y - pt1.y;
            // float avg_ratio = status_frame[i]->rect_height_ukf_ /
            // status_frame[i]->rect_width_ukf_;
            float veh_h_classify = VEHICLE_HEIGHT;
            GetVehheightByType(Vehicle, veh_h_classify);
            if (Vehicle.rear_box.score > 0.8 || Vehicle.front_box.score > 0.8)
            {
                height = height;
            }
            else if (Vehicle.cate_score > 0.8)
            {
                height = veh_h_classify;
            }
            else
            {
                height =
                    height * (1 - Vehicle.cate_score) + veh_h_classify * Vehicle.cate_score;
            }

            //    VehOutput_.height_[i]= pt.y2 - pt.y1;
            //    VehOutput_.height_[i] = std::min(4 + 0.5f, VehOutput_.height_[i]);
            // }
            // else if (imagemssage->vehicles[i].has_front_box&&
            // imagemssage->vehicles[i].front_box.score>0.5){
            //  BBox_2d pt = imagemssage->vehicles[i].front_box;
            // float *p = reinterpret_cast<float *>(Qinv_.data);
            //   float p[9] = {-0.213887259,0.000447383267,-1.89844968e-06,
            //   0.667621017,-8.44449896e-05,-0.000671758549,
            //   0.654391408,0.000701600628,8.14549276e-06};
            //     float t1 = p[6] * pt.y1 + p[7] * pt.x2 + p[8];
            //     pt.y1 = (p[0] * pt.y1 + p[1] * pt.x2 + p[2]) / t1;
            //     float t2 = p[6] * pt.y2 + p[7] * pt.x1 + p[8];
            //     pt.y2 = (p[0] * pt.y2 + p[1] * pt.x1 + p[2]) / t2;
            //    VehOutput_.height_[i]= pt.y2 - pt.y1;
            //    VehOutput_.height_[i] = std::min(3 + 0.5f, VehOutput_.height_[i]);
            // }
            // else{
            //   VehOutput_.height_[i] = 3;
            // }
            // LOG(INFO) << "height[i]: "<< VehOutput_.height_[i]<<"\n";
            // imagemssage->vehicles[i].height = height[i];
            //}
            return height;
        }
        void vehicleattribute::GetVehWidthByType(Vehicle2D &Vehicle,
                                                 float &veh_w_classify)
        {
            switch (Vehicle.cate)
            {
            case 6:
            {
                veh_w_classify = VEHICLE_WIDTH;
                break;
            }
            case 8:
            {
                veh_w_classify = BUS_WIDTH;
                break;
            }
            case 7:
            {
                veh_w_classify = BIG_CAR_WIDTH;
                break;
            }
            case 5:
            {
                veh_w_classify = TINY_VEHICLE_WIDTH;
                break;
            }
            case 9:
            {
                veh_w_classify = BIG_CAR_WIDTH;
                break;
            }
            default:
            {
                veh_w_classify = VEHICLE_WIDTH;
                break;
            }
            }
        }
        void vehicleattribute::GetVehheightByType(Vehicle2D &Vehicle,
                                                  float &veh_h_classify)
        {
            switch (Vehicle.cate)
            {
            case 6:
            {
                veh_h_classify = VEHICLE_HEIGHT;
                break;
            }
            case 8:
            {
                veh_h_classify = BUS_HEIGHT;
                break;
            }
            case 7:
            {
                veh_h_classify = BIG_CAR_HEIGHT;
                break;
            }
            case 5:
            {
                veh_h_classify = TINY_VEHICLE_HEIGHT;
                break;
            }
            case 9:
            {
                veh_h_classify = BIG_CAR_HEIGHT;
                break;
            }
            default:
            {
                veh_h_classify = VEHICLE_HEIGHT;
                break;
            }
            }
        }
        void vehicleattribute::GetVehLeightRangeByType(Vehicle2D &Vehicle,
                                                       float &min_value_,
                                                       float &mid_value_,
                                                       float &max_value_)
        {
            switch (Vehicle.cate)
            {
            case 6:
            {
                min_value_ = 3.8f;
                mid_value_ = 4.3f;
                max_value_ = 5.5f;
                break;
            }
            case 8:
            {
                min_value_ = 7.0f;
                mid_value_ = 10.0f;
                max_value_ = 12.0f;
                break;
            }
            case 7:
            {
                min_value_ = 5.0f;
                mid_value_ = 11.5f;
                max_value_ = 18.0f;
                break;
            }
            case 9:
            {
                min_value_ = 4.0f;
                mid_value_ = 5.5f;
                max_value_ = 6.0f;
                break;
            }
            case 5:
            {
                min_value_ = 2.3f;
                mid_value_ = 3.0f;
                max_value_ = 3.8f;
                break;
            }
            default:
            {
                min_value_ = 3.5f;
                mid_value_ = 6.0f;
                max_value_ = 11.5f;
                break;
            }
            }
        }
        bool vehicleattribute::InitVehLByWidth(Vehicle2D &Vehicle, float W,
                                               float &length)
        {
            if (W < 0.1f)
            {
                return false;
            }

            float slope = 0.0f, l_by_inter = 0.0f;
            float width_array[2] = {0.0f}, length_array[2] = {0.0f};
            float min_value_, mid_value_, max_value_;
            GetVehLeightRangeByType(Vehicle, min_value_, mid_value_, max_value_);
            struct length_range
            {
                float min_value_;
                float max_value_;
            } length_range;
            length_range.min_value_ = min_value_;
            length_range.max_value_ = max_value_;
            switch (Vehicle.cate)
            {
            case 6:
            {
                width_array[0] = 1.5f;
                width_array[1] = 2.0f;
                length_array[0] = length_range.min_value_;
                length_array[1] = length_range.max_value_;
                break;
            }
            case 8:
            {
                width_array[0] = 2.0f;
                width_array[1] = 2.8f;
                length_array[0] = length_range.min_value_;
                length_array[1] = length_range.max_value_;
                break;
            }
            case 7:
            {
                width_array[0] = 1.8f;
                width_array[1] = 2.8f;
                length_array[0] = 4.2f;
                length_array[1] = 12.0f;
                break;
            }
            case 9:
            {
                width_array[0] = 1.6f;
                width_array[1] = 2.0f;
                length_array[0] = length_range.min_value_;
                length_array[1] = length_range.max_value_;
                break;
            }
            case 5:
            {
                width_array[0] = 1.0f;
                width_array[1] = 1.7f;
                length_array[0] = length_range.min_value_;
                length_array[1] = length_range.max_value_;
                break;
            }
            default:
            {
                width_array[0] = 1.0f;
                width_array[1] = 2.8f;
                length_array[0] = 2.3f;
                length_array[1] = 10.0f;
                break;
            }
            }

            W = std::max(W, width_array[0]);
            W = std::min(W, width_array[1]);

            slope =
                (length_array[1] - length_array[0]) / (width_array[1] - width_array[0]);
            l_by_inter = length_array[0] + slope * (W - width_array[0]);
            length = l_by_inter;

            return true;
        }
        bool vehicleattribute::GetVehLenByBoardsidePoints(Vehicle2D &Vehicle,
                                                          float &length,
                                                          float &min_value_,
                                                          float &mid_value_,
                                                          float &max_value_)
        {
            float yaw_cam = 0.0f;
            const float yaw_thrld = 0.5f;
            float dist_pts = 0.0f, dist_inters = 0.0f, img_dist_pts = 0.0f,
                  max_scale = 1.5f;
            float slope = 0.0f, dx = 0.0f, dy = 0.0f;
            Point2f pt_front, pt_back, pt_front_vcs, pt_back_vcs;
            bool is_car = !(Vehicle.cate == 7 || Vehicle.cate == 8 || Vehicle.cate == 9);

            float length_min = min_value_;
            float length_mid = mid_value_;
            float length_max = max_value_;
            bool cut_status = Vehicle.occluded > 0.5 ? true : false;

            if (cut_status && !(Vehicle.has_rear_box || Vehicle.has_front_box))
            {
                return false;
            }
            if (Vehicle.earch_points[0].visible && Vehicle.earch_points[1].visible)
            {
                pt_front.x = Vehicle.earch_points[0].x;
                pt_front.y = Vehicle.earch_points[0].y;
                pt_back.x = Vehicle.earch_points[1].x;
                pt_back.y = Vehicle.earch_points[1].y;
                dx = pt_front.x - pt_back.x;
                dy = pt_front.y - pt_back.y;
                img_dist_pts = sqrtf(dx * dx + dy * dy);
                if (img_dist_pts < 10)
                {
                    return false;
                }
                pt_front_vcs = CvtImageToVcsGnd(pt_front);
                pt_back_vcs = CvtImageToVcsGnd(pt_back);
                dx = pt_front_vcs.x - pt_back_vcs.x;
                dy = pt_front_vcs.y - pt_back_vcs.y;
                dist_pts = sqrtf(dx * dx + dy * dy);
                if (dist_pts > length_max)
                {
                    return false;
                }

                dx = pt_front.x - pt_back.x;
                dy = pt_front.y - pt_back.y;
                if (fabsf(dx) < 0.0000001f)
                {
                    slope = 0.0f;
                }
                else
                {
                    slope = dy / dx;
                }

                pt_front.x = Vehicle.main_box.x1;
                pt_front.y = slope * ((Vehicle.main_box.x1 - pt_back.x) + pt_back.y);
                pt_back.x = Vehicle.main_box.x2;
                pt_back.y = slope * ((Vehicle.main_box.x2 - pt_back.x) + pt_back.y);

                pt_front_vcs = CvtImageToVcsGnd(pt_front);
                pt_back_vcs = CvtImageToVcsGnd(pt_back);
                dx = pt_front_vcs.x - pt_back_vcs.x;
                dy = pt_front_vcs.y - pt_back_vcs.y;
                dist_inters = sqrtf(dx * dx + dy * dy);

                if (dist_inters < length_min)
                {
                    return false;
                }

                if (dist_inters > length_max * max_scale)
                {
                    return false;
                }

                yaw_cam = dx < 0.0001f ? M_PI / 2.0f : atan2f(dy, dx);
                yaw_cam = fabsf(yaw_cam);
                if (is_car && yaw_cam < yaw_thrld)
                {
                    if (dist_inters < length_min)
                    {
                        dist_inters = dist_inters + 0.5f;
                    }
                    else if (dist_inters < length_mid)
                    {
                        dist_inters = dist_inters + 0.2f;
                    }
                    else
                    {
                    }
                }

                length = std::max(dist_inters, length_min);
                length = std::min(length, length_max);
            }
            else
            {
                return false;
            }
            return true;
        }

        float vehicleattribute::ReGet3dInfoByClusteringConeForCase(Vehicle2D &Vehicle,
                                                                   float W, float L)
        {
            float yaw_vcs;
            float pt_a = 0.0f;
            float pt_b = 0.0f;
            float pt_c = 0.0f;
            float norm_a, norm_b, norm_c, norm_l;
            float len_a = 0.0f;
            float len_b = 0.0f;
            float len_bias = 0.0f;
            // float f_u = camera->focal_u_;            // 光芯到成像的距离
            // float vp_x = -camera->optical_center_.x; // 光芯的x坐标
            float f_u = 3;   // 光芯到成像的距离
            float vp_x = -2; // 光芯的x坐标
            float sita = 0.0f;
            // float sitagp[2];
            // float sita_a, sita_b;
            float cossita_a, sinsita_a, cossita_b, sinsita_b, cossita_c, sinsita_c,
                cossita_l = 0.0f, sinsita_l = 0.0f;
            float k1, k2, k3, k4, k5, k6;
            float para_A, para_B;

            // if (type == CLUSTERING_RECT_CASE1 ||
            // type == CLUSTERING_RECT_CASE2)
            //{
            // if (Vehicle.rear_box.score < 0.5 || Vehicle.front_box.score < 0.5)
            // {
            //   rect_sub = Rect<float>(rect_main.l,
            //                          rect_main.t, rect_main.l, rect_main.b);
            // }
            // pt_a = (rect_main.l + rect_sub.l) / 2.0f;
            pt_a = Vehicle.main_box.x1;
            pt_b = Vehicle.main_box.x2; //得到在光芯下的a b c点
            pt_c = Vehicle.rear_box.x2;
            len_a = W;
            len_b = L - len_bias; //车尾框与全车框不能完全贴合，所以在实际计中，
            //需要把对应边和对应角度减掉即为len_bias，此处默认为0，也就是结合的
            norm_l = sqrtf(len_a * len_a + len_bias * len_bias);
            cossita_l = len_a / norm_l;
            sinsita_l = len_bias / norm_l;
            //}
            /*
            if (type == CLUSTERING_RECT_CASE3 ||
                type == CLUSTERING_RECT_CASE4)
            {
              if (sub_miss)
              {
                rect_sub = Rect<float>(rect_main.r,
                                       rect_main.t, rect_main.r, rect_main.b);
              }
              pt_a = rect_main.l;
              pt_b = (rect_main.r + rect_sub.r) / 2.0f;
              pt_c = rect_sub.l;
              len_a = L - len_bias;
              len_b = W;
              norm_l = sqrtf(len_b * len_b + len_bias * len_bias);
              cossita_l = len_b / norm_l;
              sinsita_l = len_bias / norm_l;
            }*/
            pt_a = -pt_a;
            pt_b = -pt_b;
            pt_c = -pt_c;
            norm_a = std::sqrt(
                f_u * f_u +
                (pt_a - vp_x) *
                    (pt_a - vp_x)); // 实际是vp_x - pt_a 也就是a到光芯o的横向距离
            norm_b = std::sqrt(f_u * f_u + (pt_b - vp_x) * (pt_b - vp_x));
            norm_c = std::sqrt(f_u * f_u + (pt_c - vp_x) * (pt_c - vp_x));
            cossita_a = f_u / norm_a;
            sinsita_a = (pt_a - vp_x) / norm_a;
            cossita_b = f_u / norm_b;
            sinsita_b = (pt_b - vp_x) / norm_b;
            cossita_c = f_u / norm_c;
            sinsita_c = (pt_c - vp_x) / norm_c;
            k1 = -sinsita_b * cossita_c + cossita_b * sinsita_c;
            k4 = -sinsita_a * cossita_c + cossita_a * sinsita_c;
            k2 = cossita_b * len_b;
            k3 = -sinsita_b * len_b;
            k5 = sinsita_a * len_a;
            k6 = cossita_a * len_a;

            // if (type == CLUSTERING_RECT_CASE1 ||
            //     type == CLUSTERING_RECT_CASE2)
            // {
            k5 = sinsita_a * len_a * cossita_l - cossita_a * len_a * sinsita_l;
            k6 = cossita_a * len_a * cossita_l + sinsita_a * len_a * sinsita_l;
            //}
            // if (type == CLUSTERING_RECT_CASE3 ||
            //     type == CLUSTERING_RECT_CASE4)
            // {
            k2 = cossita_b * len_b * cossita_l - sinsita_b * len_b * sinsita_l;
            k3 = -sinsita_b * len_b * cossita_l - cossita_b * len_b * sinsita_l;
            //}

            para_A = k2 * k4 - k5 * k1;
            para_B = k3 * k4 - k6 * k1;
            sita = std::atan(para_B / para_A);
            //往下这块没用 因为第三个参数为0，计算结果为false;
            // if (SovTrigonEquByPara(para_A, para_B, 0.0f, sitagp))
            // {
            //   sita_a = CalArcAngle(sinsita_a, cossita_a, use_map);
            //   sita_b = CalArcAngle(sinsita_b, cossita_b, use_map);
            //   int val_cnt = 0;
            //   for (int i = 0; i < 2; i++)
            //   {
            //     float sita_t = sitagp[i];
            //     if (sita_t <= sita_a &&
            //         sita_t <= sita_b &&
            //         sita_t >= sita_a - PI / 2.0f &&
            //         sita_t >= sita_b - PI / 2.0f)
            //     {
            //       sita = sita_t;
            //       val_cnt++;
            //     }
            //   }
            //   if (val_cnt == 0)
            //   {
            //     return false;
            //   }
            //   else if (val_cnt == 2)
            //   {
            //     return false;
            //   }
            //   else
            //   {
            //   }
            // }
            // else
            // {
            //   return false;
            // }

            // if (type == CLUSTERING_RECT_CASE1 ||
            //     type == CLUSTERING_RECT_CASE2)
            // {
            // }
            // else
            // {
            // sita = sita + PI / 2.0f;
            //}
            // sita = sita + camera->yaw_ +
            //        camera->vcs_cvt_->rotation_[2]; //相机自身角度补偿
            yaw_vcs = (sita + ComputeBoardSideYaw(Vehicle)) / 2;
            // yaw_vcs = FixAngleIntoDefination(yaw_vcs); //限幅处理
            return yaw_vcs;
        }

        float vehicleattribute::ComputeBoardSideYaw(Vehicle2D &Vehicle)
        {
            Point2f pt_front, pt_back, pt_front_vcs, pt_back_vcs;
            float sita = 0;
            if (Vehicle.earch_points[0].visible && Vehicle.earch_points[1].visible)
            {
                pt_front.x = Vehicle.earch_points[0].x;
                pt_front.y = Vehicle.earch_points[0].y;
                pt_back.x = Vehicle.earch_points[1].x;
                pt_back.y = Vehicle.earch_points[1].y;
                float dx = pt_front.x - pt_back.x;
                float dy = pt_front.y - pt_back.y;
                float img_dist_pts = sqrtf(dx * dx + dy * dy);
                if (img_dist_pts < 10)
                {
                    return false;
                }
                pt_front_vcs = CvtImageToVcsGnd(pt_front);
                pt_back_vcs = CvtImageToVcsGnd(pt_back);
                dx = pt_front_vcs.x - pt_back_vcs.x;
                dy = pt_front_vcs.y - pt_back_vcs.y;
                float sita = std::tan(dy / dx);
                if (sita < -M_PI / 2)
                {
                    sita = sita + M_PI;
                }
                else if (sita > M_PI / 2)
                {
                    sita = sita - M_PI;
                }
                else
                {
                }
            }
            return sita;
            /*
            TrackingCubeVehStatusFrame *tf =
                dynamic_cast<TrackingCubeVehStatusFrame *>(
                    GetLastStatusFrame());
            if (!tf || !ClusteringOBJConfig::cls_bsyaw_cfg_.boardside_en_)
            {
              return;
            }

            Rect<float> rect_main = Rect<float>();
            if (tf->geom_info_.has_full_rect_)
            {
              rect_main = tf->geom_info_.full_rect_; //全车框赋值
            }
            else
            {
              return;
            }
            bool has_rear = false;
            Rect<float> rect_sub = rect_main;
            if (tf->geom_info_.has_rear_rect_)
            {
              rect_sub = tf->geom_info_.rear_rect_; //车尾框判断和赋值
              has_rear = true;
            }

            CLUSTERING_RECT_CASETYPE rect_type = CLUSTERING_RECT_CASE_INIT;
            if (!GetSCaseType(rect_main, rect_sub, true, !has_rear,
                              rect_type,
            ClusteringOBJConfig::cls_com_cfg_.comekf_widthrate_scase_,
                              cameras[0]->vp_i_.x))
            {
              rect_type = GetCaseType(rect_main, rect_sub, true, !has_rear);
            }
            if (rect_type != tf->rect_type_)
            {
              tf->rect_type_ = rect_type; //框的类型赋值
            }

            bool has_key_pts = tf->geom_info_.board_side_.enabled_;
            if (!has_key_pts)
            {
              float yaw_hist = 0.0f;
              if (GetYawByHistoryPts(yaw_hist))
              {
                tf->geom_info_.boardside_yaw_vcs_ = yaw_hist;
                tf->geom_info_.boardside_yaw_vcs_obs_ = yaw_hist;
                tf->geom_info_.boardside_yaw_enable_ = true;
                tf->geom_info_.boardside_yaw_cnt_ = 1;
                tf->geom_info_.boardside_yaw_ori_ = yaw_hist;
                tf->geom_info_.boardside_yaw_ori_en_ = true;
                return;
              }
              return;
            }
            Point2f pt1, pt2;
            pt1 = tf->geom_info_.board_side_.back_pt_;
            pt2 = tf->geom_info_.board_side_.front_pt_;
            tf->geom_info_.board_side_.back_pt_vcs_ =
                cameras[0]->CvtImageToVcsGnd(pt1);
            tf->geom_info_.board_side_.front_pt_vcs_ =
            //获取车轮前后点并转化到车辆坐标系下 cameras[0]->CvtImageToVcsGnd(pt2);

            if (tf->geom_info_.board_side_.direct_angle_en_) //是否已经有了车辆角度
            {
              float direct_yaw = 0.0f;

              float sin_theta = sinf(tf->geom_info_.board_side_.direct_angle_);
              float cos_theta = cosf(tf->geom_info_.board_side_.direct_angle_);
              Point2f pt1_ori = tf->geom_info_.board_side_.back_pt_ori_;
              Point2f pt1_dire = Point2f(pt1_ori.x + cos_theta,
                                                 pt1_ori.y + sin_theta);
              pt1_dire = cameras[0]->undistort_->UndistortPt_bi(pt1_dire);
              if (CalDestYawByOriPointsForPerspective(pt1, pt1_dire,
                                                      cameras[0], direct_yaw))
              {
                tf->geom_info_.multibin_yaw_ = direct_yaw;
                tf->geom_info_.multibin_yaw_en_ = true;
              }
            }

            float conf_min = 0.0f;
            if (!BoardSidePointsFilter(rect_main, rect_sub, cameras,
                                       tf, has_rear, conf_min)) //点的过滤 去掉不合格的
            {
              return;
            }

            float boardside_yaw_vcs = 0.0f;
            bool has_vcs_yaw = false, has_rect_refine = false;
            has_rect_refine = GetYawByBoardSide(rect_main, pt1, pt2,
                                                rect_sub, boardside_yaw_vcs, cameras[0],
            rect_type, !has_rear, has_vcs_yaw, !has_rear);

            float yaw_cone = 0.0f;
            bool has_yaw_cone = false;
            CLUSTERING_COM_TYPE veh_type = tf->obj_size_type_;
            CLUSTERING_RECT_CASETYPE rect_type_cone = rect_type;
            Rect<float> rect_full = rect_main;
            Rect<float> rect_rear = rect_sub;
            bool fisheye_is_cutin = false;
            if (cameras[0]->camera_type_ == Camera::FISH_EYE)
            {
              if (rect_full.l < 0.0f ||
                  rect_full.r > static_cast<float>(image_width_))
              {
                fisheye_is_cutin = true;
              }
            }
            bool need_cone_verify = !has_rear &&
                                    tf->cover_ < 0.1f &&
                                    !(tf->geom_info_.veh3d_cutin_status_ ||
                                      tf->geom_info_.cutin_status_vertical_ ||
                                      fisheye_is_cutin) &&
                                    (veh_type == CLUSTERING_COM_CAR ||
                                     veh_type == CLUSTERING_COM_MINI_CAR);
            if (need_cone_verify)
            {
              yaw_cone = boardside_yaw_vcs;
              ////基于cone方法的朝向校验
              has_yaw_cone = YawVerifyByCone(veh_type, cameras[0],
                                             rect_full, rect_rear, pt1, pt2,
            rect_type_cone, yaw_cone); if (has_yaw_cone)
              {
                rect_main = rect_full;
                rect_sub = rect_rear;
                boardside_yaw_vcs = yaw_cone;
                rect_type = rect_type_cone;
              }
              else
              {
                return;
              }
            }

            float width_obs = VEHICLE_WIDTH;
            if (!tf->geom_info_.veh3d_cutin_status_)
            {
              GetVehWidth(tf, rect_sub, width_obs, cameras);
            ////计算车宽，通过车宽校验朝向 if (!YawVerifyByVehWidth(tf, cameras[0],
                                       boardside_yaw_vcs, width_obs))
              {
                return;
              }
            }
            // 对截断 错误车框 是否需要重新定义车框等 判断和赋值
            bool has_full_cut = !has_rear &&
                                (tf->geom_info_.veh3d_cutin_status_ ||
                                 tf->geom_info_.cutin_status_vertical_);
            if (has_full_cut || (tf->cover_ > 0.1f && has_rear))
            {
              has_rect_refine = false;
            }
            bool rect_error = rect_main.GetWidth() < 1.0f ||
                              rect_main.GetHeight() < 1.0f ||
                              rect_sub.GetWidth() < 1.0f ||
                              rect_sub.GetHeight() < 1.0f;
            bool need_refine_rect = has_rect_refine && !rect_error;
            if (cameras[0]->camera_type_ == Camera::FISH_EYE)
            {
              if (need_refine_rect)
              {
                tf->geom_info_.rect_rear_refine_ = rect_sub;
                tf->geom_info_.rect_full_refine_ = rect_main;
                tf->geom_info_.rect_refine_ = true;
                tf->rect_type_ = rect_type;
              }
            }
            else
            {
              if (need_refine_rect && has_yaw_cone)
              {
                tf->geom_info_.rect_rear_refine_ = rect_sub;
                tf->geom_info_.rect_full_refine_ = rect_main;
                tf->geom_info_.rect_refine_ = true;
                tf->geom_info_.yaw_cone_verify_ = yaw_cone;
                tf->rect_type_ = rect_type;
              }
            }

            if (has_vcs_yaw)
            {
              bool use_wheel_yaw = false;
              float yaw_pt = 0.0f;
              std::pair<bool, float> fit_wheels_yaw =
                  tf->obj_wheels_.fit_wheels_yaw_;
              if (fit_wheels_yaw.first) //车轮角相关
              {
                GetNearestYaw(boardside_yaw_vcs, fit_wheels_yaw.second);
                if (fabsf(boardside_yaw_vcs) > fabsf(fit_wheels_yaw.second))
                {
                  boardside_yaw_vcs = fit_wheels_yaw.second;
                  use_wheel_yaw = true;
                }
              }
              CalDestYawByOriPointsForPerspective(pt1, pt2,
                                                  cameras[0], yaw_pt);
              GetNearestYaw(yaw_pt, boardside_yaw_vcs);
              tf->geom_info_.boardside_yaw_vcs_ = boardside_yaw_vcs;
              tf->geom_info_.boardside_yaw_vcs_obs_ = boardside_yaw_vcs;
              tf->geom_info_.boardside_yaw_enable_ = true;
              tf->geom_info_.boardside_yaw_cnt_ = 1;
              tf->geom_info_.boardside_yaw_ori_ = use_wheel_yaw ? boardside_yaw_vcs :
            yaw_pt; tf->geom_info_.boardside_yaw_ori_en_ = true;
            }*/
        }
    } // namespace vehicleprocess
} // namespace pg