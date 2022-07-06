#include <algorithm>
#include "unscented_kalman_filter.h"
namespace pg
{
    namespace preprocess
    {
        UnscentedKalmanFilterParams::UnscentedKalmanFilterParams(int dp, int mp, int cp,
                                                                 float processNoiseCovDiag, float measurementNoiseCovDiag,
                                                                 UkfSystemModel *dynamicalSystem, int type)
        {
            Init(dp, mp, cp, processNoiseCovDiag,
                 measurementNoiseCovDiag, dynamicalSystem, type);
        }
        void UnscentedKalmanFilterParams::Init(int dp, int mp, int cp,
                                               float processNoiseCovDiag, float measurementNoiseCovDiag,
                                               UkfSystemModel *dynamicalSystem, int type)
        {
            CV_Assert(dp > 0 && mp > 0);
            DP_ = dp;
            MP_ = mp;
            CP_ = std::max(cp, 0);
            CV_Assert(type == CV_32F || type == CV_64F);
            dataType_ = type;

            this->model_ = dynamicalSystem;

            stateInit_ = cv::Mat::zeros(DP_, 1, type);
            errorCovInit_ = cv::Mat::eye(DP_, DP_, type);

            processNoiseCov_ = processNoiseCovDiag * cv::Mat::eye(DP_, DP_, type);
            measurementNoiseCov_ = measurementNoiseCovDiag * cv::Mat::eye(MP_, MP_, type);

            alpha_ = 1e-3f;
            k_ = 0.0f;
            beta_ = 2.0f;
        }
        void RectSystemModel::StateConversionFunction(const cv::Mat &x_k,
                                                      const cv::Mat &u_k, const cv::Mat &v_k, cv::Mat &x_kplus1)
        {
            float *x_k_p = reinterpret_cast<float *>(x_k.data);
            float w_k = x_k_p[0];
            float h_k = x_k_p[x_k.step1(0) * 1];
            float r_w = x_k_p[x_k.step1(0) * 2];
            float r_h = x_k_p[x_k.step1(0) * 3];
            float dt = *(reinterpret_cast<float *>(u_k.data));

            float *x_kplus1_p = reinterpret_cast<float *>(x_kplus1.data);
            x_kplus1_p[0] = w_k + w_k * r_w * dt;
            x_kplus1_p[x_kplus1.step1(0) * 1] = h_k + h_k * r_h * dt;
            x_kplus1_p[x_kplus1.step1(0) * 2] = r_w;
            x_kplus1_p[x_kplus1.step1(0) * 3] = r_h;
        }

        void RectSystemModel::MeasurementFunction(const cv::Mat &x_k,
                                                  const cv::Mat &n_l, cv::Mat &z_k)
        {
            float *x_k_p = reinterpret_cast<float *>(x_k.data);
            float *z_k_p = reinterpret_cast<float *>(z_k.data);

            z_k_p[0] = x_k_p[0];
            z_k_p[z_k.step1(0) * 1] = x_k_p[x_k.step1(0) * 1];
        }

        UnscentedKalmanFilterImpl::UnscentedKalmanFilterImpl(
            const UnscentedKalmanFilterParams &params)
        {
            UnscentedKalmanFilterImpl::Init(params);
        }

        bool UnscentedKalmanFilterImpl::Init(
            const UnscentedKalmanFilterParams &params)
        {
            alpha_ = params.alpha_;
            beta_ = params.beta_;
            k_ = params.k_;

            CV_Assert(params.DP_ > 0 && params.MP_ > 0);
            CV_Assert(params.dataType_ == CV_32F || params.dataType_ == CV_64F);
            DP_ = params.DP_;
            MP_ = params.MP_;
            CP_ = std::max(params.CP_, 0);
            dataType_ = params.dataType_;

            model_ = params.model_;

            CV_Assert(params.stateInit_.cols == 1 && params.stateInit_.rows == DP_);
            CV_Assert(params.errorCovInit_.cols == DP_ && params.errorCovInit_.rows == DP_);
            state_ = params.stateInit_.clone();
            errorCov_ = params.errorCovInit_.clone();

            CV_Assert(params.processNoiseCov_.cols == DP_ && params.processNoiseCov_.rows == DP_);
            CV_Assert(params.measurementNoiseCov_.cols == MP_ && params.measurementNoiseCov_.rows == MP_);
            processNoiseCov_ = params.processNoiseCov_.clone();
            measurementNoiseCov_ = params.measurementNoiseCov_.clone();

            measurementEstimate_ = cv::Mat::zeros(MP_, 1, dataType_);

            q_ = cv::Mat::zeros(DP_, 1, dataType_);
            r_ = cv::Mat::zeros(MP_, 1, dataType_);

            gain_ = cv::Mat::zeros(DP_, DP_, dataType_);

            transitionSPFuncVals_ = cv::Mat::zeros(DP_, 2 * DP_ + 1, dataType_);
            measurementSPFuncVals_ = cv::Mat::zeros(MP_, 2 * DP_ + 1, dataType_);

            transitionSPFuncValsCenter_ = cv::Mat::zeros(DP_, 2 * DP_ + 1, dataType_);
            measurementSPFuncValsCenter_ = cv::Mat::zeros(MP_, 2 * DP_ + 1, dataType_);

            lambda_ = alpha_ * alpha_ * (static_cast<float>(DP_) + k_) -
                      static_cast<float>(DP_);
            tmpLambda_ = lambda_ + static_cast<float>(DP_);
            sqrtTmpLambda_ = sqrtf(tmpLambda_);

            float tmp2Lambda = 0.5f / tmpLambda_;

            Wm_ = tmp2Lambda * cv::Mat::ones(2 * DP_ + 1, 1, dataType_);
            Wc_ = tmp2Lambda * cv::Mat::eye(2 * DP_ + 1, 2 * DP_ + 1, dataType_);

            if (dataType_ == CV_64F)
            {
                float cals_res_tmp = lambda_ / tmpLambda_;
                Wm_.at<double>(0, 0) = static_cast<double>(cals_res_tmp);
                Wc_.at<double>(0, 0) = static_cast<double>(
                    lambda_ / tmpLambda_ + 1.0 - alpha_ * alpha_ + beta_);
            }
            else
            {
                Wm_.at<float>(0, 0) = lambda_ / tmpLambda_;
                Wc_.at<float>(0, 0) = lambda_ / tmpLambda_ + 1.0f - alpha_ * alpha_ + beta_;
            }
            return true;
        }

        UnscentedKalmanFilterImpl::~UnscentedKalmanFilterImpl()
        {
            state_.release();
            errorCov_.release();

            processNoiseCov_.release();
            measurementNoiseCov_.release();

            measurementEstimate_.release();

            sigmaPoints_.release();

            transitionSPFuncVals_.release();
            measurementSPFuncVals_.release();

            transitionSPFuncValsCenter_.release();
            measurementSPFuncValsCenter_.release();

            Wm_.release();
            Wc_.release();

            gain_.release();
            xyCov_.release();
            yyCov_.release();

            r_.release();
            q_.release();
        }

        cv::Mat UnscentedKalmanFilterImpl::getSigmaPoints(
            const cv::Mat &mean, const cv::Mat &covMatrix, float coef) const
        {

            int n = mean.rows;
            cv::Mat points = repeat(mean, 1, 2 * n + 1);

            cv::Mat covMatrixL = covMatrix.clone();

            if (dataType_ == CV_64F)
            {
                choleskyDecomposition<double>(
                    covMatrix.ptr<double>(), covMatrix.step, covMatrix.rows,
                    covMatrixL.ptr<double>(), covMatrixL.step);
            }
            else if (dataType_ == CV_32F)
            {
                choleskyDecomposition<float>(
                    covMatrix.ptr<float>(), covMatrix.step, covMatrix.rows,
                    covMatrixL.ptr<float>(), covMatrixL.step);
            }
            else
            {
            }

            covMatrixL = coef * covMatrixL;

            cv::Mat p_plus = points(cv::Rect(1, 0, n, n));
            cv::Mat p_minus = points(cv::Rect(n + 1, 0, n, n));

            add(p_plus, covMatrixL, p_plus);
            subtract(p_minus, covMatrixL, p_minus);

            return points;
        }

        cv::Mat UnscentedKalmanFilterImpl::getSigmaPoints32f(
            const cv::Mat &mean, const cv::Mat &covMatrix, float coef)
        {
            int n = mean.rows;
            cv::Mat points = repeat(mean, 1, 2 * n + 1);

            cv::Mat covMatrixL = covMatrix.clone();

            choleskyDecomposition<float>(
                covMatrix.ptr<float>(), covMatrix.step, covMatrix.rows,
                covMatrixL.ptr<float>(), covMatrixL.step);

            covMatrixL = coef * covMatrixL;

            cv::Mat p_plus = points(cv::Rect(1, 0, n, n));
            cv::Mat p_minus = points(cv::Rect(n + 1, 0, n, n));

            add(p_plus, covMatrixL, p_plus);
            subtract(p_minus, covMatrixL, p_minus);

            return points;
        }

        cv::Mat UnscentedKalmanFilterImpl::Predict(const cv::Mat &control)
        {
            sigmaPoints_ = getSigmaPoints(state_, errorCov_, sqrtTmpLambda_);
            return FastPredict(control);
        }

        cv::Mat UnscentedKalmanFilterImpl::FastPredict(const cv::Mat &control,
                                                       bool add_noise)
        {

            cv::Mat x, fx;
            for (int i = 0; i < sigmaPoints_.cols; i++)
            {
                x = sigmaPoints_(cv::Rect(i, 0, 1, sigmaPoints_.rows));
                fx = transitionSPFuncVals_(cv::Rect(i, 0, 1, DP_));
                model_->StateConversionFunction(x, control, q_, fx);
            }
            state_ = transitionSPFuncVals_ * Wm_;

            cv::subtract(transitionSPFuncVals_,
                         cv::repeat(state_, 1, transitionSPFuncVals_.cols),
                         transitionSPFuncValsCenter_);

            if (add_noise)
            {
                errorCov_ = transitionSPFuncValsCenter_ * Wc_ *
                                transitionSPFuncValsCenter_.t() +
                            processNoiseCov_;
            }
            else
            {
                errorCov_ = transitionSPFuncValsCenter_ * Wc_ *
                            transitionSPFuncValsCenter_.t();
            }

            return state_.clone();
        }

        cv::Mat UnscentedKalmanFilterImpl::Correct(const cv::Mat &measurement)
        {
            sigmaPoints_ = getSigmaPoints(state_, errorCov_, sqrtTmpLambda_);
#if 0
    cv::subtract(sigmaPoints_,
    cv::repeat(state_, 1, 2 * DP_ + 1), transitionSPFuncValsCenter_);
#endif

            return FastCorrect(measurement);
        }

        cv::Mat UnscentedKalmanFilterImpl::FastCorrect(const cv::Mat &measurement)
        {

            cv::Mat x, hx;
            for (int i = 0; i < sigmaPoints_.cols; i++)
            {
                x = sigmaPoints_(cv::Rect(i, 0, 1, DP_));
                hx = measurementSPFuncVals_(cv::Rect(i, 0, 1, MP_));
                model_->MeasurementFunction(x, r_, hx);
            }

            measurementEstimate_ = measurementSPFuncVals_ * Wm_;

            cv::subtract(measurementSPFuncVals_,
                         cv::repeat(measurementEstimate_, 1, measurementSPFuncVals_.cols),
                         measurementSPFuncValsCenter_);

            yyCov_ = measurementSPFuncValsCenter_ * Wc_ *
                         measurementSPFuncValsCenter_.t() +
                     measurementNoiseCov_;

            xyCov_ = transitionSPFuncValsCenter_ * Wc_ * measurementSPFuncValsCenter_.t();

            gain_ = xyCov_ * yyCov_.inv(cv::DECOMP_SVD);

            state_ = state_ + gain_ * (measurement - measurementEstimate_);

            errorCov_ = errorCov_ - gain_ * xyCov_.t();

            return state_.clone();
        }

        cv::Mat UnscentedKalmanFilterImpl::GetProcessNoiseCov() const
        {
            return processNoiseCov_.clone();
        }

        cv::Mat UnscentedKalmanFilterImpl::GetMeasurementNoiseCov() const
        {
            return measurementNoiseCov_.clone();
        }

        cv::Mat UnscentedKalmanFilterImpl::GetErrorCov() const
        {
            return errorCov_.clone();
        }

        cv::Mat UnscentedKalmanFilterImpl::GetState() const
        {
            return state_.clone();
        }

        void UnscentedKalmanFilterImpl::SetProcessNoiseCov(
            const cv::Mat &process_noise)
        {
            processNoiseCov_ = process_noise.clone();
        }

        void UnscentedKalmanFilterImpl::SetMeasurementNoiseCov(
            const cv::Mat &measurement_noise)
        {
            measurementNoiseCov_ = measurement_noise.clone();
        }

        void UnscentedKalmanFilterImpl::SetErrorCov(
            const cv::Mat &error_cov)
        {
            errorCov_ = error_cov.clone();
        }

        void UnscentedKalmanFilterImpl::SetState(
            const cv::Mat &state)
        {
            state_ = state.clone();
        }

        const float RectUKF::Max_Dist = 100.0f;

        bool RectUKF::Init(float init_w, float init_h, TimeStamp time)
        {
            prev_time_ = time;
            miss_correct_ = 0;
            bad_correct_w_ = 0;
            bad_correct_h_ = 0;
            frame_id_ = 0;
            output_id_ = -1;
            UkfSystemModel *model_;
            const int DP = 4, MP = 2, CP = 1;
            const int type = CV_32F;
            UnscentedKalmanFilterParams init(DP, MP, CP, 0.0f, 0.0f, model_, type);

            const float alpha = 0.6f;
            const float beta = 2.0f;
            const float kappa = 0.0f;
            init.alpha_ = alpha;
            init.beta_ = beta;
            init.k_ = kappa;

            init.stateInit_ = (cv::Mat_<float>(DP, 1) << init_w, init_h, 0.0f, 0.0f);
            init.errorCovInit_ = (cv::Mat_<float>(DP, DP) << powf(init_w * 0.1f, 2.f), 0.0f, 0.0f, 0.f,
                                  0.0f, powf(init_h * 0.1f, 2.f), 0.0f, 0.f,
                                  0.0f, 0.0f, 1e-2f, 0.0f,
                                  0.0f, 0.0f, 0.0f, 1e-2f);
            init.processNoiseCov_ = (cv::Mat_<float>(DP, DP) << powf(init_w * 0.01f, 2.f), 0.0f, 0.0f, 0.f,
                                     0.0f, powf(init_h * 0.01f, 2.f), 0.0f, 0.f,
                                     0.0f, 0.0f, 1e-4f, 0.0f,
                                     0.0f, 0.0f, 0.0f, 1e-4f);
            init.measurementNoiseCov_ = (cv::Mat_<float>(MP, MP) << powf(init_w * 0.05f, 2.f), 0.0f,
                                         0.0f, powf(init_h * 0.05f, 2.f));
            return UnscentedKalmanFilterImpl::Init(init);
        }

        cv::Mat RectUKF::Predict(float w, float h, TimeStamp time)
        {
            TimeStamp dt_ts = time - prev_time_;
            float dt_ms = static_cast<float>(dt_ts);
            float dt = dt_ms / 1000.f;
            prev_time_ = time;
            miss_correct_++;

            float delta_w = 0.005f, delta_h = 0.005f;
            cv::Mat noise = (cv::Mat_<float>(DP_, DP_) << powf(w * delta_w, 2.f), 0.f, 0.f, 0.f,
                             0.f, powf(h * delta_h, 2.f), 0.f, 0.f,
                             0.f, 0.f, powf(delta_w, 2.f), 0.f,
                             0.f, 0.f, 0.f, powf(delta_h, 2.f));
            SetProcessNoiseCov(noise);

            cv::Mat control(1, 1, CV_32F);
            float *control_p = reinterpret_cast<float *>(control.data);
            control_p[0] = dt;
            return UnscentedKalmanFilterImpl::Predict(control);
        }

        cv::Mat RectUKF::Correct(float w, float h)
        {
            miss_correct_ = 0;

            float delta_w = 0.03f, delta_h = 0.03f;
            auto predict_w_h_dist = CalcDistByErrorCov(w, h);
            if (w > 120.f)
            {
                if (predict_w_h_dist.first < 2.0f)
                {
                    delta_w = 0.02f;
                }
                else if (predict_w_h_dist.first < 3.0f)
                {
                    delta_w = 0.03f;
                }
                else
                {
                    delta_w = 0.04f;
                }
            }
            else
            {
                if (predict_w_h_dist.first > 3.0f)
                {
                    delta_w = 0.05f;
                }
            }
            if (h > 120.f)
            {
                if (predict_w_h_dist.second < 2.0f)
                {
                    delta_h = 0.02f;
                }
                else if (predict_w_h_dist.second < 3.0f)
                {
                    delta_h = 0.03f;
                }
                else
                {
                    delta_h = 0.04f;
                }
            }
            else
            {
                if (predict_w_h_dist.second > 3.0f)
                {
                    delta_h = 0.05f;
                }
            }

            cv::Mat nosie = (cv::Mat_<float>(MP_, MP_) << powf(w * delta_w, 2.f), 0.0f,
                             0.0f, powf(h * delta_h, 2.f));
            SetMeasurementNoiseCov(nosie);

            cv::Mat measure(MP_, 1, CV_32F);
            float *measure_p = reinterpret_cast<float *>(measure.data);

            measure_p[0] = w;
            measure_p[1] = h;
            cv::Mat res = UnscentedKalmanFilterImpl::Correct(measure);

            predict_w_h_dist = CalcDistByErrorCov(w, h);
            bad_correct_w_ = predict_w_h_dist.first > 2.f ? bad_correct_w_ + 1 : 0;
            bad_correct_h_ = predict_w_h_dist.second > 2.f ? bad_correct_h_ + 1 : 0;
            float *state_p = reinterpret_cast<float *>(state_.data);
            float *cov_p = reinterpret_cast<float *>(errorCov_.data);
            if (bad_correct_w_ >= 3)
            {
                float std_w = sqrtf(cov_p[0]);
                float tmp_w = state_p[0];
                state_p[0] = std::min(w, tmp_w + 3.f * std_w);
                state_p[0] = std::max(state_p[0], tmp_w - 3.f * std_w);
                cov_p[0] *= 2.f;
                cov_p[2 * errorCov_.step1(0) + 2] *= 2.0f;
            }
            if (bad_correct_h_ >= 3)
            {
                float std_h = sqrtf(cov_p[1 * errorCov_.step1(0) + 1]);
                float tmp_h = state_p[1];
                state_p[1] = std::min(h, tmp_h + 3.f * std_h);
                state_p[1] = std::max(state_p[1], tmp_h - 3.f * std_h);
                cov_p[1 * errorCov_.step1(0) + 1] *= 2.f;
                cov_p[3 * errorCov_.step1(0) + 3] *= 2.f;
            }

            return res;
        }

        std::pair<float, float> RectUKF::CalcDistByErrorCov(
            float measure_w, float measure_h)
        {
            const auto state = GetState();
            float *state_p = reinterpret_cast<float *>(state.data);
            float w = state_p[0];
            float h = state_p[1];
            const auto cov = GetErrorCov();
            float *cov_p = reinterpret_cast<float *>(cov.data);
            float w_std = sqrtf(cov_p[0]);
            float h_std = sqrtf(cov_p[1 * cov.step1(0) + 1]);

            std::pair<float, float> w_h_dist(Max_Dist, Max_Dist);
            if (w_std > 1e-6f)
            {
                w_h_dist.first = fabsf(w - measure_w) / w_std;
            }
            if (h_std > 1e-6f)
            {
                w_h_dist.second = fabsf(h - measure_h) / h_std;
            }
            return w_h_dist;
        }
    } // namespace preprocess
} // namespace pg