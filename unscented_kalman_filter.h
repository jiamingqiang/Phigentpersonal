//#include "opencv2/core.hpp"
//#include "opencv2/core/hal/hal.hpp"
#include <limits>
#include <vector>
namespace pg
{
    namespace preprocess
    {

        typedef int64_t TimeStamp;
        template <typename _Tp>
        bool inline callHalCholesky(_Tp *L, size_t lstep, int lsize);

        template <>
        bool inline callHalCholesky<float>(float *L, size_t lstep, int lsize)
        {
            return cv::hal::Cholesky32f(L, lstep, lsize, NULL, 0, 0);
        }

        template <>
        bool inline callHalCholesky<double>(double *L, size_t lstep, int lsize)
        {
            return cv::hal::Cholesky64f(L, lstep, lsize, NULL, 0, 0);
        }

        template <typename _Tp>
        bool inline choleskyDecomposition(const _Tp *A, size_t astep,
                                          int asize, _Tp *L, size_t lstep)
        {
            bool success = false;

            astep /= sizeof(_Tp);
            lstep /= sizeof(_Tp);

            for (int i = 0; i < asize; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    L[i * lstep + j] = A[i * astep + j];
                }
            }

            success = callHalCholesky(L, lstep * sizeof(_Tp), asize);

            if (success)
            {
                for (int i = 0; i < asize; i++)
                {
#if CV_VERSION_MINOR < 4
                    L[i * asize + i] = 1.0f / L[i * asize + i];
#endif
                    for (int j = i + 1; j < asize; j++)
                    {
                        L[i * lstep + j] = 0.0;
                    }
                }
            }

            return success;
        }
        class UnscentedKalmanFilter
        {
        public:
            virtual ~UnscentedKalmanFilter() {}

            virtual cv::Mat Predict(const cv::Mat &control) = 0;

            virtual cv::Mat Correct(const cv::Mat &measurement) = 0;

            virtual cv::Mat GetProcessNoiseCov() const = 0;

            virtual cv::Mat GetMeasurementNoiseCov() const = 0;

            virtual cv::Mat GetErrorCov() const = 0;

            virtual cv::Mat GetState() const = 0;

            virtual void SetProcessNoiseCov(const cv::Mat &process_noise) = 0;
            virtual void SetMeasurementNoiseCov(const cv::Mat &measurement_noise) = 0;
            virtual void SetErrorCov(const cv::Mat &error_cov) = 0;
            virtual void SetState(const cv::Mat &State) = 0;
        };
        class UkfSystemModel
        {
        public:
            virtual ~UkfSystemModel() {}

            virtual void StateConversionFunction(const cv::Mat &x_k,
                                                 const cv::Mat &u_k, const cv::Mat &v_k, cv::Mat &x_kplus1) = 0;

            virtual void MeasurementFunction(const cv::Mat &x_k,
                                             const cv::Mat &n_k, cv::Mat &z_k) = 0;
        };
        class UnscentedKalmanFilterParams
        {
        public:
            int DP_;
            int MP_;
            int CP_;
            int dataType_;

            cv::Mat stateInit_;
            cv::Mat errorCovInit_;

            cv::Mat processNoiseCov_;
            cv::Mat measurementNoiseCov_;
            UkfSystemModel *model_;
            float alpha_;
            float k_;
            float beta_;

            UnscentedKalmanFilterParams() : DP_(0), MP_(0), CP_(0), dataType_(0),
                                            stateInit_(), errorCovInit_(),
                                            processNoiseCov_(), measurementNoiseCov_(),
                                            model_(nullptr),
                                            alpha_(0.0f), k_(0.0f), beta_(0.0f) {}

            UnscentedKalmanFilterParams(int dp, int mp, int cp,
                                        float processNoiseCovDiag, float measurementNoiseCovDiag,
                                        UkfSystemModel *dynamicalSystem, int type = CV_32F);
            virtual ~UnscentedKalmanFilterParams() {}

            void Init(int dp, int mp, int cp,
                      float processNoiseCovDiag, float measurementNoiseCovDiag,
                      UkfSystemModel *dynamicalSystem, int type = CV_32F);
        };
        class UnscentedKalmanFilterImpl : public UnscentedKalmanFilter
        {
        protected:
            int DP_;
            int MP_;
            int CP_;
            int dataType_;

            cv::Mat state_;
            cv::Mat errorCov_;

            cv::Mat processNoiseCov_;
            cv::Mat measurementNoiseCov_;

            UkfSystemModel *model_;

            float alpha_;
            float k_;
            float beta_;
            float lambda_;
            float tmpLambda_;
            float sqrtTmpLambda_;
            cv::Mat measurementEstimate_;

            cv::Mat sigmaPoints_;

            cv::Mat transitionSPFuncVals_;

            cv::Mat measurementSPFuncVals_;

            cv::Mat transitionSPFuncValsCenter_;

            cv::Mat measurementSPFuncValsCenter_;
            cv::Mat Wm_;
            cv::Mat Wc_;
            cv::Mat gain_;
            cv::Mat xyCov_;
            cv::Mat yyCov_;
            cv::Mat r_;
            cv::Mat q_;

            cv::Mat getSigmaPoints(const cv::Mat &mean,
                                   const cv::Mat &covMatrix, float coef) const;

        public:
            UnscentedKalmanFilterImpl() : UnscentedKalmanFilter(),
                                          DP_(0), MP_(0), CP_(0), dataType_(0),
                                          state_(), errorCov_(), processNoiseCov_(),
                                          measurementNoiseCov_(), model_(nullptr),
                                          alpha_(0.0f), k_(0.0f), beta_(0.0f),
                                          lambda_(0.0f), tmpLambda_(0.0f),
                                          sqrtTmpLambda_(0.0f),
                                          measurementEstimate_(),
                                          sigmaPoints_(),
                                          transitionSPFuncVals_(),
                                          measurementSPFuncVals_(),
                                          transitionSPFuncValsCenter_(),
                                          measurementSPFuncValsCenter_(),
                                          Wm_(), Wc_(), gain_(), xyCov_(),
                                          yyCov_(), r_(), q_() {}
            explicit UnscentedKalmanFilterImpl(const UnscentedKalmanFilterParams &params);
            virtual ~UnscentedKalmanFilterImpl();

            static cv::Mat getSigmaPoints32f(const cv::Mat &mean,
                                             const cv::Mat &covMatrix, float coef);
            virtual bool Init(const UnscentedKalmanFilterParams &params);
            cv::Mat Predict(const cv::Mat &control) override;
            virtual cv::Mat FastPredict(const cv::Mat &measurement,
                                        bool add_noise = true);

            cv::Mat Correct(const cv::Mat &measurement) override;
            virtual cv::Mat FastCorrect(const cv::Mat &measurement);

            cv::Mat GetProcessNoiseCov() const override;
            cv::Mat GetMeasurementNoiseCov() const override;
            cv::Mat GetErrorCov() const override;

            void SetProcessNoiseCov(const cv::Mat &process_noise) override;
            void SetMeasurementNoiseCov(const cv::Mat &measurement_noise) override;
            void SetErrorCov(const cv::Mat &error_cov) override;
            void SetState(const cv::Mat &State) override;

            cv::Mat GetState() const override;
        };

        class RectSystemModel : public UkfSystemModel
        {
        public:
            RectSystemModel() {}
            virtual ~RectSystemModel() {}

            void StateConversionFunction(const cv::Mat &x_k,
                                         const cv::Mat &u_k, const cv::Mat &v_k, cv::Mat &x_kplus1) override;
            void MeasurementFunction(const cv::Mat &x_k,
                                     const cv::Mat &n_k, cv::Mat &z_k) override;
        };
        class RectUKF : public UnscentedKalmanFilterImpl
        {
        public:
            RectUKF() : UnscentedKalmanFilterImpl(),
                        prev_time_(0), miss_correct_(0),
                        bad_correct_w_(0), bad_correct_h_(0),
                        frame_id_(0), output_id_(-1),
                        max_miss_correct_(3)
            {
                model_ = new RectSystemModel();
            }

            virtual ~RectUKF()
            {
                if (model_)
                {
                    delete model_;
                    model_ = NULL;
                }
            }

            using UnscentedKalmanFilterImpl::Correct;
            using UnscentedKalmanFilterImpl::Init;
            using UnscentedKalmanFilterImpl::Predict;
            bool Init(float init_w, float init_h, TimeStamp time);
            cv::Mat Predict(float w, float h, TimeStamp time);
            cv::Mat Correct(float w, float h);
            std::pair<float, float> CalcDistByErrorCov(
                float measure_w, float measure_h);

            void Reset(float init_w, float init_h, TimeStamp time)
            {
                Init(init_w, init_h, time);
            }
            int GetMaxMissCorrect() const
            {
                return max_miss_correct_;
            }

            TimeStamp prev_time_;
            int miss_correct_;
            int bad_correct_w_;
            int bad_correct_h_;
            uint32_t frame_id_;
            int output_id_;
            static const float Max_Dist;

        private:
            int max_miss_correct_;
        };
    }
}