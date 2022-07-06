#pragma once
#include <algorithm>
#include <mutex>
#include <utility>

template <typename T>
class RingBuffer
{
public:
    RingBuffer()
        : buf_size_(0),
          elements_cnt_(1),
          mem_start_ptr_(nullptr),
          mem_end_ptr_(nullptr),
          head_ptr_(nullptr),
          tail_ptr_(nullptr),
          write_ptr_(nullptr),
          need_reset_(false) {}
    explicit RingBuffer(int buf_size)
        : buf_size_(buf_size),
          elements_cnt_(1),
          mem_start_ptr_(nullptr),
          mem_end_ptr_(nullptr),
          head_ptr_(nullptr),
          tail_ptr_(nullptr),
          write_ptr_(nullptr),
          need_reset_(false)
    {
        buf_size = std::max<size_t>(2, buf_size);
        mem_start_ptr_ = new T[buf_size];
        head_ptr_ = mem_start_ptr_;
        write_ptr_ = mem_start_ptr_;
        tail_ptr_ = mem_start_ptr_;
        mem_end_ptr_ = mem_start_ptr_ + buf_size - 1;
    }
    explicit RingBuffer(T *head_ptr, size_t buf_size)
        : buf_size_(buf_size),
          elements_cnt_(1),
          mem_start_ptr_(head_ptr),
          mem_end_ptr_(head_ptr + buf_size - 1),
          head_ptr_(head_ptr),
          tail_ptr_(head_ptr),
          write_ptr_(head_ptr),
          need_reset_(false) {}
    ~RingBuffer()
    {
        if (mem_start_ptr_)
        {
            delete[] mem_start_ptr_;
            mem_start_ptr_ = nullptr;
        }
        mem_end_ptr_ = nullptr;
        head_ptr_ = nullptr;
        write_ptr_ = nullptr;
        tail_ptr_ = nullptr;
    }
    void ResetMemBuf(size_t buf_size)
    {
        buf_size = std::max<size_t>(2, buf_size);
        elements_cnt_ = 1;
        if (buf_size == buf_size_)
        {
            head_ptr_ = mem_start_ptr_;
            write_ptr_ = mem_start_ptr_;
            tail_ptr_ = mem_start_ptr_;
            write_ptr_->Reset();
            return;
        }
        if (mem_start_ptr_ != nullptr)
        {
            delete[] mem_start_ptr_;
            mem_start_ptr_ = nullptr;
        }
        buf_size_ = buf_size;
        mem_start_ptr_ = new T[buf_size_];
        head_ptr_ = mem_start_ptr_;
        write_ptr_ = mem_start_ptr_;
        tail_ptr_ = mem_start_ptr_;
        mem_end_ptr_ = mem_start_ptr_ + buf_size - 1;
    }
    void Reset()
    {
        elements_cnt_ = 1;
        head_ptr_ = mem_start_ptr_;
        write_ptr_ = mem_start_ptr_;
        tail_ptr_ = mem_start_ptr_;
        write_ptr_->Reset();
        need_reset_ = true;
    }
    size_t GetElemensCnt() const { return elements_cnt_; }

    T *Pop()
    {
        std::lock_guard<std::mutex> asyn_lock(lock_);
        if (elements_cnt_ < 2)
        {
            return nullptr;
        }
        return tail_ptr_;
    }

    void Push(const T *data)
    {
        write_ptr_->DeepCopy(data);
        MoveToNext();
    }

    T *GetWriteBuff() const { return write_ptr_; }

    T *GetBuffById(size_t id)
    {
        if (id >= elements_cnt_)
        {
            return nullptr;
        }
        if (head_ptr_ + id <= mem_end_ptr_)
        {
            return head_ptr_ + id;
        }
        else
        {
            return mem_start_ptr_ + (id - (mem_end_ptr_ - head_ptr_) - 1);
        }
    }

    T *GetHeadBuff() const { return head_ptr_; }

    void MoveToNext()
    {
        if (elements_cnt_ >= buf_size_)
        {
            if (head_ptr_ == mem_end_ptr_)
            {
                head_ptr_ = mem_start_ptr_;
            }
            else
            {
                head_ptr_ += 1;
            }
            need_reset_ = true;
        }

        if (write_ptr_ == mem_end_ptr_)
        {
            write_ptr_ = mem_start_ptr_;
        }
        else
        {
            write_ptr_ += 1;
        }
        if (need_reset_)
        {
            write_ptr_->Reset();
        }
        std::lock_guard<std::mutex> asyn_lock(lock_);
        if (elements_cnt_ > 1)
        {
            if (tail_ptr_ == mem_end_ptr_)
            {
                tail_ptr_ = mem_start_ptr_;
            }
            else
            {
                tail_ptr_ += 1;
            }
        }
        if (elements_cnt_ < buf_size_)
        {
            elements_cnt_ += 1;
        }
    }

private:
    RingBuffer &operator=(const RingBuffer &other) = delete;

    size_t buf_size_;
    size_t elements_cnt_;
    T *mem_start_ptr_;
    T *mem_end_ptr_;
    T *head_ptr_;
    T *tail_ptr_;
    T *write_ptr_;
    bool need_reset_;
    std::mutex lock_;
};
