#ifndef ADX_CURVE_VECTOR_HPP
#define ADX_CURVE_VECTOR_HPP

#ifdef __linux__
#include <vector>

#else // Erika

#ifndef STATIC_VECTOR_SIZE
#define STATIC_VECTOR_SIZE 500
#endif

template<typename T>
class StaticVector
{
  public:
    StaticVector()
      : vec_{}
      , size_(0)
    {}

    void push_back(T val)
    {
        if (size_ >= STATIC_VECTOR_SIZE)
            return;

        vec_[size_] = val;
        size_++;
    }

    T back() const { return size_ > 0 ? vec_[size_ - 1] : vec_[0]; }

    unsigned int size() const { return size_; }

    T& operator[](unsigned int idx) { return idx < size_ ? vec_[idx] : vec_[0]; }

    const T& operator[](unsigned int idx) const { return idx < size_ ? vec_[idx] : vec_[0]; }

  private:
    T vec_[STATIC_VECTOR_SIZE];
    unsigned int size_;
};

using Vec_d = StaticVector<double>;

#endif

#endif // ADX_CURVE_VECTOR_HPP
