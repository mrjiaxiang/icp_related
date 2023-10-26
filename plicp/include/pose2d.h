#ifndef POSE2D_H_
#define POSE2D_H_

#include <inttypes.h>
#include <math.h>

#include <algorithm>

#ifndef M_PI
    #define M_PI 3.14159265358979323846   // pi
#endif

/**
 * @brief 角度转弧度
 */
#ifndef DEG2RAD
    #define DEG2RAD(x) ((x)*0.017453292519943)
#endif

/**
 * @brief 弧度转角度
 */
#ifndef RAD2DEG
    #define RAD2DEG(x) ((x)*57.2957795130823)
#endif

class Pose2D {
 public:
    float x, y;

    Pose2D();
    Pose2D(const float _x, const float _y, const float phi);

    /**
     * @brief 加法重构：注意是带有矩阵旋转的重构
     * 相对于世界坐标系的A点 + 相对于A点坐标系的b 得到相对于世界坐标系的 C
     * @param b
     * @return Pose2D
     */
    Pose2D operator+(const Pose2D &b) const;

    /**
     * @brief 减法重构
     * 相对于世界坐标系的A - 相对于世界坐标系的B = A相对于B的坐标变换
     * @param b
     * @return Pose2D
     */
    Pose2D operator-(const Pose2D &b) const;

    /**
     * @brief A - pose(0,0,0)
     * @return Pose2D
     */
    Pose2D operator-();

    /**
     * @brief 等比例放大 x y phi 随后归一化角度
     * @param s
     */
    void operator*=(const float s);

    /**
     * @brief Set the Phi object
     * 设置角度，角度会被归一化到(-pi , pi)
     * @param phi
     */
    void SetPhi(float phi);

    /**
     * @brief 得到phi角
     * @return float
     */
    inline float Phi() const { return phi_; }

    /**
     * @brief 得到模长
     * @return float
     */
    inline float Normal() const { return sqrt(x * x + y * y); }

    /**
     * @brief 归一化角度
     */
    void NormalizePhi();

    /**
     * @brief 重置
     * @param x
     * @param y
     * @param phi
     */
    void Reset(float _x = 0.0f, float _y = 0.0, float phi = 0.0f);

    /**
     * @brief Get the Distance object
     * 得到两点之间的距离
     * @param point
     * @return float
     */
    float GetDistance(Pose2D point);

    /**
     * @brief 得到A相对于B直接的变化的逆矩阵(B相对于A的变换)
     * @param A
     * @param B
     */
    void InverseComposeFrom(const Pose2D &A, const Pose2D &B);

    /**
     * @brief 将数值归一化到[0,2pi)
     */
    template <class T>
    inline void WrapTo2PiInPlace(T &a);

    /**
     * @brief 将数值归一化到[0,2pi)
     * @tparam T
     * @param a
     * @return T
     */
    template <class T>
    inline T WrapTo2Pi(T a);

    /**
     * @brief 将数值归一化到[-Pi,pi)
     */
    template <class T>
    inline T WrapToPi(T a);

 private:
    float phi_;
};

inline float Pose2D::GetDistance(Pose2D point) {
    return sqrt((x - point.x) * (x - point.x) + (y - point.y) * (y - point.y));
}

inline void Pose2D::SetPhi(float phi) {
    phi_ = phi;
    NormalizePhi();
}

inline Pose2D Pose2D::operator+(const Pose2D &b) const {
    float cos_phi_ = cos(phi_);
    float sin_phi_ = sin(phi_);

    return Pose2D(x + b.x * cos_phi_ - b.y * sin_phi_, y + b.x * sin_phi_ + b.y * cos_phi_, phi_ + b.phi_);
}

inline Pose2D Pose2D::operator-(const Pose2D &b) const {
    Pose2D ret;
    ret.InverseComposeFrom(*this, b);
    return ret;
}

inline void Pose2D::InverseComposeFrom(const Pose2D &a, const Pose2D &b) {
    float cos_phi = cos(b.phi_);
    float sin_phi = sin(b.phi_);
    x             = (a.x - b.x) * cos_phi + (a.y - b.y) * sin_phi;
    y             = -(a.x - b.x) * sin_phi + (a.y - b.y) * cos_phi;

    SetPhi(a.phi_ - b.phi_);
}

template <class T>
inline void Pose2D::WrapTo2PiInPlace(T &a) {
    bool was_neg = a < 0;
    a            = fmod(a, static_cast<T>(2.0 * M_PI));
    if (was_neg)
        a += static_cast<T>(2.0 * M_PI);
}

template <class T>
inline T Pose2D::WrapTo2Pi(T a) {
    WrapTo2PiInPlace(a);
    return a;
}

template <class T>
inline T Pose2D::WrapToPi(T a) {
    return WrapTo2Pi(a + static_cast<T>(M_PI)) - static_cast<T>(M_PI);
}

#endif
