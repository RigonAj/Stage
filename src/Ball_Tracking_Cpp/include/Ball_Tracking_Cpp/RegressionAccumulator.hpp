#pragma once

#include <cmath>
#include <cstddef>

struct coef {
    float a = 0.0f;
    float b = 0.0f;
};

struct coef2 {
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
};

class LinearRegression {
public:
    void reset() {
        n_ = 0;
        sum_x_ = 0.0;
        sum_y_ = 0.0;
        sum_xx_ = 0.0;
        sum_xy_ = 0.0;
    }

    void add(double x, double y) {
        ++n_;
        sum_x_ += x;
        sum_y_ += y;
        sum_xx_ += x * x;
        sum_xy_ += x * y;
    }

    coef fit() const {
        if (n_ < 2) {
            return {0.0f, 0.0f};
        }

        const double n = static_cast<double>(n_);
        const double denom = n * sum_xx_ - sum_x_ * sum_x_;

        if (std::abs(denom) < 1e-12) {
            return {0.0f, 0.0f};
        }

        const double a = (n * sum_xy_ - sum_x_ * sum_y_) / denom;
        const double b = (sum_y_ - a * sum_x_) / n;

        return {
            static_cast<float>(a),
            static_cast<float>(b)
        };
    }

    std::size_t size() const {
        return n_;
    }

private:
    std::size_t n_ = 0;
    double sum_x_ = 0.0;
    double sum_y_ = 0.0;
    double sum_xx_ = 0.0;
    double sum_xy_ = 0.0;
};

class QuadraticRegression {
public:
    void reset() {
        n_ = 0;
        sum_x_ = 0.0;
        sum_x2_ = 0.0;
        sum_x3_ = 0.0;
        sum_x4_ = 0.0;
        sum_y_ = 0.0;
        sum_xy_ = 0.0;
        sum_x2y_ = 0.0;
    }

    void add(double x, double y) {
        const double x2 = x * x;

        n_++;
        sum_x_ += x;
        sum_x2_ += x2;
        sum_x3_ += x2 * x;
        sum_x4_ += x2 * x2;
        sum_y_ += y;
        sum_xy_ += x * y;
        sum_x2y_ += x2 * y;
    }

    coef2 fit() const {
        if (n_ < 3) {
            return {0.0f, 0.0f, 0.0f};
        }

        const double n = static_cast<double>(n_);

        const double avg_x = sum_x_ / n;
        const double avg_x2 = sum_x2_ / n;
        const double avg_y = sum_y_ / n;

        const double S11 = sum_x2_ - (sum_x_ * sum_x_) / n;
        const double S12 = sum_x3_ - (sum_x_ * sum_x2_) / n;
        const double S22 = sum_x4_ - (sum_x2_ * sum_x2_) / n;

        const double Sy1 = sum_xy_ - (sum_y_ * sum_x_) / n;
        const double Sy2 = sum_x2y_ - (sum_y_ * sum_x2_) / n;

        const double denom = S22 * S11 - S12 * S12;

        if (std::abs(denom) < 1e-12) {
            return {0.0f, 0.0f, 0.0f};
        }

        const double b = (Sy1 * S22 - Sy2 * S12) / denom;
        const double a = (Sy2 * S11 - Sy1 * S12) / denom;
        const double c = avg_y - b * avg_x - a * avg_x2;

        return {
            static_cast<float>(a),
            static_cast<float>(b),
            static_cast<float>(c)
        };
    }

    std::size_t size() const {
        return n_;
    }

private:
    std::size_t n_ = 0;
    double sum_x_ = 0.0;
    double sum_x2_ = 0.0;
    double sum_x3_ = 0.0;
    double sum_x4_ = 0.0;
    double sum_y_ = 0.0;
    double sum_xy_ = 0.0;
    double sum_x2y_ = 0.0;
};