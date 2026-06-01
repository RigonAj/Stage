#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

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

inline double RobustRegressionWeight(double residual, double scale) {
    if (!std::isfinite(residual) || !std::isfinite(scale) || scale <= 1.0e-9) {
        return 1.0;
    }

    const double normalized = residual / scale;
    return 1.0 / (1.0 + normalized * normalized);
}

inline double RecentRegressionWeight(double x, double newestX, double rangeX) {
    if (!std::isfinite(x) || !std::isfinite(newestX) || !std::isfinite(rangeX) || rangeX <= 1.0e-9) {
        return 1.0;
    }

    const double age = std::clamp((newestX - x) / rangeX, 0.0, 1.0);
    return std::exp(-3.0 * age);
}

inline coef WeightedLinearFit(
    const std::vector<float> &xs,
    const std::vector<float> &ys,
    coef initialFit = {}) {

    const std::size_t n = std::min(xs.size(), ys.size());
    if (n < 2) {
        return initialFit;
    }

    if (initialFit.a == 0.0f && initialFit.b == 0.0f) {
        LinearRegression initial;
        for (std::size_t i = 0; i < n; ++i) {
            initial.add(xs[i], ys[i]);
        }
        initialFit = initial.fit();
    }

    const auto [minIt, maxIt] = std::minmax_element(xs.begin(), xs.begin() + static_cast<std::ptrdiff_t>(n));
    const double rangeX = static_cast<double>(*maxIt - *minIt);
    const double newestX = static_cast<double>(*maxIt);

    std::vector<double> residuals;
    residuals.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double predicted = static_cast<double>(initialFit.a) * xs[i] + static_cast<double>(initialFit.b);
        residuals.push_back(std::fabs(static_cast<double>(ys[i]) - predicted));
    }

    const std::size_t medianIndex = residuals.size() / 2;
    std::nth_element(residuals.begin(), residuals.begin() + static_cast<std::ptrdiff_t>(medianIndex), residuals.end());
    const double residualScale = std::max(1.0e-6, residuals[medianIndex] * 1.4826);

    double sw = 0.0;
    double sx = 0.0;
    double sy = 0.0;
    double sxx = 0.0;
    double sxy = 0.0;

    for (std::size_t i = 0; i < n; ++i) {
        const double x = xs[i];
        const double y = ys[i];
        const double predicted = static_cast<double>(initialFit.a) * x + static_cast<double>(initialFit.b);
        const double residual = std::fabs(y - predicted);
        const double weight = RecentRegressionWeight(x, newestX, rangeX)
                            * RobustRegressionWeight(residual, residualScale);
        sw += weight;
        sx += weight * x;
        sy += weight * y;
        sxx += weight * x * x;
        sxy += weight * x * y;
    }

    const double denom = sw * sxx - sx * sx;
    if (sw <= 1.0e-9 || std::fabs(denom) < 1.0e-12) {
        return initialFit;
    }

    const double a = (sw * sxy - sx * sy) / denom;
    const double b = (sy - a * sx) / sw;
    return {static_cast<float>(a), static_cast<float>(b)};
}

inline coef2 WeightedQuadraticFit(
    const std::vector<float> &xs,
    const std::vector<float> &ys,
    coef2 initialFit = {}) {

    const std::size_t n = std::min(xs.size(), ys.size());
    if (n < 3) {
        return initialFit;
    }

    if (initialFit.a == 0.0f && initialFit.b == 0.0f && initialFit.c == 0.0f) {
        QuadraticRegression initial;
        for (std::size_t i = 0; i < n; ++i) {
            initial.add(xs[i], ys[i]);
        }
        initialFit = initial.fit();
    }

    const auto [minIt, maxIt] = std::minmax_element(xs.begin(), xs.begin() + static_cast<std::ptrdiff_t>(n));
    const double rangeX = static_cast<double>(*maxIt - *minIt);
    const double newestX = static_cast<double>(*maxIt);

    std::vector<double> residuals;
    residuals.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double x = xs[i];
        const double predicted = static_cast<double>(initialFit.a) * x * x
                               + static_cast<double>(initialFit.b) * x
                               + static_cast<double>(initialFit.c);
        residuals.push_back(std::fabs(static_cast<double>(ys[i]) - predicted));
    }

    const std::size_t medianIndex = residuals.size() / 2;
    std::nth_element(residuals.begin(), residuals.begin() + static_cast<std::ptrdiff_t>(medianIndex), residuals.end());
    const double residualScale = std::max(1.0e-6, residuals[medianIndex] * 1.4826);

    double sw = 0.0;
    double sx = 0.0;
    double sx2 = 0.0;
    double sx3 = 0.0;
    double sx4 = 0.0;
    double sy = 0.0;
    double sxy = 0.0;
    double sx2y = 0.0;

    for (std::size_t i = 0; i < n; ++i) {
        const double x = xs[i];
        const double y = ys[i];
        const double x2 = x * x;
        const double predicted = static_cast<double>(initialFit.a) * x2
                               + static_cast<double>(initialFit.b) * x
                               + static_cast<double>(initialFit.c);
        const double residual = std::fabs(y - predicted);
        const double weight = RecentRegressionWeight(x, newestX, rangeX)
                            * RobustRegressionWeight(residual, residualScale);
        sw += weight;
        sx += weight * x;
        sx2 += weight * x2;
        sx3 += weight * x2 * x;
        sx4 += weight * x2 * x2;
        sy += weight * y;
        sxy += weight * x * y;
        sx2y += weight * x2 * y;
    }

    if (sw <= 1.0e-9) {
        return initialFit;
    }

    const double avgX = sx / sw;
    const double avgX2 = sx2 / sw;
    const double avgY = sy / sw;
    const double s11 = sx2 - (sx * sx) / sw;
    const double s12 = sx3 - (sx * sx2) / sw;
    const double s22 = sx4 - (sx2 * sx2) / sw;
    const double sy1 = sxy - (sy * sx) / sw;
    const double sy2 = sx2y - (sy * sx2) / sw;
    const double denom = s22 * s11 - s12 * s12;

    if (std::fabs(denom) < 1.0e-12) {
        return initialFit;
    }

    const double b = (sy1 * s22 - sy2 * s12) / denom;
    const double a = (sy2 * s11 - sy1 * s12) / denom;
    const double c = avgY - b * avgX - a * avgX2;
    return {static_cast<float>(a), static_cast<float>(b), static_cast<float>(c)};
}
