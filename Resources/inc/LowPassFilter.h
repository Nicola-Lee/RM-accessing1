#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

class LowPassFilter {
public:
    // alpha 范围：(0, 1]；越小越平滑，但响应越慢
    explicit LowPassFilter(float alpha);

    float update(float input);
    void reset();
    float value() const;

private:
    float alpha_;
    float value_;
    bool initialized_;
};

#endif