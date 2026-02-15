#ifndef FILTER_HPP
#define FILTER_HPP

class Filter
{
public:
    Filter(float alpha = 0.1f);
    void lowPassFilterUpdate(const float input[3]);
    void lowPassFilterUpdate4(const float input[4]);
    void getFiltered(float output[3]) const;
    void getFiltered4(float output[4]) const;
    void setAlpha(float alpha);

private:
    float alpha_;
    float filtered_value_[3];
    float filtered_value4_[4];
    bool initialized_;
    bool initialized4_;
};

#endif
