#ifndef DEADZONE_H_
#define DEADZONE_H_

template<typename T>
class Deadzone
{
public:
Deadzone(T deadzone)
    : deadzone_(deadzone), slope_(1.0), offset_(0.0)
{
    if (deadzone_ < 1.0 && deadzone_ >= 0.0) {
        slope_ = 1.0 / (1.0 - deadzone_);
        offset_ = -slope_ * deadzone_;
    } else {
        deadzone_ = 1.0;
        slope_ = 0.0;
        offset_ = 0.0;
    }
}

T operator()(T input) const
{
    if (input >= deadzone_) {
        if (input > 1.0)
            input = 1.0;
        return slope_ * input + offset_;
    } else if (input <= -deadzone_) {
      if (input < -1.0)
        input = -1.0;
      return slope_ * input - offset_;
    }
    return 0.0;
}

private:
  T deadzone_;
  T slope_;
  T offset_;
};

#endif /* DEADZONE_H_ */
