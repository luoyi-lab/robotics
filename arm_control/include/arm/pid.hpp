#pragma once // 防重复包含

namespace arm { // 命名空间，绝对不写 using namespace std;

enum class AntiWindup { None, Clamp, Conditional };

class PID {
public:
    PID(double kp, double ki, double kd, double u_lim = 50.0, AntiWindup mode = AntiWindup::Conditional);
    void reset();
    double update(double target, double current, double dt);

private:
    double kp_, ki_, kd_, u_lim_;
    AntiWindup mode_;
    double integral_ = 0.0;
    double prev_val_ = 0.0;
    bool first_ = true;
};

} // namespace arm