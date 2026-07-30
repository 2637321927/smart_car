#ifndef CONTROL_PROFILE_HPP
#define CONTROL_PROFILE_HPP

enum ControlProfileMode {
    CONTROL_PROFILE_STABLE = 0,
    CONTROL_PROFILE_PRO = 1,
};

struct ControlProfileValues {
    float target_speed_rps;
    float speed_p;
    float speed_i;
    float speed_d;
    float direction_p;
    float direction_d;
    float aim_m;
    int speed_slow_ratio;
    float gyro_outer_p;
    float gyro_outer_d;
    float gyro_inner_p;
    float gyro_inner_i;
    float gyro_target_max_dps;
    float gyro_turn_max_rps;
    float rescue_target_dps;
    float rescue_base_rps;
    float rescue_turn_max_rps;
};

class ControlProfile {
public:
    explicit ControlProfile(const ControlProfileValues &defaults);

    const ControlProfileValues &defaults() const;
    const ControlProfileValues &session() const;
    void reset_session();
    void set_session(const ControlProfileValues &values);

private:
    ControlProfileValues defaults_;
    ControlProfileValues session_;
};

extern ControlProfile stable_profile;
extern ControlProfile pro_profile;

void control_profile_init();
bool control_profile_switch(ControlProfileMode mode);
ControlProfileMode control_profile_mode();
bool control_profile_is_pro();
float control_profile_target_speed_rps();
float control_profile_set_target_speed_rps(float speed_rps);
void control_profile_capture_active();

#endif
