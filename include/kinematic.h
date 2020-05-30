

static inline void inv_kinematic(const svel_s& vel, swheel_speed_s& wheel) {
    float vx = vel.vx;
    float vy = vel.vy;
    float wds = vel.wz;
    float ro = (L1 + L2) * wds;
    wheel.speed[0] = (vx + vy - ro) / R;
    wheel.speed[1] = (vx - vy + ro) / R;
    wheel.speed[2] = (vx - vy - ro) / R;
    wheel.speed[3] = (vx + vy + ro) / R;
}
static inline void fwd_kinematic(const swheel_speed_s& wheel, svel_s& out) {
    out.vx = 0.25 * R * (wheel.speed[0] + wheel.speed[1] + wheel.speed[2] + wheel.speed[3]);
    out.vy = 0.25 * R * (wheel.speed[0] - wheel.speed[1] - wheel.speed[2] + wheel.speed[3]);
    out.wz = 0.25 * R * wheelk*(-wheel.speed[0] + wheel.speed[1] - wheel.speed[2] + wheel.speed[3]);
}