#include "cho_hardware_openarm_mit_mujoco/mit_limiter.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace cho_hardware_openarm_mit_mujoco {
namespace {double clamp(double x,double lo,double hi){return std::max(lo,std::min(x,hi));} double slew(double x,double old,double rate,double dt){return clamp(x,old-rate*dt,old+rate*dt);}}
Limiter::Limiter(const cho_openarm_mit_core::SafetyProfile&p,std::size_t rate):limits_{p.position_lower,p.position_upper,p.command_velocity,p.kp_max,p.kd_max,p.kp_slew,p.kd_slew,p.safe_stiffness,p.safe_damping,p.tau_ff_max,p.tau_ff_slew,p.final_torque,p.final_slew,p.lease_cap}{if(p.backend!=cho_openarm_mit_core::SafetyBackend::MUJOCO||p.name.empty()||rate!=p.update_rate_hz)throw std::invalid_argument("explicit MuJoCo safety profile/update rate mismatch");}
void Limiter::reset(const std::array<double,N>&q){for(std::size_t i=0;i<N;++i)command_[i]=applied_[i]={q[i],0,limits_.safe_kp[i],limits_.safe_kd[i],0};last_tau_.fill(0);ack_=lease_=age_=0;safe_=true;fault_=false;capture_safe_position_=false;}
bool Limiter::submit(const std::array<Tuple,N>&r,std::uint64_t gen,std::uint64_t lease){
 if(fault_||gen==0||gen<=ack_||lease==0||lease>limits_.lease_cap)return false;
 for(std::size_t i=0;i<N;++i)if(!std::isfinite(r[i].q)||!std::isfinite(r[i].dq)||!std::isfinite(r[i].kp)||!std::isfinite(r[i].kd)||!std::isfinite(r[i].tau)||r[i].kp<0||r[i].kd<0)return false;
 command_=r;lease_=lease;age_=0;ack_=gen;safe_=false;return true;
}
std::array<double,N> Limiter::update(const std::array<double,N>&q,const std::array<double,N>&dq,double dt){std::array<double,N>out{};if(fault_){last_tau_.fill(0);return out;}if(!safe_&&++age_>=lease_){safe_=true;capture_safe_position_=true;}if(capture_safe_position_){for(std::size_t i=0;i<N;++i)command_[i].q=q[i];capture_safe_position_=false;}for(std::size_t i=0;i<N;++i){auto target=command_[i];if(safe_)target={command_[i].q,0,limits_.safe_kp[i],limits_.safe_kd[i],0};target.q=clamp(target.q,limits_.q_lo[i],limits_.q_hi[i]);target.dq=clamp(target.dq,-limits_.dq[i],limits_.dq[i]);applied_[i].q=target.q;applied_[i].dq=target.dq;applied_[i].kp=slew(clamp(target.kp,0,limits_.kp[i]),applied_[i].kp,limits_.kp_slew[i],dt);applied_[i].kd=slew(clamp(target.kd,0,limits_.kd[i]),applied_[i].kd,limits_.kd_slew[i],dt);applied_[i].tau=slew(clamp(target.tau,-limits_.tau_ff[i],limits_.tau_ff[i]),applied_[i].tau,limits_.tau_ff_slew[i],dt);double raw=applied_[i].kp*(applied_[i].q-q[i])+applied_[i].kd*(applied_[i].dq-dq[i])+applied_[i].tau;out[i]=slew(clamp(raw,-limits_.tau[i],limits_.tau[i]),last_tau_[i],limits_.tau_slew[i],dt);}last_tau_=out;return out;}
}
