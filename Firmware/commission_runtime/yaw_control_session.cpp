#include "session.hpp"
#include "io.hpp"
#include "axis_control_core.hpp"
#include "servo.hpp"
#include <algorithm>
#include <deque>
#include <numbers>

namespace ota::commission {
using namespace detail;
namespace {
struct ReferenceSegment { double duration, target; };
struct ReferenceSample { double time, position, velocity, acceleration; };
struct DepartureWindow { double begin,end; int direction; };
struct AppliedCurrent { bool success; double actual; int64_t accepted; };

// Algebraic steady-state feedforward only. The shared core remains the sole
// observer, transition, PI, limiter and accepted-current owner.
struct SelectedFamilyFeedforward {
  bool enabled=false;
  double a{},viscous{},negative{},positive{},load_offset{},load_slope{},q_origin{};
  double gain{},bias{},static_balance{},current_cap{},intent_threshold{},rest_speed{};
  double q_min{},q_max{},velocity_max{},acceleration_max{};
  std::string start_policy_readback;
  int planned_direction{};
  static int demand(void* context,const axis::PosteriorState* state,
                    const axis::Reference* reference,double* command) {
    auto& f=*static_cast<SelectedFamilyFeedforward*>(context);
    const int wanted=f.planned_direction?f.planned_direction:
      reference->velocity>f.intent_threshold?1:reference->velocity<-f.intent_threshold?-1:0;
    const bool unresolved=std::abs(state->velocity)<=f.rest_speed;
    double friction=f.static_balance;
    if(!unresolved || wanted) {
      // A nonzero posterior owns friction direction even at unresolved low
      // speed. Only a stationary posterior may borrow departure intent.
      const int direction=state->velocity!=0.?(state->velocity>0.?1:-1):wanted;
      friction=direction>0?f.positive:direction<0?-f.negative:f.static_balance;
    }
    const double load=f.load_offset+f.load_slope*(state->position-f.q_origin);
    *command=(f.a*reference->acceleration+load+f.viscous*reference->velocity+friction-f.bias)/f.gain;
    return std::isfinite(*command) && std::abs(*command)<=f.current_cap;
  }
  void load(const YAML::Node& node,const YAML::Node& limits,const axis::Parameters& p) {
    if(!node) return;
    enabled=true;
    require(node["policy"].as<std::string>()=="STEADY_STATE_REFERENCE" &&
            node["qualification"].as<std::string>()=="UNQUALIFIED" &&
            node["owner_authorized_unqualified"].as<bool>(),
            "DATA_INVALID: explicit owner authorization and unqualified steady-state policy required");
    require(p.acceleration_cap==0.,"INTEGRATION_MISMATCH: selected-family reference owns acceleration shaping");
    const auto model=node["model"];
    require(model["friction"].as<std::string>()=="coulomb" &&
            (model["load"].as<std::string>()=="constant" || model["load"].as<std::string>()=="affine"),
            "DATA_INVALID: selected-family bridge supports Coulomb constant/affine load");
    a=model["a"].as<double>(); viscous=model["viscous"].as<double>();
    negative=model["coulomb_negative"].as<double>(); positive=model["coulomb_positive"].as<double>();
    load_offset=model["load_offset"].as<double>();
    load_slope=model["load_slope"]?model["load_slope"].as<double>():0.;
    q_origin=model["q_origin"]?model["q_origin"].as<double>():0.;
    gain=model["actuator_gain"].as<double>(); bias=model["actuator_bias"].as<double>();
    static_balance=node["static_balance_A"].as<double>();
    q_min=limits["q_min_rad"].as<double>(); q_max=limits["q_max_rad"].as<double>();
    velocity_max=limits["velocity_max_rad_s"].as<double>();
    acceleration_max=limits["acceleration_max_rad_s2"].as<double>();
    for(const auto value:{a,viscous,negative,positive,load_offset,load_slope,q_origin,gain,bias,
                         static_balance,q_min,q_max,velocity_max,acceleration_max})
      require(std::isfinite(value),"DATA_INVALID: finite selected-family model/reference support required");
    const double sixty=std::numbers::pi/3.;
    require(a>0. && viscous>=0. && negative>=0. && positive>=0. && gain>0. && q_min<q_max &&
            velocity_max>0. && velocity_max<=sixty+1e-12 && acceleration_max>0. && acceleration_max<=sixty+1e-12 &&
            (model["load"].as<std::string>()!="constant" || load_slope==0.) &&
            std::abs(static_balance)<=std::max(negative,positive),
            "DATA_INVALID: invalid selected-family coefficient or owner reference envelope");
    current_cap=p.current_cap; intent_threshold=p.intent_threshold; rest_speed=p.rest_speed;
    const auto start=node["experimental_start_policy"];
    require(start && start["qualification"].as<std::string>()=="UNQUALIFIED_FLOOR" &&
            start["owner_authorized"].as<bool>() && p.start_timeout_s<=.2 && p.current_cap<=.9,
            "DATA_INVALID: bounded owner-authorized experimental START floor required");
    const auto identified=start["identified_start_censored"].as<std::vector<int>>();
    require(identified.size()==unsigned(6*p.model.n),"DATA_INVALID: preserve complete identified START censoring");
    const double static_negative=model["static_negative"].as<double>();
    const double static_positive=model["static_positive"].as<double>();
    require(std::isfinite(static_negative) && std::isfinite(static_positive) &&
            static_negative>=negative && static_positive>=positive,
            "DATA_INVALID: explicit finite physical-fit START floor points required");
    std::ostringstream start_log; start_log.precision(17);
    start_log<<"{\"kind\":\"experimental_start_policy_readback\",\"qualification\":\"UNQUALIFIED_FLOOR\","
             <<"\"owner_authorized\":true,\"identified_threshold_qualification\":\"UNKNOWN\","
             <<"\"execution_flags_meaning\":\"authorized_experimental_floor\",\"identified_start_censored\":[";
    for(int k=0;k<6*p.model.n;++k) {
      require((identified[k]==0 || identified[k]==1) && p.start_censored[k]==0,
              "DATA_INVALID: preserve identified flags separately from approved execution floor");
      const int direction=k<3*p.model.n?-1:1;
      const double load=load_offset+load_slope*(p.model.q[k%p.model.n]-q_origin);
      const double floor=(load+direction*(direction<0?static_negative:static_positive)-bias)/gain;
      require(std::abs(p.start_total[k]-floor)<=1e-12 && std::abs(floor)<=p.current_cap,
              "DATA_INVALID: experimental START must retain physical-fit floor without added excess");
      if(k) start_log<<',';
      start_log<<identified[k];
    }
    start_log<<"],\"start_timeout_s\":"<<p.start_timeout_s<<",\"current_cap_A\":"<<p.current_cap<<'}';
    start_policy_readback=start_log.str();
  }
  void validate(const ReferenceSample& sample) const {
    if(!enabled) return;
    require(sample.position>=q_min && sample.position<=q_max &&
            std::abs(sample.velocity)<=velocity_max+1e-12 &&
            std::abs(sample.acceleration)<=acceleration_max+1e-12,
            "DATA_INVALID: selected-family sample outside finite owner reference limits");
  }
  std::string readback() const {
    std::ostringstream out; out.precision(17);
    out<<"{\"kind\":\"selected_family_readback\",\"qualification\":\"UNQUALIFIED\","
       <<"\"owner_authorized_unqualified\":true,\"policy\":\"STEADY_STATE_REFERENCE\","
       <<"\"friction_direction\":\"current_posterior\",\"reference_shaping\":\"manifest_qva\","
       <<"\"model\":{\"a\":"<<a<<",\"viscous\":"<<viscous<<",\"coulomb_negative\":"<<negative
       <<",\"coulomb_positive\":"<<positive<<",\"load_offset\":"<<load_offset<<",\"load_slope\":"<<load_slope
       <<",\"q_origin\":"<<q_origin<<",\"actuator_gain\":"<<gain<<",\"actuator_bias\":"<<bias
       <<"},\"static_balance_A\":"<<static_balance<<",\"reference_limits\":{\"q_min_rad\":"<<q_min
       <<",\"q_max_rad\":"<<q_max<<",\"velocity_max_rad_s\":"<<velocity_max
       <<",\"acceleration_max_rad_s2\":"<<acceleration_max<<"}}";
    return out.str();
  }
};

template<class T> void flatten(const YAML::Node& node,std::vector<T>& result) {
  if (node.IsSequence()) for (const auto& item:node) flatten(item,result);
  else result.push_back(node.as<T>());
}
axis::Parameters load_parameters(const YAML::Node& node,bool selected_family=false) {
  axis::Parameters p{};
  const auto model=node["model"],observer=node["observer"];
  p.model.n=model["n"].as<int>(); p.model.periodic=model["periodic"].as<int>();
  require(p.model.n==5 || p.model.n==8,"DATA_INVALID: shared model node count");
  const auto q=model["q"].as<std::vector<double>>(),z=model["z"].as<std::vector<double>>();
  const auto theta=model["theta"].as<std::vector<double>>();
  require(q.size()==unsigned(p.model.n) && z.size()==3 && theta.size()==unsigned(7+6*p.model.n),
          "DATA_INVALID: complete shared model arrays required");
  std::copy(q.begin(),q.end(),p.model.q.begin()); std::copy(z.begin(),z.end(),p.model.z.begin());
  std::copy(theta.begin(),theta.end(),p.model.theta.begin());
#define OTA_OBSERVER_VALUE(name) p.observer.name=observer[#name].as<double>();
  OTA_OBSERVER_VALUE(encoder_variance) OTA_OBSERVER_VALUE(gyro_variance)
  OTA_OBSERVER_VALUE(process_variance) OTA_OBSERVER_VALUE(max_encoder_age_s)
  OTA_OBSERVER_VALUE(max_gyro_age_s) OTA_OBSERVER_VALUE(initial_position_variance)
  OTA_OBSERVER_VALUE(initial_velocity_variance)
#undef OTA_OBSERVER_VALUE
  p.observer.encoder_only_verified=observer["encoder_only_verified"].as<int>();
#define OTA_CONTROL_VALUE(name) p.name=node[#name].as<double>();
  OTA_CONTROL_VALUE(kp) OTA_CONTROL_VALUE(ki) OTA_CONTROL_VALUE(kpos) OTA_CONTROL_VALUE(kaw)
  OTA_CONTROL_VALUE(current_cap) OTA_CONTROL_VALUE(slew) OTA_CONTROL_VALUE(integral_cap)
  OTA_CONTROL_VALUE(velocity_cap) OTA_CONTROL_VALUE(dt_min) OTA_CONTROL_VALUE(dt_max)
  OTA_CONTROL_VALUE(intent_threshold) OTA_CONTROL_VALUE(rest_speed) OTA_CONTROL_VALUE(sustained_s)
  OTA_CONTROL_VALUE(start_timeout_s)
  OTA_CONTROL_VALUE(acceleration_cap) OTA_CONTROL_VALUE(acceleration_noise_sigma)
  OTA_CONTROL_VALUE(acceleration_sample_period_s)
  OTA_CONTROL_VALUE(acceleration_current_window_enabled)
#undef OTA_CONTROL_VALUE
  std::vector<double> start; std::vector<int> censored;
  flatten(node["start_total"],start); flatten(node["start_censored"],censored);
  require(start.size()==unsigned(6*p.model.n) && censored.size()==start.size(),
          "DATA_INVALID: complete shared startup arrays required");
  std::copy(start.begin(),start.end(),p.start_total.begin());
  std::copy(censored.begin(),censored.end(),p.start_censored.begin());
  require(axis::valid(p),"DATA_INVALID: shared controller parameters invalid");
  require((selected_family?p.acceleration_cap==0:p.acceleration_cap>0) && p.acceleration_sample_period_s>0,
          "DATA_INVALID: live yaw acceleration constraint must be explicitly bound");
  return p;
}
template<class T> void array_json(std::ostream& out,const T& values,int count) {
  out<<'['; for (int i=0;i<count;++i) { if (i) out<<','; out<<values[i]; } out<<']';
}
std::string parameter_json(const axis::Parameters& p) {
  std::ostringstream out; out.precision(17);
  out<<"{\"model\":{\"n\":"<<p.model.n<<",\"periodic\":"<<p.model.periodic<<",\"q\":";
  array_json(out,p.model.q,p.model.n); out<<",\"z\":"; array_json(out,p.model.z,3);
  out<<",\"theta\":"; array_json(out,p.model.theta,7+6*p.model.n); out<<"},\"observer\":{";
#define OTA_OBSERVER_JSON(name) out<<"\"" #name "\":"<<p.observer.name<<',';
  OTA_OBSERVER_JSON(encoder_variance) OTA_OBSERVER_JSON(gyro_variance) OTA_OBSERVER_JSON(process_variance)
  OTA_OBSERVER_JSON(max_encoder_age_s) OTA_OBSERVER_JSON(max_gyro_age_s)
  OTA_OBSERVER_JSON(initial_position_variance) OTA_OBSERVER_JSON(initial_velocity_variance)
#undef OTA_OBSERVER_JSON
  out<<"\"encoder_only_verified\":"<<p.observer.encoder_only_verified<<"},";
#define OTA_CONTROL_JSON(name) out<<"\"" #name "\":"<<p.name<<',';
  OTA_CONTROL_JSON(kp) OTA_CONTROL_JSON(ki) OTA_CONTROL_JSON(kpos) OTA_CONTROL_JSON(kaw)
  OTA_CONTROL_JSON(current_cap) OTA_CONTROL_JSON(slew) OTA_CONTROL_JSON(integral_cap)
  OTA_CONTROL_JSON(velocity_cap) OTA_CONTROL_JSON(dt_min) OTA_CONTROL_JSON(dt_max)
  OTA_CONTROL_JSON(intent_threshold) OTA_CONTROL_JSON(rest_speed) OTA_CONTROL_JSON(sustained_s)
  OTA_CONTROL_JSON(start_timeout_s)
  OTA_CONTROL_JSON(acceleration_cap) OTA_CONTROL_JSON(acceleration_noise_sigma)
  OTA_CONTROL_JSON(acceleration_sample_period_s)
  OTA_CONTROL_JSON(acceleration_current_window_enabled)
#undef OTA_CONTROL_JSON
  out<<"\"start_total\":"; array_json(out,p.start_total,6*p.model.n);
  out<<",\"start_censored\":"; array_json(out,p.start_censored,6*p.model.n); out<<'}';
  return out.str();
}

void load_servo_extra(const YAML::Node& node,axis::ServoParameters& p) {
  p.friction_learning_rate=node["friction_learning_rate"].as<double>();
  p.friction_learning_speed=node["friction_learning_speed"].as<double>();
  p.friction_map_limit=node["friction_map_limit"].as<double>();
  const auto pos=node["friction_map_positive"].as<std::vector<double>>(),neg=node["friction_map_negative"].as<std::vector<double>>();
  require(pos.size()==axis::ServoParameters::kFrictionBins && neg.size()==pos.size(),"DATA_INVALID: friction map size");
  std::copy(pos.begin(),pos.end(),p.friction_map_positive); std::copy(neg.begin(),neg.end(),p.friction_map_negative);
  p.crosstalk_delay_s=node["crosstalk_delay_s"]?node["crosstalk_delay_s"].as<double>():0.;
  if (const auto stall=node["stall_recovery"]) {
    p.stall_error=stall["error_rad"].as<double>(); p.stall_speed=stall["speed_rad_s"].as<double>();
    p.stall_current=stall["current_A"].as<double>(); p.stall_time_s=stall["time_s"].as<double>();
    p.rock_current=stall["rock_current_A"].as<double>(); p.rock_s=stall["rock_s"].as<double>();
  }
  if (const auto map=node["crosstalk_map"]) {
    const auto values=map.as<std::vector<double>>();
    require(values.size()==axis::ServoParameters::kCrosstalkBins,"DATA_INVALID: crosstalk map size");
    std::copy(values.begin(),values.end(),p.crosstalk_map);
  }
}
axis::ServoParameters load_servo(const YAML::Node& node) {
  axis::ServoParameters p{};
#define OTA_SERVO_VALUE(name) p.name=node[#name].as<double>();
  OTA_SERVO_VALUE(encoder_variance) OTA_SERVO_VALUE(gyro_variance) OTA_SERVO_VALUE(process_variance)
  OTA_SERVO_VALUE(max_encoder_age_s) OTA_SERVO_VALUE(max_gyro_age_s)
  OTA_SERVO_VALUE(inertia) OTA_SERVO_VALUE(coulomb_positive) OTA_SERVO_VALUE(coulomb_negative)
  OTA_SERVO_VALUE(stribeck_positive) OTA_SERVO_VALUE(stribeck_negative) OTA_SERVO_VALUE(stribeck_speed)
  OTA_SERVO_VALUE(viscous) OTA_SERVO_VALUE(friction_band) OTA_SERVO_VALUE(load)
  OTA_SERVO_VALUE(friction_correction_rate) OTA_SERVO_VALUE(friction_correction_deadband)
  OTA_SERVO_VALUE(dither_amplitude) OTA_SERVO_VALUE(dither_period_s) OTA_SERVO_VALUE(kq) OTA_SERVO_VALUE(kv) OTA_SERVO_VALUE(ki)
  OTA_SERVO_VALUE(integral_cap) OTA_SERVO_VALUE(error_clamp) OTA_SERVO_VALUE(hold_band) OTA_SERVO_VALUE(hold_speed)
  OTA_SERVO_VALUE(hold_relax_tau_s) OTA_SERVO_VALUE(rms_limit) OTA_SERVO_VALUE(rms_tau_s)
  OTA_SERVO_VALUE(current_cap) OTA_SERVO_VALUE(slew) OTA_SERVO_VALUE(dt_min) OTA_SERVO_VALUE(dt_max)
  OTA_SERVO_VALUE(following_error_limit)
#undef OTA_SERVO_VALUE
  p.use_gyro=node["use_gyro"].as<int>();
  load_servo_extra(node,p);
  require(axis::valid(p),"DATA_INVALID: servo parameters invalid");
  return p;
}
std::string servo_json(const axis::ServoParameters& p) {
  // 9 significant digits keeps the full record (maps included) inside one journal line.
  std::ostringstream out; out.precision(9); out<<'{';
#define OTA_SERVO_JSON(name) out<<"\"" #name "\":"<<p.name<<',';
  OTA_SERVO_JSON(encoder_variance) OTA_SERVO_JSON(gyro_variance) OTA_SERVO_JSON(process_variance)
  OTA_SERVO_JSON(max_encoder_age_s) OTA_SERVO_JSON(max_gyro_age_s)
  OTA_SERVO_JSON(inertia) OTA_SERVO_JSON(coulomb_positive) OTA_SERVO_JSON(coulomb_negative)
  OTA_SERVO_JSON(stribeck_positive) OTA_SERVO_JSON(stribeck_negative) OTA_SERVO_JSON(stribeck_speed)
  OTA_SERVO_JSON(viscous) OTA_SERVO_JSON(friction_band) OTA_SERVO_JSON(load)
  OTA_SERVO_JSON(friction_correction_rate) OTA_SERVO_JSON(friction_correction_deadband)
  OTA_SERVO_JSON(dither_amplitude) OTA_SERVO_JSON(dither_period_s) OTA_SERVO_JSON(kq) OTA_SERVO_JSON(kv) OTA_SERVO_JSON(ki)
  OTA_SERVO_JSON(integral_cap) OTA_SERVO_JSON(error_clamp) OTA_SERVO_JSON(hold_band) OTA_SERVO_JSON(hold_speed)
  OTA_SERVO_JSON(hold_relax_tau_s) OTA_SERVO_JSON(rms_limit) OTA_SERVO_JSON(rms_tau_s)
  OTA_SERVO_JSON(current_cap) OTA_SERVO_JSON(slew) OTA_SERVO_JSON(dt_min) OTA_SERVO_JSON(dt_max)
  OTA_SERVO_JSON(following_error_limit)
#undef OTA_SERVO_JSON
  out<<"\"use_gyro\":"<<p.use_gyro<<",\"friction_learning_rate\":"<<p.friction_learning_rate
     <<",\"friction_learning_speed\":"<<p.friction_learning_speed<<",\"friction_map_limit\":"<<p.friction_map_limit;
  out<<",\"friction_map_positive\":"; array_json(out,p.friction_map_positive,axis::ServoParameters::kFrictionBins);
  out<<",\"friction_map_negative\":"; array_json(out,p.friction_map_negative,axis::ServoParameters::kFrictionBins);
  out<<",\"crosstalk_delay_s\":"<<p.crosstalk_delay_s<<",\"crosstalk_map\":";
  array_json(out,p.crosstalk_map,axis::ServoParameters::kCrosstalkBins);
  out<<'}';
  return out.str();
}

class YawControlSession {
 public:
  explicit YawControlSession(const YAML::Node& config)
      : config_(config),limits_(config["limits"]),
        synthetic_(config["provenance"].as<std::string>()=="SYNTHETIC"),readback_(127,0) {
    require(config["schema"].as<std::string>()=="adr0022.yaw-control/1" &&
            config["purpose"].as<std::string>()=="yaw_shared_core_3a","INTEGRATION_MISMATCH: yaw control manifest");
    require(synthetic_ || config["provenance"].as<std::string>()=="MEASURED","DATA_INVALID: provenance");
    require(config["transport"].as<std::string>()==(synthetic_?"loopback_udp":"socketcan"),
            "INTEGRATION_MISMATCH: transport/provenance mismatch");
    require(config["pitch_supported_when_disabled"].as<bool>(),"HARD_ABORT: pitch support required");
    candidate_=config["candidate_label"].as<std::string>();
    require(!candidate_.empty(),"DATA_INVALID: descriptive candidate label required");
    uid_text_=config["expected_pitch_uid"].as<std::string>();
    require(uid_text_.size()==16 && uid_text_.find_first_not_of("0123456789abcdef")==std::string::npos,
            "DATA_INVALID: pitch UID");
    uid_=std::stoull(uid_text_,nullptr,16);
    baseline_s_=positive("baseline_s"); stop_s_=positive("stop_observation_s");
    require(baseline_s_>=2. && stop_s_==2.,"DATA_INVALID: two-second baseline and zero observation required");
    current_bound_=positive("yaw_current_bound_A");
    // Continuous authority stays at the GM6020 0.9 A stall rating. The servo may
    // use short peaks up to 1.5 A (rated 1.62 A) only under an RMS budget <=0.9 A.
    require(current_bound_<=(config["servo_parameters"]?1.5:.9),"HARD_ABORT: yaw current authority");
    pitch_temperature_=positive("pitch_maximum_temperature_C");
    posture_=config["other_axis_posture_rad"].as<double>();
    position_offset_=config["yaw_position_offset_rad"]?config["yaw_position_offset_rad"].as<double>():0.;
    require(std::isfinite(posture_) && std::isfinite(position_offset_),"DATA_INVALID: finite measured coordinates");
    servo_mode_=bool(config["servo_parameters"]);
    if (servo_mode_) {
      // Position servo: one PID + reference feedforward, event-driven on encoder frames.
      require(!config["controller_parameters"] && !config["selected_family_feedforward"],
              "DATA_INVALID: choose servo_parameters or controller_parameters");
      const auto servo=load_servo(config["servo_parameters"]);
      require(servo.current_cap==current_bound_,"INTEGRATION_MISMATCH: current authority differs from servo");
      require(servo.rms_limit<=.9 && servo.rms_tau_s<=5.,"HARD_ABORT: servo RMS budget above the 0.9 A continuous rating");
      motor_temperature_limit_=config["servo_motor_temperature_limit_C"].as<double>();
      require(motor_temperature_limit_>0 && motor_temperature_limit_<=70.,"DATA_INVALID: servo motor temperature limit");
      require(servo_.configure(servo),"DATA_INVALID: servo parameter apply failed");
      speed_limit_=config["servo_speed_limit_rad_s"].as<double>();
      // Owner rule 2026-10-02: stay below 100 RPM (10.47 rad/s).
      require(std::isfinite(speed_limit_) && speed_limit_>0 && speed_limit_<=10.,"DATA_INVALID: servo speed limit");
      hold_after_s_=config["servo_hold_after_s"].as<double>();
      require(std::isfinite(hold_after_s_) && hold_after_s_>=0 && hold_after_s_<=10.,"DATA_INVALID: servo hold window");
      event_control_=config["servo_event_control"].as<bool>();
      if (const auto schedule=config["servo_gain_schedule"]) {
        for (const auto& item:schedule)
          gain_schedule_.push_back({item["begin_s"].as<double>(),item["kq"].as<double>(),item["kv"].as<double>(),item["ki"].as<double>()});
        require(!gain_schedule_.empty() && gain_schedule_.size()<=64,"DATA_INVALID: servo gain schedule");
      }
      if (const auto x=config["servo_excitation"]) {
        // Identification only: a log-swept sine added to the servo output.
        excitation_amplitude_=x["amplitude_A"].as<double>(); excitation_f0_=x["f0_hz"].as<double>();
        excitation_f1_=x["f1_hz"].as<double>(); excitation_begin_=x["begin_s"].as<double>();
        excitation_duration_=x["duration_s"].as<double>();
        require(excitation_amplitude_>0 && excitation_amplitude_<=.4 && excitation_f0_>0 && excitation_f1_>excitation_f0_ &&
                excitation_f1_<=200 && excitation_duration_>0,"DATA_INVALID: servo excitation");
      }
      parameter_readback_=servo_json(servo_.parameters());
    } else {
    const auto parameters=load_parameters(config["controller_parameters"],bool(config["selected_family_feedforward"]));
    family_.load(config["selected_family_feedforward"],config["reference_limits"],parameters);
    require(parameters.current_cap==current_bound_,"INTEGRATION_MISMATCH: current authority differs from runtime controller");
    require(controller_.configure(parameters),"DATA_INVALID: shared controller parameter apply failed");
    parameter_readback_=parameter_json(controller_.parameters());
    }
    const auto calibration=config["gyro_calibration"];
    const auto column=calibration["yaw_column"].as<std::vector<double>>();
    const auto bias=calibration["baseline_sensor_bias"].as<std::vector<double>>();
    require(column.size()==3 && bias.size()==3,"DATA_INVALID: frozen yaw gyro column and bias required");
    double norm=0.; for (const auto value:column) { require(std::isfinite(value),"DATA_INVALID: gyro column"); norm+=value*value; }
    require(norm>0.,"DATA_INVALID: gyro column has no yaw projection");
    for (unsigned i=0;i<3;++i) { require(std::isfinite(bias[i]),"DATA_INVALID: gyro bias"); projection_[i]=column[i]/norm; bias_[i]=bias[i]; }
    const auto samples=config["reference_samples"];
    if (samples) {
      require(!config["reference_segments"],"DATA_INVALID: choose reference samples or position segments");
      require(samples.IsSequence() && samples.size()>=2,"DATA_INVALID: finite reference sample table required");
      double previous=-1.;
      for (const auto& item:samples) {
        ReferenceSample sample{item["time_s"].as<double>(),item["position_rad"].as<double>(),
                               item["velocity_rad_s"].as<double>(),item["acceleration_rad_s2"].as<double>()};
        require(std::isfinite(sample.time) && std::isfinite(sample.position) &&
                std::isfinite(sample.velocity) && std::isfinite(sample.acceleration),
                "DATA_INVALID: finite reference time and q/v/a required");
        require((!reference_samples_.empty() || sample.time==0.) && sample.time>previous,
                "DATA_INVALID: reference samples must start at zero and increase in time");
        family_.validate(sample);
        reference_samples_.push_back(sample); previous=sample.time;
      }
      reference_s_=reference_samples_.back().time;
    } else {
      require(!family_.enabled,"DATA_INVALID: selected-family mode requires shaped reference samples");
      const auto references=config["reference_segments"];
      require(references.IsSequence() && references.size()>0,"DATA_INVALID: finite shaped reference segments required");
      for (const auto& item:references) {
        ReferenceSegment segment{item["duration_s"].as<double>(),item["target_position_rad"].as<double>()};
        require(std::isfinite(segment.duration) && segment.duration>0 && std::isfinite(segment.target),
                "DATA_INVALID: finite shaped reference required");
        references_.push_back(segment); reference_s_+=segment.duration;
      }
    }
    require((baseline_s_+reference_s_+hold_after_s_+stop_s_)*1e9<limits_.duration,"DATA_INVALID: control sequence deadline");
    if(family_.enabled) {
      const auto cases=config["reference_profile"]["cases"];
      require(cases.IsSequence() && cases.size()>0 && cases.size()<=128,
              "DATA_INVALID: finite selected-family departure cases required");
      double previous=-1.;
      for(const auto& item:cases) {
        DepartureWindow window{item["begin_s"].as<double>(),item["plateau_begin_s"].as<double>(),
                               item["direction"].as<int>()};
        require(std::isfinite(window.begin) && std::isfinite(window.end) && window.begin>=0. &&
                window.begin>previous && window.begin<window.end && window.end<=reference_s_ &&
                (window.direction==-1 || window.direction==1),
                "DATA_INVALID: finite ordered departure anchors required");
        departures_.push_back(window); previous=window.end;
      }
    }
    imu_fd_=config["imu_fd"].as<int>(); struct stat info{};
    require(imu_fd_>2 && imu_fd_!=8 && fstat(imu_fd_,&info)==0 && S_ISFIFO(info.st_mode),
            "DATA_INVALID: inherited IMU pipe required");
    require(fcntl(imu_fd_,F_SETFL,fcntl(imu_fd_,F_GETFL)|O_NONBLOCK)==0,"DATA_INVALID: IMU nonblocking pipe");
    if (!synthetic_) {
      check_launcher_lease();
      require(config["yaw"]["interface"].as<std::string>()=="can0" && config["pitch"]["interface"].as<std::string>()=="can1",
              "INTEGRATION_MISMATCH: station topology");
    }
    journal_=std::make_unique<Journal>(config["output"].as<std::string>(),
      "{\"kind\":\"header\",\"schema\":\"adr0022.yaw-control/1\",\"purpose\":\"yaw_shared_core_3a\",\"provenance\":"+
      quoted(synthetic_?"SYNTHETIC":"MEASURED")+",\"candidate_label\":"+quoted(candidate_)+",\"parameter_qualified\":false,\"manifest_yaml\":"+quoted(json(config))+"}",
      // 1 kHz servo sessions log ~4 rows/ms; absorb SD-card write stalls.
      config["servo_parameters"]?16384:4096);
    buses_[0].open(config["yaw"],synthetic_,limits_); buses_[1].open(config["pitch"],synthetic_,limits_);
  }
  int run() {
    interrupted=0; std::signal(SIGINT,on_signal); std::signal(SIGTERM,on_signal); begin_=monotonic_ns();
    std::cout<<"{\"kind\":\"capture_ready\",\"excitation\":true,\"purpose\":\"yaw_shared_core_3a\"}\n"<<std::flush;
    std::string failure; bool complete=false,zero_completed=false;
    try {
      record("{\"kind\":\"session_begin\",\"time_ns\":"+std::to_string(begin_)+"}");
      record("{\"kind\":\"controller_parameters_readback\",\"candidate_label\":"+quoted(candidate_)+
             ",\"source\":"+quoted(servo_mode_?"configured_servo":"configured_shared_core")+",\"parameters\":"+parameter_readback_+"}");
      if(family_.enabled) { record(family_.readback()); record(family_.start_policy_readback); }
      const auto first_zero=yaw(0.,"baseline");
      require(first_zero.success,"HARD_ABORT: yaw baseline zero TX failed");
      baseline_current_time_=first_zero.accepted;
      discovery_begin_=pitch(cybergear::make_discovery_request(0,127),"discovery");
      while (!complete) {
        pump(); const auto now=monotonic_ns();
        require(!interrupted,"HARD_ABORT: yaw shared-core session interrupted");
        require(now-begin_<limits_.duration,"MEASUREMENT_LIMITED: yaw control deadline");
        require(identified_ || now-discovery_begin_<limits_.read_timeout,"MEASUREMENT_LIMITED: pitch discovery timeout");
        readback_.check_deadline(now); supervise(now); service_pitch(now);
        if (!control_begin_ && now-begin_>=int64_t(baseline_s_*1e9) && identified_ && disabled_ && have_mode_) {
          check_streams(now); initial_position_=position(); control_begin_=now;
          if (servo_mode_) require(servo_.reset(seconds(now),initial_position_,0.,0.),"DATA_INVALID: servo reset failed");
          else
          require(controller_.reset(seconds(now),initial_position_,gyro_rate_,0.,1,seconds(baseline_current_time_)),"DATA_INVALID: shared controller reset failed");
          next_control_=now+5'000'000;
          record("{\"kind\":\"yaw_control_begin\",\"time_ns\":"+std::to_string(now)+"}");
        }
        if (servo_mode_ && control_begin_) {
          // The servo holds the final reference sample, then the session ends.
          if ((now-control_begin_)*1e-9>=reference_s_+hold_after_s_) { complete=true; servo_done_=true; }
          else if (!event_control_ && now>=next_control_) {
            control_servo(now); next_control_+=((now-next_control_)/5'000'000+1)*5'000'000;
          }
        }
        else if (control_begin_ && !braking_begin_ && (now-control_begin_)*1e-9>=reference_s_) {
          braking_begin_=now;
          record("{\"kind\":\"controlled_stop_begin\",\"time_ns\":"+std::to_string(now)+"}");
        }
        if (!servo_mode_ && control_begin_ && now>=next_control_) {
          control(now); next_control_+=((now-next_control_)/5'000'000+1)*5'000'000;
          if(braking_begin_) {
            const auto& parameters=controller_.parameters();
            if(std::abs(gyro_rate_)<=parameters.rest_speed &&
               std::abs(last_shaped_velocity_)<=parameters.rest_speed) {
              if(!quiet_since_) quiet_since_=now;
              if((now-quiet_since_)*1e-9>=parameters.sustained_s) complete=true;
            } else quiet_since_=0;
          }
        }
        else if (!control_begin_ && now>=next_control_) {
          require(yaw(0.,"baseline").success,"HARD_ABORT: yaw baseline TX failed"); next_control_=now+5'000'000;
        }
        require(journal_->healthy(),"DATA_INVALID: capture writer failed");
      }
    } catch (const std::exception& e) { failure=e.what(); }
    servo_done_=true;  // nothing but zero current from here on, whatever ended the loop
    if (servo_mode_) record_best("{\"kind\":\"servo_learned\",\"parameters\":"+servo_json(servo_.learned())+"}");
    const auto stop_begin=monotonic_ns();
    try { zero_completed=yaw(0.,"stop").success; require(zero_completed,"HARD_ABORT: final zero TX failed"); }
    catch (const std::exception& e) { if (failure.empty()) failure=e.what(); }
    record_best("{\"kind\":\"stop_observation_begin\",\"time_ns\":"+std::to_string(stop_begin)+"}");
    next_control_=stop_begin+5'000'000;
    while (monotonic_ns()-stop_begin<int64_t(stop_s_*1e9)) {
      try {
        pump(); const auto now=monotonic_ns();
        if (now>=next_control_) { const auto sent=yaw(0.,"stop"); zero_completed|=sent.success; require(sent.success,"HARD_ABORT: stop zero TX failed"); next_control_=now+5'000'000; }
        service_pitch(now); supervise(now);
      } catch (const std::exception& e) {
        if (failure.empty()) failure=e.what();
        const auto now=monotonic_ns();
        if (now>=next_control_) { try { zero_completed|=yaw(0.,"stop").success; } catch (...) {} next_control_=now+5'000'000; }
      }
    }
    return finish(complete && failure.empty() && zero_completed,complete,zero_completed,failure);
  }
 private:
  double positive(const char* key) const { const auto value=config_[key].as<double>(); require(std::isfinite(value) && value>0,"DATA_INVALID: positive runtime boundary required"); return value; }
  double seconds(int64_t time) const { return (time-begin_)*1e-9; }
  double position() const { return position_offset_+encoder_counts_*(2.*std::numbers::pi/8192.); }
  void record(const std::string& row) { require(journal_->append(row),"DATA_INVALID: capture writer failed"); }
  void record_best(const std::string& row) { if (journal_->healthy()) journal_->append(row); }
  std::pair<bool,int64_t> transmit(unsigned axis,const can::RawFrame& frame) {
    can_frame wire{}; wire.can_id=frame.id|(frame.extended?CAN_EFF_FLAG:0); wire.can_dlc=frame.dlc;
    std::memcpy(wire.data,frame.data,8); const auto before=monotonic_ns(); ssize_t count;
    auto& bus=buses_[axis];
    if (synthetic_) count=sendto(bus.fd.value,&wire,sizeof(wire),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&bus.peer),sizeof(bus.peer));
    else count=::send(bus.fd.value,&wire,sizeof(wire),MSG_DONTWAIT);
    tx_begin_=before; tx_accepted_=monotonic_ns(); return {count==sizeof(wire),before};
  }
  AppliedCurrent yaw(double requested,const char* phase) {
    require(std::isfinite(requested),"DATA_INVALID: yaw current is not finite");
    const auto limited=std::clamp(requested,-current_bound_,current_bound_);
    const int raw=gm6020::current_raw_uncapped(limited); auto frame=gm6020::current_zero_frame(1);
    const auto encoded=static_cast<uint16_t>(static_cast<int16_t>(raw)); frame.data[0]=uint8_t(encoded>>8); frame.data[1]=uint8_t(encoded);
    const auto [success,_]=transmit(0,frame); const double actual=raw*gm6020::kAmpsPerRaw;
    std::ostringstream out; out.precision(17);
    out<<"{\"kind\":\"yaw_current_tx\",\"phase\":"<<quoted(phase)<<",\"requested_A\":"<<requested
       <<",\"limited_A\":"<<limited<<",\"successful_tx_A\":";
    if (success) out<<actual; else out<<"null";
    out<<",\"current_raw\":"<<raw<<",\"id\":510,\"unused_slots_zero\":true,\"begin_ns\":"<<tx_begin_
       <<",\"kernel_accepted_ns\":"<<tx_accepted_<<",\"success\":"<<(success?"true":"false")<<'}'; record(out.str());
    return {success,actual,tx_accepted_};
  }
  int64_t pitch(const cybergear::CanFrame& source,const char* operation) {
    const auto id=cybergear::unpack_ext_id(source.id);
    require(id.target==127 && id.data2==0 && (id.comm_type==0 || id.comm_type==4 || id.comm_type==17),
            "HARD_ABORT: pitch command must be discovery, STOP or read");
    can::RawFrame frame; frame.id=source.id; frame.dlc=source.dlc; std::memcpy(frame.data,source.data,8);
    const auto [success,before]=transmit(1,frame);
    record("{\"kind\":\"pitch_tx\",\"operation\":"+quoted(operation)+",\"id\":"+std::to_string(source.id)+
      ",\"begin_ns\":"+std::to_string(before)+",\"kernel_accepted_ns\":"+std::to_string(tx_accepted_)+",\"success\":"+(success?"true":"false")+"}");
    require(success,"HARD_ABORT: pitch STOP/read TX failed"); return before;
  }
  axis::Reference reference(double elapsed) const {
    if (!reference_samples_.empty()) {
      const auto upper=std::upper_bound(reference_samples_.begin(),reference_samples_.end(),elapsed,
        [](double time,const ReferenceSample& sample) { return time<sample.time; });
      if (upper==reference_samples_.begin()) {
        const auto& first=*upper;
        return {first.position,first.velocity,first.acceleration,posture_};
      }
      if (upper==reference_samples_.end()) {
        const auto& last=reference_samples_.back();
        return {last.position,last.velocity,last.acceleration,posture_};
      }
      const auto& lower=*(upper-1);
      const double fraction=(elapsed-lower.time)/(upper->time-lower.time);
      return {std::lerp(lower.position,upper->position,fraction),
              std::lerp(lower.velocity,upper->velocity,fraction),
              std::lerp(lower.acceleration,upper->acceleration,fraction),posture_};
    }
    double from=initial_position_;
    for (const auto& segment:references_) {
      if (elapsed<=segment.duration) {
        const double s=std::clamp(elapsed/segment.duration,0.,1.),delta=segment.target-from;
        const double s2=s*s,s3=s2*s,s4=s3*s,s5=s4*s;
        return {from+delta*(10*s3-15*s4+6*s5),delta*(30*s2-60*s3+30*s4)/segment.duration,
                delta*(60*s-180*s2+120*s3)/(segment.duration*segment.duration),posture_};
      }
      elapsed-=segment.duration; from=segment.target;
    }
    return {from,0.,0.,posture_};
  }
  void control_servo(int64_t now) {
    const double since=(now-control_begin_)*1e-9;
    while (next_gain_<gain_schedule_.size() && since>=gain_schedule_[next_gain_][0]) {
      const auto& g=gain_schedule_[next_gain_++];
      require(servo_.set_gains(g[1],g[2],g[3]),"DATA_INVALID: servo gain schedule entry");
      record("{\"kind\":\"servo_gains\",\"time_ns\":"+std::to_string(now)+",\"kq\":"+std::to_string(g[1])+
             ",\"kv\":"+std::to_string(g[2])+",\"ki\":"+std::to_string(g[3])+"}");
    }
    const auto r=reference(since);
    // Servo references are relative to the position where control began.
    const auto out=servo_.step(seconds(now),initial_position_+r.position,r.velocity,r.acceleration);
    std::ostringstream log; log.precision(9);
    require(motor_temperature_<motor_temperature_limit_,"HARD_ABORT: yaw motor temperature limit");
    log<<"{\"kind\":\"servo_cycle\",\"time_ns\":"<<now<<",\"qr\":"<<initial_position_+r.position<<",\"vr\":"<<r.velocity<<",\"ar\":"<<r.acceleration
       <<",\"q\":"<<out.position<<",\"v\":"<<out.velocity<<",\"gyro\":"<<gyro_rate_<<",\"req\":"<<out.requested<<",\"u\":"<<out.limited
       <<",\"ff\":"<<out.feedforward<<",\"fr\":"<<out.friction<<",\"p\":"<<out.proportional<<",\"d\":"<<out.derivative
       <<",\"i\":"<<out.integral<<",\"rms\":"<<out.rms<<",\"cap\":"<<out.cap<<",\"sat\":"<<out.saturated<<",\"rock\":"<<out.rocking<<",\"stalls\":"<<out.stall_events<<",\"stale\":"<<servo_.stale_samples()<<",\"status\":"<<out.status<<'}'; record(log.str());
    require(out.status!=int(axis::ServoStatus::FollowingError),"HARD_ABORT: servo following error limit");
    require(out.status==int(axis::ServoStatus::Ok),"MEASUREMENT_LIMITED: servo sensor data stale or invalid");
    // Physical speed check: encoder displacement over >=20 ms and the independent
    // gyro. The observer's instantaneous estimate overshoots for a few ms after a
    // breakaway and is not, by itself, evidence of real overspeed.
    speed_history_.push_back({now,position()});
    while (speed_history_.size()>2 && now-speed_history_[1].first>=20'000'000) speed_history_.pop_front();
    const auto& [then,where]=speed_history_.front();
    const double encoder_speed=now-then>=20'000'000?(position()-where)/((now-then)*1e-9):0.;
    if (std::abs(encoder_speed)>speed_limit_ || std::abs(gyro_rate_)>speed_limit_)
      throw std::runtime_error("HARD_ABORT: yaw speed limit (encoder "+std::to_string(encoder_speed)+
                               " rad/s, gyro "+std::to_string(gyro_rate_)+" rad/s)");
    double excitation=0.;
    const double elapsed=(now-control_begin_)*1e-9-excitation_begin_;
    if (excitation_amplitude_>0 && elapsed>=0 && elapsed<excitation_duration_) {
      const double k=std::log(excitation_f1_/excitation_f0_)/excitation_duration_;
      excitation=excitation_amplitude_*std::sin(2*std::numbers::pi*excitation_f0_*(std::exp(k*elapsed)-1)/k);
    }
    const auto sent=yaw(std::clamp(out.limited+excitation,-current_bound_,current_bound_),excitation!=0.?"excitation":"control");
    servo_.acknowledge(sent.success,sent.actual);
    require(sent.success,"HARD_ABORT: yaw control TX failed"); ++control_cycles_;
  }
  void control(int64_t now) {
    const auto r=braking_begin_?axis::Reference{position(),0.,0.,posture_}:
      reference((now-control_begin_)*1e-9);
    const axis::Observation o{seconds(now),seconds(encoder_time_),seconds(gyro_time_),position(),gyro_rate_,
      encoder_sequence_,gyro_sequence_,1,encoder_time_>=control_begin_,gyro_time_>=control_begin_};
    family_.planned_direction=0;
    auto phase=r.velocity*r.acceleration<0.?axis::ReferencePhase::Braking:axis::ReferencePhase::Legacy;
    if(family_.enabled && !braking_begin_) {
      const double elapsed=(now-control_begin_)*1e-9;
      for(const auto& window:departures_) if(elapsed>=window.begin && elapsed<window.end &&
          window.direction*r.velocity>=0. && window.direction*r.acceleration>=0.) {
        family_.planned_direction=window.direction; phase=axis::ReferencePhase::Departure; break;
      }
    }
    const auto output=family_.enabled?
      controller_.step_with_posterior_phase(o,r,SelectedFamilyFeedforward::demand,&family_,phase,family_.planned_direction):
      controller_.step(o,r);
    std::ostringstream out; out.precision(17);
    out<<"{\"kind\":\"yaw_control_cycle\",\"time_ns\":"<<now<<",\"sequence\":"<<output.sequence
       <<",\"observation\":{\"encoder_ns\":"<<encoder_time_<<",\"gyro_ns\":"<<gyro_time_
       <<",\"encoder_sequence\":"<<encoder_sequence_<<",\"gyro_sequence\":"<<gyro_sequence_
       <<",\"position_rad\":"<<o.position<<",\"gyro_rate_rad_s\":"<<o.gyro_rate<<"},\"reference\":{\"position_rad\":"<<r.position
       <<",\"velocity_rad_s\":"<<r.velocity<<",\"acceleration_rad_s2\":"<<r.acceleration<<",\"posture_rad\":"<<r.posture
       <<"},\"core\":{\"status\":"<<output.status<<",\"motion\":"<<output.motion<<",\"encoder_only\":"<<output.encoder_only
       <<",\"requested_A\":"<<output.requested<<",\"limited_A\":"<<output.limited<<",\"position_rad\":"<<output.position
       <<",\"velocity_rad_s\":"<<output.velocity<<",\"integral_A\":"<<output.integral<<",\"feedforward_A\":"<<output.feedforward
       <<",\"start_increment_A\":"<<output.start_increment
       <<",\"requested_reference_velocity_rad_s\":"<<output.requested_reference_velocity
       <<",\"shaped_reference_velocity_rad_s\":"<<output.shaped_reference_velocity
       <<",\"requested_reference_acceleration_rad_s2\":"<<output.requested_reference_acceleration
       <<",\"shaped_reference_acceleration_rad_s2\":"<<output.shaped_reference_acceleration
       <<"},\"acceleration_control\":{\"fresh\":"<<(output.acceleration_fresh?"true":"false")
       <<",\"valid\":"<<(output.acceleration_valid?"true":"false")
       <<",\"sample_time_s\":"<<output.acceleration_sample_time
       <<",\"sample_interval_s\":"<<output.acceleration_interval_s
       <<",\"measured_rad_s2\":"<<output.measured_acceleration
       <<",\"noise_sigma_rad_s2\":"<<output.acceleration_noise_sigma
       <<",\"feedback_horizon_s\":"<<output.acceleration_feedback_horizon_s
       <<",\"delayed_interval_current_A\":"<<output.delayed_applied_current
       <<",\"current_min_A\":"<<output.acceleration_current_min
       <<",\"current_max_A\":"<<output.acceleration_current_max
       <<",\"limited_request_A\":"<<output.acceleration_limited_request
       <<",\"reason_bits\":"<<output.acceleration_limit_reason
       <<",\"startup_load_provisional\":"<<((output.acceleration_limit_reason&128)?"true":"false")
       <<",\"electrical_window_conflict\":"<<((output.acceleration_limit_reason&64)?"true":"false")
       <<",\"policy\":"<<quoted(controller_.parameters().acceleration_current_window_enabled?"provisional_current_window":"guidance")
       <<",\"history_timestamp_provenance\":"<<quoted(output.current_history_actual_time?"successful_tx_accepted_time":"controller_decision_time")
       <<",\"phase\":"<<quoted(braking_begin_?"controlled_stop":"reference")<<"}}"; record(out.str());
    require(output.status==int(axis::Status::Ok) || output.status==int(axis::Status::EnvelopeLimited),
            "MEASUREMENT_LIMITED: shared-core step cannot continue");
    const auto sent=yaw(output.limited,braking_begin_?"controlled_stop":"control");
    require(controller_.acknowledge_at(output.sequence,sent.success,sent.actual,seconds(sent.accepted)),"HARD_ABORT: shared-core actual TX acknowledgement failed");
    last_shaped_velocity_=output.shaped_reference_velocity;
    require(sent.success,"HARD_ABORT: yaw control TX failed"); ++control_cycles_;
  }
  void service_pitch(int64_t now) {
    if (!identified_) return;
    if (now>=next_stop_) { if (!pitch_stop_begin_) pitch_stop_begin_=now; pitch(cybergear::make_stop(0,127),"stop_poll"); next_stop_=now+limits_.stop_period; }
    require(disabled_ || now-pitch_stop_begin_<limits_.read_timeout,"HARD_ABORT: pitch STOP feedback timeout");
    if (disabled_ && !readback_.pending() && now>=next_read_) {
      const auto reg=have_mode_?((read_index_++%2)?cybergear::Reg::Iqf:cybergear::Reg::MechPos):cybergear::Reg::RunMode;
      const auto raw=readback_.begin(reg,now,limits_.read_timeout); cybergear::CanFrame frame; frame.id=raw.id; std::memcpy(frame.data,raw.data,8);
      pitch(frame,"register_read"); readback_.accepted(monotonic_ns(),true); next_read_=now+limits_.read_period;
    }
  }
  void check_streams(int64_t now) {
    for (const auto& bus:buses_) require(bus.last_feedback && now-bus.last_feedback<=limits_.can_gap,"HARD_ABORT: CAN feedback absent/stale");
    require(disabled_,"HARD_ABORT: pitch is not disabled"); require(imu_.ready() && gyro_sequence_,"MEASUREMENT_LIMITED: IMU streams absent");
    require(have_posture_,"MEASUREMENT_LIMITED: actual pitch posture readback absent");
    for (const auto& [_,sensor]:imu_.sensors) require(now-sensor.received<=limits_.imu_gap,"MEASUREMENT_LIMITED: IMU stream stale");
  }
  void supervise(int64_t now) {
    for (const auto& bus:buses_) if (bus.last_feedback) require(now-bus.last_feedback<=limits_.can_gap,"HARD_ABORT: CAN feedback lost");
    for (const auto& [_,sensor]:imu_.sensors) if (sensor.count) require(now-sensor.received<=limits_.imu_gap,"MEASUREMENT_LIMITED: IMU stream stale");
    if (control_begin_) require(disabled_,"HARD_ABORT: pitch became enabled during yaw control");
  }
  void imu_line(const std::string& raw) {
    imu_.line(raw,limits_,*journal_); const auto sample=YAML::Load(raw);
    if (sample["kind"].as<std::string>()!="sample" || sample["sensor"].as<std::string>()!="gyro") return;
    const int generation=sample["generation"].as<int>();
    require(!control_begin_ || generation==gyro_generation_,"HARD_ABORT: gyro generation changed during control");
    gyro_generation_=generation; gyro_time_=sample["sample_ns"].as<int64_t>();
    const auto values=sample["values"].as<std::vector<double>>(); gyro_rate_=0.;
    for (unsigned i=0;i<3;++i) gyro_rate_+=(values[i]-bias_[i])*projection_[i];
    ++gyro_sequence_;
    if (servo_mode_ && control_begin_) servo_.observe_gyro(seconds(gyro_time_),gyro_rate_);
  }
  void pump() {
    std::array<pollfd,3> fds{{{buses_[0].fd.value,POLLIN,0},{buses_[1].fd.value,POLLIN,0},{imu_fd_,POLLIN,0}}};
    const auto result=poll(fds.data(),fds.size(),1); if (result<0 && errno==EINTR) return;
    require(result>=0,"DATA_INVALID: acquisition poll failed");
    for (unsigned axis=0;axis<2;++axis) {
      auto& bus=buses_[axis]; require(!(fds[axis].revents&(POLLERR|POLLHUP|POLLNVAL)),"HARD_ABORT: CAN endpoint failed"); Receipt receipt;
      for (unsigned drained=0;drained<256 && bus.receiver->receive(receipt);++drained) {
        ++bus.sequence;
        // Servo mode: the decoded yaw_feedback row already carries the kernel timestamps.
        if (axis || !servo_mode_) record(receipt_json(receipt,axis?"pitch":"yaw",bus.sequence));
        require(!receipt.frame.error,"HARD_ABORT: CAN error frame"); require(!receipt.drop_delta,"DATA_INVALID: socket receive loss");
        if (!axis) {
          gm6020::Feedback value;
          if (gm6020::decode(receipt.frame,1,value)) {
            if (have_encoder_) { int delta=int(value.angle_count)-int(previous_encoder_); if (delta>4096) delta-=8192; if (delta<-4096) delta+=8192; encoder_counts_+=delta; }
            if (!have_encoder_ && servo_mode_) position_offset_=value.angle_count*(2.*std::numbers::pi/8192.);
            have_encoder_=true; previous_encoder_=value.angle_count; motor_temperature_=value.temperature_raw; encoder_time_=receipt.kernel_monotonic_ns; ++encoder_sequence_; bus.last_feedback=encoder_time_;
            std::ostringstream out; out.precision(17);
            out<<"{\"kind\":\"yaw_feedback\",\"encoder_raw\":"<<value.angle_count<<",\"encoder_unwrapped_counts\":"<<encoder_counts_
               <<",\"q_relative_rad\":"<<position()<<",\"speed_rpm\":"<<value.speed_rpm<<",\"current_raw\":"<<value.current_raw
               <<",\"current_A\":"<<value.current_a()<<",\"temperature_raw\":"<<unsigned(value.temperature_raw)
               <<",\"kernel_realtime_ns\":"<<receipt.kernel_realtime_ns<<",\"kernel_monotonic_ns\":"<<encoder_time_
               <<",\"dequeue_ns\":"<<receipt.dequeue_ns<<'}'; record(out.str());
            if (servo_mode_ && control_begin_ && !servo_done_) {
              if (!servo_.observe_encoder(seconds(encoder_time_),position()))
                throw std::runtime_error("MEASUREMENT_LIMITED: servo encoder update rejected (ready="+
                  std::to_string(servo_.ready())+", t="+std::to_string(seconds(encoder_time_))+")");
              if (event_control_) control_servo(monotonic_ns());
            }
          }
          continue;
        }
        if (!receipt.frame.extended || receipt.frame.rtr || receipt.frame.dlc!=8) continue;
        const auto id=cybergear::unpack_ext_id(receipt.frame.id); cybergear::CanFrame frame; frame.id=receipt.frame.id; std::memcpy(frame.data,receipt.frame.data,8);
        if (id.comm_type==0) {
          cybergear::DiscoveryResponse response;
          require(cybergear::parse_discovery_response(frame,response) && id.data2==127 && response.unique_id==uid_,"INTEGRATION_MISMATCH: pitch identity"); identified_=true;
          record("{\"kind\":\"pitch_identity\",\"uid_hex\":"+quoted(uid_text_)+",\"receive_ns\":"+std::to_string(receipt.kernel_monotonic_ns)+"}");
        } else if (id.comm_type==17) {
          const auto observation=readback_.observe(receipt); record(readback_json(observation));
          if (observation.reg==cybergear::Reg::RunMode) { require(observation.value.has_value(),"MEASUREMENT_LIMITED: pitch mode readback unavailable"); have_mode_=true; }
          if (observation.reg==cybergear::Reg::MechPos) {
            require(observation.value && std::isfinite(*observation.value),"MEASUREMENT_LIMITED: pitch posture readback unavailable");
            posture_=*observation.value;have_posture_=true;
          }
        } else if (id.comm_type==2) {
          cybergear::Feedback feedback;
          require(cybergear::parse_feedback(frame,feedback) && feedback.motor_id==127 && feedback.host_id==0,"INTEGRATION_MISMATCH: pitch feedback identity");
          require(!feedback.faults && feedback.temp_c<pitch_temperature_,"HARD_ABORT: pitch fault or temperature");
          const bool disabled=feedback.mode==cybergear::MotorMode::Reset; require(!disabled_ || disabled,"HARD_ABORT: pitch became enabled");
          if (pitch_stop_begin_ && receipt.kernel_monotonic_ns>=pitch_stop_begin_) disabled_=disabled;
          bus.last_feedback=receipt.kernel_monotonic_ns;
        } else if (id.comm_type==21) require(false,"HARD_ABORT: pitch fault response");
      }
    }
    require(!(fds[2].revents&(POLLERR|POLLNVAL)),"DATA_INVALID: IMU pipe failed");
    if (fds[2].revents&(POLLIN|POLLHUP)) {
      char bytes[8192]; const auto count=::read(imu_fd_,bytes,sizeof(bytes));
      if (count<0 && (errno==EAGAIN || errno==EINTR)) return;
      require(count>0,"DATA_INVALID: IMU producer EOF or read failure"); imu_pending_.append(bytes,count);
      size_t end; while ((end=imu_pending_.find('\n'))!=std::string::npos) { imu_line(imu_pending_.substr(0,end)); imu_pending_.erase(0,end+1); }
      require(imu_pending_.size()<3000,"DATA_INVALID: IMU line exceeded capture bound");
    }
  }
  int finish(bool complete,bool sequence,bool zero,std::string detail) {
    try {
      for (const auto& bus:buses_) {
        require(!bus.receiver->kernel_drops(),"DATA_INVALID: final socket receive loss");
        if (!synthetic_) require(bus.counter("rx_dropped")==bus.drops_begin && bus.counter("rx_errors")==bus.errors_begin,"DATA_INVALID: interface receive loss");
      }
    } catch (const std::exception& e) { complete=false; if (detail.empty()) detail=e.what(); }
    const auto footer="{\"kind\":\"footer\",\"status\":"+quoted(complete?"COMPLETE":"INVALID")+",\"detail\":"+quoted(detail)+
      ",\"sequence_complete\":"+(sequence?"true":"false")+",\"zero_request_completed\":"+(zero?"true":"false")+
      ",\"stop_observation_s\":2,\"motion_qualified\":false,\"parameter_qualified\":false,\"shared_core_control_cycles\":"+
      std::to_string(control_cycles_)+",\"yaw_frames\":"+std::to_string(buses_[0].sequence)+",\"pitch_frames\":"+std::to_string(buses_[1].sequence)+
      ",\"end_ns\":"+std::to_string(monotonic_ns())+"}";
    const bool written=journal_->finish(footer); std::cout<<footer<<'\n'; return complete && written?0:1;
  }
  YAML::Node config_; Limits limits_; bool synthetic_; Readback readback_; axis::Controller controller_;
  SelectedFamilyFeedforward family_;
  axis::Servo servo_; bool servo_mode_{},event_control_{},servo_done_{}; double speed_limit_{},hold_after_s_{};
  double motor_temperature_{},motor_temperature_limit_{};
  std::vector<std::array<double,4>> gain_schedule_; std::size_t next_gain_{};
  std::deque<std::pair<int64_t,double>> speed_history_;
  double excitation_amplitude_{},excitation_f0_{},excitation_f1_{},excitation_begin_{},excitation_duration_{};
  std::array<Endpoint,2> buses_; ImuStream imu_; std::unique_ptr<Journal> journal_;
  std::vector<ReferenceSegment> references_; std::vector<ReferenceSample> reference_samples_;
  std::vector<DepartureWindow> departures_;
  std::array<double,3> projection_{},bias_{};
  std::string candidate_,uid_text_,parameter_readback_,imu_pending_;
  uint64_t uid_{},encoder_sequence_{},gyro_sequence_{},read_index_{},control_cycles_{};
  int imu_fd_{},gyro_generation_{-1}; uint16_t previous_encoder_{}; int64_t encoder_counts_{};
  bool identified_{},disabled_{},have_mode_{},have_encoder_{},have_posture_{};
  double baseline_s_{},stop_s_{},current_bound_{},pitch_temperature_{},posture_{},position_offset_{},reference_s_{},initial_position_{},gyro_rate_{},last_shaped_velocity_{};
  int64_t begin_{},discovery_begin_{},pitch_stop_begin_{},control_begin_{},next_control_{},next_stop_{},next_read_{},encoder_time_{},gyro_time_{},tx_begin_{},tx_accepted_{};
  int64_t baseline_current_time_{},braking_begin_{},quiet_since_{};
};
}
int yaw_control_session(const char* path) {
  try { return YawControlSession(YAML::LoadFile(path)).run(); }
  catch (const std::exception& e) { std::cerr<<"{\"status\":\"INVALID\",\"detail\":"<<quoted(e.what())<<"}\n"; return 1; }
}
}
