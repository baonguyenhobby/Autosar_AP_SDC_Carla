// av_ap/av_services.hpp — the av-stack ara::com SERVICE INTERFACES.
//
// Grouped <Service>Skeleton / <Service>Proxy classes, one per interface, each exposing
// typed events. These mirror what an ARXML service-interface code generator produces and
// follow the standard AUTOSAR AP pattern (see Adaptive-AUTOSAR
// user_apps/.../vehicle_status/{skeleton,proxy}.h): the classes derive from
// ara::com::ServiceSkeletonBase / ServiceProxyBase and hold ara::com::SkeletonEvent<T> /
// ProxyEvent<T> members built from a transport binding (SOME/IP / vsomeip here).
//
//   -DAP_HAVE_ARA  -> real ara::com (Adaptive-AUTOSAR), SOME/IP binding.
//   (host default) -> the in-process ara::com shim in ara_com.hpp (Path A).
//
// Payload (event) types are the av-stack's own plain structs, (de)serialized by
// ara::com::Serializer<T>: the trivially-copyable ones use the default memcpy serializer;
// the three that hold a std::vector get a small Serializer specialization below.
#ifndef AV_AP_AV_SERVICES_HPP
#define AV_AP_AV_SERVICES_HPP

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "av_ap/ara_com.hpp"
#include "av_common/types.hpp"

namespace av {
namespace services {

// ---- payload (event) types (plain structs, member access) ----
struct ImuSample        { double stamp{0}, yaw_rate{0}, yaw{0}; };
struct GpsSample        { double stamp{0}, x{0}, y{0}, yaw{0}; };
struct SpeedSample      { double stamp{0}, v{0}; };
struct LidarSample      { double stamp{0}; av::PointCloud points; };
struct ObjectsSample    { double stamp{0}; av::DetectedObjectArray objects; };
struct TrajectorySample { double stamp{0}; av::Trajectory points; };
using  VehicleStateSample = av::VehicleState;
using  ControlSample      = av::ControlCommand;

}  // namespace services
}  // namespace av

#if defined(AP_HAVE_ARA)
// ===========================================================================================
//  TARGET: real Adaptive-AUTOSAR ara::com
// ===========================================================================================

// --- Serializer<> for the three composite payloads (scalar prefix + a std::vector<T>). The
//     element types (Point3, DetectedObject, TrajectoryPoint) are trivially copyable, so the
//     framework's Serializer<std::vector<T>> bulk-copies them. ---
namespace ara {
namespace com {

template <>
struct Serializer<av::services::LidarSample, void> {
  static std::vector<std::uint8_t> Serialize(const av::services::LidarSample& v) {
    std::vector<std::uint8_t> buf(sizeof(double));
    std::memcpy(buf.data(), &v.stamp, sizeof(double));
    auto pts = Serializer<av::PointCloud>::Serialize(v.points);
    buf.insert(buf.end(), pts.begin(), pts.end());
    return buf;
  }
  static core::Result<av::services::LidarSample> Deserialize(const std::uint8_t* d, std::size_t n) {
    if (n < sizeof(double))
      return core::Result<av::services::LidarSample>::FromError(MakeErrorCode(ComErrc::kFieldValueIsNotValid));
    av::services::LidarSample o;
    std::memcpy(&o.stamp, d, sizeof(double));
    auto r = Serializer<av::PointCloud>::Deserialize(d + sizeof(double), n - sizeof(double));
    if (!r.HasValue()) return core::Result<av::services::LidarSample>::FromError(r.Error());
    o.points = std::move(r).Value();
    return core::Result<av::services::LidarSample>::FromValue(std::move(o));
  }
};

template <>
struct Serializer<av::services::ObjectsSample, void> {
  static std::vector<std::uint8_t> Serialize(const av::services::ObjectsSample& v) {
    std::vector<std::uint8_t> buf(sizeof(double));
    std::memcpy(buf.data(), &v.stamp, sizeof(double));
    auto o = Serializer<av::DetectedObjectArray>::Serialize(v.objects);
    buf.insert(buf.end(), o.begin(), o.end());
    return buf;
  }
  static core::Result<av::services::ObjectsSample> Deserialize(const std::uint8_t* d, std::size_t n) {
    if (n < sizeof(double))
      return core::Result<av::services::ObjectsSample>::FromError(MakeErrorCode(ComErrc::kFieldValueIsNotValid));
    av::services::ObjectsSample o;
    std::memcpy(&o.stamp, d, sizeof(double));
    auto r = Serializer<av::DetectedObjectArray>::Deserialize(d + sizeof(double), n - sizeof(double));
    if (!r.HasValue()) return core::Result<av::services::ObjectsSample>::FromError(r.Error());
    o.objects = std::move(r).Value();
    return core::Result<av::services::ObjectsSample>::FromValue(std::move(o));
  }
};

template <>
struct Serializer<av::services::TrajectorySample, void> {
  static std::vector<std::uint8_t> Serialize(const av::services::TrajectorySample& v) {
    std::vector<std::uint8_t> buf(sizeof(double));
    std::memcpy(buf.data(), &v.stamp, sizeof(double));
    auto p = Serializer<av::Trajectory>::Serialize(v.points);
    buf.insert(buf.end(), p.begin(), p.end());
    return buf;
  }
  static core::Result<av::services::TrajectorySample> Deserialize(const std::uint8_t* d, std::size_t n) {
    if (n < sizeof(double))
      return core::Result<av::services::TrajectorySample>::FromError(MakeErrorCode(ComErrc::kFieldValueIsNotValid));
    av::services::TrajectorySample o;
    std::memcpy(&o.stamp, d, sizeof(double));
    auto r = Serializer<av::Trajectory>::Deserialize(d + sizeof(double), n - sizeof(double));
    if (!r.HasValue()) return core::Result<av::services::TrajectorySample>::FromError(r.Error());
    o.points = std::move(r).Value();
    return core::Result<av::services::TrajectorySample>::FromValue(std::move(o));
  }
};

}  // namespace com
}  // namespace ara

namespace av {
namespace services {

// Service / instance / event IDs — aligned with config/manifests/av_service_instances.arxml.
namespace ids {
constexpr std::uint8_t  kMajor = 1U;
constexpr std::uint16_t kGrp   = 0x0001U;
constexpr std::uint16_t kSensorSvc = 0x1001U, kSensorInst = 0x0001U;
constexpr std::uint16_t kEvImu = 0x8001U, kEvGps = 0x8002U, kEvSpeed = 0x8003U, kEvLidar = 0x8004U;
constexpr std::uint16_t kVehStateSvc = 0x1002U, kVehStateInst = 0x0001U, kEvState = 0x8001U;
constexpr std::uint16_t kObjectsSvc  = 0x1003U, kObjectsInst  = 0x0001U, kEvObjects = 0x8001U;
constexpr std::uint16_t kTrajSvc     = 0x1004U, kTrajInst     = 0x0001U, kEvTraj = 0x8001U;
constexpr std::uint16_t kControlSvc  = 0x1005U, kControlInst  = 0x0001U, kEvCmd = 0x8001U;
}  // namespace ids

inline std::unique_ptr<ara::com::internal::SkeletonEventBinding>
MakeSkelBinding(std::uint16_t svc, std::uint16_t inst, std::uint16_t ev) {
  return ara::com::internal::BindingFactory::CreateSkeletonEventBinding(
      ara::com::internal::TransportBinding::kVsomeip,
      ara::com::internal::EventBindingConfig{svc, inst, ev, ids::kGrp, ids::kMajor});
}
inline std::unique_ptr<ara::com::internal::ProxyEventBinding>
MakeProxyBinding(const ara::com::ServiceHandleType& h, std::uint16_t ev) {
  return ara::com::internal::BindingFactory::CreateProxyEventBinding(
      ara::com::internal::TransportBinding::kVsomeip,
      ara::com::internal::EventBindingConfig{h.GetServiceId(), h.GetInstanceId(), ev, ids::kGrp, ids::kMajor});
}

// ---- SensorService : imu, gps, speed, lidar ----
class SensorServiceSkeleton : public ara::com::ServiceSkeletonBase {
 public:
  ara::com::SkeletonEvent<ImuSample>   imu;
  ara::com::SkeletonEvent<GpsSample>   gps;
  ara::com::SkeletonEvent<SpeedSample> speed;
  ara::com::SkeletonEvent<LidarSample> lidar;
  explicit SensorServiceSkeleton(const std::string& inst)
      : ara::com::ServiceSkeletonBase(ara::core::InstanceSpecifier(inst + "/SensorService"),
            ids::kSensorSvc, ids::kSensorInst, ids::kMajor, 0U,
            ara::com::MethodCallProcessingMode::kEvent),
        imu(MakeSkelBinding(ids::kSensorSvc, ids::kSensorInst, ids::kEvImu)),
        gps(MakeSkelBinding(ids::kSensorSvc, ids::kSensorInst, ids::kEvGps)),
        speed(MakeSkelBinding(ids::kSensorSvc, ids::kSensorInst, ids::kEvSpeed)),
        lidar(MakeSkelBinding(ids::kSensorSvc, ids::kSensorInst, ids::kEvLidar)) {}
  void OfferService() {
    ara::com::ServiceSkeletonBase::OfferService();
    imu.Offer(); gps.Offer(); speed.Offer(); lidar.Offer();
  }
  void StopOfferService() {
    imu.StopOffer(); gps.StopOffer(); speed.StopOffer(); lidar.StopOffer();
    ara::com::ServiceSkeletonBase::StopOfferService();
  }
};
class SensorServiceProxy : public ara::com::ServiceProxyBase {
 public:
  using HandleType = ara::com::ServiceHandleType;
  ara::com::ProxyEvent<ImuSample>   imu;
  ara::com::ProxyEvent<GpsSample>   gps;
  ara::com::ProxyEvent<SpeedSample> speed;
  ara::com::ProxyEvent<LidarSample> lidar;
  explicit SensorServiceProxy(HandleType h)
      : ara::com::ServiceProxyBase(h),
        imu(MakeProxyBinding(h, ids::kEvImu)),
        gps(MakeProxyBinding(h, ids::kEvGps)),
        speed(MakeProxyBinding(h, ids::kEvSpeed)),
        lidar(MakeProxyBinding(h, ids::kEvLidar)) {}
  static ara::com::ServiceHandleContainer<HandleType> FindService(const std::string&) {
    auto r = ara::com::ServiceProxyBase::FindService(ids::kSensorSvc, ids::kSensorInst);
    return r.HasValue() ? std::move(r).Value() : ara::com::ServiceHandleContainer<HandleType>{};
  }
};

// single-event services (VehicleState, Objects, Trajectory, Control)
#define AV_DEFINE_ARA_SERVICE(NAME, EVT, TYPE, SVC, INST, EV)                                   \
  class NAME##Skeleton : public ara::com::ServiceSkeletonBase {                                 \
   public:                                                                                      \
    ara::com::SkeletonEvent<TYPE> EVT;                                                          \
    explicit NAME##Skeleton(const std::string& inst)                                            \
        : ara::com::ServiceSkeletonBase(ara::core::InstanceSpecifier(inst + "/" #NAME),         \
              SVC, INST, ids::kMajor, 0U, ara::com::MethodCallProcessingMode::kEvent),          \
          EVT(MakeSkelBinding(SVC, INST, EV)) {}                                                \
    void OfferService() { ara::com::ServiceSkeletonBase::OfferService(); EVT.Offer(); }         \
    void StopOfferService() { EVT.StopOffer(); ara::com::ServiceSkeletonBase::StopOfferService(); } \
  };                                                                                            \
  class NAME##Proxy : public ara::com::ServiceProxyBase {                                       \
   public:                                                                                      \
    using HandleType = ara::com::ServiceHandleType;                                             \
    ara::com::ProxyEvent<TYPE> EVT;                                                             \
    explicit NAME##Proxy(HandleType h)                                                          \
        : ara::com::ServiceProxyBase(h), EVT(MakeProxyBinding(h, EV)) {}                        \
    static ara::com::ServiceHandleContainer<HandleType> FindService(const std::string&) {       \
      auto r = ara::com::ServiceProxyBase::FindService(SVC, INST);                              \
      return r.HasValue() ? std::move(r).Value()                                                \
                          : ara::com::ServiceHandleContainer<HandleType>{};                     \
    }                                                                                           \
  };

AV_DEFINE_ARA_SERVICE(VehicleStateService, state,      VehicleStateSample, ids::kVehStateSvc, ids::kVehStateInst, ids::kEvState)
AV_DEFINE_ARA_SERVICE(ObjectsService,      objects,    ObjectsSample,      ids::kObjectsSvc,  ids::kObjectsInst,  ids::kEvObjects)
AV_DEFINE_ARA_SERVICE(TrajectoryService,   trajectory, TrajectorySample,   ids::kTrajSvc,     ids::kTrajInst,     ids::kEvTraj)
AV_DEFINE_ARA_SERVICE(ControlService,      command,    ControlSample,      ids::kControlSvc,  ids::kControlInst,  ids::kEvCmd)

#undef AV_DEFINE_ARA_SERVICE

}  // namespace services
}  // namespace av

#else  // ============================= HOST SHIM (Path A) ====================================
namespace av {
namespace services {

using ara::com::InstanceIdentifier;
using ara::com::ProxyEvent;
using ara::com::ServiceHandleContainer;
using ara::com::SkeletonEvent;

inline std::string svc_key(const InstanceIdentifier& i, const char* s) { return i + "/" + s; }

// ---- SensorService : imu, gps, speed, lidar (provided by the gateway) ----
class SensorServiceSkeleton {
 public:
  explicit SensorServiceSkeleton(const InstanceIdentifier& i)
      : imu(svc_key(i, "SensorService/imu")), gps(svc_key(i, "SensorService/gps")),
        speed(svc_key(i, "SensorService/speed")), lidar(svc_key(i, "SensorService/lidar")) {}
  void OfferService() { imu.Offer(); gps.Offer(); speed.Offer(); lidar.Offer(); }
  void StopOfferService() { imu.StopOffer(); gps.StopOffer(); speed.StopOffer(); lidar.StopOffer(); }
  SkeletonEvent<ImuSample> imu; SkeletonEvent<GpsSample> gps;
  SkeletonEvent<SpeedSample> speed; SkeletonEvent<LidarSample> lidar;
};
class SensorServiceProxy {
 public:
  using HandleType = InstanceIdentifier;
  static ServiceHandleContainer<HandleType> FindService(const InstanceIdentifier& i) { return {i}; }
  explicit SensorServiceProxy(const HandleType& i)
      : imu(svc_key(i, "SensorService/imu")), gps(svc_key(i, "SensorService/gps")),
        speed(svc_key(i, "SensorService/speed")), lidar(svc_key(i, "SensorService/lidar")) {}
  ProxyEvent<ImuSample> imu; ProxyEvent<GpsSample> gps;
  ProxyEvent<SpeedSample> speed; ProxyEvent<LidarSample> lidar;
};

// single-event service definition
#define AV_DEFINE_SERVICE(NAME, EVT, TYPE)                                                  \
  class NAME##Skeleton {                                                                     \
   public:                                                                                   \
    explicit NAME##Skeleton(const InstanceIdentifier& i) : EVT(svc_key(i, #NAME "/" #EVT)) {}\
    void OfferService() { EVT.Offer(); }                                                     \
    void StopOfferService() { EVT.StopOffer(); }                                             \
    SkeletonEvent<TYPE> EVT;                                                                 \
  };                                                                                          \
  class NAME##Proxy {                                                                         \
   public:                                                                                    \
    using HandleType = InstanceIdentifier;                                                    \
    static ServiceHandleContainer<HandleType> FindService(const InstanceIdentifier& i) {      \
      return {i};                                                                             \
    }                                                                                         \
    explicit NAME##Proxy(const HandleType& i) : EVT(svc_key(i, #NAME "/" #EVT)) {}            \
    ProxyEvent<TYPE> EVT;                                                                      \
  };

AV_DEFINE_SERVICE(VehicleStateService, state,      VehicleStateSample)
AV_DEFINE_SERVICE(ObjectsService,      objects,    ObjectsSample)
AV_DEFINE_SERVICE(TrajectoryService,   trajectory, TrajectorySample)
AV_DEFINE_SERVICE(ControlService,      command,    ControlSample)

#undef AV_DEFINE_SERVICE

}  // namespace services
}  // namespace av
#endif  // AP_HAVE_ARA

#endif  // AV_AP_AV_SERVICES_HPP
