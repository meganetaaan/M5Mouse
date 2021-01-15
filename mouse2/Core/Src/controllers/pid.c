#define M5DEFAULT_PGAIN 0.1f
#define M5DEFAULT_IGAIN 0.1f
#define M5DEFAULT_DGAIN 0.1f
typedef int m5Value;

struct m5PIDGainRecord {
  float P;
  float I;
  float D;
};
typedef m5PIDGainRecord m5PIDGainRecord;
typedef m5PIDGainRecord *m5PIDGain;

struct m5PIDContextRecord {
  int initialized;
  m5Value last_value;
  m5Value integrated_value;
};
typedef m5PIDContextRecord m5PIDContextRecord;
typedef m5PIDContextRecord *m5PIDContext;

struct m5PIDControllerRecord {
  m5PIDGain gain;
  m5PIDContext context;
};
typedef m5PIDControllerRecord m5PIDControllerRecord;
typedef m5PIDControllerRecord *m5PIDController;

m5PIDController m5PIDCOntroller() {
  m5PIDGainRecord gain = {
    M5DEFAULT_PGAIN,
    M5DEFAULT_IGAIN,
    M5DEFAULT_DGAIN
  };
  m5PIDContextRecord ctx = {
    0,
    (m5Value)0,
    (m5Value)0
  };
  m5PIDControllerRecord record = {
    &gain,
    &ctx
  };
  return &record;
}