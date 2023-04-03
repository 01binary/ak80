#include <SPI.h>
#include <mcp2515.h>

const float MAX_PWM = 100000.0f;
const float MAX_CURRENT = 60000.0f;
const float MAX_VELOCITY = 100000.0f;
const float MAX_POSITION = 360000000.0f;
const float MAX_POSITION_VELOCITY = float(SHRT_MAX);
const float MIN_POSITION_VELOCITY = float(SHRT_MIN);
const float MAX_ACCELERATION = float(SHRT_MAX);
const float MAX_MIT_POSITION = 12.5f;
const float MAX_MIT_VELOCITY = 8.0f;
const float MAX_MIT_TORQUE = 144.0f;
const float MAX_MIT_KP = 500.0f;
const float MAX_MIT_KD = 5.0f;

enum AKMode {
  AK_PWM,
  AK_EFFORT,
  AK_HOLD,
  AK_VELOCITY,
  AK_POSITION,
  AK_ORIGIN,
  AK_POSITION_VELOCITY,
  AK_RAW,
  AK_UNINITIALIZED
};

enum AKError {
  AK_NONE,
  AK_OVER_TEMPERATURE,
  AK_OVER_CURRENT,
  AK_OVER_VOLTAGE,
  AK_UNDER_VOLTAGE,
  AK_ENCODER,
  AK_PHASE
};

struct AKPwm {
  int32_t dutyCycle;
};

struct AKEffort {
  int32_t effort;
};

struct AKVelocity {
  int32_t velocity;
};

struct AKPosition {
  int32_t position;
};

struct AKPositionVelocityAcceleration {
  int32_t position;
  int16_t velocity;
  int16_t acceleration;
};

struct AKPositionVelocityTorqueKpKd {
  int16_t position: 16;
  int16_t velocity: 12;
  int16_t kp: 12;
  int16_t kd: 12;
  int16_t torque: 12;
};

struct AKStatus {
  int16_t position;
  int16_t velocity;
  int16_t current;
  int8_t temperature;
  uint8_t error;
};

MCP2515 mcp2515(D10);

uint32_t canId(int id, AKMode mode)
{
  return uint32_t(id | mode << 8);
}

void setMode(int id, AKMode mode)
{
  bool switchMode = akMode[id] != mode && (
    akMode[id] == AKMode::AK_UNINITIALIZED ||
    akMode[id] == AKMode::AK_RAW ||
    mode == AKMode::AK_RAW
  );

  if (switchMode)
  {
    can_frame frame;
    frame.can_id = uint32_t(id);
    memset(frame.data, sizeof(frame.data) - 1, 0xFF);
    frame.data[sizeof(frame.data) - 1] = mode == AKMode::AK_RAW
      // Switch to MIT mode
      ? 0xFC
      // Switch to servo mode
      : 0xFE;
    mcp2515.sendMessage(&frame);
  }

  akMode[id] = mode;
}

void setPwm(uint32_t id, float dutyCycle)
{
  setMode(id, AKMode::AK_PWM);

  can_frame frame;
  frame.can_id = canId(id, AKMode::AK_PWM);
  frame.can_dlc = sizeof(frame.data);
  ((AKPwm*)frame.data)->dutyCycle = int32_t(dutyCycle * MAX_PWM);

  mcp2515.sendMessage(&frame);
}

void setEffort(int id, float effort)
{
  setMode(id, AKMode::AK_EFFORT);

  can_frame frame;
  frame.can_id = canId(id, AKMode::AK_EFFORT);
  frame.can_dlc = sizeof(frame.data);
  ((AKEffort*)frame.data)->effort = int32_t(effort * MAX_CURRENT);

  mcp2515.sendMessage(&frame);
}

void setHold(int id, float effort)
{
  setMode(id, AKMode::AK_HOLD);
  
  can_frame frame;
  frame.can_id = canId(id, AKMode::AK_HOLD);
  frame.can_dlc = sizeof(frame.data);
  ((AKEffort*)frame.data)->effort = int32_t(effort * MAX_CURRENT);

  mcp2515.sendMessage(&frame);
}

void setVelocity(int id, float velocity)
{
  setMode(id, AKMode::AK_VELOCITY);
  
  can_frame frame;
  frame.can_id = canId(id, AKMode::AK_VELOCITY);
  frame.can_dlc = sizeof(frame.data);
  ((AKVelocity*)frame.data)->velocity = int32_t(velocity * MAX_VELOCITY);

  mcp2515.sendMessage(&frame);
}

void setPosition(int id, float position)
{
  setMode(id, AKMode::AK_POSITION);

  can_frame frame;
  frame.can_id = canId(id, AKMode::AK_POSITION);
  frame.can_dlc = sizeof(frame.data);
  ((AKPosition*)frame.data)->position = int32_t(position * MAX_POSITION);

  mcp2515.sendMessage(&frame);
}

void setOrigin(int id, float position)
{
  setMode(id, AKMode::AK_ORIGIN);
 
  can_frame frame;
  frame.can_id = canId(id, AKMode::AK_ORIGIN);
  frame.can_dlc = sizeof(frame.data);
  ((AKPosition*)frame.data)->position = int32_t(position * MAX_POSITION);

  mcp2515.sendMessage(&frame);
}

void setPositionVelocityAcceleration(
  int id, float position, float velocity, float acceleration)
{
  setMode(id, AKMode::AK_POSITION_VELOCITY);

  can_frame frame;
  frame.can_id = canId(id, AKMode::AK_POSITION_VELOCITY);
  frame.can_dlc = sizeof(frame.data);

  AKPositionVelocityAcceleration* data =
    (AKPositionVelocityAcceleration*)frame.data;

  data->position = int32_t(position * MAX_POSITION);
  data->velocity = int16_t(velocity < 0
    ? (-velocity) * MIN_POSITION_VELOCITY
    : velocity * MAX_POSITION_VELOCITY);
  data->acceleration = int16_t(acceleration * MAX_ACCELERATION);
 
  mcp2515.sendMessage(&frame);
}

void setPositionVelocityTorqueStiffnessDamping(
  int id, float position, float velocity, float torque, float kp, float kd)
{
  setMode(id, AKMode::AK_RAW);

  can_frame frame;
  frame.can_id = uint32_t(id);
  frame.can_dlc = sizeof(frame.data);

  AKPositionVelocityTorqueKpKd* data = (AKPositionVelocityTorqueKpKd*)frame.data;

  data->position = position * MAX_MIT_POSITION;
  data->velocity = velocity * MAX_MIT_VELOCITY;
  data->torque = torque * MAX_MIT_TORQUE;
  data->kp = kp * MAX_MIT_KP;
  data->kd = kd * MAX_MIT_KD;
 
  mcp2515.sendMessage(&frame);
}

float normalize(int16_t value)
{
  return value < 0
    ? float(value) / float(-SHRT_MIN)
    : float(value) / float(SHRT_MAX);
}

float normalize(uint16_t value)
{
  return float(value) / float(USHRT_MAX);
}

float normalize(uint8_t value)
{
  return float(value) / float(UCHAR_MAX);
}

void writeServo(const str1ker::Servo& msg)
{
  for (uint32_t n = 0; n < msg.channels_length; n++)
  {
    str1ker::ServoChannel& request = msg.channels[n];

    switch (request.mode)
    {
      case str1ker::ServoChannel::MODE_PWM:
        setPwm(request.id, normalize(request.velocity));
        break;
      case str1ker::ServoChannel::MODE_EFFORT:
        setEffort(request.id, normalize(request.effort));
        break;
      case str1ker::ServoChannel::MODE_HOLD:
        setHold(request.id, normalize(request.effort));
        break;
      case str1ker::ServoChannel::MODE_VELOCITY:
        setVelocity(request.id, normalize(request.velocity));
        break;
      case str1ker::ServoChannel::MODE_POSITION:
        setPosition(request.id, normalize(request.position));
        break;
      case str1ker::ServoChannel::MODE_ORIGIN:
        setOrigin(request.id, normalize(request.position));
        break;
      case str1ker::ServoChannel::MODE_POSITION_VELOCITY:
        setPositionVelocityAcceleration(
          request.id,
          normalize(request.position),
          normalize(request.velocity),
          normalize(request.effort)
        );
        break;
      case str1ker::ServoChannel::MODE_RAW:
        setPositionVelocityTorqueStiffnessDamping(
          request.id,
          normalize(request.position),
          normalize(request.velocity),
          normalize(request.effort),
          normalize(request.kp),
          normalize(request.kd)
        );
        break;
    }
  }
}

void setup() {
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
}

void loop() {
}
