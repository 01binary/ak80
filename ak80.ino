#include <SPI.h>
#include <mcp2515.h>

const float MAX_PWM = 100000.0;
const float MAX_CURRENT = 60000.0;
const float MAX_VELOCITY = 100000.0;
const float MAX_POSITION = 360000000.0;
const float MAX_POSITION_VELOCITY = 32767.0;
const float MAX_ACCELERATION = 32767.0;
const float MAX_MIT_POSITION = 12.5f;
const float MAX_MIT_VELOCITY = 8.0f;
const float MAX_MIT_TORQUE  = 144.0f;
const float MAX_MIT_KP = 500.0f;
const float MAX_MIT_KD = 5.0f;

enum AKModes {
  PWM,
  EFFORT,
  BRAKE,
  VELOCITY,
  POSITION,
  ORIGIN,
  POSITION_VELOCITY
};

enum AKError {
  NONE,
  OVER_TEMPERATURE,
  OVER_CURRENT,
  OVER_VOLTAGE,
  UNDER_VOLTAGE,
  ENCODER,
  PHASE,
};

struct AKDuty {
  int32_t dutyCycle;
};

struct AKCurrent {
  int32_t current;
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

struct AKPositionVelocityStiffnessDamping {
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

MCP2515 mcp2515(10);

uint32_t motorMode(int motorId, AKModes mode)
{
  return uint32_t(motorId | mode << 8);
}

void setDutyCycle(uint32_t motorId, float dutyCycle)
{
  can_frame frame;
  frame.can_id = motorMode(motorId, AKModes::PWM);
  frame.can_dlc = sizeof(frame.data);
  ((AKDuty*)frame.data)->dutyCycle = int32_t(dutyCycle * MAX_PWM);
  mcp2515.sendMessage(&frame);
}

void setEffort(int motorId, float effort)
{
  can_frame frame;
  frame.can_id = motorMode(motorId, AKModes::EFFORT);
  frame.can_dlc = sizeof(frame.data);
  ((AKCurrent*)frame.data)->current = int32_t(effort * MAX_CURRENT);
}

void setBrake(int motorId, float effort)
{
  can_frame frame;
  frame.can_id = motorMode(motorId, AKModes::BRAKE);
  frame.can_dlc = sizeof(frame.data);
  ((AKCurrent*)frame.data)->current = int32_t(effort * MAX_CURRENT);
}

void setVelocity(int motorId, float velocity)
{
  can_frame frame;
  frame.can_id = motorMode(motorId, AKModes::VELOCITY);
  frame.can_dlc = sizeof(frame.data);
  ((AKVelocity*)frame.data)->velocity = int32_t(velocity * MAX_VELOCITY);
  mcp2515.sendMessage(&frame);
}

void setPosition(int motorId, float position)
{
  can_frame frame;
  frame.can_id = motorMode(motorId, AKModes::POSITION);
  frame.can_dlc = sizeof(frame.data);
  ((AKPosition*)frame.data)->position = int32_t(position * MAX_POSITION);
  mcp2515.sendMessage(&frame);
}

void setOrigin(int motorId, float position)
{
  can_frame frame;
  frame.can_id = motorMode(motorId, AKModes::ORIGIN);
  frame.can_dlc = sizeof(frame.data);
  ((AKPosition*)frame.data)->position = int32_t(position * MAX_POSITION);
  mcp2515.sendMessage(&frame);
}

void setPositionVelocityAcceleration(int motorId, float position, float velocity, float acceleration)
{
  can_frame frame;
  frame.can_id = motorMode(motorId, AKModes::POSITION_VELOCITY);
  frame.can_dlc = sizeof(frame.data);

  AKPositionVelocityAcceleration* data =
    (AKPositionVelocityAcceleration*)frame.data;

  data->position = int32_t(position * MAX_POSITION);
  data->velocity = int16_t(velocity * MAX_POSITION_VELOCITY);
  data->acceleration = int16_t(acceleration * MAX_ACCELERATION);
 
  mcp2515.sendMessage(&frame);
}

AKStatus* read(int motorId)
{
  static can_frame frame;

  if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    if (frame.can_id != motorId) return NULL;
    return (AKStatus*)frame.data;
  }

  return NULL;
}

void setMode(int motorId, bool mitModeOn)
{
  can_frame frame;
  frame.can_id = uint32_t(motorId);
  memset(frame.data, sizeof(frame.data) - 1, 0xFF);
  frame.data[sizeof(frame.data) - 1] = mitModeOn ? 0xFC : 0xFE;
  mcp2515.sendMessage(&frame);
}

void setPositionVelocityTorqueStiffnessDamping(int motorId, float position, float velocity, float torque, float kp, float kd)
{
  can_frame frame;
  frame.can_id = uint32_t(motorId);
  frame.can_dlc = sizeof(frame.data);

  AKPositionVelocityStiffnessDamping* data =
    (AKPositionVelocityStiffnessDamping*)frame.data;

  data->position = position * MAX_MIT_POSITION;
  data->velocity = velocity * MAX_MIT_VELOCITY;
  data->kp = kp * MAX_MIT_KP;
  data->kd = kd * MAX_MIT_KD;
  data->torque = torque * MAX_MIT_TORQUE;
 
  mcp2515.sendMessage(&frame);
}

void setup() {
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
}

void loop() {
  // put your main code here, to run repeatedly:

}
