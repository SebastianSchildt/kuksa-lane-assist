/********************************************************************************
 * Copyright (c) 2022 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Apache License 2.0 which is available at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * SPDX-License-Identifier: Apache-2.0
 ********************************************************************************/
#include <ostream>
#include <stdio.h>
#include <unistd.h>
#include <cstdint>

#include <kuksa/val/v2/val.pb.h>
#include <kuksa/val/v2/types.pb.h>
#include <kuksaclient/kuksaclient.h>
#include <vector>

using namespace kuksa;

uint32_t steeringWheelAngle = 0;
uint32_t steeringWheelTargetAngle = 0;
double errSum = 0;
double lastError = 0;

double kp = 0.5;
double ki = 0.01;
double kd = 0;

KuksaClient *kuksaconnection=NULL;

void handleValue(const kuksa::val::v2::Value &value) {
  switch (value.typed_value_case()) {
  case kuksa::val::v2::Value::kString:
    std::cout << "String value: " << value.string() << std::endl;
    break;
  case kuksa::val::v2::Value::kBool:
    std::cout << "Bool value: " << value.bool_() << std::endl;
    break;
  case kuksa::val::v2::Value::kInt32:
    std::cout << "Int32 value: " << value.int32() << std::endl;
    break;
  case kuksa::val::v2::Value::kUint32:
    std::cout << "Uint32 value: " << value.uint32() << std::endl;
    break;

  case kuksa::val::v2::Value::kFloat:
    std::cout << "Float value: " << value.float_() << std::endl;
    break;
  case kuksa::val::v2::Value::kDouble:
    std::cout << "Double value: " << value.double_() << std::endl;
    break;
  // Handle initial callback on subscription confirmation
  // No value is set by the broker
  case kuksa::val::v2::Value::TYPED_VALUE_NOT_SET:
    break;
  default:
    std::cout << "Unsupported value type: " << value.typed_value_case()
              << std::endl;
    break;
  }
}

void on_data_reception_v2(const std::string &path,
                          const kuksa::val::v2::Value &value) {
  std::cout << "Subscription callback invoked on VSS point " << path
            << std::endl;

  handleValue(value);
}

void on_steeringwheel_current_update(const std::string &path, const kuksa::val::v2::Value &value) {
  if ( value.typed_value_case() == kuksa::val::v2::Value::kInt32 ) {
    steeringWheelAngle = value.int32();
    std::cout << "Vehicle.Chassis.SteeringWheel.Angle: " << steeringWheelAngle << std::endl;
  }
  else {
    std::cout << " Invalid update for Vehicle.Chassis.SteeringWheel.Angle: "<< std::endl;
  }
  double elapsedTime = 0.01; // just assume this is the time elapsed between two updates
  std::cout << "Steering Wheel Angle: " << steeringWheelAngle << " Target Angle: " << steeringWheelTargetAngle << std::endl;
  double error = (double)steeringWheelTargetAngle - (double)steeringWheelAngle;
  errSum += (error * elapsedTime);
  double diffErr = (error - lastError) / elapsedTime;

  std :: cout << "Error: " << error << " errSum: " << errSum << " diffErr: " << diffErr << std::endl;
  //no time, d is weird
  //double torque = kp * error + ki * errSum + kd * diffErr;
  double torque = kp * error + ki * errSum;

  if (torque > 5) {
    std::cout << "Limit Steering Torque from " << torque << " to 5 " << std::endl;
    torque = 5;
  }
  else if (torque < -5) {
    std::cout << "Limit Steering Torque from " << torque << " to -5 " << std::endl;
    torque = -5;
  }

  std::cout << "Publish  Vehicle.ADAS.LaneAssist.SteeringTorque=" << torque << std::endl;
  
  kuksa::val::v2::Value newtorque{};
  newtorque.set_int32((int32_t)torque);
  kuksaconnection->publishValue("Vehicle.ADAS.LaneAssist.SteeringTorque", newtorque);

  lastError = error;
//  Vehicle.ADAS.LaneAssist.SteeringTorque
}


void on_steeringwheel_target_update(const std::string &path, const kuksa::val::v2::Value &value) {
  if ( value.typed_value_case() == kuksa::val::v2::Value::kInt32 ) {
    steeringWheelTargetAngle = value.int32();
    std::cout << "Vehicle.ADAS.LaneAssist.TargetSteeringWheelAngle: " << steeringWheelTargetAngle << std::endl;
  }
  else {
    std::cout << " Invalid update for Vehicle.ADAS.LaneAssist.TargetSteeringWheelAngle "<< std::endl;
  }
}

void on_data_reception_v1(const std::string &path,
                          const kuksa::val::v1::Datapoint &value) {
  std::cout << "Received " << path << std::endl;
}

int main() {
  std::cout << "Starting lane-assist example for v2 ..." << std::endl;
  KuksaClient instance;

  // Connect to the databroker
  bool connectionStatus = instance.connect_v2("127.0.0.1:55555");
  printf("Connection is %s \n",
         (connectionStatus == true) ? "Succesful" : "Failed");

  kuksaconnection = &instance;
  sleep(1);

  // Get info of the databroker server
  kuksa::val::v2::GetServerInfoResponse serverInfo{};
  if (instance.getServerInfo(serverInfo)) {
    std::cout << "Server Name: " << serverInfo.name() << std::endl;
    std::cout << "Version    : " << serverInfo.version() << std::endl;
    std::cout << "Commit Hash: " << serverInfo.commit_hash() << std::endl;
  }

  // Publish Vehicle.Speed signal
/*  kuksa::val::v2::Value value{};
  value.set_float_(52.47f);
  instance.publishValue("Vehicle.Speed", value);
  */

  /*
  // Read back the value
  if (instance.getValue("Vehicle.Speed", value)) {
    handleValue(value);
  }
    */

    /*

  kuksa::val::v2::Value value_1{};
  value_1.set_uint32(73);
  instance.publishValue("Vehicle.Chassis.Accelerator.PedalPosition", value_1);
*/
  std::vector<kuksa::val::v2::Datapoint> datapoints;
  /*
  std::vector<std::string> signals_to_publish = {
      "Vehicle.Speed", "Vehicle.Chassis.Accelerator.PedalPosition"};

  datapoints = instance.getValues(signals_to_publish);

  if (!datapoints.empty()) {
    for (const auto &datapoint : datapoints) {
      handleValue(datapoint.value());
    }
  }
*/
  sleep(1);

  
  instance.subscribe({"Vehicle.Chassis.SteeringWheel.Angle"}, on_steeringwheel_current_update);
  instance.subscribe({"Vehicle.ADAS.LaneAssist.TargetSteeringWheelAngle"}, on_steeringwheel_target_update);

  // Actuate via signal
//  value.set_bool_(true);
  // This will fail in the absence of a provider
//  instance.actuate("Vehicle.Body.Trunk.Rear.IsOpen", value);
  while (true) {
    sleep(1);
  }

  return 0;
}
