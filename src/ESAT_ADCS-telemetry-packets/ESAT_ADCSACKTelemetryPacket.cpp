/*
 * Copyright (C) 2017, 2018 Theia Space, Universidad Politécnica de Madrid
 *
 * This file is part of Theia Space's ESAT ADCS library.
 *
 * Theia Space's ESAT ADCS library is free software: you can
 * redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Theia Space's ESAT ADCS library is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Theia Space's ESAT ADCS library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "ESAT_ADCS-telemetry-packets/ESAT_ADCSACKTelemetryPacket.h"
#include "ESAT_ADCS.h"
#include "ESAT_ADCS-actuators/ESAT_Magnetorquer.h"
#include "ESAT_ADCS-actuators/ESAT_Wheel.h"
#include "ESAT_ADCS-controllers/ESAT_AttitudePIDController.h"
#include "ESAT_ADCS-controllers/ESAT_WheelPIDController.h"
#include "ESAT_ADCS-measurements/ESAT_CoarseSunSensor.h"
#include "ESAT_ADCS-measurements/ESAT_Gyroscope.h"
#include "ESAT_ADCS-measurements/ESAT_Magnetometer.h"
#include "ESAT_ADCS-telecommand-handlers/ESAT_AttitudeTelecommandHandler.h"
#include "ESAT_ADCS-telecommand-handlers/ESAT_ADCSClockTelecommandHandler.h"
#include "ESAT_ADCS-telecommand-handlers/ESAT_DiagnosticsTelecommandHandler.h"
#include "ESAT_ADCS-telecommand-handlers/ESAT_MagnetorquerTelecommandHandler.h"
#include "ESAT_ADCS-telecommand-handlers/ESAT_StopActuatorsTelecommandHandler.h"
#include "ESAT_ADCS-telecommand-handlers/ESAT_WheelTelecommandHandler.h"

byte ESAT_ADCSACKTelemetryPacketClass::packetIdentifier()
{
  return PACKET_IDENTIFIER;
}

boolean ESAT_ADCSACKTelemetryPacketClass::makeAvailable(bool isAvailable)
{
  return activated = isAvailable;
}

boolean ESAT_ADCSACKTelemetryPacketClass::available()
{
  if (activated)
  {
    return true;
  }
  else
  {
    return false;
  }
}

ESAT_CCSDSSecondaryHeader ESAT_ADCSACKTelemetryPacketClass::saveSecondaryHeader(ESAT_CCSDSPacket &packet)
{
  return datum = packet.readSecondaryHeader();
}

boolean ESAT_ADCSACKTelemetryPacketClass::handlerIsCompatibleWithPacket(byte packetIdentifier, ESAT_CCSDSSecondaryHeader datum)
{
  if (packetIdentifier == datum.packetIdentifier)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void ESAT_ADCSACKTelemetryPacketClass::readUserData(ESAT_CCSDSPacket &packet)
{
  byte handlers[] = {
      ESAT_ADCSClockTelecommandHandler.SET_TIME,
      ESAT_AttitudeTelecommandHandler.FOLLOW_MAGNETIC_TARGET,
      ESAT_AttitudeTelecommandHandler.FOLLOW_SOLAR_TARGET,
      ESAT_AttitudeTelecommandHandler.DETUMBLE,
      ESAT_AttitudeTelecommandHandler.ATTITUDE_CONTROLLER_SET_PROPORTIONAL_GAIN,
      ESAT_AttitudeTelecommandHandler.ATTITUDE_CONTROLLER_SET_INTEGRAL_GAIN,
      ESAT_AttitudeTelecommandHandler.ATTITUDE_CONTROLLER_SET_DERIVATIVE_GAIN,
      ESAT_AttitudeTelecommandHandler.ATTITUDE_CONTROLLER_RESET_ERROR_INTEGRAL,
      ESAT_AttitudeTelecommandHandler.ATTITUDE_CONTROLLER_SET_GYROSCOPE_USAGE,
      ESAT_AttitudeTelecommandHandler.ATTITUDE_CONTROLLER_SET_ACTUATORS,
      ESAT_AttitudeTelecommandHandler.ATTITUDE_CONTROLLER_SET_DEADBAND,
      ESAT_AttitudeTelecommandHandler.ATTITUDE_CONTROLLER_SET_DETUMBLING_THRESHOLD,
      ESAT_DiagnosticsTelecommandHandler.DIAGNOSTICS_SET_COARSE_SUN_SENSOR_READINGS_SOURCES,
      ESAT_DiagnosticsTelecommandHandler.DIAGNOSTICS_CHANGE_MAGNETORQUER_AXES_AND_POLARITIES,
      ESAT_DiagnosticsTelecommandHandler.DIAGNOSTICS_CONFIGURE_GYROSCOPE_BIAS_CORRECTION,
      ESAT_DiagnosticsTelecommandHandler.DIAGNOSTICS_CONFIGURE_MAGNETOMETER_GEOMETRY_CORRECTION,
      ESAT_DiagnosticsTelecommandHandler.DIAGNOSTICS_RECONFIGURE_GYROSCOPE,
      ESAT_MagnetorquerTelecommandHandler.MAGNETORQUER_ENABLE,
      ESAT_MagnetorquerTelecommandHandler.MAGNETORQUER_SET_X_POLARITY,
      ESAT_MagnetorquerTelecommandHandler.MAGNETORQUER_SET_Y_POLARITY,
      ESAT_MagnetorquerTelecommandHandler.MAGNETORQUER_APPLY_MAXIMUM_TORQUE,
      ESAT_MagnetorquerTelecommandHandler.MAGNETORQUER_DEMAGNETIZE,
      ESAT_StopActuatorsTelecommandHandler.STOP_ACTUATORS,
      ESAT_WheelTelecommandHandler.WHEEL_SET_DUTY_CYCLE,
      ESAT_WheelTelecommandHandler.WHEEL_SET_SPEED,
      ESAT_WheelTelecommandHandler.WHEEL_CONTROLLER_SET_PROPORTIONAL_GAIN,
      ESAT_WheelTelecommandHandler.WHEEL_CONTROLLER_SET_INTEGRAL_GAIN,
      ESAT_WheelTelecommandHandler.WHEEL_CONTROLLER_SET_DERIVATIVE_GAIN,
      ESAT_WheelTelecommandHandler.WHEEL_CONTROLLER_RESET_ERROR_INTEGRAL,
      ESAT_WheelTelecommandHandler.WHEEL_RESET_ELECTRONIC_SPEED_CONTROLLER,
  };

  for (int i = 0; i < sizeof(handlers); i++)
  {
    if (handlerIsCompatibleWithPacket(handlers[i], datum))
    {
      packet.writeByte(handlers[i]);
    }
  }
  activated = false;
}

ESAT_ADCSACKTelemetryPacketClass ESAT_ADCSACKTelemetryPacket;
