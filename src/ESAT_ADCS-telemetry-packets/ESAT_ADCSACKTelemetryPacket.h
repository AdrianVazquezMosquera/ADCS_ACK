/*
 * Copyright (C) 2017 Theia Space, Universidad Politécnica de Madrid
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

#ifndef ESAT_ADCSACKTelemetry_h
#define ESAT_ADCSACKTelemetry_h

#include <Arduino.h>
#include <ESAT_CCSDSPacket.h>
#include "ESAT_ADCS-telemetry-packets/ESAT_ADCSTelemetryPacket.h"
#include "ESAT_ADCS-measurements/ESAT_AttitudeStateVector.h"

// ADCS Acknowledgement system packet.
// Use the public instance ESAT_ADCSACKTelemetryPacket.
class ESAT_ADCSACKTelemetryPacketClass : public ESAT_ADCSTelemetryPacket
{
  public:
    // Return true when a new telemetry packet is available; otherwise
    // return false.
    boolean available();
    boolean makeAvailable(bool isAvailable);
    bool activated = false;

    // Return the ADCS ACK telemetry packet identifier.
    byte packetIdentifier();

    ESAT_CCSDSSecondaryHeader saveSecondaryHeader(ESAT_CCSDSPacket &packet);
    ESAT_CCSDSSecondaryHeader datum;

    boolean handlerIsCompatibleWithPacket(byte packetIdentifier,
                                        ESAT_CCSDSSecondaryHeader secondaryHeader);

    // Fill a packet with ADCS ACK telemetry.
    void readUserData(ESAT_CCSDSPacket &packet);

  private:
    // Packet identifier of ADCS ACK telemetry.
    static const byte PACKET_IDENTIFIER = 0x01;
};

// Public instance of the ADCS ACK telemetry packet library.
extern ESAT_ADCSACKTelemetryPacketClass ESAT_ADCSACKTelemetryPacket;

#endif /* ESAT_ADCSACKTelemetryPacket_h */
