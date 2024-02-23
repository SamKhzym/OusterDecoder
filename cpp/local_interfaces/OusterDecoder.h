////////////////////////////////////////////////////////////////////////////////
//     This file is part of RTMaps                                            //
//     Copyright (c) Intempora S.A. All rights reserved.                      //
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////
// SDK Programmer samples
////////////////////////////////

#pragma once

// Includes maps sdk library header
#include <maps.hpp>
// Includes the MAPS::InputReader class and its dependencies
#include <maps/input_reader/maps_input_reader.hpp>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <memory>

#include "ouster/client.h"
#include "ouster/impl/build.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"

using namespace ouster;

// Declares a new MAPSComponent child class
class OusterDecoder : public MAPSComponent
{
    // Use standard header definition macro
    MAPS_COMPONENT_STANDARD_HEADER_CODE(OusterDecoder)

    // Place here your specific methods and attributes
    void QueryAndSendData();
private:

    // Init lidar client, get metadata
    std::string hostname;
    std::shared_ptr<sensor::client> handle;

    // Parse sensor metadata, create XYZ LUT
    std::string metadata;
    sensor::sensor_info sensorInfo;
    XYZLut lut;

    int w; int h;
    const sensor::packet_format* pf;
    ScanBatcher* batchToScan;
    
};
