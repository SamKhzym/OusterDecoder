#include "OusterDecoder.h"    // Includes the header of this component

// COMPONENT CONSTANTS
#define VERSION_NUMBER "1.0.1"
#define COMPONENT_NAME OusterDecoder

#define NUM_INPUTS 0

#define NUM_OUTPUTS 1
#define OUTPUT_0 "pointCloud"
#define OUTPUT_0_SIZE 5000000 // up to 2048 points, each with 4 floats (x, y, z, reflectivity)

#define NUM_PROPERTIES 1
#define PROPERTY_0 "hostname"

//MAPS COMPONENT DEFINITION MACROS
MAPS_BEGIN_INPUTS_DEFINITION(COMPONENT_NAME)
MAPS_END_INPUTS_DEFINITION

MAPS_BEGIN_OUTPUTS_DEFINITION(COMPONENT_NAME)
    MAPS_OUTPUT(OUTPUT_0,MAPS::Float32,nullptr,nullptr,OUTPUT_0_SIZE)
MAPS_END_OUTPUTS_DEFINITION

MAPS_BEGIN_PROPERTIES_DEFINITION(COMPONENT_NAME)
    MAPS_PROPERTY(PROPERTY_0,"os-122308000453.local",false,false)
MAPS_END_PROPERTIES_DEFINITION

// No action here.
MAPS_BEGIN_ACTIONS_DEFINITION(COMPONENT_NAME)
MAPS_END_ACTIONS_DEFINITION

MAPS_COMPONENT_DEFINITION(COMPONENT_NAME,"OusterDecoder",VERSION_NUMBER,128,
    MAPS::Threaded,MAPS::Threaded,
    NUM_INPUTS,
    NUM_OUTPUTS,
    NUM_PROPERTIES,
    0
)

void WriteFloat32(
    MAPS::OutputGuard<MAPSFloat32>* guard,
    std::vector<float>* floatVec
)
{

    // set each byte to output of guard
    for (int i = 0; i < floatVec->size(); i++)
    {
        guard->Data(i) = floatVec->at(i);
    }

    guard->VectorSize() = floatVec->size();

}

void OusterDecoder::Birth()
{

    // Init lidar client, get metadata
    this->hostname = static_cast<std::string>(GetStringProperty(PROPERTY_0));
    this->handle = sensor::init_client(this->hostname, "");

    if (!this->handle) { ReportInfo("UNABLE TO CONNECT TO LIDAR"); }

    // Parse sensor metadata, create XYZ LUT
    this->metadata = sensor::get_metadata(*this->handle, 10, false);
    this->sensorInfo = sensor::parse_metadata(this->metadata);
    this->lut = ouster::make_xyz_lut(this->sensorInfo);

    this->w = this->sensorInfo.format.columns_per_frame;
    this->h = this->sensorInfo.format.pixels_per_column;

    // A ScanBatcher can be used to batch packets into scans
    this->pf = &(sensor::get_format(this->sensorInfo));
    this->batchToScan = new ScanBatcher(this->sensorInfo.format.columns_per_frame, *this->pf);

    ReportInfo("COMPLETED OUSTER SETUP!");

}

void OusterDecoder::Core()
{
    this->QueryAndSendData();
}

void OusterDecoder::Death()
{

}

//COMPONENT LOGIC STARTS HERE

void OusterDecoder::QueryAndSendData() 
{

    // check if sensor datat is available
    sensor::client_state st = sensor::poll_client(*this->handle);

    // if so, wait until full scan is complete, then parse and send it
    if (st & sensor::LIDAR_DATA) {

        auto lidar_packet = sensor::LidarPacket();
        auto scan = LidarScan{(size_t)this->w, (size_t)this->h, this->sensorInfo.format.udp_profile_lidar};

        // loop over lidar packets until sensor data is saved to scan variables
        bool done = false;
        while (!done) {

            if (!sensor::read_lidar_packet(*handle, lidar_packet, *this->pf)) {
                //ReportInfo("Failed to read a packet of the expected size!");
            }

            // batcher will return "true" when the current scan is complete
            if (this->batchToScan->operator()(lidar_packet, scan)) {
                // retry until we receive a full set of valid measurements
                // (accounting for azimuth_window settings if any)

                if (scan.complete(this->sensorInfo.format.column_window)) {
                    done = true;
                }
            }

        }

        auto cloud = ouster::cartesian(scan.field(sensor::ChanField::RANGE), this->lut);
        //Eigen::Array<uint32_t, -1, -1, Eigen::RowMajor> reflectivity = (scan.field<uint16_t>(sensor::ChanField::REFLECTIVITY)).cast<uint32_t>(); //this line is problematic, susceptible to config changes...

        // UNTESTED !!
        Eigen::Array<uint32_t, -1, -1, Eigen::RowMajor> reflectivity;
        if (this->sensorInfo.format.udp_profile_lidar == sensor::UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
            reflectivity = scan.field(sensor::ChanField::REFLECTIVITY);
        } else if (this->sensorInfo.format.udp_profile_lidar == sensor::UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL) {
            reflectivity = scan.field<uint8_t>(sensor::ChanField::REFLECTIVITY).cast<uint32_t>();
        } else {  // legacy or single return profile
            reflectivity = scan.field<uint16_t>(sensor::ChanField::REFLECTIVITY).cast<uint32_t>();
        }

        // get and destagger x, y, z, reflectivity vals
        auto x = Eigen::Map<const img_t<double>>(cloud.col(0).data(), this->h, this->w);
        auto y = Eigen::Map<const img_t<double>>(cloud.col(1).data(), this->h, this->w);
        auto z = Eigen::Map<const img_t<double>>(cloud.col(2).data(), this->h, this->w);

        auto x_destaggered = destagger<double>(x, this->sensorInfo.format.pixel_shift_by_row);
        auto y_destaggered = destagger<double>(y, this->sensorInfo.format.pixel_shift_by_row);
        auto z_destaggered = destagger<double>(z, this->sensorInfo.format.pixel_shift_by_row);
        auto reflectivity_destaggered = destagger<uint32_t>(reflectivity, this->sensorInfo.format.pixel_shift_by_row);

        std::vector<float> pointCloudVals;

        // iterate through destaggered matrix and make a series of bytes that can be sent through protobuf
        for (int row = 0; row < this->h; row++) {
            for (int col = 0; col < this->w; col++) {
                pointCloudVals.push_back((float)x_destaggered(row, col));
                pointCloudVals.push_back((float)y_destaggered(row, col));
                pointCloudVals.push_back((float)z_destaggered(row, col));
                pointCloudVals.push_back((float)reflectivity_destaggered(row, col));
            }
        }

        // write output
        MAPS::OutputGuard<MAPSFloat32> guard0(this, Output(0));
        WriteFloat32(&guard0, &pointCloudVals);

    }

}