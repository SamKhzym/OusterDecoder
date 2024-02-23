Some Ouster helpers for decoding LiDAR data!

In python folder, there are two files -- record_lidar.py which saves Ouster data to pcap, and pcap_parser.py which can post-process the pcap and read the point cloud data in python.

In cpp folder, there is an rtmaps component under bin (built only for linux, most recent version untested) that takes in hostname as a parameter and outputs a float stream output representing a flattened array of [x,y,z,r] data. Source code is also there if modifications need to be made, dependent on eigen3, jsoncpp, curl, spdlog (we recommend using vcpkg to get these dependencies).
