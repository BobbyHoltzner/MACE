{
  "targets": [
    {
      "target_name": "mace-api",
      "sources": [ "./src/mace_api_wrap.cxx" ],
      "include_dirs": [
        "C:/Program Files (x86)/Eigen/include/eigen3",
        "C:/Code/MACE/src/",
      ],
      'link_settings': {
        'libraries': [
            '../../../build_MSVC2013_64bit-Debug/mace_core/debug/mace_core',
            '../../../build_MSVC2013_64bit-Debug/comms/debug/comms',
            '../../../build_MSVC2013_64bit-Debug/module_path_planning_NASAPhase2/debug/module_path_planning_NASAPhase2',
            '../../../build_MSVC2013_64bit-Debug/module_RTA_NASAPhase2/debug/module_RTA_NASAPhase2',
            '../../../build_MSVC2013_64bit-Debug/module_vehicle_MAVLINK/debug/module_vehicle_MAVLINK'
          ],
      },
      'cflags': ['-std=c++11', '-fPIC'],
      "cflags_cc": ['-std=c++11', '-fPIC'],
      'cflags!': [ '-fno-exceptions' ],
      'cflags_cc!': [ '-fno-exceptions' ],
      "ldflags": ["-fPIC"]
    }
  ]
}