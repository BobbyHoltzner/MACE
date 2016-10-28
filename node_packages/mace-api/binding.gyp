{
  "targets": [
    {
      "target_name": "mace-api",
      "sources": [ "./src/mace_api_wrap.cxx" ],
      "include_dirs": [],
      "libraries": [],
      'cflags': ['-std=c++11', '-fPIC'],
      "cflags_cc": ['-std=c++11', '-fPIC'],
      'cflags!': [ '-fno-exceptions' ],
      'cflags_cc!': [ '-fno-exceptions' ],
      "ldflags": ["-fPIC"]
    }
  ]
}