%module MaceAPI

%{
#include "./mace_api.cpp"
%}

%include <windows.i>
%include "./mace_api.h"