#ifndef GLOBALS_H
#define GLOBALS_H

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <filesystem>

extern boost::filesystem::path full_path_;

// Debug 
extern int nFrontCloudMPMatchNum;
extern int nBackCloudMPMatchNum;

extern double nLostTimeStamp;
extern double nNewMapTimeStamp;
extern int   nLostTime;

#endif