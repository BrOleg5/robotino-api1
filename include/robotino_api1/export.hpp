// Symbol visibility for the shared library build.
#pragma once

#if defined(ROBOTINO_API1_STATIC)
#define ROBOTINO_API1_EXPORT
#elif defined(_WIN32)
#if defined(ROBOTINO_API1_BUILDING)
#define ROBOTINO_API1_EXPORT __declspec(dllexport)
#else
#define ROBOTINO_API1_EXPORT __declspec(dllimport)
#endif
#else
#define ROBOTINO_API1_EXPORT __attribute__((visibility("default")))
#endif
