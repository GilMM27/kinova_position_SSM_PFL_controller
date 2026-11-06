#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define KinovaPositionSSMPFLController_DLLIMPORT __declspec(dllimport)
#  define KinovaPositionSSMPFLController_DLLEXPORT __declspec(dllexport)
#  define KinovaPositionSSMPFLController_DLLLOCAL
#elif defined __GNUC__ && __GNUC__ >= 4
#  define KinovaPositionSSMPFLController_DLLIMPORT __attribute__((visibility("default")))
#  define KinovaPositionSSMPFLController_DLLEXPORT __attribute__((visibility("default")))
#  define KinovaPositionSSMPFLController_DLLLOCAL __attribute__((visibility("hidden")))
#else
#  define KinovaPositionSSMPFLController_DLLIMPORT
#  define KinovaPositionSSMPFLController_DLLEXPORT
#  define KinovaPositionSSMPFLController_DLLLOCAL
#endif

#ifdef KinovaPositionSSMPFLController_STATIC
#  define KinovaPositionSSMPFLController_DLLAPI
#else
#  ifdef KinovaPositionSSMPFLController_EXPORTS
#    define KinovaPositionSSMPFLController_DLLAPI KinovaPositionSSMPFLController_DLLEXPORT
#  else
#    define KinovaPositionSSMPFLController_DLLAPI KinovaPositionSSMPFLController_DLLIMPORT
#  endif
#endif
