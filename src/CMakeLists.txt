rock_library(slam-tilt_scan
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(slam-tilt_scan_bin Main.cpp
    DEPS slam-tilt_scan)

