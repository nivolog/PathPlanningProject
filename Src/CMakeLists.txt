
set(HEADERS         config.h
                    environmentoptions.h
                    gl_const.h
                    ilogger.h
                    map.h
                    mission.h
                    node.h
                    search.h
                    searchresult.h
                    tinyxml2.h
                    xmllogger.h
                    open.h
                    closed.h)

set(SOURCES         config.cpp
                    environmentoptions.cpp
                    map.cpp
                    mission.cpp
                    search.cpp
                    tinyxml2.cpp
                    xmllogger.cpp
                    node.cpp
                    open.cpp
                    closed.cpp)

add_executable(PathPlanning ${HEADERS} ${SOURCES} main.cpp)

add_library(PPLIB ${HEADERS} ${SOURCES})

install(TARGETS PathPlanning 
        CONFIGURATIONS Debug
        DESTINATION Bin/Debug)

install(TARGETS PathPlanning 
        CONFIGURATIONS Release
        DESTINATION Bin/Release)
