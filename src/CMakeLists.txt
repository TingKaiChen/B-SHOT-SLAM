add_library( myslam SHARED
    ViewerHandler.cpp
)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)