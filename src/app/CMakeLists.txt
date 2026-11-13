qt_add_executable(rov_ground_station
    main.cpp
    RovController.cpp
    RovController.h
)

qt_add_qml_module(rov_ground_station
    URI RovGroundStation
    VERSION 1.0
    QML_FILES
        qml/Main.qml
    DEPENDENCIES
        QtQuick
)

target_compile_definitions(rov_ground_station PRIVATE
    ROV_APP_VERSION="${PROJECT_VERSION}"
)

target_link_libraries(rov_ground_station PRIVATE
    Qt6::Qml
    Qt6::Quick
    Qt6::QuickControls2
    Qt6::Location
    Qt6::Positioning
    rovsim
)

set_target_properties(rov_ground_station PROPERTIES
    MACOSX_BUNDLE TRUE
    MACOSX_BUNDLE_GUI_IDENTIFIER com.coratia.assessment.rovgroundstation
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION}
    WIN32_EXECUTABLE TRUE
)
