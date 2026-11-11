# Headless tests for the simulation core. These link rovsim only - no Qt, no
# display, no event loop - which is the point of keeping the physics in its own
# target.

add_executable(test_simulator test_simulator.cpp)
target_link_libraries(test_simulator PRIVATE rovsim)

add_test(NAME simulator COMMAND test_simulator)
