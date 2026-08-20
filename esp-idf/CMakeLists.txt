# spangap/tinylcd — paged status UI for tiny mono OLEDs.
#
# Cross-straddle deps flow through ${SPANGAP_REQUIRES} only (the staging lint
# rejects a hand-written sibling repo name). u8g2 is this straddle's own nested
# component (components/u8g2/), REQUIRES-public because tinylcd.h exposes
# u8g2_t* to page draw callbacks.
include(${CMAKE_CURRENT_LIST_DIR}/spangap_requires.cmake)
idf_component_register(
    SRCS
        "src/tinylcd.cpp"
        ${SPANGAP_CONDITIONAL_SRCS}
    INCLUDE_DIRS "include" ${SPANGAP_CONDITIONAL_INCLUDE_DIRS}
    REQUIRES
        ${SPANGAP_REQUIRES}
        u8g2
        driver
)
