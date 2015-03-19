
# tools
# yuv2mpeg

SET(COMPILE_DEFINITIONS -Werror)

include_directories(${PROJECT_SOURCE_DIR}/host_applications/linux/libs/bcm_host/include)
include_directories(${PROJECT_SOURCE_DIR}/host_applications/linux/apps/raspicam/)

set (COMMON_SOURCES
   yuv2mpeg.c
   )

add_executable(yuv2mpeg  ${COMMON_SOURCES} yuv2mpeg.c)

set (MMAL_LIBS mmal_core mmal_util mmal_vc_client)

target_link_libraries(yuv2mpeg   ${MMAL_LIBS} vcos bcm_host)

install(TARGETS yuv2mpeg RUNTIME DESTINATION bin)
