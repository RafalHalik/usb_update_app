# Add set(CONFIG_USE_middleware_usb_host_ehci true) in config.cmake to use this component

include_guard(GLOBAL)
message("${CMAKE_CURRENT_LIST_FILE} component is included.")

if((CONFIG_DEVICE_ID STREQUAL MIMXRT1041xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1042xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1051xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1052xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1064xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1064xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1021xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1165xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1166xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1171xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1172xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1173xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1175xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1176xxxxx) 
AND CONFIG_USE_middleware_usb_host_ehci_config_header 
AND CONFIG_USE_middleware_usb_host_common_header 
AND ((CONFIG_USE_middleware_usb_phy AND (CONFIG_DEVICE_ID STREQUAL MIMXRT1021xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1041xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1042xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1051xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1052xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1064xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1064xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1165xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1166xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1171xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1172xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1173xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1175xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1176xxxxx OR CONFIG_DEVICE_ID STREQUAL LPC5514 OR CONFIG_DEVICE_ID STREQUAL LPC5516 OR CONFIG_DEVICE_ID STREQUAL LPC5526 OR CONFIG_DEVICE_ID STREQUAL LPC5528 OR CONFIG_DEVICE_ID STREQUAL LPC55S14 OR CONFIG_DEVICE_ID STREQUAL LPC55S16 OR CONFIG_DEVICE_ID STREQUAL LPC55S26 OR CONFIG_DEVICE_ID STREQUAL LPC55S28 OR CONFIG_DEVICE_ID STREQUAL LPC55S66 OR CONFIG_DEVICE_ID STREQUAL LPC55S69)) OR (NOT (CONFIG_NOT STREQUAL MIMXRT1021xxxxx OR CONFIG_NOT STREQUAL MIMXRT1041xxxxB OR CONFIG_NOT STREQUAL MIMXRT1042xxxxB OR CONFIG_NOT STREQUAL MIMXRT1051xxxxB OR CONFIG_NOT STREQUAL MIMXRT1052xxxxB OR CONFIG_NOT STREQUAL MIMXRT1061xxxxA OR CONFIG_NOT STREQUAL MIMXRT1061xxxxB OR CONFIG_NOT STREQUAL MIMXRT1062xxxxA OR CONFIG_NOT STREQUAL MIMXRT1062xxxxB OR CONFIG_NOT STREQUAL MIMXRT1064xxxxA OR CONFIG_NOT STREQUAL MIMXRT1064xxxxB OR CONFIG_NOT STREQUAL MIMXRT1165xxxxx OR CONFIG_NOT STREQUAL MIMXRT1166xxxxx OR CONFIG_NOT STREQUAL MIMXRT1171xxxxx OR CONFIG_NOT STREQUAL MIMXRT1172xxxxx OR CONFIG_NOT STREQUAL MIMXRT1173xxxxx OR CONFIG_NOT STREQUAL MIMXRT1175xxxxx OR CONFIG_NOT STREQUAL MIMXRT1176xxxxx OR CONFIG_NOT STREQUAL LPC5514 OR CONFIG_NOT STREQUAL LPC5516 OR CONFIG_NOT STREQUAL LPC5526 OR CONFIG_NOT STREQUAL LPC5528 OR CONFIG_NOT STREQUAL LPC55S14 OR CONFIG_NOT STREQUAL LPC55S16 OR CONFIG_NOT STREQUAL LPC55S26 OR CONFIG_NOT STREQUAL LPC55S28 OR CONFIG_NOT STREQUAL LPC55S66 OR CONFIG_NOT STREQUAL LPC55S69))))

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/host/usb_host_ehci.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/host
  ${CMAKE_CURRENT_LIST_DIR}/include
)

else()

message(SEND_ERROR "middleware_usb_host_ehci dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()
