cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

target_sources(app PRIVATE 
    src/main.c
    src/host_msd_fatfs.c
)

set(MCUX_SDK_PROJECT_NAME app)

target_include_directories(app PUBLIC 
    include
    )

include_directories(include)


# USB Example 
add_compile_definitions(DATA_SECTION_IS_CACHEABLE=1)
add_compile_definitions(HOST_ECHO=1)

include(config.cmake)

add_definitions(-DFSL_RTOS_ZEPHYR)
add_definitions(-DCONFIG_ZEPHYR)
add_definitions(-DHOST_ECHO=1)

# OSA
set(CONFIG_USE_component_osa 1)

# set(CONFIG_USE_middleware_baremetal 1)
set(CONFIG_USE_driver_common 1)
set(CONFIG_USE_component_lists 1)
set(CONFIG_USE_component_osa_interface 1)
set(CONFIG_USE_osa_zephyr 1)

include(lists/component_lists.MIMXRT1062.cmake)
include(osa/component_osa.cmake)
include(osa/component_osa_zephyr.cmake)

# USB
set(CONFIG_USE_middleware_usb_common_header 1)
set(CONFIG_DEVICE_ID "MIMXRT1062xxxxB")
set(CONFIG_USE_middleware_usb_host_stack 1)
set(CONFIG_USE_middleware_usb_host_ehci 1) 
set(CONFIG_USE_middleware_usb_phy 1)
set(CONFIG_USE_middleware_usb_host_common_header 1)
set(CONFIG_USE_middleware_usb_host_ehci_config_header 1)
set(CONFIG_USE_driver_memory 1)

include(usb/middleware_usb_common_header.cmake)
include(usb/middleware_usb_host_common_header.cmake)
include(usb/middleware_usb_host_stack.cmake)
include(usb/middleware_usb_host_msd.cmake)
include(usb/middleware_usb_host_ehci.cmake)
include(usb/middleware_usb_phy.cmake)

# FATFS
set(CONFIG_USE_middleware_baremetal 1)
include(fatfs/middleware_fatfs.cmake)
include(fatfs/middleware_fatfs_usb.cmake)

# LVGL Demo
set(LVGL_DIR ${ZEPHYR_LVGL_MODULE_DIR})

target_include_directories(app PRIVATE
    ${LVGL_DIR}/demos/
)

target_sources_ifdef(CONFIG_LV_USE_DEMO_MUSIC app PRIVATE
    ${LVGL_DIR}/demos/music/lv_demo_music_list.c
    ${LVGL_DIR}/demos/music/lv_demo_music.c
    ${LVGL_DIR}/demos/music/lv_demo_music_main.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_next.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_wave_top_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_icon_4_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_icon_4.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_corner_right.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_cover_1.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_slider_knob_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_icon_3.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_pause.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_pause_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_wave_bottom_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_icon_2_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_list_play_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_wave_top.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_play_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_cover_1_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_wave_bottom.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_corner_left_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_play.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_list_border.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_icon_2.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_next_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_list_play.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_list_border_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_rnd.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_cover_3.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_prev_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_loop.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_icon_1.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_slider_knob.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_corner_right_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_corner_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_corner_left.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_cover_2_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_icon_3_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_icon_1_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_cover_2.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_prev.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_rnd_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_list_pause_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_cover_3_large.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_list_pause.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_logo.c
    ${LVGL_DIR}/demos/music/assets/img_lv_demo_music_btn_loop_large.c
)

target_sources_ifdef(CONFIG_LV_USE_DEMO_BENCHMARK app PRIVATE
    ${LVGL_DIR}/demos/benchmark/lv_demo_benchmark.c
    ${LVGL_DIR}/demos/benchmark/assets/img_benchmark_cogwheel_indexed16.c
    ${LVGL_DIR}/demos/benchmark/assets/lv_font_bechmark_montserrat_28_compr_az.c.c
    ${LVGL_DIR}/demos/benchmark/assets/lv_font_bechmark_montserrat_16_compr_az.c.c
    ${LVGL_DIR}/demos/benchmark/assets/img_benchmark_cogwheel_argb.c
    ${LVGL_DIR}/demos/benchmark/assets/img_benchmark_cogwheel_alpha16.c
    ${LVGL_DIR}/demos/benchmark/assets/img_benchmark_cogwheel_rgb.c
    ${LVGL_DIR}/demos/benchmark/assets/lv_font_bechmark_montserrat_12_compr_az.c.c
    ${LVGL_DIR}/demos/benchmark/assets/img_benchmark_cogwheel_chroma_keyed.c
    ${LVGL_DIR}/demos/benchmark/assets/img_benchmark_cogwheel_rgb565a8.c
)

target_sources_ifdef(CONFIG_LV_USE_DEMO_STRESS app PRIVATE
    ${LVGL_DIR}/demos/stress/lv_demo_stress.c
)

target_sources_ifdef(CONFIG_LV_USE_DEMO_WIDGETS app PRIVATE
    ${LVGL_DIR}/demos/widgets/assets/img_clothes.c
    ${LVGL_DIR}/demos/widgets/assets/img_demo_widgets_avatar.c
    ${LVGL_DIR}/demos/widgets/assets/img_lvgl_logo.c
    ${LVGL_DIR}/demos/widgets/lv_demo_widgets.c
)