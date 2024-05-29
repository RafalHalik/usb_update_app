/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
// flash includes...

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>

#include "board.h"
#include "usb_host_config.h"
#include "usb_host.h"
#include "fsl_device_registers.h"
#include "fsl_common.h"

// Display
#include <zephyr/drivers/display.h>
#include <lvgl.h>
#include <lv_demos.h>


#include "usb_phy.h"
#include "usb_host_msd.h"
#include "host_msd_fatfs.h"
#include "app.h"

#include "zephyr/dfu/mcuboot.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);


#if ((!USB_HOST_CONFIG_KHCI) && (!USB_HOST_CONFIG_EHCI) && (!USB_HOST_CONFIG_OHCI) && (!USB_HOST_CONFIG_IP3516HS))
#error Please enable USB_HOST_CONFIG_KHCI, USB_HOST_CONFIG_EHCI, USB_HOST_CONFIG_OHCI, or USB_HOST_CONFIG_IP3516HS in file usb_host_config.
#endif

#define USB_THREAD_STACK_SIZE 4096
#define USB_THREAD_PRIOROTY 7

#define LVGL_THREAD_PRIOROTY 8

#define USB_UPDATE_THREAD_PRIOROTY 2



/*!
 * @brief host callback function.
 *
 * device attach/detach callback function.
 *
 * @param deviceHandle        device handle.
 * @param configurationHandle attached device's configuration descriptor information.
 * @param eventCode           callback event code, please reference to enumeration host_event_t.
 *
 * @retval kStatus_USB_Success              The host is initialized successfully.
 * @retval kStatus_USB_NotSupported         The application don't support the configuration.
 */
static usb_status_t USB_HostEvent(usb_device_handle deviceHandle,
                                  usb_host_configuration_handle configurationHandle,
                                  uint32_t eventCode);



static void USB_HostApplicationInit(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/

K_THREAD_STACK_DEFINE(usb_thread_stack, USB_THREAD_STACK_SIZE);
struct k_thread usb_thread_data;

K_THREAD_STACK_DEFINE(lvgl_thread_stack, USB_THREAD_STACK_SIZE);
struct k_thread lvgl_thread_data;

K_THREAD_STACK_DEFINE(usb_update_thread_stack, USB_THREAD_STACK_SIZE);
struct k_thread usb_update_thread_data;

/*! @brief USB host msd fatfs instance global variable */
extern usb_host_msd_fatfs_instance_t g_MsdFatfsInstance;
usb_host_handle g_HostHandle;

/* Run USB stack on a separate thread */
static k_tid_t usb_thread_id;

static k_tid_t lvgl_thread_id; 

static k_tid_t usb_update_thread_id; 


/******************************************************************************/
/* Functions */
/******************************************************************************/

/******************************************************************************/
void usb_thread(void *p1, void *p2, void *p3);

void lvgl_thread(void *p1, void *p2, void *p3);

void usb_update_thread(void *p1, void *p2, void *p3);

static void lv_btn_click_callback(lv_event_t *e);

void start_update_process(bool updating);
void stop_read_usb(void);

static lv_style_t style_indic;
static lv_obj_t *hello_world_label;
static lv_obj_t *count_label;
static lv_obj_t *update_label;
static uint32_t count;
 
static lv_obj_t *mainScreen;
static lv_obj_t *updateScreen;
static lv_obj_t *bar1;

static bool stopReadingUsb = false;

char count_str[11] = {0};
bool updateInProgress = false;

#define SEM_COUNT_INITAL 0
#define SEM_COUNT_MAX 1


/******************************************************************************/
void start_update_process(bool updating)
{
    updateInProgress = updating;
}


void dynamic_usb_isr(const void *param)
{
    USB_HostEhciIsrFunction(g_HostHandle);
}

static void lv_btn_click_callback(lv_event_t *e)
{
	ARG_UNUSED(e);

	count = 0;
}

static struct gpio_dt_spec button_gpio = GPIO_DT_SPEC_GET_OR(
		DT_ALIAS(sw0), gpios, {0});
static struct gpio_callback button_callback;

static void button_isr_callback(const struct device *port,
				struct gpio_callback *cb,
				uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	count = 0;
}
/******************************************************************************/



int main(void)
{
    // LVGL
    const struct device *display_dev;
    bool image_ok = boot_is_img_confirmed();
    if(!image_ok)
    {
        bool ret = boot_write_img_confirmed();
        if(ret)
        {
            usb_echo("image cant be confirmed \r\n");
        }
    }
    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev))
    {
        LOG_ERR("Device not ready, aborting test");
        return 0;
    }



    // Initialize USB and start thread
    USB_HostApplicationInit();
    usb_thread_id = k_thread_create(&usb_thread_data, usb_thread_stack,
                                    K_THREAD_STACK_SIZEOF(usb_thread_stack),
                                    usb_thread,
                                    NULL, NULL, NULL,
                                    USB_THREAD_PRIOROTY,
                                    0,
                                    K_NO_WAIT);

    lvgl_thread_id = k_thread_create(&lvgl_thread_data, lvgl_thread_stack,
                                   K_THREAD_STACK_SIZEOF(lvgl_thread_stack),
                                   lvgl_thread,
                                   NULL, NULL, NULL,
                                   LVGL_THREAD_PRIOROTY,
                                   0,
                                   K_NO_WAIT);    

    usb_update_thread_id = k_thread_create(&usb_update_thread_data, usb_update_thread_stack,
                                   K_THREAD_STACK_SIZEOF(usb_update_thread_stack),
                                   usb_update_thread,
                                   NULL, NULL, NULL,
                                   USB_UPDATE_THREAD_PRIOROTY,
                                   0,
                                   K_NO_WAIT);                                    

    while (1)
    {
        k_msleep(100);
    }

    return 0;
}

/*******************************************************************************/
void USB_HostClockInit(void)
{
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };

    if (CONTROLLER_ID == kUSB_ControllerEhci0)
    {
        CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);
    }
    else
    {
        CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M, 480000000U);
    }
    usb_status_t stat = USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);
    assert(stat == kStatus_USB_Success);
}

void USB_HostIsrEnable(void)
{
    uint8_t irqNumber;

    uint8_t usbHOSTEhciIrq[] = USBHS_IRQS;
    irqNumber = usbHOSTEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];

    irq_connect_dynamic(irqNumber, USB_HOST_INTERRUPT_PRIORITY, dynamic_usb_isr, NULL, 0);
    irq_enable(irqNumber);
}

/*******************************************************************************/
void USB_HostTaskFn(void *param)
{
    USB_HostEhciTaskFunction(param);
}

/*******************************************************************************/
/*!
 * @brief USB isr function.
 */

static usb_status_t USB_HostEvent(usb_device_handle deviceHandle,
                                  usb_host_configuration_handle configurationHandle,
                                  uint32_t eventCode)
{
    usb_status_t status = kStatus_USB_Success;
    switch (eventCode & 0x0000FFFFU)
    {
    case kUSB_HostEventAttach:

        status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
        stopReadingUsb = false;
        break;

    case kUSB_HostEventNotSupported:
        usb_echo("Unsupported Device\r\n");

        break;

    case kUSB_HostEventEnumerationDone:
        status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
        break;

    case kUSB_HostEventDetach:
        usb_echo("kUSB_HostEventDetach\r\n");
        status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
        stopReadingUsb = true;
        break;

    case kUSB_HostEventEnumerationFail:
        usb_echo("enumeration failed\r\n");
        break;

    default:
        break;
    }
    return status;
}

/******************************************************************************/
static void USB_HostApplicationInit(void)
{
    usb_status_t status = kStatus_USB_Success;

    USB_HostClockInit();

#if ((defined FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT))
    SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

    status = USB_HostInit(CONTROLLER_ID, &g_HostHandle, USB_HostEvent);
    if (status != kStatus_USB_Success)
    {
        usb_echo("host init error\r\n");
        return;
    }
    USB_HostIsrEnable();

    usb_echo("host init done\r\n");
};

/******************************************************************************/
void usb_thread(void *p1, void *p2, void *p3)
{
    usb_echo("USB Thread started\r\n");

    while (1)
    {
        USB_HostTaskFn(g_HostHandle);
        USB_HostMsdTask(&g_MsdFatfsInstance);       
        k_msleep(1);      // make sure this dosent break anything
    }
}



 void lvgl_thread(void *p1, void *p2, void *p3)
{
#if defined(CONFIG_LV_USE_DEMO_MUSIC)
     lv_demo_music();
#elif defined(CONFIG_LV_USE_DEMO_BENCHMARK)
     lv_demo_benchmark();
#elif defined(CONFIG_LV_USE_DEMO_STRESS)
     lv_demo_stress();
#elif defined(CONFIG_LV_USE_DEMO_WIDGETS)
     lv_demo_widgets();
#endif

    lv_obj_t *prevScreen = lv_scr_act();;
    usb_echo("LVGL Thread started\r\n");
    lvgl_setup();   
    
    
    while (1)
    {
        if (updateInProgress)
        {    
           
            if(lv_scr_act() != updateScreen)
            {
                lv_scr_load(updateScreen); 
                lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_BLUE));
                lv_obj_add_style(bar1, &style_indic, LV_PART_INDICATOR);
                lv_label_set_text(update_label, "USB update in progress...");
                lv_bar_set_value(bar1, 0, LV_ANIM_OFF);
                usb_echo("SWITCH TO UPDATE SCREEN \r\n");
            }
        }
        else
        { 
            if (lv_scr_act() != prevScreen)
            {
                lv_scr_load(prevScreen);
                usb_echo("SWITCH TO MAIN SCREEN \r\n");
            }       
        }
        
		lv_task_handler();		
		k_sleep(K_MSEC(10));
    }
}

void update_progress_bar(uint32_t fileSizeInBytes, size_t bytesRead)
{
    uint32_t percentageComplete = (bytesRead * 100) / fileSizeInBytes ;
    lv_bar_set_value(bar1, percentageComplete, LV_ANIM_OFF);
    if (percentageComplete >= 100)
    {      
        lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_GREEN));
        lv_obj_add_style(bar1, &style_indic, LV_PART_INDICATOR);
        lv_label_set_text(update_label, "Remove USB, Rebooting shortly...");
    }
    lv_task_handler();	
}

 void usb_update_thread(void *p1, void *p2, void *p3)
{
    usb_echo("USB update Thread started\r\n");
    
    while (1)
    {
        usb_update_task();  
        k_msleep(1);         
    }
}



int lvgl_setup(void)
{
    const struct device *display_dev;
	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	if (!device_is_ready(display_dev)) {
		LOG_ERR("Device not ready, aborting test");
		return 0;
	}

    lv_style_init(&style_indic);
    lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
	
    if (gpio_is_ready_dt(&button_gpio)) {
		int err;

		err = gpio_pin_configure_dt(&button_gpio, GPIO_INPUT);
		if (err) {
			LOG_ERR("failed to configure button gpio: %d", err);
			return 0;
		}

		gpio_init_callback(&button_callback, button_isr_callback,
				   BIT(button_gpio.pin));

		err = gpio_add_callback(button_gpio.port, &button_callback);
		if (err) {
			LOG_ERR("failed to add button callback: %d", err);
			return 0;
		}

		err = gpio_pin_interrupt_configure_dt(&button_gpio,
						      GPIO_INT_EDGE_TO_ACTIVE);
		if (err) {
			LOG_ERR("failed to enable button callback: %d", err);
			return 0;
		}
	}

    //main_screen_init();
    update_screen_init();

    if (mainScreen)
    {
        lv_scr_load(mainScreen);
    }


	lv_task_handler();
	display_blanking_off(display_dev);
    return 0;
}


void display_read_error(void * displayString)
{
    if(displayString == NULL)
    {
        displayString = "Error during update..."; // Generic error if no message passed.
    }
    lv_bar_set_value(bar1, 100, LV_ANIM_OFF);
    static lv_style_t style_indic;
    lv_style_init(&style_indic);
    lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
    lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_RED));
    lv_obj_add_style(bar1, &style_indic, LV_PART_INDICATOR);
    lv_label_set_text(update_label, displayString);
}

void main_screen_init(void)
{
    mainScreen = lv_scr_act();

	lv_obj_t *arc;
	lv_group_t *arc_group;

	arc = lv_arc_create(mainScreen);
	lv_obj_align(arc, LV_ALIGN_CENTER, 0, -15);
	lv_obj_set_size(arc, 150, 150);

	arc_group = lv_group_create();
	lv_group_add_obj(arc_group, arc);
	

	lv_obj_t *btn_matrix;
	lv_group_t *btn_matrix_group;
	static const char *const btnm_map[] = {"1", "2", "3", "4", ""};

	btn_matrix = lv_btnmatrix_create(mainScreen);
	lv_obj_align(btn_matrix, LV_ALIGN_CENTER, 0, 70);
	lv_btnmatrix_set_map(btn_matrix, (const char **)btnm_map);
	lv_obj_set_size(btn_matrix, 100, 50);

	btn_matrix_group = lv_group_create();
	lv_group_add_obj(btn_matrix_group, btn_matrix);

	
	lv_obj_t *hello_world_button;

	hello_world_button = lv_btn_create(mainScreen);
	lv_obj_align(hello_world_button, LV_ALIGN_CENTER, 0, -15);
	lv_obj_add_event_cb(hello_world_button, lv_btn_click_callback, LV_EVENT_CLICKED,
			    NULL);
	hello_world_label = lv_label_create(hello_world_button);

	lv_label_set_text(hello_world_label, "STATEMACHINE");
	lv_obj_align(hello_world_label, LV_ALIGN_CENTER, 0, 0);

	count_label = lv_label_create(mainScreen);
	lv_obj_align(count_label, LV_ALIGN_CENTER, 0, 100);
}

void update_screen_init(void)
{
    updateScreen = lv_obj_create(NULL); 

    lv_obj_clean(updateScreen);
    bar1 = lv_bar_create(updateScreen);
    lv_obj_set_size(bar1, 300, 25);
    lv_obj_center(bar1);
    lv_bar_set_value(bar1, 0, LV_ANIM_OFF);
    lv_bar_set_range(bar1, 0, 100);

    update_label = lv_label_create(updateScreen);
	lv_obj_align(update_label, LV_ALIGN_CENTER, 0, -100);
    lv_label_set_text(update_label, "USB update in progress...");
}