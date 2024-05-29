/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016, 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "usb_host_config.h"
#include "usb_host.h"
#include "usb_host_msd.h"
#include "host_msd_fatfs.h"
#include "ff.h"
#include "diskio.h"
#include "fsl_device_registers.h"
#include "app.h"

 //===========================================
#include <zephyr/init.h>
#include <stdio.h>
#include <errno.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>

#include <zephyr/dfu/mcuboot.h>
#include <zephyr/storage/stream_flash.h>
#include <zephyr/dfu/flash_img.h>

#include <zephyr/sys/reboot.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define SLOT1_LABEL slot1_partition
#define SLOT1_SIZE FIXED_PARTITION_SIZE(SLOT1_LABEL)
#define SLOT1_ID FIXED_PARTITION_ID(SLOT1_LABEL)
#define SLOT1_OFFSET FIXED_PARTITION_OFFSET(SLOT1_LABEL)

#define STORAGE_LABEL storage_partition
#define STORAGE_DEV FIXED_PARTITION_DEVICE(STORAGE_LABEL)
#define STORAGE_OFFSET FIXED_PARTITION_OFFSET(STORAGE_LABEL)

#if MSD_FATFS_THROUGHPUT_TEST_ENABLE
#include "fsl_device_registers.h"
#define THROUGHPUT_BUFFER_SIZE (512)//(32 * 1024) /* throughput test buffer */
#define MCU_CORE_CLOCK (120000000)         /* mcu core clock, user need to configure it. */
#endif                                     /* MSD_FATFS_THROUGHPUT_TEST_ENABLE */

typedef enum _usb_update_state
{
    USB_update_mount_drive = 0,
    USB_update_read_file,
    USB_update_write_to_flash,
    USB_update_move_to_next_section,
    USB_check_image,
    USB_update_idle
} usb_update_state;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/




 /*!
  * @brief host msd control transfer callback.
  *
  * This function is used as callback function for control transfer .
  *
  * @param param      the host msd fatfs instance pointer.
  * @param data       data buffer pointer.
  * @param dataLength data length.
  * @status           transfer result status.
  */
void USB_HostMsdControlCallback(void* param, uint8_t* data, uint32_t dataLength, usb_status_t status);

/*!
 * @brief display file information.
 */
static void USB_HostMsdFatfsDisplayFileInfo(FILINFO* fileInfo);

/*!
 * @brief list files and sub-directory in one directory, the function don't check all sub-directories recursively.
 */
static FRESULT USB_HostMsdFatfsListDirectory(const TCHAR* path);



/*******************************************************************************
 * Veethree Fnc
 ******************************************************************************/

 /*!
  * @brief Unblocks/block flash write thread
  *
  * This function sets a flag to either block or unblock the write flash task
  * This is called when a USB drive is attached enumerated and mounted with
  * update file present.
  *
  * @param updating  Flag, if USB drive is set up and ready for reading
  */
extern void start_update_process(bool updating);

/*!
 * @brief changes value on update screens progress bar to indicate amount left
 *
 * Takes the size of the file read and total bytes of data read, calculating a %
 * which is then displayed on screen in the form a LVGL bar object
 *
 * @param fileSizeInBytes size of the file being read
 * @param bytesRead amount of data read from usb so far in bytes
 */
extern void update_progress_bar(uint32_t fileSizeInBytes, size_t bytesRead);


/*!
 * @brief Takes read result from fatfs and writes it to flash
 *
 * After fatfs reads the max amount of bytes it can hold in its buffer this function is called
 * to write the buffered data to flash in chuncks that can be user altered with the #define THROUGHPUT_BUFFER_SIZE
 *
 * @param bytesLeftToRead total data to be written to memory
 */
void write_to_flash(uint32_t bytesLeftToRead);



/*!
 * @brief very basic image validity check
 */
void check_image(void);


/*******************************************************************************
 * Variables
 ******************************************************************************/

static struct flash_img_context flash_ctx;
static usb_update_state usb_update = USB_update_idle;

static bool ReadyToRead = false;
static bool FileWrittenToFlash = false;

static FRESULT fatfsCode;
static FIL file;
static FILINFO fileInfo;
static uint8_t driverNumberBuffer[3];

 /*! @brief msd class handle array for fatfs */
extern usb_host_class_handle g_UsbFatfsClassHandle;

usb_host_msd_fatfs_instance_t g_MsdFatfsInstance; /* global msd fatfs instance */
static FATFS fatfs;
/* control transfer on-going state. It should set to 1 when start control transfer, it is set to 0 in the callback */
volatile uint8_t controlIng;
/* control transfer callback status */
volatile usb_status_t controlStatus;

#define THROUGHPUT_BUFFER_SIZE (512) //(16 * 1024) //(64 * 1024)
static uint32_t testThroughputBuffer[THROUGHPUT_BUFFER_SIZE / 4];


/*******************************************************************************
 * Code
 ******************************************************************************/



void USB_HostMsdControlCallback(void* param, uint8_t* data, uint32_t dataLength, usb_status_t status)
{
    usb_host_msd_fatfs_instance_t* msdFatfsInstance = (usb_host_msd_fatfs_instance_t*)param;

    if (msdFatfsInstance->runWaitState == kUSB_HostMsdRunWaitSetInterface) /* set interface finish */
    {
        msdFatfsInstance->runWaitState = kUSB_HostMsdRunIdle;
        msdFatfsInstance->runState = kUSB_HostMsdRunMassStorageTest;
    }
    controlIng = 0;
    controlStatus = status;
}


static void USB_HostMsdFatfsDisplayFileInfo(FILINFO* fileInfo)
{
    char* fileName;
    fileName = fileInfo->fname;
    /* note: if this file/directory don't have one attribute, '_' replace the attribute letter ('R' - readonly, 'H' -
     * hide, 'S' - system) */
    usb_echo("    %s - %c%c%c - %s - %dBytes - %d-%d-%d %d:%d:%d\r\n", (fileInfo->fattrib & AM_DIR) ? "dir" : "fil",
        (fileInfo->fattrib & AM_RDO) ? 'R' : '_', (fileInfo->fattrib & AM_HID) ? 'H' : '_',
        (fileInfo->fattrib & AM_SYS) ? 'S' : '_', fileName, (fileInfo->fsize),
        (uint32_t)((fileInfo->fdate >> 9) + 1980) /* year */,
        (uint32_t)((fileInfo->fdate >> 5) & 0x000Fu) /* month */, (uint32_t)(fileInfo->fdate & 0x001Fu) /* day */,
        (uint32_t)((fileInfo->ftime >> 11) & 0x0000001Fu) /* hour */,
        (uint32_t)((fileInfo->ftime >> 5) & 0x0000003Fu) /* minute */,
        (uint32_t)(fileInfo->ftime & 0x0000001Fu) /* second */
    );
}

static FRESULT USB_HostMsdFatfsListDirectory(const TCHAR* path)
{
    FRESULT fatfsCode = FR_OK;
    FILINFO fileInfo;
    DIR dir;
    uint8_t outputLabel = 0;

    fatfsCode = f_opendir(&dir, path);
    if (fatfsCode)
    {
        return fatfsCode;
    }
    while (1)
    {
        fatfsCode = f_readdir(&dir, &fileInfo);
        if ((fatfsCode) || (!fileInfo.fname[0]))
        {
            break;
        }
        outputLabel = 1;
        USB_HostMsdFatfsDisplayFileInfo(&fileInfo);
    }
    if (!outputLabel)
    {
        // usb_echo("\r\n");
    }

    return fatfsCode;
}

void USB_HostMsdTask(void* arg)
{
    usb_status_t status;
    usb_host_msd_fatfs_instance_t* msdFatfsInstance = (usb_host_msd_fatfs_instance_t*)arg;

    if (msdFatfsInstance->deviceState != msdFatfsInstance->prevDeviceState)
    {
        msdFatfsInstance->prevDeviceState = msdFatfsInstance->deviceState;
        switch (msdFatfsInstance->deviceState)
        {
        case kStatus_DEV_Idle:          
            break;

        case kStatus_DEV_Attached: /* deivce is attached and numeration is done */
         //usb_echo("kStatus_DEV_Attached \r\n");
            status = USB_HostMsdInit(msdFatfsInstance->deviceHandle,
                &msdFatfsInstance->classHandle); /* msd class initialization */
            g_UsbFatfsClassHandle = msdFatfsInstance->classHandle;
            if (status != kStatus_USB_Success)
            {
                usb_echo("usb host msd init fail\r\n");
                return;
            }
            msdFatfsInstance->runState = kUSB_HostMsdRunSetInterface;
            break;

        case kStatus_DEV_Detached: /* device is detached */
          //usb_echo("kStatus_DEV_Detached \r\n");
            msdFatfsInstance->deviceState = kStatus_DEV_Idle;
            msdFatfsInstance->runState = kUSB_HostMsdRunIdle;
            USB_HostMsdDeinit(msdFatfsInstance->deviceHandle,
                msdFatfsInstance->classHandle); /* msd class de-initialization */
            msdFatfsInstance->classHandle = NULL;
            //g_MsdFatfsInstance.deviceState = kStatus_DEV_Detached;
            start_update_process(false);
            usb_echo("mass storage device detached\r\n");
            break;

        default:
            break;
        }
    }

    /* run state */
    switch (msdFatfsInstance->runState)
    {
    case kUSB_HostMsdRunIdle:
        break;

    case kUSB_HostMsdRunSetInterface: /* set msd interface */
    //usb_echo("kUSB_HostMsdRunSetInterface \r\n");
        msdFatfsInstance->runState = kUSB_HostMsdRunIdle;
        msdFatfsInstance->runWaitState = kUSB_HostMsdRunWaitSetInterface;
        status = USB_HostMsdSetInterface(msdFatfsInstance->classHandle, msdFatfsInstance->interfaceHandle, 0,
            USB_HostMsdControlCallback, msdFatfsInstance);
        if (status != kStatus_USB_Success)
        {
            // usb_echo("set interface fail\r\n");
        }
        break;

    case kUSB_HostMsdRunMassStorageTest: /* set interface succeed */
    //usb_echo("kUSB_HostMsdRunMassStorageTest \r\n");
        usb_update = USB_update_mount_drive;
        ReadyToRead = true;
        start_update_process(true);
        msdFatfsInstance->runState = kUSB_HostMsdRunIdle;
        //USB_MountStorageDeviceFatfs(msdFatfsInstance);
        // msdFatfsInstance->runState = kUSB_HostMsdRunUpdate;
        break;
    case kUSB_HostMsdRunUpdate:
    //usb_echo("kUSB_HostMsdRunUpdate \r\n");
        start_update_process(true);
        usb_echo("USB update case... \r\n");
        msdFatfsInstance->runState = kUSB_HostMsdRunIdle;
        break;
    default:
        break;
    }
}



void usb_update_task(void)
{
    static uint32_t bytesLeftToRead = 0;
    static uint32_t resultSize = 1;
    static uint8_t fileReadCounter = 0;
    static size_t resultSizeTotal = 0;

    static uint8_t DEBUG_number_of_writes = 0;

    switch (usb_update)
    {
    case USB_update_mount_drive:       
        // Reinitalize varibles incase somethign went wrong and update had to be attempted again...
        bytesLeftToRead = 0;
        resultSize = 1;
        fileReadCounter = 0;
        resultSizeTotal = 0;
        DEBUG_number_of_writes = 0;

        sprintf((char*)&driverNumberBuffer[0], "%c:", USBDISK + '0');
        fatfsCode = f_mount(&fatfs, (char const*)&driverNumberBuffer[0], 0);
        if (fatfsCode)
        {
            return;
        }

        fatfsCode = USB_HostMsdFatfsListDirectory((char const*)&driverNumberBuffer[0]);
        if (fatfsCode)
        {
            return;
        }

        fatfsCode = f_open(&file, _T("1:/zephyr~1.bin"), FA_READ | FA_OPEN_EXISTING);
        if (fatfsCode)
        {
            usb_echo("error file no able to be opended\r\n");
        }
        else
        {
            fatfsCode = f_stat(_T("1:/zephyr~1.bin"), &fileInfo);
            if (fatfsCode)
            {
                return;
            }
        }
        flash_img_init_id(&flash_ctx, SLOT1_ID);
        flash_area_erase(flash_ctx.flash_area, SLOT1_ID, flash_ctx.flash_area->fa_size);

        bytesLeftToRead = fileInfo.fsize;
        usb_update = USB_update_read_file;
        break;
    case USB_update_read_file:
            fatfsCode = f_read(&file, testThroughputBuffer, THROUGHPUT_BUFFER_SIZE, &resultSize);
            if (fatfsCode)
            {
                usb_echo("read error %d \r\n", fatfsCode);                        
                update_failed_clean_up("Error Reading! try again...");
                return;
            }           

            bytesLeftToRead -= resultSize;
            usb_update = USB_update_write_to_flash;
        break;
    case USB_update_write_to_flash:       
        write_to_flash(bytesLeftToRead);
        DEBUG_number_of_writes++;
        resultSizeTotal += resultSize;
        update_progress_bar(fileInfo.fsize, resultSizeTotal);

        usb_update = USB_update_move_to_next_section;
        break;
    case USB_update_move_to_next_section:     
        fatfsCode = f_lseek(&file, resultSizeTotal);
        if (fatfsCode)
        {
            usb_echo("seek error\r\n");
            usb_update = USB_update_idle;
            f_close(&file);
            return;
        }
        if (flash_img_bytes_written(&flash_ctx) >= fileInfo.fsize)
        {
            usb_update = USB_check_image;
        }
        else
        {
            usb_update = USB_update_read_file;
        }
        break;
    case USB_check_image:
        usb_echo("USB_check_image \r\n");
        check_image();
        usb_update = USB_update_idle;
        ReadyToRead = false;
        break;
    case USB_update_idle:
        // wait
        if (ReadyToRead == true)
        {
            usb_update = USB_update_mount_drive;
        }
        else
        {
            k_msleep(10);
            usb_update = USB_update_idle;
        }        
        break;
    default:
        usb_echo("Unknown state... \r\n");
        break;
    }

}

void update_failed_clean_up(void* displayString)
{
    display_read_error(displayString);   
    ReadyToRead = false;
    FileWrittenToFlash = false;
    usb_update = USB_update_idle; 
    f_close(&file);
}


void check_image(void)
{
    struct mcuboot_img_header header;
    boot_read_bank_header(FIXED_PARTITION_ID(slot1_partition), &header, sizeof(header));
    usb_echo("mcuboot_version: %d \r\n", header.mcuboot_version);
    usb_echo("image_size: %d \r\n", header.h.v1.image_size);
    usb_echo("img version: %d.%d \r\n", header.h.v1.sem_ver.major, header.h.v1.sem_ver.minor);
    usb_echo("Image check... \r\n");
    
    if(((header.h.v1.sem_ver.major == 0) && ( header.h.v1.sem_ver.minor == 0)) || (header.h.v1.image_size == 0))
    {
        usb_echo("Image invalid... \r\n");
        update_failed_clean_up("Image invalid...");
        return;
    }

    if (boot_request_upgrade(BOOT_UPGRADE_PERMANENT))
    {
        usb_echo("Upgrade request FAIL \r\n");
        return;
    }
   
    FileWrittenToFlash = true;
}

void write_to_flash(uint32_t bytesLeftToRead)
{  
    int ret = 0;

    bool flush = false;
    if (bytesLeftToRead == 0)
    {
        // whole file read now force flush/write of buffered data before reboot
        flush = true;       
    }

    ret = flash_img_buffered_write(&flash_ctx, (uint8_t *)testThroughputBuffer, THROUGHPUT_BUFFER_SIZE, flush);
    if (ret)
    {
        usb_echo("flash write failed \r\n");
        return;
    }
}

usb_status_t USB_HostMsdEvent(usb_device_handle deviceHandle,
    usb_host_configuration_handle configurationHandle,
    uint32_t eventCode)
{
    usb_status_t status = kStatus_USB_Success;
    usb_host_configuration_t* configuration;
    uint8_t interfaceIndex;
    usb_host_interface_t* interface;
    uint32_t infoValue = 0U;
    uint8_t id;

    switch (eventCode)
    {
    case kUSB_HostEventAttach:
        /* judge whether is configurationHandle supported */
        configuration = (usb_host_configuration_t*)configurationHandle;
        for (interfaceIndex = 0; interfaceIndex < configuration->interfaceCount; ++interfaceIndex)
        {
            interface = &configuration->interfaceList[interfaceIndex];
            id = interface->interfaceDesc->bInterfaceClass;
            if (id != USB_HOST_MSD_CLASS_CODE)
            {
                continue;
            }
            id = interface->interfaceDesc->bInterfaceSubClass;
            if ((id != USB_HOST_MSD_SUBCLASS_CODE_UFI) && (id != USB_HOST_MSD_SUBCLASS_CODE_SCSI))
            {
                continue;
            }
            id = interface->interfaceDesc->bInterfaceProtocol;
            if (id != USB_HOST_MSD_PROTOCOL_BULK)
            {
                continue;
            }
            else
            {
                if (g_MsdFatfsInstance.deviceState == kStatus_DEV_Idle)
                {
                    /* the interface is supported by the application */
                    g_MsdFatfsInstance.deviceHandle = deviceHandle;
                    g_MsdFatfsInstance.interfaceHandle = interface;
                    g_MsdFatfsInstance.configHandle = configurationHandle;
                    return kStatus_USB_Success;
                }
                else
                {
                    continue;
                }
            }
        }
        status = kStatus_USB_NotSupported;
        break;

    case kUSB_HostEventNotSupported:
        break;

    case kUSB_HostEventEnumerationDone:
        if (g_MsdFatfsInstance.configHandle == configurationHandle)
        {
            if ((g_MsdFatfsInstance.deviceHandle != NULL) && (g_MsdFatfsInstance.interfaceHandle != NULL))
            {
                /* the device enumeration is done */
                if (g_MsdFatfsInstance.deviceState == kStatus_DEV_Idle)
                {
                    g_MsdFatfsInstance.deviceState = kStatus_DEV_Attached;

                    USB_HostHelperGetPeripheralInformation(deviceHandle, kUSB_HostGetDevicePID, &infoValue);
                    // usb_echo("mass storage device attached:pid=0x%x", infoValue);
                    USB_HostHelperGetPeripheralInformation(deviceHandle, kUSB_HostGetDeviceVID, &infoValue);
                    // usb_echo("vid=0x%x ", infoValue);
                    USB_HostHelperGetPeripheralInformation(deviceHandle, kUSB_HostGetDeviceAddress, &infoValue);
                    // usb_echo("address=%d\r\n", infoValue);
                }
                else
                {
                    // usb_echo("not idle msd instance\r\n");
                    status = kStatus_USB_Error;
                }
            }
        }
        break;

    case kUSB_HostEventDetach:
        if (g_MsdFatfsInstance.configHandle == configurationHandle)
        {
            /* the device is detached */
            g_UsbFatfsClassHandle = NULL;
            g_MsdFatfsInstance.configHandle = NULL;
            if (g_MsdFatfsInstance.deviceState != kStatus_DEV_Idle)
            {
                usb_echo("status DETACHED \r\n");
                if (FileWrittenToFlash == true)
                {
                    NVIC_SystemReset(); //reboot, restart, powercycle, load new image
                }
                g_MsdFatfsInstance.deviceState = kStatus_DEV_Detached;
            }
        }

        break;

    default:
        break;
    }
    return status;
}
