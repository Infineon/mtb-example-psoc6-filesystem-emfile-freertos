/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the emFile Filesystem on SD Card
*              and QSPI NOR Flash code example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FS.h"
#include <string.h>

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"

/*******************************************************************************
* Macros
********************************************************************************/

/* Last byte in the buffer is used for terminating the string using a NULL
 * character. Therefore, the number of bytes read from the file is one less than
 * the value of the macro.
 */
#define NUM_BYTES_TO_READ_FROM_FILE         (256U)
#define STRING_TO_WRITE                     "This is an emFile filesystem example for ModusToolbox."
#define FILE_NAME                           "File.txt"

#define EMFILE_TASK_STACK_SIZE              (512U)
#define USER_BUTTON_INTERRUPT_PRIORITY      (7U)

/* Debounce delay for the user button. */
#define DEBOUNCE_DELAY_MS                   (50U)


/*******************************************************************************
 * Global Variables
 ******************************************************************************/
static char fileData[NUM_BYTES_TO_READ_FROM_FILE];
static TaskHandle_t emfile_task_handle;

/* This enables RTOS aware debugging */
volatile int uxTopUsedPriority;


/*******************************************************************************
* Function Name: check_error
****************************************************************************//**
* Summary:
*  Prints the message and halts the execution by running an infinite loop, when
*  the error value is negative.
*
* Parameters: 
*  message - message to print if error value is negative.
*  error - error value for evaluation.
*
*******************************************************************************/
static void check_error(char *message, int error)
{
    if (error < 0)
    {
        printf("\n================================================================================\n");
        printf("\nFAIL: %s\n", message);
        printf("Error Value: %d\n", error);
        printf("emFile-defined Error Message: %s", FS_ErrorNo2Text(error));
        printf("\n================================================================================\n");
        
        while(true);
    }
}


/*******************************************************************************
* Function Name: user_button_interrupt_handler
********************************************************************************
* Summary:
*   User button interrupt handler.
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_gpio_irq_event_t (unused)
*
*******************************************************************************/
static void user_button_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    (void) handler_arg;
    (void) event;

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(emfile_task_handle, &higher_priority_task_woken);

    /* Yield if xHigherPriorityTaskWoken was set to true */
    portYIELD_FROM_ISR( higher_priority_task_woken );
}


/*******************************************************************************
* Function Name: emfile_task
********************************************************************************
* Summary:
*   Formats the storage device, reads the content from a file and prints the
*   content to the UART terminal, writes a message to the same file, and waits
*   for the user button press. When the button is pressed, deletes the file and
*   returns.
*
* Parameters:
*  arg - Unused.
*
*******************************************************************************/
static void emfile_task(void* arg)
{
    U32         varU32;
    U32         numBytesToRead;
    int         error;
    FS_FILE    *filePtr;
    const char *volumeName = "";

#if defined(USE_SD_CARD)
    printf("Using SD card as storage device\n");
#else
    printf("Using NOR flash as storage device\n");
#endif /* #if defined(USE_SD_CARD) */

    /* Initialize the file system. */
    FS_Init();

#if !defined(USE_SD_CARD)
    /* Check if low-level format is required. Applicable only for NOR flash. */
    error = FS_FormatLLIfRequired(volumeName);
    check_error("Error in low-level formatting", error);
#endif /* #if !defined(USE_SD_CARD) */

    /* Check if volume needs to be high-level formatted. */
    error = FS_IsHLFormatted(volumeName);
    check_error("Error in checking if volume is high-level formatted", error);

    /* Return value of 0 indicates that high-level format is required. */
    if (error == 0)
    {
        printf("Perform high-level format\n");
        error = FS_Format(volumeName, NULL);
        check_error("Error in high-level formatting", error);
    }

    varU32 = FS_GetVolumeSizeKB(volumeName);
    printf("Volume size: %lu KB\n\n", varU32);

    if(0U == varU32)
    {
        printf("Error in checking the volume size\n");
        CY_ASSERT(0U);
    }

    printf("Opening the file for reading...\n");

    /* Open the file for reading. */
    filePtr = FS_FOpen(FILE_NAME, "r");

    if (filePtr != NULL)
    {
        /* Last byte is for storing the NULL character. */
        numBytesToRead = sizeof(fileData) - 1U;
        varU32 = FS_GetFileSize(filePtr);

        if(varU32 < numBytesToRead)
        {
            numBytesToRead = varU32;
        }

        printf("Reading %lu bytes from the file. ", numBytesToRead);
        varU32 = FS_Read(filePtr, fileData, numBytesToRead);

        if(varU32 != numBytesToRead)
        {
            error = FS_FError(filePtr);
            check_error("Error in reading from the file", error);
        }
        
        /* Terminate the string using NULL. */
        fileData[numBytesToRead] = '\0';

        /* Display the file content. */
        printf("File Content:\n\"%s\"\n", fileData);

        error = FS_FClose(filePtr);
        check_error("Error in closing the file", error);
        
        printf("\nOpening the file for overwriting...\n");
    }
    else
    {
        printf("Unable to read. File not found.\n");
        printf("\nOpening the file for writing...\n");
    }
    
    /* Mode 'w' truncates the file size to zero if the file exists otherwise
     * creates a new file.
     */
    filePtr = FS_FOpen(FILE_NAME, "w");

    if(filePtr != NULL)
    {
        varU32 = FS_Write(filePtr, STRING_TO_WRITE, strlen(STRING_TO_WRITE));

        if(varU32 != strlen(STRING_TO_WRITE))
        {
            error = FS_FError(filePtr);
            check_error("Error in writing to the file", error);
        }

        printf("File is written with the following message:\n");
        printf("\"%s\"\n\n", STRING_TO_WRITE);

#if defined(USE_SD_CARD)
        printf("You can now view the file content in your PC. File name is \"%s\"\n", FILE_NAME);
#endif /* #if defined(USE_SD_CARD) */

        error = FS_FClose(filePtr);
        check_error("Error in closing the file", error);

        /* Enable the user button interrupt */
        cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, USER_BUTTON_INTERRUPT_PRIORITY, true);

        printf("\nPress the user button to delete the file or press reset to run the example again.\n\n");

        /* Wait until the user button press is notified through the interrupt */
        while (true)
        {
            if(1UL == ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
            {
                /* Debounce the button press. */
                vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));

                if(!cyhal_gpio_read(CYBSP_USER_BTN)) { break; }
            }
        }

        printf("Deleting the file... \n");

        /* User button is pressed. Delete the file. */
        error = FS_Remove(FILE_NAME);
        check_error("Error in deleting the file", error);

        FS_Unmount(volumeName);

        printf("Filesystem operations completed successfully!\n");
        printf("Press reset to the run the example again.\n");
    }
    else
    {
        printf("Unable to open the file for writing! Exiting...\n");
    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU. It does...
*    1. Initializes the UART for redirecting printf output
*    2. Intializes the user button GPIO
*    3. Creates a FreeRTOS task to perform file I/O operations
*    4. Starts the FreeRTOS scheduler
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* This enables RTOS aware debugging in OpenOCD */
    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    CY_ASSERT (result == CY_RSLT_SUCCESS);

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT (result == CY_RSLT_SUCCESS);

    /* Initialize the user button used for erasing the block device. */
    result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    CY_ASSERT (result == CY_RSLT_SUCCESS);

    /* Configure & the user button interrupt */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, user_button_interrupt_handler, NULL);

    /* Enable global interrupts */
    __enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("************* "
           "emFile FAT Filesystem on SD Card and QSPI NOR Flash "
           "************* \n\n");
    
    /* Create the user tasks. See the respective task definition for more
     * details of these tasks.
     */
    xTaskCreate(emfile_task, "emFile Task", EMFILE_TASK_STACK_SIZE,
                NULL, (configMAX_PRIORITIES - 1), &emfile_task_handle);

    /* Start the RTOS scheduler. This function should never return */
    vTaskStartScheduler();

    (void) result; /* To avoid compiler warning */

    for (;;)
    {
    }
}

/* [] END OF FILE */
