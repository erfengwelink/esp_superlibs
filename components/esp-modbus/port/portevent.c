/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portevent.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
/* ----------------------- Variables ----------------------------------------*/
#if ( MB_DEVICE_USE_TYPE == MB_DEVICE_SLAVE)

static xQueueHandle xQueueSlaveHdl;

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortEventInit( void )
{
    BOOL    bStatus = FALSE;
    if( 0 != ( xQueueSlaveHdl = xQueueCreate( 1, sizeof( eMBEventType ) ) ) )
    {
        bStatus = TRUE;
    }
    return bStatus;
}

void
vMBPortEventClose( void )
{
    if( 0 != xQueueSlaveHdl )
    {
        vQueueDelete( xQueueSlaveHdl );
        xQueueSlaveHdl = 0;
    }
}

BOOL
xMBPortEventPost( eMBEventType eEvent )
{
    BOOL    bStatus = TRUE;
    if( bMBPortIsWithinException(  ) )
    {
        ( void )xQueueSendFromISR( xQueueSlaveHdl, ( const void * )&eEvent, pdFALSE );
    }
    else
    {
        ( void )xQueueSend( xQueueSlaveHdl, ( const void * )&eEvent, pdFALSE );
    }

    return bStatus;
}

BOOL
xMBPortEventGet( eMBEventType * eEvent )
{
    BOOL    xEventHappened = FALSE;
    if( pdTRUE == xQueueReceive( xQueueSlaveHdl, eEvent, portTICK_RATE_MS * 50 ) )
    {
        xEventHappened = TRUE;
    }
    return xEventHappened;
}

#else 

#include "mbport.h"
#include "freertos/semphr.h"

static xQueueHandle xQueueMasterHdl;
static SemaphoreHandle_t SemaphorMasterHdl;

BOOL
xMBMasterPortEventInit( void )
{
    BOOL    bStatus = FALSE;
    if( 0 != ( xQueueMasterHdl = xQueueCreate( 1, sizeof( eMBMasterEventType ) ) ) )
    {
        bStatus = TRUE;
    }
    return bStatus;
}

void
vMBMasterPortEventClose( void )
{
    if( 0 != xQueueMasterHdl )
    {
        vQueueDelete( xQueueMasterHdl );
        xQueueMasterHdl = 0;
    }
}

BOOL
xMBMasterPortEventPost( eMBMasterEventType eEvent)
{
    BOOL    bStatus = TRUE;
    if( bMBPortIsWithinException(  ) )
    {
        ( void )xQueueSendFromISR( xQueueMasterHdl, ( const void * )&eEvent, pdFALSE );
    }
    else
    {
        ( void )xQueueSend( xQueueMasterHdl, ( const void * )&eEvent, pdFALSE );
    }

    return bStatus;
}

BOOL
xMBMasterPortEventGet( eMBMasterEventType * eEvent)
{
    BOOL    xEventHappened = FALSE;
    if( pdTRUE == xQueueReceive( xQueueMasterHdl, eEvent, portTICK_RATE_MS * 50 ) )
    {
        xEventHappened = TRUE;
    }
    return xEventHappened;
}

void vMBMasterErrorCBRespondTimeout(UCHAR ucDestAddress, const UCHAR* pucPDUData, USHORT ucPDULength) 
{
    BOOL ret = xMBMasterPortEventPost(EV_MASTER_ERROR_RESPOND_TIMEOUT);
    if(!ret)
        printf("xMBMasterPortEventPost event 'EV_MASTER_ERROR_RESPOND_TIMEOUT' failed!!!\r\n");

}
void vMBMasterCBRequestScuuess( void ) 
{
    BOOL ret = xMBMasterPortEventPost(EV_MASTER_PROCESS_SUCESS);
    if(!ret)
        printf("xMBMasterPortEventPost event 'EV_MASTER_PROCESS_SUCESS' failed!!!\r\n");
}

void vMBMasterErrorCBReceiveData(UCHAR ucDestAddress, const UCHAR* pucPDUData, USHORT ucPDULength) 
{
    BOOL ret = xMBMasterPortEventPost(EV_MASTER_ERROR_RECEIVE_DATA);
    if(!ret)
        printf("xMBMasterPortEventPost event 'EV_MASTER_ERROR_RECEIVE_DATA' failed!!!\r\n");

}

void vMBMasterErrorCBExecuteFunction(UCHAR ucDestAddress, const UCHAR* pucPDUData, USHORT ucPDULength) 
{
   BOOL ret = xMBMasterPortEventPost(EV_MASTER_ERROR_EXECUTE_FUNCTION);
    if(!ret)
        printf("xMBMasterPortEventPost event 'EV_MASTER_ERROR_EXECUTE_FUNCTION' failed!!!\r\n");

}

eMBMasterReqErrCode eMBMasterWaitRequestFinish( void ) 
{
    eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;
    eMBMasterEventType recvedEvent;
    xMBMasterPortEventGet( &recvedEvent );
    switch (recvedEvent)
    {
        case EV_MASTER_PROCESS_SUCESS:
            break;
        case EV_MASTER_ERROR_RESPOND_TIMEOUT:
        {
            eErrStatus = MB_MRE_TIMEDOUT;
            break;
        }
        case EV_MASTER_ERROR_RECEIVE_DATA:
        {
            eErrStatus = MB_MRE_REV_DATA;
            break;
        }
        case EV_MASTER_ERROR_EXECUTE_FUNCTION:
        {
            eErrStatus = MB_MRE_EXE_FUN;
            break;
        }
        default:
            break;

    }
    return eErrStatus;
}

BOOL xMBMasterRunResTake( LONG lTimeOut )
{
    /*If waiting time is -1 .It will wait forever */
    if( xSemaphoreTake( SemaphorMasterHdl, lTimeOut ) == pdTRUE )  
    {  
        //printf("%s:xSemaphoreTake ok\r\n", __func__);  
        return FALSE;
    }  
    return TRUE;
}

void vMBMasterRunResRelease( void )
{
    static BaseType_t xHigherPriorityTaskWoken;  
    xSemaphoreGiveFromISR( SemaphorMasterHdl, &xHigherPriorityTaskWoken );  
}

void vMBMasterOsResInit( void )
{
    SemaphorMasterHdl = xSemaphoreCreateBinary();  
}

#endif