/*!
 *	\file		extIg.h
 *  \author		SBG-Systems (Alexis Guinamard)
 *	\date		29/11/10
 *
 *	\brief		Remote IG-Device specific commands implementation<br>
 *				Copyright 2007-2010 SBG Systems. All rights reserved.
 */
#ifndef __EXTIG_H__
#define __EXTIG_H__

#include "../../sbgCommon.h"
#include "../protocol.h"

//----------------------------------------------------------------------//
//- IG-Device definition                                                -//
//----------------------------------------------------------------------//

#define SBG_IG_AUTO_OFFSET_XY		(0x04)					/*!< Automatic offset in XY axes between the IG-Device and the IG-500E */

//----------------------------------------------------------------------//
//- IG-Device Types definitions                                        -//
//----------------------------------------------------------------------//

/*!
 * IG-Device protocol external configuration commands
 */
typedef enum _SbgIgdCommand
{
	SBG_EXT_CMD_IG_ACK							= 0x00,		/*!< Acknowledge returned by the IG-Device management module */
	SBG_EXT_CMD_IG_SET_AUTO_OFFSET				= 0x01,		/*!< Set an automatic offset (XY axes) between the IG-Device and the IG-500E */
	SBG_EXT_CMD_IG_SET_MATRIX_OFFSET			= 0x02,		/*!< Set a manual rotation matrix offset for device alignment */
	SBG_EXT_CMD_IG_GET_MATRIX_OFFSET			= 0x03,		/*!< Get a manual rotation matrix offset for device alignment */
	SBG_EXT_CMD_IG_RET_MATRIX_OFFSET			= 0x04,		/*!< Return a manual rotation matrix offset for device alignment */
	SBG_EXT_CMD_IG_SEND_REMOTE_FRAME			= 0x05,		/*!< Send an encapsulated frame to be sent to the remote IG-Device */
	SBG_EXT_CMD_IG_RET_REMOTE_FRAME				= 0x06,		/*!< Return the encapsulated frame received from the remote IG-Device */
} SbgIgdCommand;

//----------------------------------------------------------------------//
//- Remote IG-Device operations                                        -//
//----------------------------------------------------------------------//

/*!
 *	Set an automatic orientation offset between the IG-500E and the IG-Device to realign them
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *  \param[in]	type				Offset type. Value allowed is:
 *									- SBG_IG_AUTO_OFFSET_XY
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtIgSetAutoOffset(SbgProtocolHandle handle, bool permanent, uint8 type);

/*!
 *	Set a manual orientation offset between the IG-500E and remote device, in a matrix form
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *	\param[in]	matrixOffset		orientation offset between the IG-Device and IG-500E
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtIgSetMatrixOffset(SbgProtocolHandle handle, bool permanent, const float matrixOffset[9]);

/*!
 *	Get the offset between the IG-500E orientation and the IG-Device orientation. <br>
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[out]	matrixOffset		Orientation offset between the remote IG-Device and IG-500E
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtIgGetMatrixOffset(SbgProtocolHandle handle, float matrixOffset[9]);

/*!
 *	Send an encapsulated frame to the remote IG-Device, and return its answer
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	cmdIn				Command to be sent to the remote IG-Device
 *  \param[in]	sizeIn				Size of the frame to be sent
 *  \param[in]	pDataIn				Pointer to the data to be sent to the IG-Device
 *  \param[out]	pCmdOut				Pointer to the command returned by the Remote IG-Device
 *  \param[out]	pSizeOut			Pointer to the actual data size returned by the remote device
 *  \param[out]	pDataOut			Pointer to the data output buffer returned by the remote IG-Device
 *  \param[in]	maxSizeOut			Maximum output size allowed in pDataOut field
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtIgEncapsulatedFrame(SbgProtocolHandle handle, uint8 cmdIn, uint16 sizeIn, const void *pDataIn, 
									   uint8 *pCmdOut, uint16 *pSizeOut, void *pDataOut, uint16 maxSizeOut);

#endif
