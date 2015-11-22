/*!
 *	\file		commandsNav.h
 *  \author		SBG-Systems (Alexis Guinamard)
 *	\date		25/07/08
 *
 *	\brief		Commands used to define navigation options on IG-500N and IG-500E<br>
 *				 Copyright 20010 SBG Systems. All rights reserved.
 */
#ifndef __COMMANDS_NAV_H__
#define __COMMANDS_NAV_H__

#include "../sbgCommon.h"
#include "protocol.h"
#include "commands.h"

//----------------------------------------------------------------------//
//-  Definitions concerning GPS                                        -//
//----------------------------------------------------------------------//

#define SBG_SV_USED_FOR_NAV						(0x01)			/*!< SV is used for navigation */
#define SBG_SV_DIFF_AVAILABLE					(0x02)			/*!< Differential correction data is available for this SV */
#define SBG_SV_ORBIT_AVAILABLE					(0x04)			/*!< Orbit information is available for this SV (Ephemeris or Almanach) */
#define SBG_SV_ORBIT_EPHEMERIS					(0x08)			/*!< Orbit information is Ephemeris */
#define SBG_SV_UNHEALTHY						(0x10)			/*!< SV is unhealthy / shall not be used */

#define SBG_SV_QUAL_IDLE						(0x00)			/*!< This channel is idle */
#define SBG_SV_QUAL_SEARCHING_1					(0x01)			/*!< Channel is searching */
#define SBG_SV_QUAL_SERACHING_2					(0x02)			/*!< Channel is searching */
#define SBG_SV_QUAL_DETECTED_UNUSABLE			(0x03)			/*!< Signal detected but unusable */
#define SBG_SV_QUAL_CODE_LOCK_ON				(0x04)			/*!< Code Lock on Signal */
#define SBG_SV_QUAL_CODE_AND_CARRIER_LOCKED_1	(0x05)			/*!< Code and Carrier locked */
#define SBG_SV_QUAL_CODE_AND_CARRIER_LOCKED_2	(0x06)			/*!< Code and Carrier locked */
#define SBG_SV_QUAL_RECEIVING_DATA				(0x07)			/*!< Code and Carrier locked, receiving 50bps data */

//----------------------------------------------------------------------//
//- Definition concerning velocity constraints                         -//
//----------------------------------------------------------------------//

#define SBG_CONSTRAINT_X						(0x0001)		/*!< Bit mask for a constraint on X axis (for example, Vx = 0) */
#define SBG_CONSTRAINT_Y						(0x0002)		/*!< Bit mask for a constraint on Y axis (for example, Vy = 0) */
#define SBG_CONSTRAINT_Z						(0x0004)		/*!< Bit mask for a constraint on Z axis (for example, Vz = 0) */

/*!
 *	Defines our constraints error level.
 */
typedef enum _SbgConstraintError
{
	SBG_CONSTRAINT_STRICT		= 0,							/*!< Defines a strict constraint ( rapidly recenter parameter to 0 ) */
	SBG_CONSTRAINT_NORMAL		= 1,							/*!< Defines a normal constraint */
	SBG_CONSTRAINT_LOOSE		= 2								/*!< Defines a loose constraint ( slowly recenter parameter to 0 ) */
} SbgConstraintError;

//----------------------------------------------------------------------//
//- Definition concerning GPS and navigation                           -//
//----------------------------------------------------------------------//

/*!
 *	GPS Space vehicles information struct.<br>
 *	Angles are in 32/45th of degrees.
 */
typedef struct _SbgGpsSVInfo
{
	uint8	satelliteID;					/*!< ID of the satellite followed */
	uint8	flagsQuality;					/*!< flags and signal quality indicator (quality bits 7,6,5 flags 4,3,2,1,0) */
	uint8	signalStrength;					/*!< Carrier to noise Ratio */
	int8	azimuth;						/*!< Azimuth of the SV(signed) (1LSB = 32/45 degrees) */
	int8	elevation; 						/*!< Elevation of the SV(signed) (1LSB = 32/45 degrees) */
} SbgGpsSVInfo;

/*!
 *	Define the position aiding source to use in the Kalman Naviagation filter.
 */
typedef enum _SbgAidingPosSrc
{
	SBG_POS_SRC_GPS				= 0x00,		/*!< We are using the GPS position as the aiding source for the navigation filter */
	SBG_POS_SRC_GPS_AND_BARO	= 0x01,		/*!< We are using the GPS position and the barometric altitude as the aiding source for the navigation filter */
	SBG_POS_SRC_USER			= 0x02		/*!< We are using the user provided source for position as the aiding source for the navigation filter */
} SbgAidingPosSrc;

/*!
 *	Define the velocity aiding source to use in the Kalman Naviagation filter.
 */
typedef enum _SbgAidingVelSrc
{
	SBG_VEL_SRC_GPS				= 0x00,		/*!< We are using the GPS velocity as the aiding source for the navigation filter */
	SBG_VEL_SRC_USER			= 0x02,		/*!< We are using the a user provided source for velocity as the aiding source for the navigation filter */
	SBG_VEL_SRC_ODO				= 0x03,		/*! We are using the external odometer as velocity source for the navigation filter */
} SbgAidingVelSrc;

//----------------------------------------------------------------------//
//- GPS relative commands                                              -//   
//----------------------------------------------------------------------//

/*!
 *	Get the advanced information about Space Vehicles seen by GPS. (IG-30G and IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[out]	pSvInfo				Pointer to an array of SbgGpsSVInfo structure that will contain SV information (std size 16 channels)
 *	\param[out]	pNbSV				Pointer to the number of satellites managed
 *	\param[in]	maxNbSv				Maximum number of nbSv that can be contained in our pSvInfo list.
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgGetSvInfo(SbgProtocolHandle handle, SbgGpsSVInfo *pSvInfo, uint8 *pNbSV, uint8 maxNbSv);

/*!
 *	Defines which source to use to aid the velocity in our navigation filter. (IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *  \param[in]	aidingSrc			The aiding source to use in our navigation filter.<br>
 *									Valid options are:<br>
 *									- SBG_SRC_GPS<br>
 *									- SBG_SRC_GPS_AND_BARO<br>
 *									- SBG_SRC_EXTERNAL<br>
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetNavVelocitySrc(SbgProtocolHandle handle, bool permanent, SbgAidingVelSrc aidingSrc);

/*!
 *	Returns which source is used to aid the velocity in our navigation filter. (IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	pAidingSrc			The aiding source used in our navigation filter.<br>
 *									Valid options are:<br>
 *									- SBG_SRC_GPS<br>
 *									- SBG_SRC_GPS_AND_BARO<br>
 *									- SBG_SRC_EXTERNAL<br>
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetNavVelocitySrc(SbgProtocolHandle handle, SbgAidingVelSrc *pAidingSrc);

/*!
 *	Defines which source to use to aid the position in our navigation filter. (IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *  \param[in]	aidingSrc			The aiding source to use in our navigation filter.<br>
 *									Valid options are:<br>
 *									- SBG_SRC_GPS<br>
 *									- SBG_SRC_GPS_AND_BARO<br>
 *									- SBG_SRC_EXTERNAL<br>
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetNavPositionSrc(SbgProtocolHandle handle, bool permanent, SbgAidingPosSrc aidingSrc);

/*!
 *	Returns which source is used to aid the position in our navigation filter. (IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	pAidingSrc			The aiding source used in our navigation filter.<br>
 *									Valid options are:<br>
 *									- SBG_SRC_GPS<br>
 *									- SBG_SRC_GPS_AND_BARO<br>
 *									- SBG_SRC_EXTERNAL<br>
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetNavPositionSrc(SbgProtocolHandle handle, SbgAidingPosSrc *pAidingSrc);

/*!
 *	Defines the GPS lever arm. (IG-500N only).<br>
 *	Use this command to specifiy the vector between the device and the GPS antenna.<br>
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *  \param[in]	gpsLeverArm			X,Y,Z vector representing the distance between the device and the GPS antenna.<br>
 *									The distance is exressed in meters in the device coordinate system.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetGpsLeverArm(SbgProtocolHandle handle, bool permanent, const float gpsLeverArm[3]);

/*!
 *	Returns the GPS lever arm. (IG-500N only).<br>
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	gpsLeverArm			X,Y,Z vector representing the distance between the device and the GPS antenna.<br>
 *									The distance is exressed in meters in the device coordinate system.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetGpsLeverArm(SbgProtocolHandle handle, float gpsLeverArm[3]);

/*!
 *	Defines the local gravity magnitude. (IG-500N only).
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *  \param[in]	gravityMagnitude	The local gravity magnitude in m.s^-2.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetGravityMagnitude(SbgProtocolHandle handle, bool permanent, float gravityMagnitude);

/*!
 *	Returns the local gravity magnitude. (IG-500N only).<br>
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	pGravityMagnitude	The local gravity magnitude in m.s^-2.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetGravityMagnitude(SbgProtocolHandle handle, float *pGravityMagnitude);

/*!
 *	Defines the constraints applied on velocity. If an axis is constrained, its velocity will be recentered to 0 by the kalman filter
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *  \param[in]	constraints			Bitmask for the constraints to apply: CONSTRAINT_X, CONSTRAINT_Y and CONSTRAINT_Z can be set
 *  \param[in]	errorLevel			Error factor on the constraint applied: SBG_CONSTRAINT_STRICT, SBG_CONSTRAINT_NORMAL or SBG_CONSTRAINT_LOOSE
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetVelocityConstraints(SbgProtocolHandle handle, bool permanent, uint16 constraints, SbgConstraintError errorLevel);

/*!
 *	Returns the Constraints applied on velocity. If an axis is constrained, its velocity will be recentered to 0 by the kalman filter
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	pConstraints		Bitmask for the constraints to apply: CONSTRAINT_X, CONSTRAINT_Y and CONSTRAINT_Z can be set
 *  \param[out]	pErrorLevel			Error factor on the constraint applied: SBG_CONSTRAINT_STRICT, SBG_CONSTRAINT_NORMAL or SBG_CONSTRAINT_LOOSE
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetVelocityConstraints(SbgProtocolHandle handle, uint16 *pConstraints, SbgConstraintError *pErrorLevel);

/*!
 *	Send a new velocity information to our Navigation filter.
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	velocity			The new X,Y,Z velocity that should be used by the navigation filter in m/s.
 *	\param[in]	accuracy			The velocity accuracy in m/s.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSendNavVelocity(SbgProtocolHandle handle, const float velocity[3], float accuracy);

/*!
 *	Send a new position information to our Navigation filter.
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	position			The new WGS84 position : latitude, longitude and altitude (above ellipsoid) in [deg, deg, meters].
 *	\param[in]	hAccuracy			The horizontal accuracy in meters.
 *	\param[in]	vAccuracy			The vertical accuracy in meters.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSendNavPosition(SbgProtocolHandle handle, const double position[3], float hAccuracy, float vAccuracy);
#endif
