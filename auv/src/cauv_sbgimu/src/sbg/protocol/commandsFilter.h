/*!
 *	\file		commandsFilter.h
 *  \author		SBG-Systems (Raphael Siryani)
 *	\date		02/04/07
 *
 *	\brief		Filter related commands used to define Kalman Filter options and sensors filtering settings.<br>
 *				Copyright 2007-2008 SBG Systems. All rights reserved.
 */
#ifndef _COMMANDS_FILTER_H_
#define _COMMANDS_FILTER_H_

#include "../sbgCommon.h"
#include "protocol.h"

//--------------------------------------------------------------------------------//
// Masks definitons about kalman filter                                           //
//--------------------------------------------------------------------------------//
#define SBG_FILTER_OPTION_FILTER_ACCELERATIONS			(0x01)	/*!< If enabled, the device has a better immunity when large accelerations occurs. */
#define SBG_FILTER_OPTION_FILTER_MAG_DISTORTIONS		(0x02)	/*!< This option reduces heading errors in disturbed magnetic environements. */
#define SBG_FILTER_OPTION_ESTIMATE_GYRO_BIAS			(0x04)	/*!< Enable the gyroscopes bias estimation. */
#define SBG_FILTER_OPTION_ESTIMATE_ACCEL_BIAS			(0x08)	/*!< Enable the acclerometrs bias estimation */
#define SBG_FILTER_OPTION_ENABLE_ATTITUDE				(0x10)	/*!< Enable the attitude/navigation computation.<br>If disabled, the device can outputs calibrated data at 512 Hz. */

/*!
 *	List of available sources for heading corrections.
 */
typedef enum _SbgHeadingSource
{
	SBG_HEADING_SOURCE_NONE							=	0x00,	/*!< Our heading is calculated using gyroscopes only. */
	SBG_HEADING_SOURCE_MAGNETOMETERS				=	0x01,	/*!< Our heading is based on magnetometers information and gyroscopes. */
	SBG_HEADING_SOURCE_GPS_COURSE					=	0x02,	/*!< GPS course is used */
	SBG_HEADING_SOURCE_USER							=	0x05,	/*!< Heading is fed by user via the main protocol */
	SBG_HEADING_SOURCE_REMOTE_MAG					=	0x06,	/*!< Remote magnetometers used as a heading source */
	SBG_HEADING_SOURCE_REMOTE_TRUE_HEADING			=	0x07	/*!< Remote true heading sensor used (dual antenna)  */
} SbgHeadingSource;

//----------------------------------------------------------------------//
//  Kalman Filter commands                                              //
//----------------------------------------------------------------------//

/*!
 *	Defines our Kalman Filter errors coefficiens.<br>
 *	Using this command, you can balance between accelerometers/gyroscopes and magnetometers/gyroscopes<br>
 *	as the attitude estimation is concerned.<br>
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	permanent				If true, stores this settings in flash memory.
 *	\param[in]	accelerometersError		Errors between 0.1 to 10.0 associated with our accelerometers.<br>
 *										If this settings is set to 0.1, the device will be more accurate in static conditions but less in dynamic one.<br>
 *										With a setting near 10.0, the device will be less sensitive to transients accelerations in dynamic conditions<br>
 *	\param[in]	magnetometersError		Errors between 0.1 to 10.0 associated with our magnetometers.<br>
 *										If this settings is set to 0.1, the device will be more accurate in static conditions but less in dynamic one.<br>
 *										With a setting near 10.0, the device will be less sensitive to magnetic disturbances and in dynamic conditions<br>
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetFilterAttitudeCoef(SbgProtocolHandle handle, bool permanent, float accelerometersError, float magnetometersError);

/*!
 *	Retreives our Kalman Filter errors coefficiens.<br>
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pAccelerometersError	Errors between 0.1 to 10.0 associated with our accelerometers.<br>
 *										If this settings is set to 0.1, the device will be more accurate in static conditions but less in dynamic one.<br>
 *										With a setting near 10.0, the device will be less sensitive to transients accelerations in dynamic conditions<br>
 *	\param[out]	pMagnetometersError		Errors between 0.1 to 10.0 associated with our magnetometers.<br>
 *										If this settings is set to 0.1, the device will be more accurate in static conditions but less in dynamic one.<br>
 *										With a setting near 10.0, the device will be less sensitive to magnetic disturbances and in dynamic conditions<br>
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetFilterAttitudeCoef(SbgProtocolHandle handle, float *pAccelerometersError, float *pMagnetometersError);

/*!
 *	Defines some options regarding the Kalman Filter.<br>
 *	It's possible, for example, to enable/disable the attitude computation or to enable/disable gyro-bias estimation.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	permanent				If true, stores this settings in flash memory.
 *	\param[in]	filterOptions			Our Kalman filter options mask.<br>
 *										Possible values are:<br>
 *										- #SBG_FILTER_OPTION_FILTER_ACCELERATIONS
 *										- #SBG_FILTER_OPTION_FILTER_MAG_DISTORTIONS
 *										- #SBG_FILTER_OPTION_ESTIMATE_GYRO_BIAS
 *										- #SBG_FILTER_OPTION_ESTIMATE_ACCEL_BIAS
 *										- #SBG_FILTER_OPTION_ENABLE_ATTITUDE
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetFilterAttitudeOptions(SbgProtocolHandle handle, bool permanent, uint32 filterOptions);

/*!
 *	Retreives some options regarding the Kalman Filter.<br>
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pFilterOptions			Our Kalman filter options mask.<br>
 *										Possible values are:<br>
 *										- #SBG_FILTER_OPTION_FILTER_ACCELERATIONS
 *										- #SBG_FILTER_OPTION_FILTER_MAG_DISTORTIONS
 *										- #SBG_FILTER_OPTION_ESTIMATE_GYRO_BIAS
 *										- #SBG_FILTER_OPTION_ESTIMATE_ACCEL_BIAS
 *										- #SBG_FILTER_OPTION_ENABLE_ATTITUDE
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetFilterAttitudeOptions(SbgProtocolHandle handle, uint32 *pFilterOptions);

/*!
 *	Defines our sensors filter cut-off frequencies and the update rate for the Kalman Filter.<br>
 *	If you set a setting to 0.0, the value will remain unchanged.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	permanent				If true, stores this settings in flash memory.
 *	\param[in]	gyroAccelsSampling		Reserved for backward compatibility. Leave to 0.
 *	\param[in]	cutoffGyro				Gyroscopes low-pass filter cut-off frequency in Hz.
 *	\param[in]	cutoffAccel				Accelerometers low-pass filter cut-off frequency in Hz.
 *	\param[in]	cutoffMagneto			Magnetometers low-pass filter cut-off frequency in Hz.
 *	\param[in]	kalmanFreq				Our Kalman filter refresh rate.<br>
 *										Max 100 Hz for IG-500N and 160 Hz for IG-500A.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetFilterFrequencies(SbgProtocolHandle handle, bool permanent, float gyroAccelsSampling, float cutoffGyro, float cutoffAccel, float cutoffMagneto, float kalmanFreq);

/*!
 *	Retrives our sensors filter cut-off frequencies and the update rate for the Kalman Filter.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pGyroAccelsSampling		Our accelerometers and gyroscopes sampling frequency in Hz.
 *	\param[out]	pCutoffGyro				Gyroscopes low-pass filter cut-off frequency in Hz.
 *	\param[out]	pCutoffAccel			Accelerometers low-pass filter cut-off frequency in Hz.
 *	\param[out]	pCutoffMagneto			Magnetometers low-pass filter cut-off frequency in Hz.
 *	\param[out]	pKalmanFreq				Our Kalman filter refresh rate.<br>
 *										Max 100 Hz for IG-500N and 160 Hz for IG-500A.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetFilterFrequencies(SbgProtocolHandle handle, float *pGyroAccelsSampling, float *pCutoffGyro, float *pCutoffAccel, float *pCutoffMagneto, float *pKalmanFreq);

/*!
 *	Defines the kalman filter source for heading estimate.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	permanent				If true, stores this settings in flash memory.
 *	\param[in]	source					Source used in heading calculation.<br>
 *										Possible values are:<br>
 *										- SBG_HEADING_SOURCE_NONE, unavailable for IG-500N devices<br>
 *										- SBG_HEADING_SOURCE_MAGNETOMETERS<br>
 *										- SBG_HEADING_SOURCE_GPS, only for IG-30G and IG-500N devices<br>
 *										- SBG_HEADING_SOURCE_EXTERNAL<br>
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetFilterHeadingSource(SbgProtocolHandle handle, bool permanent, SbgHeadingSource source);

/*!
 *	Retrives the kalman filter source for heading estimate.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pSource					Source used in heading calculation.<br>
 *										Possible values are:<br>
 *										- SBG_HEADING_SOURCE_NONE, unavailable for IG-500N devices<br>
 *										- SBG_HEADING_SOURCE_MAGNETOMETERS<br>
 *										- SBG_HEADING_SOURCE_GPS, only for IG-30G and IG-500N devices<br>
 *										- SBG_HEADING_SOURCE_EXTERNAL<br>
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetFilterHeadingSource(SbgProtocolHandle handle, SbgHeadingSource *pSource);

/*!
 *	Defines the magnetic declination in radians.<br>
 *	The declination is important for good results in navigation estimation with IG-500N devices.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	permanent				If true, stores this settings in flash memory.
 *	\param[in]	declination				The local magnetic declination in radians from -PI to +PI.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetMagneticDeclination(SbgProtocolHandle handle, bool permanent, float declination);

/*!
 *	Returns the magnetic declination in radians.<br>
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pDeclination			The local magnetic declination in radians from -PI to +PI.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetMagneticDeclination(SbgProtocolHandle handle, float *pDeclination);

/*!
 *	Send a new heading inforamtion to our Kalman filter.
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[in]	heading					The new heading in radians.
 *	\param[in]	accuracy				Our heading accuracy in radians.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSendFilterHeading(SbgProtocolHandle handle, float heading, float accuracy);
#endif
