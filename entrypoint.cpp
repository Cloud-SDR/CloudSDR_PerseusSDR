/*
#==========================================================================================
# + + +   This Software is released under the "Simplified BSD License"  + + +
# Copyright 2014 F4GKR Sylvain AZARIAN . All rights reserved.
#
#Redistribution and use in source and binary forms, with or without modification, are
#permitted provided that the following conditions are met:
#
#   1. Redistributions of source code must retain the above copyright notice, this list of
#	  conditions and the following disclaimer.
#
#   2. Redistributions in binary form must reproduce the above copyright notice, this list
#	  of conditions and the following disclaimer in the documentation and/or other materials
#	  provided with the distribution.
#
#THIS SOFTWARE IS PROVIDED BY Sylvain AZARIAN F4GKR ``AS IS'' AND ANY EXPRESS OR IMPLIED
#WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
#FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Sylvain AZARIAN OR
#CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#The views and conclusions contained in the software and documentation are those of the
#authors and should not be interpreted as representing official policies, either expressed
#or implied, of Sylvain AZARIAN F4GKR.
#
# Adds PerseusSDR capability to SDRNode
#==========================================================================================
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <pthread.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "entrypoint.h"
#define DEBUG_DRIVER (0)
#define STAGES_COUNT (1)
#define BUFSIZ 512

#include "perseus/perseus-sdr.h"

char *driver_name ;
void* acquisition_thread( void *params ) ;

typedef struct __attribute__ ((__packed__)) _sCplx
{
    float re;
    float im;
} TYPECPX;

struct t_sample_rates {
    unsigned int *sample_rates ;
    int enum_length ;
    int preffered_sr_index ;
};

// this structure stores the device state
struct t_rx_device {
    perseus_descr *device ;
    char *device_name ;
    char *device_serial_number ;

    struct t_sample_rates* rates;
    int current_sample_rate ;

    int64_t min_frq_hz ; // minimal frequency for this device
    int64_t max_frq_hz ; // maximal frequency for this device
    int64_t center_frq_hz ; // currently set frequency

    float gainv[STAGES_COUNT] ;

    char *uuid ;
    bool running ;
    bool acq_stop ;

    // for DC removal
    TYPECPX xn_1 ;
    TYPECPX yn_1 ;
    struct ext_Context ext_context ;


};

int device_count ;
char *stage_name[STAGES_COUNT] ;
char *stage_unit ;

struct t_rx_device *rxtab;

_tlogFun* sdrNode_LogFunction ;
_pushSamplesFun *acqCbFunction ;

#ifdef _WIN64
#include <windows.h>
// Win  DLL Main entry
BOOL WINAPI DllMain( HINSTANCE hInstance, DWORD dwReason, LPVOID *lpvReserved ) {
    return( TRUE ) ;
}
#endif

void log( int device_id, int level, char *msg ) {
    if( sdrNode_LogFunction != NULL ) {
        (*sdrNode_LogFunction)(rxtab[device_id].uuid,level,msg);
        return ;
    }
    printf("Trace:%s\n", msg );
}



/*
 * First function called by SDRNode - must return 0 if hardware is not present or problem
 */
/**
 * @brief initLibrary is called when the DLL is loaded, only for the first instance of the devices (when the getBoardCount() function returns
 *        more than 1)
 * @param json_init_params a JSOn structure to pass parameters from scripting to drivers
 * @param ptr pointer to function for logging
 * @param acqCb pointer to RF IQ processing function
 * @return
 */
LIBRARY_API int initLibrary(char *json_init_params,
                            _tlogFun* ptr,
                            _pushSamplesFun *acqCb )
{

    int rc ;
    int buf[BUFSIZ];
    struct t_rx_device *tmp ;
    eeprom_prodid prodid;

    sdrNode_LogFunction = ptr ;
    acqCbFunction = acqCb ;

    if( DEBUG_DRIVER ) fprintf(stderr,"%s\n", __func__);

    driver_name = (char *)malloc( 100*sizeof(char));
    snprintf(driver_name,100,"PerseuSDR");

    // disable debug mode
    perseus_set_debug(0);

    // Step 1 : count how many devices we have
    device_count = perseus_init();
    if( device_count == 0 ) {
        perseus_exit();
        return(0);
    }

   rxtab = (struct t_rx_device *)malloc(device_count*sizeof(struct t_rx_device));
   if( rxtab == NULL ) {
       return(0);
   }

   for( int i=0 ; i < device_count ; i++ ){
          tmp = &rxtab[i] ;
          tmp->device = perseus_open(i) ;
          if( tmp->device==NULL ) {
              device_count = i ;
              break ;
          }

          if( perseus_firmware_download( tmp->device, NULL) <0 ) {
              // wait a bit and retry
              usleep(1e6);
              if (perseus_firmware_download( tmp->device , NULL )<0) {
                  // no way
                  device_count = i ;
                  break ;
              }

          }

          tmp->device_name = (char *)malloc( 64 *sizeof(char));
          sprintf( tmp->device_name, "PerseusSDR");

          tmp->device_serial_number = (char *)malloc( 128 *sizeof(char));
          if (perseus_get_product_id( tmp->device ,&prodid)<0) {
              sprintf( tmp->device_serial_number, "00000-00-00-00" );
          } else
              sprintf( tmp->device_serial_number, "%05d-%02hX%02hX-%02hX%02hX-%02hX%02hX",
                     (uint16_t) prodid.sn,
                     (uint16_t) prodid.signature[5],
                     (uint16_t) prodid.signature[4],
                     (uint16_t) prodid.signature[3],
                     (uint16_t) prodid.signature[2],
                     (uint16_t) prodid.signature[1],
                     (uint16_t) prodid.signature[0]);


          tmp->uuid = NULL ;
          tmp->running = false ;
          tmp->acq_stop = false ;

          tmp->min_frq_hz = 100e3 ;
          tmp->max_frq_hz = 40e6 ;
          tmp->center_frq_hz = tmp->min_frq_hz + 1e6 ; // arbitrary startup freq

          tmp->rates = (struct t_sample_rates*)malloc( sizeof(struct t_sample_rates));
          tmp->rates->enum_length = 0 ;

          // recover sampling rates from device
          if (perseus_get_sampling_rates ( tmp->device, buf, sizeof(buf)/sizeof(buf[0])) >= 0) {
              int k = 0 ;
              while (buf[k]) {
                  tmp->rates->enum_length++ ;
                  if(( buf[k] > 512e3 ) && (buf[k]< 1024e3 )) {
                      tmp->rates->preffered_sr_index = k ;
                  }
                  k++;
              }
              tmp->rates->sample_rates = (unsigned int *)malloc( tmp->rates->enum_length * sizeof( unsigned int )) ;
              k = 0 ;
              while (buf[k]) {
                   tmp->rates->sample_rates[k] = buf[k];
                   k++ ;
              }

          }

          // configure device
          tmp->ext_context.ctx_version = 0 ;
          tmp->ext_context.center_freq = tmp->center_frq_hz ;
          tmp->current_sample_rate = tmp->rates->sample_rates[tmp->rates->preffered_sr_index] ;
          tmp->ext_context.sample_rate = tmp->current_sample_rate ;

          if (perseus_set_sampling_rate( tmp->device, tmp->current_sample_rate) < 0) {
              device_count = i-1 ;
              break ;
          }

          perseus_set_ddc_center_freq( tmp->device, (double)(tmp->center_frq_hz ), 1);

    }
    // go through linked list and build table
    if( device_count == 0 ) {
        return(0);
    }

    // set names for stages
    stage_name[0] = (char *)malloc( 10*sizeof(char));
    snprintf( stage_name[0],10,"GAIN");

    stage_unit = (char *)malloc( 10*sizeof(char));
    snprintf( stage_unit,10,"dB");

    return(RC_OK);
}


/**
 * @brief setBoardUUID this function is called by SDRNode to assign a unique ID to each device managed by the driver
 * @param device_id [0..getBoardCount()[
 * @param uuid the unique ID
 * @return
 */
LIBRARY_API int setBoardUUID( int device_id, char *uuid ) {
    int len = 0 ;

    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%s)\n", __func__, device_id, uuid );

    if( uuid == NULL ) {
        return(RC_NOK);
    }
    if( device_id >= device_count )
        return(RC_NOK);

    len = strlen(uuid);
    if( rxtab[device_id].uuid != NULL ) {
        free( rxtab[device_id].uuid );
    }
    rxtab[device_id].uuid = (char *)malloc( len * sizeof(char));
    strcpy( rxtab[device_id].uuid, uuid);
    return(RC_OK);
}

/**
 * @brief getHardwareName called by SDRNode to retrieve the name for the nth device
 * @param device_id [0..getBoardCount()[
 * @return a string with the hardware name, this name is listed in the 'devices' admin page and appears 'as is' in the scripts
 */
LIBRARY_API char *getHardwareName(int device_id) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s\n", __func__);
    if( device_id >= device_count )
        return(NULL);
    struct t_rx_device *dev = &rxtab[device_id] ;
    return( dev->device_name );
}

/**
 * @brief getBoardCount called by SDRNode to retrieve the number of different boards managed by the driver
 * @return the number of devices managed by the driver
 */
LIBRARY_API int getBoardCount() {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s\n", __func__);
    return(device_count);
}

/**
 * @brief getPossibleSampleRateCount called to know how many sample rates are available. Used to fill the select zone in admin
 * @param device_id
 * @return sample rate in Hz
 */
LIBRARY_API int getPossibleSampleRateCount(int device_id) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s\n", __func__);
    if( device_id >= device_count )
        return(0);
    struct t_rx_device *dev = &rxtab[device_id] ;
    return( dev->rates->enum_length );
}

/**
 * @brief getPossibleSampleRateValue
 * @param device_id
 * @param index
 * @return
 */
LIBRARY_API unsigned int getPossibleSampleRateValue(int device_id, int index) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d)\n", __func__, index );
    if( device_id >= device_count )
        return(0);
    struct t_rx_device *dev = &rxtab[device_id] ;

    struct t_sample_rates* rates = dev->rates ;
    if( index > rates->enum_length )
        return(0);

    return( rates->sample_rates[index] );
}

LIBRARY_API unsigned int getPrefferedSampleRateValue(int device_id) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s\n", __func__);
    if( device_id >= device_count )
        return(0);
    struct t_rx_device *dev = &rxtab[device_id] ;
    struct t_sample_rates* rates = dev->rates ;
    int index = rates->preffered_sr_index ;
    return( rates->sample_rates[index] );
}
//-------------------------------------------------------------------
LIBRARY_API int64_t getMin_HWRx_CenterFreq(int device_id) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s\n", __func__);
    if( device_id >= device_count )
        return(0);
    struct t_rx_device *dev = &rxtab[device_id] ;
    return( dev->min_frq_hz ) ;
}

LIBRARY_API int64_t getMax_HWRx_CenterFreq(int device_id) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s\n", __func__);
    if( device_id >= device_count )
        return(0);
    struct t_rx_device *dev = &rxtab[device_id] ;
    return( dev->max_frq_hz ) ;
}

//-------------------------------------------------------------------
// Gain management
// devices have stages (LNA, VGA, IF...) . Each stage has its own gain
// range, its own name and its own unit.
// each stage can be 'continuous gain' or 'discrete' (on/off for example)
//-------------------------------------------------------------------
LIBRARY_API int getRxGainStageCount(int device_id) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d)\n", __func__, device_id);
    return(STAGES_COUNT);
}

LIBRARY_API char* getRxGainStageName( int device_id, int stage) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d)\n", __func__, device_id, stage );
    if( stage >= STAGES_COUNT ) {
        stage = 0 ;
    }
    return( stage_name[stage] );
}

LIBRARY_API char* getRxGainStageUnitName( int device_id, int stage) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d)\n", __func__, device_id, stage );
    return( stage_unit );
}

LIBRARY_API int getRxGainStageType( int device_id, int stage) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d)\n", __func__, device_id, stage );
    // continuous value
    return(0);
}

LIBRARY_API float getMinGainValue(int device_id,int stage) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d)\n", __func__, device_id, stage );
    return(-30) ;
}

LIBRARY_API float getMaxGainValue(int device_id,int stage) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d)\n", __func__, device_id, stage );
    return(10) ;
}

LIBRARY_API int getGainDiscreteValuesCount( int device_id, int stage ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d)\n", __func__, device_id, stage);
    return(0);
}

LIBRARY_API float getGainDiscreteValue( int device_id, int stage, int index ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d, %d,%d)\n", __func__, device_id, stage, index);
    return(0);
}

/**
 * @brief getSerialNumber returns the (unique for this hardware name) serial number. Serial numbers are useful to manage more than one unit
 * @param device_id
 * @return
 */
LIBRARY_API char* getSerialNumber( int device_id ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d)\n", __func__, device_id);
    if( device_id >= device_count )
        return(RC_NOK);
    struct t_rx_device *dev = &rxtab[device_id] ;
    return( dev->device_serial_number );
}


/**
 * @brief setRxSampleRate configures the sample rate for the device (in Hz). Can be different from the enum given by getXXXSampleRate
 * @param device_id
 * @param sample_rate
 * @return
 */
LIBRARY_API int setRxSampleRate( int device_id , int sample_rate) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d)\n", __func__, device_id,sample_rate);
    if( device_id >= device_count )
        return(RC_NOK);

    struct t_rx_device *dev = &rxtab[device_id] ;
    if( sample_rate == dev->current_sample_rate ) {
        return(RC_OK);
    }

    int rc = perseus_set_sampling_rate( dev->device, sample_rate) ;
    if( rc == 0 ) {
        dev->current_sample_rate = sample_rate ;
        dev->ext_context.sample_rate = sample_rate ;
        dev->ext_context.ctx_version++ ;
        return(RC_OK);
    }
    return(RC_NOK);
}

/**
 * @brief getActualRxSampleRate called to know what is the actual sampling rate (hz) for the given device
 * @param device_id
 * @return
 */
LIBRARY_API int getActualRxSampleRate( int device_id ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d)\n", __func__, device_id);
    if( device_id >= device_count )
        return(RC_NOK);
    struct t_rx_device *dev = &rxtab[device_id] ;
    return(dev->current_sample_rate);
}

/**
 * @brief setRxCenterFreq tunes device to frq_hz (center frequency)
 * @param device_id
 * @param frq_hz
 * @return
 */
LIBRARY_API int setRxCenterFreq( int device_id, int64_t frq_hz ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%ld)\n", __func__, device_id, (long)frq_hz);
    if( DEBUG_DRIVER ) fflush(stderr);
    if( device_id >= device_count )
        return(RC_NOK);

    struct t_rx_device *dev = &rxtab[device_id] ;
    int rc = perseus_set_ddc_center_freq( dev->device, (double)frq_hz, 1  );
    if( rc == 0 ) {
        dev->center_frq_hz = frq_hz ;
        dev->ext_context.center_freq = frq_hz ;
        dev->ext_context.ctx_version++ ;
        return(RC_OK);
    }
    return(RC_NOK);
}

/**
 * @brief getRxCenterFreq retrieve the current center frequency for the device
 * @param device_id
 * @return
 */
LIBRARY_API int64_t getRxCenterFreq( int device_id ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d)\n", __func__, device_id);
    if( device_id >= device_count )
        return(RC_NOK);

    struct t_rx_device *dev = &rxtab[device_id] ;
    return( dev->center_frq_hz ) ;
}

/**
 * @brief setRxGain sets the current gain
 * @param device_id
 * @param stage_id
 * @param gain_value
 * @return
 */
LIBRARY_API int setRxGain( int device_id, int stage_id, float gain_value ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d,%f)\n", __func__, device_id,stage_id,gain_value);
    if( device_id >= device_count )
        return(RC_NOK);
    if( stage_id >= STAGES_COUNT )
        return(RC_NOK);

    struct t_rx_device *dev = &rxtab[device_id] ;
    perseus_descr* sdr = dev->device ;


    int code = -1 ;
    if( gain_value > 0 ) {
        gain_value = 10 ;
        perseus_set_attenuator( sdr, PERSEUS_ATT_0DB );
        perseus_set_adc( sdr, 1, 1); // turn on preamp
    } else {

        if( gain_value <= -30 ) {
            code = PERSEUS_ATT_30DB ;
        } else if( gain_value<=-20) {
            code = PERSEUS_ATT_20DB ;
        } else if( gain_value<=-10) {
            code = PERSEUS_ATT_10DB ;
        } else  {
            code = PERSEUS_ATT_0DB ;
        }

        if( code >= 0 ) {
            perseus_set_attenuator( sdr, (uint8_t)code );
            perseus_set_adc( sdr, 1, 0); // turn on preamp
        }
    }
    dev->gainv[0] = gain_value ;
    return(RC_OK);
}

/**
 * @brief getRxGainValue reads the current gain value
 * @param device_id
 * @param stage_id
 * @return
 */
LIBRARY_API float getRxGainValue( int device_id , int stage_id ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d,%d)\n", __func__, device_id,stage_id);

    if( device_id >= device_count )
        return(RC_NOK);
    if( stage_id >= STAGES_COUNT )
        return(RC_NOK);

    struct t_rx_device *dev = &rxtab[device_id] ;
    return( dev->gainv[stage_id]) ;
}

LIBRARY_API bool setAutoGainMode( int device_id ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d)\n", __func__, device_id);
    return(false);
}

//-----------------------------------------------------------------------------------------
// One thread is started by device, and each sample frame calls sdr_callback() with a block
// of IQ samples as bytes.
#define ALPHA_DC (0.9996)
typedef union {
    struct {
        int32_t	i;
        int32_t	q;
    } __attribute__((__packed__)) iq;
    struct {
        uint8_t		i1;
        uint8_t		i2;
        uint8_t		i3;
        uint8_t		i4;
        uint8_t		q1;
        uint8_t		q2;
        uint8_t		q3;
        uint8_t		q4;
    } __attribute__((__packed__)) ;
} iq_sample;

/**
 * @brief sdr_callback called by driver.
 */
int sdr_callback( void *buf, int buf_size, void *extra ) {
    TYPECPX *samples ;
    TYPECPX tmp ;
    float I,Q ;
    uint8_t	*samplebuf 	= (uint8_t*)buf;
    int sample_count ;
    iq_sample s;
    int k ;
    double fact = pow(2,23) ;

    struct t_rx_device* my_device = (struct t_rx_device*)extra ;
    if( (my_device==NULL) || (my_device->acq_stop == true )) {
            return(-1) ;
    }

    // The buffer received contains 24-bit IQ samples (6 bytes per sample)
    sample_count = buf_size/6;
    samples = (TYPECPX *)malloc( sample_count * sizeof( TYPECPX ));
    if( samples == NULL ) {
        if( DEBUG_DRIVER ) fprintf(stderr,"%s(len=%d) samples == NULL\n", __func__, sample_count );
        if( DEBUG_DRIVER ) fflush(stderr);
        return(-1) ;
    }

    // Take the raw samples from USB, convert to complex IQ
    // apply the IQ correction
    // do a high pass filter to remove DC bias
    for (k=0;k<sample_count;k++) {
        s.i2 = *samplebuf++;
        s.i3 = *samplebuf++;
        s.i4 = *samplebuf++;
        s.q2 = *samplebuf++;
        s.q3 = *samplebuf++;
        s.q4 = *samplebuf++;
        I = ((double)(s.iq.i >> 6)- fact)/fact ;
        Q = ((double)(s.iq.q >> 6)- fact)/fact ;

        // DC
        // y[n] = x[n] - x[n-1] + alpha * y[n-1]
        // see http://peabody.sapp.org/class/dmp2/lab/dcblock/
        tmp.re = I - my_device->xn_1.re + ALPHA_DC * my_device->yn_1.re ;
        tmp.im = Q - my_device->xn_1.im + ALPHA_DC * my_device->yn_1.im ;

        my_device->xn_1.re = I ;
        my_device->xn_1.im = Q ;

        my_device->yn_1.re = tmp.re ;
        my_device->yn_1.im = tmp.im ;

        samples[k].re = tmp.re ;
        samples[k].im = tmp.im ;
    }
    // push samples to SDRNode callback function
    // we only manage one channel per device
    if( (*acqCbFunction)( my_device->uuid, (float *)samples, sample_count, 1, &my_device->ext_context ) <= 0 ) {
        free(samples);
    }
    return(0) ;
}

//----------------------------------------------------------------------------------
// Manage acquisition
// SDRNode calls 'prepareRxEngine(device)' to ask for the start of acquisition
// Then, the driver shall call the '_pushSamplesFun' function passed at initLibrary( ., ., _pushSamplesFun* fun , ...)
// when the driver shall stop, SDRNode calls finalizeRXEngine()

/**
 * @brief prepareRXEngine trig on the acquisition process for the device
 * @param device_id
 * @return RC_OK if streaming has started, RC_NOK otherwise
 */
LIBRARY_API int prepareRXEngine( int device_id ) {
    int nb = 6*2;
    int bs = 1024;

    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d)\n", __func__, device_id);
    if( device_id >= device_count )
        return(RC_NOK);


    // here we keep it simple, just fire the relevant mutex
    struct t_rx_device *dev = &rxtab[device_id] ;
    // make sure we have a SR set otherwise may have device unflashed
    setRxSampleRate( device_id, dev->current_sample_rate );

    dev->acq_stop = false ;
    if (perseus_start_async_input( dev->device, nb*bs, sdr_callback, (void *)dev )>=0) {
        dev->running = true ;
        return(RC_OK);
    }
    return(RC_NOK);
}

/**
 * @brief finalizeRXEngine stops the acquisition process
 * @param device_id
 * @return
 */
LIBRARY_API int finalizeRXEngine( int device_id ) {
    if( DEBUG_DRIVER ) fprintf(stderr,"%s(%d)\n", __func__, device_id);
    if( device_id >= device_count )
        return(RC_NOK);

    struct t_rx_device *dev = &rxtab[device_id] ;
    perseus_stop_async_input( dev->device );
    dev->acq_stop = true ;
    return(RC_OK);
}

