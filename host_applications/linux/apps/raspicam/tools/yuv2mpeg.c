/*
Copyright (c) 2015, DSP Group Ltd
Copyright (c) 2015, James Hughes
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <sysexits.h>
#include <unistd.h>

#define VERSION_STRING "v0.1"

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

extern char *optarg;
extern int optind, opterr, optopt;

#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 480;
#define VIDEO_BITRATE 420000
#define VIDEO_FRAMERATE 30

/// Cross reference structure, mode string against mode id
typedef struct xref_t
{
   char *mode;
   int mmal_mode;
} XREF_T;

// Forward
typedef struct YUV2MPEG_STATE_S YUV2MPEG_STATE;

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct
{
   FILE *file_handle;                   /// File handle to write buffer data to.
   FILE *imv_file_handle;               /// File handle to write inline motion vectors to.

   YUV2MPEG_STATE *pstate;              /// pointer to our state in case required in callback
} PORT_USERDATA;

/** Structure containing all state information for the current run
 */
struct YUV2MPEG_STATE_S
{
   int width;                           /// Requested width of image
   int height;                          /// requested height of image
   int bitrate;                         /// Requested bitrate
   int fps;                             /// Requested frame rate
   int intraperiod;                     /// Intra-refresh period (key frame rate)
   int quantisationParameter;           /// Quantisation parameter - quality. Set bitrate 0 and set this for variable bitrate
   int bInlineHeaders;                  /// Insert inline headers to stream (SPS, PPS)
   char *inputfilename;            /// filename of output file
   char *outputfilename;                /// filename of output file
   int verbose;                         /// !0 if want detailed run information
   int profile;                         /// H264 profile to use for encoding
   int disable_CABAC;                   /// Flag to disable CABAC
   int intrarefresh;                    /// What intra refresh type to use. -1 to not set.

   int want_motion_vectors;             /// Encoder outputs inline Motion Vectors
   char *motion_filename;               /// filename of inline Motion Vectors output

   MMAL_COMPONENT_T *encoder_component; /// Pointer to the encoder component

   MMAL_POOL_T *encoder_input_pool;     /// Pointer to the pool of buffers used by encoder output port
   MMAL_POOL_T *encoder_output_pool;    /// Pointer to the pool of buffers used by encoder output port

   PORT_USERDATA callback_data;         /// Used to move data to the encoder callback
};


/// Structure to cross reference H264 profile strings against the MMAL parameter equivalent
static XREF_T  profile_map[] =
{
   {"baseline",     MMAL_VIDEO_PROFILE_H264_BASELINE},
   {"main",         MMAL_VIDEO_PROFILE_H264_MAIN},
   {"high",         MMAL_VIDEO_PROFILE_H264_HIGH},
//   {"constrained",  MMAL_VIDEO_PROFILE_H264_CONSTRAINED_BASELINE} // Does anyone need this?
};

static int profile_map_size = sizeof(profile_map) / sizeof(profile_map[0]);

static XREF_T  intra_refresh_map[] =
{
   {"cyclic",       MMAL_VIDEO_INTRA_REFRESH_CYCLIC},
   {"adaptive",     MMAL_VIDEO_INTRA_REFRESH_ADAPTIVE},
   {"both",         MMAL_VIDEO_INTRA_REFRESH_BOTH},
   {"cyclicrows",   MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS},
//   {"random",       MMAL_VIDEO_INTRA_REFRESH_PSEUDO_RAND} Cannot use random, crashes the encoder. No idea why.
};

static int intra_refresh_map_size = sizeof(intra_refresh_map) / sizeof(intra_refresh_map[0]);

static char *xref_to_string(int en, XREF_T*map, int mapsize)
{
   int i;

   if (en < 0)
      return "Not Set/Unknown";

   for (i=0;i<mapsize;i++)
   {
      if (en == map[i].mmal_mode)
      {
         return map[i].mode;
      }
   }
   return NULL;
}


/**
 *  buffer header callback function for encoder input buffer
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void encoder_input_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
//   printf("input buffer callback buffer %p, flag %d, length %d\n", buffer, buffer->flags, buffer->length);

   // release buffer back to the pool
   mmal_buffer_header_release(buffer);
}

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void encoder_output_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   // We pass our file handle and other stuff in via the userdata field.

   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

   if (pData)
   {
      int bytes_written = buffer->length;

      vcos_assert(pData->file_handle);

      if(pData->pstate->want_motion_vectors)
         vcos_assert(pData->imv_file_handle);

      if (buffer->length)
      {
         mmal_buffer_header_mem_lock(buffer);

         if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO)
         {
            if(pData->pstate->want_motion_vectors)
            {
               bytes_written = fwrite(buffer->data, 1, buffer->length, pData->imv_file_handle);
            }
            else
            {
               //We do not want to save inlineMotionVectors...
               bytes_written = buffer->length;
            }
         }
         else
         {
            bytes_written = fwrite(buffer->data, 1, buffer->length, pData->file_handle);
         }

         mmal_buffer_header_mem_unlock(buffer);

         if (bytes_written != buffer->length)
         {
            vcos_log_error("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
            //pData->abort = 1;
         }
      }
   }
   else
   {
      vcos_log_error("Received a encoder buffer callback with no state");
   }

   // release buffer back to the pool
   mmal_buffer_header_release(buffer);

   // and send one back to the port (if still open)
   if (port->is_enabled)
   {
      MMAL_STATUS_T status;
      MMAL_BUFFER_HEADER_T *new_buffer;

      new_buffer = mmal_queue_get(pData->pstate->encoder_output_pool->queue);

      if (new_buffer)
         status = mmal_port_send_buffer(port, new_buffer);

      if (!new_buffer || status != MMAL_SUCCESS)
         vcos_log_error("Unable to return a buffer to the encoder port");
   }
}



/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_encoder_component(YUV2MPEG_STATE *state)
{
   MMAL_COMPONENT_T *encoder = 0;
   MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
   MMAL_STATUS_T status;
   MMAL_POOL_T *inpool = NULL, *outpool = NULL;

   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to create video encoder component");
      goto error;
   }

   if (!encoder->input_num || !encoder->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Video encoder doesn't have input/output ports");
      goto error;
   }

   encoder_input = encoder->input[0];
   encoder_output = encoder->output[0];

   // We want same base format on input and output
   mmal_format_copy(encoder_output->format, encoder_input->format);

   // Input format
   encoder_input->format->encoding = MMAL_ENCODING_I420;
   encoder_input->format->flags = 0;

   encoder_input->buffer_size = encoder_input->buffer_size_recommended;

   if (encoder_input->buffer_size < encoder_input->buffer_size_min)
      encoder_input->buffer_size = encoder_input->buffer_size_min;

   encoder_input->buffer_num = encoder_input->buffer_num_recommended;

   if (encoder_input->buffer_num < encoder_input->buffer_num_min)
      encoder_input->buffer_num = encoder_input->buffer_num_min;

   encoder_input->format->es->video.frame_rate.num = state->fps;
   encoder_input->format->es->video.frame_rate.den = 1;
   encoder_input->format->es->video.width = VCOS_ALIGN_UP(state->width, 32);
   encoder_input->format->es->video.crop.width = state->width;
   encoder_input->format->es->video.height = VCOS_ALIGN_UP(state->height, 16);
   encoder_input->format->es->video.crop.height = state->height;
   encoder_input->format->es->video.crop.x = encoder_input->format->es->video.crop.y = 0;

   // Commit the port changes to the output port
   status = mmal_port_format_commit(encoder_input);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on video encoder input port");
      goto error;
   }

   // Output format

   // Only supporting H264 at the moment
   encoder_output->format->encoding = MMAL_ENCODING_H264;

   encoder_output->format->bitrate = state->bitrate;

   encoder_output->buffer_size = encoder_output->buffer_size_recommended;

   if (encoder_output->buffer_size < encoder_output->buffer_size_min)
      encoder_output->buffer_size = encoder_output->buffer_size_min;

   encoder_output->buffer_num = encoder_output->buffer_num_recommended;

   if (encoder_output->buffer_num < encoder_output->buffer_num_min)
      encoder_output->buffer_num = encoder_output->buffer_num_min;

   encoder_output->format->es->video.frame_rate.num = state->fps;
   encoder_output->format->es->video.frame_rate.den = 1;

   // Commit the port changes to the output port
   status = mmal_port_format_commit(encoder_output);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on video encoder output port");
      goto error;
   }

   if (state->intraperiod != -1)
   {
      MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_INTRAPERIOD, sizeof(param)}, state->intraperiod};
      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set intraperiod");
         goto error;
      }
   }

   if (state->quantisationParameter)
   {
      MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param)}, state->quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set initial QP");
         goto error;
      }

      MMAL_PARAMETER_UINT32_T param2 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MIN_QUANT, sizeof(param)}, state->quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param2.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set min QP");
         goto error;
      }

      MMAL_PARAMETER_UINT32_T param3 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MAX_QUANT, sizeof(param)}, state->quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param3.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set max QP");
         goto error;
      }

   }

   {
      MMAL_PARAMETER_VIDEO_PROFILE_T  param;
      param.hdr.id = MMAL_PARAMETER_PROFILE;
      param.hdr.size = sizeof(param);

      param.profile[0].profile = state->profile;
      param.profile[0].level = MMAL_VIDEO_LEVEL_H264_4; // This is the only value supported

      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set H264 profile");
         goto error;
      }
   }

   //set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
   if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, state->bInlineHeaders) != MMAL_SUCCESS)
   {
      vcos_log_error("failed to set INLINE HEADER FLAG parameters");
      // Continue rather than abort..
   }

   //set INLINE VECTORS flag to request motion vector estimates
   if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, state->want_motion_vectors) != MMAL_SUCCESS)
   {
      vcos_log_error("failed to set INLINE VECTORS parameters");
      // Continue rather than abort..
   }

   // Adaptive intra refresh settings
   if (state->intrarefresh != -1)
   {
      MMAL_PARAMETER_VIDEO_INTRA_REFRESH_T  param;
      param.hdr.id = MMAL_PARAMETER_VIDEO_INTRA_REFRESH;
      param.hdr.size = sizeof(param);

      // Get first so we don't overwrite anything unexpectedly
      status = mmal_port_parameter_get(encoder_output, &param.hdr);

      param.refresh_mode = state->intrarefresh;

      //if (state->intra_refresh_type == MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS)
      //   param.cir_mbs = 10;

      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set H264 intra-refresh values");
         goto error;
      }
   }

   //  Enable component
   status = mmal_component_enable(encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable video encoder component");
      goto error;
   }

   /* Create pool of buffer headers for the input port to consume */
   inpool = mmal_port_pool_create(encoder_input, encoder_input->buffer_num, encoder_input->buffer_size);

   /* Create pool of buffer headers for the output port to consume */
   outpool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

   if (!inpool || !outpool)
   {
      vcos_log_error("Failed to create buffer header pool for encoder input/output port %s", encoder_output->name);
      goto error;
   }

   state->encoder_output_pool = outpool;
   state->encoder_input_pool = inpool;
   state->encoder_component = encoder;

   if (state->verbose)
      fprintf(stderr, "Encoder component done\n");

   return status;

   error:

   if (inpool)
   {
      mmal_port_pool_destroy(encoder_input, inpool);
   }

   if (outpool)
   {
      mmal_port_pool_destroy(encoder_output, outpool);
   }

   if (encoder)
      mmal_component_destroy(encoder);

   state->encoder_component = NULL;

   return status;
}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component(YUV2MPEG_STATE *state)
{
   // Get rid of any port buffers first
   if (state->encoder_input_pool)
   {
      mmal_port_pool_destroy(state->encoder_component->input[0], state->encoder_input_pool);
   }

   if (state->encoder_output_pool)
   {
      mmal_port_pool_destroy(state->encoder_component->output[0], state->encoder_output_pool);
   }

   if (state->encoder_component)
   {
      mmal_component_destroy(state->encoder_component);
      state->encoder_component = NULL;
   }
}


int main(int argc, char **argv)
{
   YUV2MPEG_STATE state;
   FILE *inputfilehandle = NULL;
   int frame_size, opt;
   void *input_buffer;
   MMAL_STATUS_T status = MMAL_SUCCESS;
   MMAL_PORT_T *encoder_input_port = NULL;
   MMAL_PORT_T *encoder_output_port = NULL;
   int bytes_read = 1;
   MMAL_BUFFER_HEADER_TYPE_SPECIFIC_T input_frame_data;
   int64_t pts, pts_increment;

   bcm_host_init();

   // Register our application with the logging system
   vcos_log_register("yuv2mpeg", VCOS_LOG_CATEGORY);
   vcos_log_set_level(VCOS_LOG_CATEGORY, VCOS_LOG_TRACE);

   state.width = VIDEO_WIDTH;
   state.height = VIDEO_HEIGHT;
   state.bitrate = VIDEO_BITRATE;
   state.fps = VIDEO_FRAMERATE;
   state.intrarefresh = -1;
   state.profile = MMAL_VIDEO_PROFILE_H264_BASELINE;
   state.disable_CABAC = MMAL_FALSE;
   state.want_motion_vectors = MMAL_FALSE;
   state.motion_filename = NULL;
   state.callback_data.imv_file_handle = NULL;
   state.verbose = 0;
   state.quantisationParameter = 0;
   state.bInlineHeaders = MMAL_FALSE;

   while ((opt = getopt (argc, argv, "w:h:b:f:i:o:vr:p:cm:sx:?")) != -1)
   {
      switch (opt)
      {
         case 'w':
            state.width = atoi(optarg);
            break;
         case 'h':
            state.height = atoi(optarg);
            break;
         case 'b':
            state.bitrate = atoi(optarg);
            break;
         case 'f':
            state.fps = atoi(optarg);
            break;
         case 'i':
            state.inputfilename = optarg;
            break;
         case 'o':
            state.outputfilename = optarg;
            break;
         case 'v':
           state.verbose = 1;
            break;
         case 'r':
            state.intrarefresh = atoi(optarg);
            break;
         case 'x':
            state.intraperiod = atoi(optarg);
            break;

         case 'p':
            if (optarg)
            {
               if (*optarg == 'h')
                  state.profile = MMAL_VIDEO_PROFILE_H264_HIGH;
               else if (*optarg == 'm')
                  state.profile = MMAL_VIDEO_PROFILE_H264_MAIN;
               else
                  state.profile = MMAL_VIDEO_PROFILE_H264_BASELINE;
            }
            break;
         case 'c':
            state.disable_CABAC = MMAL_TRUE;
            break;

         case 'm':
            state.want_motion_vectors = MMAL_TRUE;
            state.motion_filename = optarg;
            break;

         case 's':
            state.bInlineHeaders = MMAL_TRUE;
            break;

         default:
         case '?':
            fprintf(stderr, "Usage: %s\n", argv[0]);
            fprintf(stderr, "-w width\n-h height\n-b bitrate \n-f framerate \n-r Intra refresh type \n-p profile [h,m,n] \n-c disable CABAC \n-m filename Output motion vectors");
            fprintf(stderr, "-x Intra period \n-s Inline headers \n-i Input file \n-o Output file \n-v Verbose\n");
            exit(EXIT_FAILURE);
      }
   }

   if (!state.inputfilename || !state.outputfilename)
   {
      printf("No input or output filename specified\n");
      exit(0);
   }

   // Open up the input file, output file and IMV file is required

   inputfilehandle = fopen(state.inputfilename, "rb");

   if (inputfilehandle == NULL)
   {
      fprintf(stderr, "Unable to open input filename %s\n", state.inputfilename);
      goto cleanup;
   }

   state.callback_data.file_handle = fopen(state.outputfilename, "wb");
   if (state.callback_data.file_handle == NULL)
   {
      fprintf(stderr, "Unable to open output filename %s\n", state.outputfilename);
      goto cleanup;
   }

   if (state.want_motion_vectors)
   {
      printf("motion\n");

      state.callback_data.imv_file_handle = fopen(state.motion_filename, "wb");
      if (state.callback_data.imv_file_handle  == NULL)
      {
         fprintf(stderr, "Unable to open motion vector filename %s\n", state.motion_filename);
         goto cleanup;|| en > mapsize-1
      }
   }

   if (state.verbose)
   {
      printf("Creating encoder component\n");
      printf("Parameters used:\n  Width = %d, height = %d, fps = %d, bitrate = %d\n", state.width, state.height, state.fps, state.bitrate);
      printf("  Input filename %s, output filename %s", state.inputfilename, state.outputfilename);
      if (state.want_motion_vectors)
         printf(", Motion vector filename %s", state.motion_filename);
      printf(". CABAC is %s, using profile %s\n", state.disable_CABAC ? "disabled" : "enabled", xref_to_string(state.profile, profile_map, profile_map_size));
      if (state.quantisationParameter)
         printf("Quantisation set to %d\n", state.quantisationParameter);
      printf("  Intra period %d, Inline headers %s\n", state.intraperiod, state.bInlineHeaders ? "Yes" : "No");
      printf("  Intra refresh type %s\n",xref_to_string(state.intrarefresh, intra_refresh_map, intra_refresh_map_size));
   }

   create_encoder_component(&state);

   state.callback_data.pstate = &state;
   encoder_input_port  = state.encoder_component->input[0];
   encoder_output_port = state.encoder_component->output[0];

   encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&state.callback_data;
   encoder_input_port->userdata = (struct MMAL_PORT_USERDATA_T *)&state.callback_data;

   if (state.verbose)
      printf("Enabling input port\n");

   // Enable the encoder input port and tell it its callback function
   status = mmal_port_enable(encoder_input_port, encoder_input_buffer_callback);

   if (state.verbose)
      printf("Enabling output port\n");

   // Enable the encoder output port and tell it its callback function
   status = mmal_port_enable(encoder_output_port, encoder_output_buffer_callback);

   // Send all the buffers to the encoder output ports
   {
      int num = mmal_queue_length(state.encoder_output_pool->queue);
      int q;
      for (q=0;q<num;q++)
      {
         MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.encoder_output_pool->queue);

         if (!buffer)
            vcos_log_error("Unable to get a required buffer %d from pool queue", q);

         if (mmal_port_send_buffer(encoder_output_port, buffer)!= MMAL_SUCCESS)
            vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
      }
   }

   if (state.verbose)
      printf("Sent all buffers\n");

   // Calculate the size of each I420 frame. We will read input file in these chunks.
   // TODO: need to ensure %16 on dimensions
   frame_size = (state.width * state.height * 3) / 2;

   input_buffer = malloc(frame_size);

   if (!input_buffer)
   {
      fprintf(stderr, "Unable to allocate input buffer \n");
      goto cleanup;
   }

   input_frame_data.video.planes = 3;
   input_frame_data.video.offset[0] = 0;
   input_frame_data.video.offset[1] = state.width * state.height;
   input_frame_data.video.offset[2] = input_frame_data.video.offset[1] + input_frame_data.video.offset[1]/4;

   input_frame_data.video.pitch[0] = state.width;
   input_frame_data.video.pitch[1] = state.width/4;
   input_frame_data.video.pitch[2] = state.width/4;

   if (state.verbose)
      printf("Starting the loop\n");

   pts = 0;
   pts_increment = 1000000 / state.fps;

   // Loop until have read all the input file and passed to the encoder
   while (bytes_read > 0)
   {
      MMAL_BUFFER_HEADER_T *enc_input_buffer;

      // Get a buffer from the encoder. This will block until one arrives.
      enc_input_buffer = mmal_queue_wait(state.encoder_input_pool->queue);

      mmal_buffer_header_mem_lock(enc_input_buffer);

      bytes_read = fread(enc_input_buffer->data, 1, enc_input_buffer->alloc_size, inputfilehandle);

      enc_input_buffer->length = bytes_read;
      enc_input_buffer->offset = 0;
      enc_input_buffer->cmd = 0;

      enc_input_buffer->flags = MMAL_BUFFER_HEADER_FLAG_FRAME | MMAL_BUFFER_HEADER_FLAG_KEYFRAME;

      if (bytes_read < enc_input_buffer->alloc_size)
         enc_input_buffer->flags |= MMAL_BUFFER_HEADER_FLAG_EOS;

      enc_input_buffer->pts = enc_input_buffer->dts = pts;
      pts += pts_increment;

      enc_input_buffer->type = &input_frame_data;

      mmal_buffer_header_mem_unlock(enc_input_buffer);

  //    if (state.verbose)
    //     printf("Input buffer free, ptr = %p, alloc size = %d, filled size %d, actually read %d\n", enc_input_buffer->data, enc_input_buffer->alloc_size, enc_input_buffer->length, bytes_read);


      if (mmal_port_send_buffer(encoder_input_port, enc_input_buffer)!= MMAL_SUCCESS)
         vcos_log_error("Unable to send a buffer to encoder output port");

      if (state.verbose)
      {
         printf(".");
         fflush(stdout);
      }
   }

   if (state.verbose)
      printf("\nReached end of input file, closing and exiting\n");

cleanup:

   if (encoder_input_port->is_enabled)
      mmal_port_disable(encoder_input_port);

   if (encoder_output_port->is_enabled)
      mmal_port_disable(encoder_output_port);

   mmal_component_disable(state.encoder_component);

   destroy_encoder_component(&state);

   if (inputfilehandle)
       fclose(inputfilehandle);

   if (state.callback_data.file_handle)
      fclose(state.callback_data.file_handle);

   if (state.callback_data.imv_file_handle)
       fclose(state.callback_data.imv_file_handle);

   return 0;
}
