/**
 * @file CameraUnit_ATIK.cpp
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief Implementation of CameraUnit interfaces for ATIK Cameras
 * @version 0.1
 * @date 2022-01-03
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "CameraUnit_TIS.hpp"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> /* sleep  */
#include <gst/gst.h>
#include <tcamprop.h>
#include "meb_print.h"

#include <string>
#include <mutex>
#include <chrono>

#if !defined(OS_Windows)
#include <unistd.h>
static inline void Sleep(int dwMilliseconds)
{
    usleep(dwMilliseconds * 1000);
}
#endif

#ifndef eprintf
#define eprintf(str, ...)                                                   \
    {                                                                       \
        fprintf(stderr, "%s, %d: " str, __func__, __LINE__, ##__VA_ARGS__); \
        fflush(stderr);                                                     \
    }
#endif
#ifndef eprintlf
#define eprintlf(str, ...) eprintf(str "\n", ##__VA_ARGS__)
#endif

static inline uint64_t getTime()
{
    return ((std::chrono::duration_cast<std::chrono::milliseconds>((std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now())).time_since_epoch())).count());
}

static void gstream_system_init(int *argc, char **argv[])
{
    static bool firstRun = true;
    if (firstRun)
    {
        gst_debug_set_default_threshold(GST_LEVEL_WARNING);
        gst_init(argc, argv);
        firstRun = false;
    }
}

bool gst_get_property(GstElement *src, const char *name, int &value)
{
    bool retval = true;
    GValue val, min, max, default_val, step_sz, type, flags, cat, group;
    gboolean ret = tcam_prop_get_tcam_property(TCAM_PROP(src),
                                               name,
                                               &val,
                                               &min,
                                               &max,
                                               &default_val,
                                               &step_sz,
                                               &type,
                                               &flags,
                                               &cat,
                                               &group);
    if (!ret)
    {
        retval = false;
        goto cleanup;
    }
    value = (int)g_value_get_int(&val);
cleanup:
    g_value_unset(&value);
    g_value_unset(&min);
    g_value_unset(&max);
    g_value_unset(&default_value);
    g_value_unset(&step_size);
    g_value_unset(&type);
    g_value_unset(&flags);
    g_value_unset(&category);
    g_value_unset(&group);
    return retval;
}

bool gst_get_property(GstElement *src, const char *name, double &value)
{
    bool retval = true;
    GValue val, min, max, default_val, step_sz, type, flags, cat, group;
    gboolean ret = tcam_prop_get_tcam_property(TCAM_PROP(src),
                                               name,
                                               &val,
                                               &min,
                                               &max,
                                               &default_val,
                                               &step_sz,
                                               &type,
                                               &flags,
                                               &cat,
                                               &group);
    if (!ret)
    {
        retval = false;
        goto cleanup;
    }
    value = (double)g_value_get_double(&val);
cleanup:
    g_value_unset(&value);
    g_value_unset(&min);
    g_value_unset(&max);
    g_value_unset(&default_value);
    g_value_unset(&step_size);
    g_value_unset(&type);
    g_value_unset(&flags);
    g_value_unset(&category);
    g_value_unset(&group);
    return retval;
}

bool gst_get_property(GstElement *src, const char *name, std::string &value)
{
    bool retval = true;
    GValue val, min, max, default_val, step_sz, type, flags, cat, group;
    gboolean ret = tcam_prop_get_tcam_property(TCAM_PROP(src),
                                               name,
                                               &val,
                                               &min,
                                               &max,
                                               &default_val,
                                               &step_sz,
                                               &type,
                                               &flags,
                                               &cat,
                                               &group);
    if (!ret)
    {
        retval = false;
        goto cleanup;
    }
    value = std::string((char *)g_value_get_string(&val));
cleanup:
    g_value_unset(&value);
    g_value_unset(&min);
    g_value_unset(&max);
    g_value_unset(&default_value);
    g_value_unset(&step_size);
    g_value_unset(&type);
    g_value_unset(&flags);
    g_value_unset(&category);
    g_value_unset(&group);
    return retval;
}

bool gst_get_property(const GstElement *src, const char *name, bool &value)
{
    bool retval = true;
    GValue val, min, max, default_val, step_sz, type, flags, cat, group;
    gboolean ret = tcam_prop_get_tcam_property(TCAM_PROP(src),
                                               name,
                                               &val,
                                               &min,
                                               &max,
                                               &default_val,
                                               &step_sz,
                                               &type,
                                               &flags,
                                               &cat,
                                               &group);
    if (!ret)
    {
        retval = false;
        goto cleanup;
    }
    value = (bool)g_value_get_bool(&val);
cleanup:
    g_value_unset(&value);
    g_value_unset(&min);
    g_value_unset(&max);
    g_value_unset(&default_value);
    g_value_unset(&step_size);
    g_value_unset(&type);
    g_value_unset(&flags);
    g_value_unset(&category);
    g_value_unset(&group);
    return retval;
}

bool gst_set_property(const GstElement *src, const char *name, int value)
{
    GValue val = G_VALUE_INIT;
    g_value_init(&val, G_TYPE_INT);
    g_value_set_int(&val, value);
    bool ret = tcam_prop_set_tcam_property(TCAM_PROP(src), name, &val);
    g_value_unset(&val);
    return ret;
}

bool gst_set_property(const GstElement *src, const char *name, double value)
{
    GValue val = G_VALUE_INIT;
    g_value_init(&val, G_TYPE_DOUBLE);
    g_value_set_double(&val, value);
    bool ret = tcam_prop_set_tcam_property(TCAM_PROP(src), name, &val);
    g_value_unset(&val);
    return ret;
}

bool gst_set_property(const GstElement *src, const char *name, std::string value)
{
    GValue val = G_VALUE_INIT;
    g_value_init(&val, G_TYPE_STRING);
    g_value_set_string(&val, (gchar *)value.c_str());
    bool ret = tcam_prop_set_tcam_property(TCAM_PROP(src), name, &val);
    g_value_unset(&val);
    return ret;
}

bool gst_set_property(const GstElement *src, const char *name, bool value)
{
    GValue val = G_VALUE_INIT;
    g_value_init(&val, G_TYPE_BOOLEAN);
    bool ret = tcam_prop_set_tcam_property(TCAM_PROP(src), name, &val);
    g_value_unset(&val);
    return ret;
}

GstFlowReturn CCameraUnit_TIS::ExposureCallback (GstElement* sink, void* user_data)
{
    GstFlowReturn retval = GST_FLOW_OK;
    GstSample* sample = NULL;
    CallbackData *callback_ptr = (CallbackData *) user_data;
    CCameraUnit_TIS *self = callback_ptr->self;
    CImageData *img = callback_ptr->img;
    // acquire lock
    std::lock_guard<std::mutex>(self->cs_);
    /* Retrieve the buffer */
    g_signal_emit_by_name(sink, "pull-sample", &sample, NULL);

    if (sample)
    {
        // we have a valid sample
        // do things with the image here
        static guint framecount = 0;
        int pixel_data = -1;

        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstMapInfo info; // contains the actual image
        if (gst_buffer_map(buffer, &info, GST_MAP_READ))
        {
            GstVideoInfo* video_info = gst_video_info_new();
            if (!gst_video_info_from_caps(video_info, gst_sample_get_caps(sample)))
            {
                // Could not parse video info (should not happen)
                g_warning("Failed to parse video info");
                retval = GST_FLOW_ERROR;
                goto ret;
            }

            /* Get a pointer to the image data */
            unsigned char* data = info.data;

            /* Get the pixel value of the center pixel */
            int stride = video_info->finfo->bits / 8;
            unsigned int pixel_offset = video_info->width / 2 * stride +
                video_info->width * video_info->height / 2 * stride;

            // this is only one pixel
            // when dealing with formats like BGRx
            // pixel_data will have to consist out of
            // pixel_offset   => B
            // pixel_offset+1 => G
            // pixel_offset+2 => R
            // pixel_offset+3 => x
            pixel_data = info.data[pixel_offset];

            gst_buffer_unmap(buffer, &info);
            gst_video_info_free(video_info);
        }

        GstClockTime timestamp = GST_BUFFER_PTS(buffer);
        g_print("Captured frame %d, Pixel Value=%03d Timestamp=%" GST_TIME_FORMAT "            \r",
                framecount, pixel_data,
                GST_TIME_ARGS(timestamp));
        framecount++;


        // delete our reference so that gstreamer can handle the sample
        gst_sample_unref (sample);
    }
ret:
    return retval;
}

bool CCameraUnit_TIS::HasError(int error, unsigned int line) const
{
    return false;
}

int CCameraUnit_TIS::ArtemisGetCameraState(ArtemisHandle h)
{
    int state = 0;
    return state;
}

CCameraUnit_TIS::CCameraUnit_TIS()
    : err(NULL),
      pipeline(NULL),
      source(NULL),
      m_initializationOK(false),
      cancelCapture_(true),
      hasshutter(false),
      numtempsensors(0),
      binningX_(1),
      binningY_(1),
      imageLeft_(0),
      imageRight_(0),
      imageTop_(0),
      imageBottom_(0),
      roiLeft(0),
      roiRight(0),
      roiTop(0),
      roiBottom(0),
      roi_updated_(false),
      CCDWidth_(0),
      CCDHeight_(0)
{
    // initialize camera
    std::string errmsg = "";

    pipeline = gst_parse_launch("tcambin name=source ! videoconvert ! appsink name=sink", &err);

    if (pipeline == NULL)
    {
        dbprintlf(RED_FG "Could not create pipeline, reason: %s", err->message);
        errmsg = err->message;
        goto final_cleanup;
    }

    // get the sink
    sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
    if (sink == NULL)
    {
        errmsg = "Could not enumerate sink.";
        goto close_pipeline;
    }

    // get the source
    source = gst_bin_get_by_name(GST_BIN(pipeline), "source");
    if (source == NULL)
    {
        errmsg = "Could not enumerate source.";
        goto close_sink;
    }

    // get cameras connected
    GSList *serials = tcam_prop_get_device_serials(TCAM_PROP(source));

    if (serials == NULL)
    {
        errmsg = "Could not enumerate devices.";
        goto close_source;
    }

    char *name, *ident, *conn_type;

    if (!tcam_prop_get_device_info(TCAM_PROP(source), serials, &name, &ident, &conn_type))
    {
        errmsg = "Could not retrieve device properties.";
        goto close_source;
    }
    memset(cam_name, 0x0, sizeof(cam_name));
    snprintf(cam_name, sizeof(cam_name) - 1, "%s", name);
    g_free(name);
    g_free(ident);
    g_free(conn_type);

    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GstStateChangeReturn::GST_STATE_CHANGE_FAILURE)
    {
        errmsg = "Could not set state to playing.";
        goto close_source;
    }

    if (!gst_get_property(source, "width", &CCDWidth_))
    {
        errmsg = "Get property for width failed.";
        goto close_source;
    }

    if (!gst_get_property(source, "height", &CCDHeight_))
    {

        errmsg = "Get property for height failed.";
        goto close_source;
    }

    if (!gst_get_property(source, "exposure_max", &maxExposureUs_))
    {
        errmsg = "Get property for exposure max failed.";
        goto close_source;
    }

    imageLeft_ = 0;
    imageRight_ = CCDWidth_;
    imageTop_ = 0;
    imageBottom_ = CCDHeight_;
    roiLeft = imageLeft_;
    roiRight = imageRight_;
    roiTop = imageTop_;
    roiBottom = imageBottom_;

    // Initialization done
    m_initializationOK = true;

    return;

close_source:
    gst_object_unref(source);

close_sink:
    gst_object_unref(source);

close_pipeline:
    g_free(err);
    gst_object_unref(pipeline);

final_cleanup:
    m_initializationOK = false;

    if (errmsg.length() > 0)
    {
        throw std::runtime_error(errmsg));
    }
}

CCameraUnit_TIS::~CCameraUnit_TIS()
{
    // CriticalSection::Lock lock(criticalSection_);
    std::lock_guard<std::mutex> lock(cs_);
    ArtemisDisconnect(hCam);
    m_initializationOK = false;
#ifdef _WIN32
    ArtemisUnLoadDLL();
#endif
}

void CCameraUnit_TIS::CancelCapture()
{
    std::lock_guard<std::mutex> lock(cs_);
    cancelCapture_ = true;
    // abort acquisition
    ArtemisAbortExposure(hCam);
}

// ----------------------------------------------------------

CImageData CCameraUnit_TIS::CaptureImage(long int &retryCount)
{
    std::unique_lock<std::mutex> lock(cs_);
    CImageData retVal;
    cancelCapture_ = false;
    BOOL image_ready;

    void *pImgBuf;

    int x, y, w, h, binx, biny;

    int sleep_time_ms = 0;

    int cameraState = 0;

    int exposure_ms = exposure_ * 1000;

    float exposure_now = exposure_;
    if (exposure_ms < 1)
        exposure_ms = 1;

    uint64_t exposure_start;

    if (!m_initializationOK)
    {
        goto exit_err;
    }

    exposure_start = getTime();
    if (HasError(ArtemisStartExposureMS(hCam, exposure_ms), __LINE__))
    {
        goto exit_err;
    }
    sleep_time_ms = 1000 * ArtemisExposureTimeRemaining(hCam); // sleep time in ms
    if (sleep_time_ms < 0)
        sleep_time_ms = 0;
    lock.unlock();

    Sleep(sleep_time_ms);

    lock.lock();
    while (!(image_ready = ArtemisImageReady(hCam)))
    {
        cameraState = ArtemisGetCameraState(hCam);
        if (cameraState == CAMERA_DOWNLOADING)
        {
            status_ += std::string(" Download: ");
            status_ += std::to_string(ArtemisDownloadPercent(hCam));
            status_ += " %";
        }
        Sleep(10);
    }
    if (HasError(ArtemisGetImageData(hCam, &x, &y, &w, &h, &binx, &biny), __LINE__))
    {
        eprintlf("Error getting image data");
        goto exit_err;
    }
    pImgBuf = ArtemisImageBuffer(hCam);

    binningX_ = binx;
    binningY_ = biny;

    retVal = CImageData(w, h);
    if (pImgBuf == NULL)
    {
        eprintlf("Image buffer is NULL");
        goto exit_err;
    }
    memcpy(retVal.GetImageData(), pImgBuf, w * h * 2);
    retVal.SetImageMetadata(exposure_now, binx, biny, GetTemperature(), exposure_start, CameraName());
exit_err:
    // printf("Exiting capture\n");
    return retVal;
}

void CCameraUnit_TIS::SetTemperature(double temperatureInCelcius)
{
    if (!m_initializationOK)
    {
        return;
    }

    int16_t settemp;
    settemp = temperatureInCelcius * 100;

    HasError(ArtemisSetCooling(hCam, settemp), __LINE__);
}

// get temperature is done
double CCameraUnit_TIS::GetTemperature() const
{
    if (!m_initializationOK)
    {
        return INVALID_TEMPERATURE;
    }

    int temperature = 0;
    for (int i = 0; i < numtempsensors; i++)
        HasError(ArtemisTemperatureSensorInfo(hCam, i + 1, &temperature), __LINE__);

    double retVal;

    retVal = double(temperature) / 100;
    return retVal;
}

void CCameraUnit_TIS::SetBinningAndROI(int binX, int binY, int x_min, int x_max, int y_min, int y_max)
{
    if (!m_initializationOK)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(cs_);
    if (!m_initializationOK)
    {
        return;
    }

    if (binX < 1)
        binX = 1;
    if (binX > 16)
        binX = 16;

    bool change_bin = false;
    if (binningX_ != binX)
    {
        change_bin = true;
    }

    if (binY < 1)
        binY = 1;
    if (binY > 16)
        binY = 16;

    if (binningY_ != binY)
    {
        change_bin = true;
    }

    if (change_bin)
    {
        if (HasError(ArtemisBin(hCam, binX, binY), __LINE__))
            return;
        binningY_ = binY;
        binningX_ = binX;
    }

    int imageLeft, imageRight, imageTop, imageBottom;

    imageLeft = x_min;
    imageRight = (x_max - x_min) + imageLeft;
    imageTop = y_min;
    imageBottom = (y_max - y_min) + imageTop;

    if (imageRight > GetCCDWidth())
        imageRight = GetCCDWidth();
    if (imageLeft < 0)
        imageLeft = 0;
    if (imageRight <= imageLeft)
        imageRight = GetCCDWidth();

    if (imageBottom > GetCCDWidth())
        imageBottom = GetCCDHeight();
    if (imageTop < 0)
        imageTop = 0;
    if (imageBottom <= imageTop)
        imageBottom = GetCCDHeight();

    if (!HasError(ArtemisSubframe(hCam, imageLeft, imageTop, imageRight - imageLeft, imageBottom - imageTop), __LINE__))
    {
        imageLeft_ = imageLeft;
        imageRight_ = imageRight;
        imageTop_ = imageTop_;
        imageBottom_ = imageBottom_;
        roiLeft = imageLeft;
        roiRight = imageRight;
        roiBottom = imageBottom;
        roiTop = imageTop;
    }
    else
    {
        imageLeft_ = 0;
        imageRight_ = GetCCDWidth();
        imageTop_ = 0;
        imageBottom_ = GetCCDHeight();
        roiLeft = imageLeft_;
        roiRight = imageRight_;
        roiBottom = imageBottom_;
        roiTop = imageTop_;
    }

    // printf("%d %d, %d %d | %d %d\n", binningX_, binningY_, imageLeft_, imageRight_, imageBottom_, imageTop_);
}

const ROI *CCameraUnit_TIS::GetROI() const
{
    static ROI roi;
    roi.x_min = roiLeft;
    roi.x_max = roiRight;
    roi.y_min = roiBottom;
    roi.y_max = roiTop;
    return &roi;
}

void CCameraUnit_TIS::SetShutter(bool open)
{
    if (!m_initializationOK)
    {
        return;
    }
    if (hasshutter)
    {
        if (open)
        {
            HasError(ArtemisOpenShutter(hCam), __LINE__);
            requestShutterOpen_ = true;
        }
        else
        {
            HasError(ArtemisCloseShutter(hCam), __LINE__);
            requestShutterOpen_ = false;
        }
    }
}

void CCameraUnit_TIS::SetShutterIsOpen(bool open)
{
    SetShutter(open);
}

// exposure time done
void CCameraUnit_TIS::SetExposure(float exposureInSeconds)
{
    if (!m_initializationOK)
    {
        return;
    }

    if (exposureInSeconds <= 0)
    {
        exposureInSeconds = 0.0;
    }

    long int maxexposurems = exposureInSeconds * 1000;

    if (maxexposurems > 10 * 60 * 1000) // max exposure 10 minutes
        maxexposurems = 10 * 60 * 1000;
    std::lock_guard<std::mutex> lock(cs_);
    exposure_ = maxexposurems * 0.001; // 1 ms increments only
}

void CCameraUnit_TIS::SetReadout(int ReadSpeed)
{
    if (!m_initializationOK)
    {
        return;
    }

    return;
}