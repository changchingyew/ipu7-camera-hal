/*
 * Copyright (C) 2026 Intel Corporation.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include "MediaControl.h"
#include "PlatformData.h"

namespace icamera {

/**
 * \class SensorNodeDiscovery
 *
 * Self discovery of MediaCtlConf / supportedStreamConfig for "simple yuv" sensors and
 * bridges (no Bayer/ISP processing, e.g. HDMI-to-CSI bridges or GMSL YUV sensors such
 * as isx031 behind a max9x serializer/deserializer pair) whose media graph links and
 * routing have already been configured out-of-band -- e.g. by a udev/board-init
 * service, or the kernel driver's default routing -- before ipu7-camera-hal starts.
 *
 * Instead of hand-authoring the MediaCtlConfig/supportedStreamConfig sections of a
 * sensor's config/linux/ipuN/sensors JSON file (which, for a multi-virtual-channel GMSL
 * sensor such as isx031, run to well over a thousand lines of routing tables), this
 * class queries the live v4l2 subdevs (VIDIOC_SUBDEV_G_FMT / VIDIOC_SUBDEV_G_ROUTING /
 * VIDIOC_SUBDEV_ENUM_FRAME_SIZE) to derive the exact same in-memory MediaCtlConf and
 * stream_array_t data structures that the JSON parser would otherwise build, so no
 * change is required anywhere downstream (CaptureUnit, PlatformData, MediaControl).
 *
 * Only the mbus codes covered by CameraUtils::getPixelFormatFromMBusCode() (YUV/RGB
 * "passthrough" formats) can be self discovered this way: Bayer/RAW sensors that need
 * ISP demosaic, multi-exposure HDR composition, or PSys tuning still require the
 * hand-authored JSON, since none of that is discoverable from the subdev alone.
 */
class SensorNodeDiscovery {
 public:
    /**
     * \struct DiscoveredVcInfo
     *
     * CSI-2 virtual channel placement of a sensor, self discovered from the routing
     * table of the CSI-2 receiver it ultimately feeds. Mirrors the hand-authored
     * "vcCount"/"vcId"/"vcGroupId" keys of a sensor JSON.
     */
    struct DiscoveredVcInfo {
        int count;    // number of VCs multiplexed onto the same CSI-2 receiver sink pad,
                      // 0 when the sensor isn't multiplexed at all (no VC handling)
        int id;       // this sensor's own VC index on that receiver
        int groupId;  // set of cameras that may be open concurrently, see discoverVcInfo()
        DiscoveredVcInfo() : count(0), id(0), groupId(-1) {}
    };

    explicit SensorNodeDiscovery(MediaControl* mediaCtl);
    ~SensorNodeDiscovery() = default;

    /**
     * \brief Discover the MediaCtlConf(s) and supportedStreamConfig entries for a
     *        sensor entity whose graph is already active.
     *
     * \param sensorEntityName: name of the sensor's own v4l2 subdev entity, e.g.
     *                          "isx031 a-0" (as reported by the kernel media graph,
     *                          not the generic sensor name "isx031").
     * \param outConfs: filled with one MediaCtlConf per discovered native sensor mode
     *                  (mcId 0..N-1), topology (links/routings) identical across
     *                  modes, size varying per mode.
     * \param outStreamMap: filled with mcId -> stream_array_t, ready to be merged into
     *                      PlatformData::StaticCfg::CameraInfo::mStreamToMcMap.
     * \param outFpsRange: optional; if non-null, filled with StaticMetadata-style
     *                     fpsRange pairs (min0, max0, min1, max1, ...) derived from
     *                     VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL on the sensor's active
     *                     mode, ready to be merged into
     *                     PlatformData::StaticCfg::CameraInfo::mStaticMetadata.mFpsRange
     *                     when the JSON doesn't already provide a "fpsRange" (that
     *                     value, if present, always wins over the discovered one).
     * \param outVcInfo: optional; if non-null, filled with the sensor's CSI-2 virtual
     *                   channel placement (the "vcCount"/"vcId"/"vcGroupId" JSON keys)
     *                   derived from the CSI-2 receiver's routing table. As with
     *                   outFpsRange, explicit JSON values always take precedence.
     *
     * \return true if at least one mode was discovered and the active topology could
     *         be read back end-to-end (sensor -> ... -> ISYS capture video node),
     *         false otherwise (e.g. graph not yet configured out-of-band, or the
     *         active mbus code isn't a self-discoverable passthrough format).
     */
    bool discover(const std::string& sensorEntityName, std::vector<MediaCtlConf>* outConfs,
                 std::map<int, stream_array_t>* outStreamMap,
                 std::vector<double>* outFpsRange = nullptr,
                 DiscoveredVcInfo* outVcInfo = nullptr);

 private:
    // One hop of the discovered active chain, augmented with the live format/routing
    // state read back from the corresponding subdev/video node.
    struct HopState {
        DiscoveredNode node;
        int sinkStream;  // stream id incoming to this hop, -1 for the start (sensor) hop
        int srcStream;   // stream id outgoing from this hop
        int width;
        int height;
        int mbusCode;
        int field;
        HopState() : sinkStream(-1), srcStream(0), width(0), height(0), mbusCode(0), field(0) {}
    };

    MediaControl* mMediaCtl;

    bool probeActiveTopology(const std::string& sensorEntityName, std::vector<HopState>* hops);
    bool queryActiveRoute(const std::string& devName, int sinkPad, int sinkStream, int* srcStream);
    MediaCtlConf buildMediaCtlConf(const std::vector<HopState>& hops, int mcId, int width,
                                   int height, int format, int field);
    void discoverFpsRange(const std::string& sensorDevName, int pad, int mbusCode, int width,
                          int height, std::vector<double>* outFpsRange);
    void discoverVcInfo(const std::vector<HopState>& hops, DiscoveredVcInfo* outVcInfo);
    bool countActiveSinkStreams(const std::string& devName, int sinkPad, int* count);
};

}  // namespace icamera
