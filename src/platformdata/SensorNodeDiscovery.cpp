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

#define LOG_TAG SensorNodeDiscovery

#include "SensorNodeDiscovery.h"

#include <fcntl.h>

#include <set>

#include <v4l2_device.h>

#include "iutils/CameraLog.h"
#include "iutils/Utils.h"

namespace icamera {

// Generous upper bound on the number of routes a single subdev (e.g. a GMSL
// deserializer multiplexing a handful of virtual channels) can report.
static const uint32_t kMaxRoutesPerSubdev = 64;
// Upper bound on the number of native modes (VIDIOC_SUBDEV_ENUM_FRAME_SIZE indices)
// a sensor is expected to enumerate before we give up.
static const int kMaxEnumeratedModes = 16;
// Upper bound on the number of discrete frame intervals (VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL
// indices) a sensor is expected to enumerate for a given mode before we give up.
static const int kMaxEnumeratedFrameIntervals = 16;

SensorNodeDiscovery::SensorNodeDiscovery(MediaControl* mediaCtl) : mMediaCtl(mediaCtl) {}

bool SensorNodeDiscovery::queryActiveRoute(const std::string& devName, int sinkPad,
                                           int sinkStream, int* srcStream) {
    V4L2Subdevice subdev(devName);
    if (subdev.Open(O_RDWR) != 0) {
        LOGW("%s: failed to open %s to query routing", __func__, devName.c_str());
        return false;
    }

    std::vector<v4l2_subdev_route> routes(kMaxRoutesPerSubdev);
    uint32_t numRoutes = kMaxRoutesPerSubdev;
    int ret = subdev.GetRouting(routes.data(), &numRoutes);
    if (ret != 0) {
        // Routing ioctl unsupported on this entity: treat it as a plain passthrough,
        // the stream id is carried through unchanged.
        subdev.Close();
        *srcStream = sinkStream;
        return true;
    }

    bool found = false;
    for (uint32_t i = 0; i < numRoutes; i++) {
        const v4l2_subdev_route& r = routes[i];
        if ((r.sink_pad == static_cast<uint32_t>(sinkPad)) &&
            (r.sink_stream == static_cast<uint32_t>(sinkStream)) &&
            (r.flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE)) {
            *srcStream = r.source_stream;
            found = true;
            break;
        }
    }
    subdev.Close();

    if (!found) {
        LOGW("%s: no active route on %s matching sink pad %d stream %d", __func__,
             devName.c_str(), sinkPad, sinkStream);
    }
    return found;
}

bool SensorNodeDiscovery::probeActiveTopology(const std::string& sensorEntityName,
                                              std::vector<HopState>* hops) {
    std::vector<DiscoveredNode> chain;
    int ret = mMediaCtl->discoverActiveChain(sensorEntityName, &chain);
    if (ret != OK || chain.size() < 2) {
        LOGW("%s: no active chain found starting at %s (ret %d); is MediaCtlConfig "
             "really configured out-of-band?",
             __func__, sensorEntityName.c_str(), ret);
        return false;
    }

    hops->clear();
    hops->reserve(chain.size());
    int curStream = 0;  // the sensor's own local stream id on its source pad

    for (size_t i = 0; i < chain.size(); i++) {
        HopState hop;
        hop.node = chain[i];
        hop.sinkStream = (i == 0) ? -1 : curStream;

        if (chain[i].entityType == MEDIA_ENT_T_V4L2_VIDEO) {
            // Terminal ISYS capture video node: read back the final captured format
            // as a sanity check that it matches what the subdev chain produced.
            V4L2VideoNode vnode(chain[i].devName);
            if (vnode.Open(O_RDWR) != 0) {
                LOGW("%s: failed to open terminal video node %s", __func__,
                     chain[i].devName.c_str());
                return false;
            }
            V4L2Format fmt;
            int fret = vnode.GetFormat(&fmt);
            vnode.Close();
            if (fret != 0) {
                LOGW("%s: failed to G_FMT terminal video node %s", __func__,
                     chain[i].devName.c_str());
                return false;
            }
            hop.width = fmt.Width();
            hop.height = fmt.Height();
            hop.mbusCode = 0;  // video node reports a pixel format, not an mbus code
            hop.field = fmt.Field();
            hop.srcStream = curStream;
            hops->push_back(hop);
            return true;
        }

        // Discover the stream id remap (if any) performed by this entity's routing
        // table on the way in, then read the resulting format on the way out.
        int srcStream = curStream;
        if (i > 0) {
            if (!queryActiveRoute(chain[i].devName, chain[i].sinkPad, curStream, &srcStream)) {
                return false;
            }
        }

        V4L2Subdevice subdev(chain[i].devName);
        if (subdev.Open(O_RDWR) != 0) {
            LOGW("%s: failed to open %s", __func__, chain[i].devName.c_str());
            return false;
        }
        int width = 0, height = 0, code = 0, field = 0;
        int fret = subdev.GetPadFormat(chain[i].srcPad, srcStream, &width, &height, &code, &field);
        subdev.Close();
        if (fret != 0) {
            LOGW("%s: failed to read active format on %s pad %d stream %d", __func__,
                 chain[i].devName.c_str(), chain[i].srcPad, srcStream);
            return false;
        }

        hop.width = width;
        hop.height = height;
        hop.mbusCode = code;
        hop.field = field;
        hop.srcStream = srcStream;
        hops->push_back(hop);
        curStream = srcStream;
    }

    // Only reached if the chain never terminated in a video node.
    return false;
}

MediaCtlConf SensorNodeDiscovery::buildMediaCtlConf(const std::vector<HopState>& hops, int mcId,
                                                    int width, int height, int format, int field) {
    MediaCtlConf mc;
    mc.mcId = mcId;
    mc.configMode.push_back(CAMERA_STREAM_CONFIGURATION_MODE_AUTO);
    mc.outputWidth = width;
    mc.outputHeight = height;
    mc.format = format;

    for (size_t i = 0; i + 1 < hops.size(); i++) {
        const HopState& cur = hops[i];
        const HopState& next = hops[i + 1];

        // link: cur's src pad -> next's sink pad (both entities/pads already known
        // from the active chain we walked; the link is already enabled out-of-band,
        // this entry only documents/re-asserts it, it is not required to be applied).
        McLink link;
        link.srcEntityName = cur.node.entityName;
        link.srcEntity = mMediaCtl->getEntityIdByName(cur.node.entityName);
        link.srcPad = cur.node.srcPad;
        link.sinkEntityName = next.node.entityName;
        link.sinkEntity = mMediaCtl->getEntityIdByName(next.node.entityName);
        link.sinkPad = next.node.sinkPad;
        link.enable = true;
        mc.links.push_back(link);

        // routing: only meaningful (and only recorded) when this hop's discovered
        // stream id actually changed, i.e. the entity has a real routing table.
        if (i > 0 && cur.srcStream != cur.sinkStream) {
            McRoute route;
            route.entityName = cur.node.entityName;
            route.entity = mMediaCtl->getEntityIdByName(cur.node.entityName);
            route.sinkPad = cur.node.sinkPad;
            route.sinkStream = cur.sinkStream;
            route.srcPad = cur.node.srcPad;
            route.srcStream = cur.srcStream;
            route.flag = V4L2_SUBDEV_ROUTE_FL_ACTIVE;
            mc.routings[route.entityName].push_back(route);
        }

        // format on the current hop's source pad, geometry substituted for the
        // requested mode (same assumption the hand-authored isx031 JSON makes: the
        // routing/pad topology is identical across native sensor modes, only the
        // width/height changes).
        if (cur.node.entityType != MEDIA_ENT_T_V4L2_VIDEO) {
            McFormat fmt;
            fmt.entityName = cur.node.entityName;
            fmt.entity = mMediaCtl->getEntityIdByName(cur.node.entityName);
            fmt.pad = cur.node.srcPad;
            fmt.stream = cur.srcStream;
            fmt.width = width;
            fmt.height = height;
            fmt.pixelCode = cur.mbusCode;
            fmt.formatType = FC_FORMAT;
            fmt.type = RESOLUTION_TARGET;
            mc.formats.push_back(fmt);
        }
    }

    const HopState& last = hops.back();
    McVideoNode videoNode;
    videoNode.name = last.node.entityName;
    videoNode.videoNodeType = VIDEO_GENERIC;
    mc.videoNodes.push_back(videoNode);

    return mc;
}

void SensorNodeDiscovery::discoverFpsRange(const std::string& sensorDevName, int pad,
                                           int mbusCode, int width, int height,
                                           std::vector<double>* outFpsRange) {
    CheckAndLogError(!outFpsRange, VOID_VALUE, "%s: nullptr output", __func__);

    V4L2Subdevice subdev(sensorDevName);
    if (subdev.Open(O_RDWR) != 0) {
        LOGW("%s: failed to open %s to enumerate frame intervals", __func__,
             sensorDevName.c_str());
        return;
    }

    double minFps = -1.0, maxFps = -1.0;
    for (int index = 0; index < kMaxEnumeratedFrameIntervals; index++) {
        int numerator = 0, denominator = 0;
        int ret = subdev.EnumFrameInterval(pad, mbusCode, width, height, index, &numerator,
                                           &denominator);
        if (ret != 0) break;
        if (numerator <= 0) continue;

        double fps = static_cast<double>(denominator) / static_cast<double>(numerator);
        if ((minFps < 0.0) || (fps < minFps)) minFps = fps;
        if ((maxFps < 0.0) || (fps > maxFps)) maxFps = fps;
    }
    subdev.Close();

    if (minFps < 0.0) {
        LOGW("%s: %s doesn't support VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL; fpsRange can't be "
             "self discovered, keep it out of the JSON or hand-author it",
             __func__, sensorDevName.c_str());
        return;
    }

    // Mirror the shape of hand-authored fpsRange entries (pairs of min/max target fps
    // the AE algorithm may pick from): a locked slow range, an auto range spanning the
    // full discovered span, and (if distinct) a locked fast range.
    outFpsRange->push_back(minFps);
    outFpsRange->push_back(minFps);
    if (maxFps > minFps) {
        outFpsRange->push_back(minFps);
        outFpsRange->push_back(maxFps);
        outFpsRange->push_back(maxFps);
        outFpsRange->push_back(maxFps);
    }

    LOGI("%s: self discovered fps range [%.2f, %.2f] on %s", __func__, minFps, maxFps,
        sensorDevName.c_str());
}

bool SensorNodeDiscovery::countActiveSinkStreams(const std::string& devName, int sinkPad,
                                                 int* count) {
    V4L2Subdevice subdev(devName);
    if (subdev.Open(O_RDWR) != 0) {
        LOGW("%s: failed to open %s to count multiplexed streams", __func__, devName.c_str());
        return false;
    }

    std::vector<v4l2_subdev_route> routes(kMaxRoutesPerSubdev);
    uint32_t numRoutes = kMaxRoutesPerSubdev;
    int ret = subdev.GetRouting(routes.data(), &numRoutes);
    subdev.Close();
    if (ret != 0) {
        // No routing support at all: nothing is multiplexed on this entity.
        return false;
    }

    std::set<uint32_t> streams;
    for (uint32_t i = 0; i < numRoutes; i++) {
        const v4l2_subdev_route& r = routes[i];
        if ((r.sink_pad == static_cast<uint32_t>(sinkPad)) &&
            (r.flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE)) {
            streams.insert(r.sink_stream);
        }
    }

    *count = static_cast<int>(streams.size());
    return true;
}

void SensorNodeDiscovery::discoverVcInfo(const std::vector<HopState>& hops,
                                         DiscoveredVcInfo* outVcInfo) {
    CheckAndLogError(!outVcInfo, VOID_VALUE, "%s: nullptr output", __func__);

    // The CSI-2 receiver is the last subdev before the terminal video node; it is the
    // entity whose sink pad actually carries the multiplexed virtual channels, and
    // whose stream ids *are* the CSI-2 VC indices. A chain too short to contain one
    // (sensor wired straight to a video node) has no virtual channels by definition.
    if (hops.size() < 3) return;
    const HopState& receiver = hops[hops.size() - 2];

    int count = 0;
    if (!countActiveSinkStreams(receiver.node.devName, receiver.node.sinkPad, &count)) {
        LOG1("%s: %s has no routing table, treating sensor as non-multiplexed", __func__,
             receiver.node.entityName.c_str());
        return;
    }

    if (count <= 1) {
        // Exactly one stream on the receiver: not actually multiplexed, so leave the
        // VC info at its "no virtual channel" default rather than claiming a 1-VC
        // group, matching how non-GMSL sensor JSONs simply omit these keys.
        LOG1("%s: %s carries a single stream, no virtual channel handling needed",
             __func__, receiver.node.entityName.c_str());
        return;
    }

    outVcInfo->count = count;
    // The stream id this sensor's data arrives on at the receiver's sink pad is its
    // CSI-2 virtual channel, already resolved hop by hop through every serializer /
    // deserializer routing table on the way here.
    outVcInfo->id = receiver.sinkStream;

    // vcGroupId is *not* simply "which aggregator am I behind": CameraHal uses it as a
    // mutual exclusion key, permitting only one group to be open at a time. Reporting
    // e.g. the CSI-2 port index here would therefore make cameras behind different
    // deserializers un-openable together, which is a behaviour change no board asked
    // for -- every existing sensor JSON pins the group to 0, and the runtime default
    // (-1) is coerced to 0 by CameraHal as well. So keep 0 and let a board that really
    // does need exclusive groups say so explicitly via "vcGroupId" in its JSON.
    outVcInfo->groupId = 0;

    LOGI("%s: self discovered virtual channel %d of %d on %s (group %d)", __func__,
        outVcInfo->id, outVcInfo->count, receiver.node.entityName.c_str(), outVcInfo->groupId);
}

bool SensorNodeDiscovery::discover(const std::string& sensorEntityName,
                                   std::vector<MediaCtlConf>* outConfs,
                                   std::map<int, stream_array_t>* outStreamMap,
                                   std::vector<double>* outFpsRange,
                                   DiscoveredVcInfo* outVcInfo) {
    CheckAndLogError(!outConfs || !outStreamMap, false, "%s: nullptr output", __func__);

    std::vector<HopState> hops;
    if (!probeActiveTopology(sensorEntityName, &hops) || hops.size() < 2) {
        return false;
    }

    // hops[0] is the sensor itself; hops[hops.size()-2] is the last subdev before the
    // terminal video node, whose discovered mbus code drives the ISYS output format.
    int mbusCode = hops[hops.size() - 2].mbusCode;
    int format = CameraUtils::getPixelFormatFromMBusCode(mbusCode);
    if (format == 0) {
        LOGW("%s: mbus code %s on %s is not a self-discoverable passthrough format; "
             "this sensor needs a hand-authored JSON (Bayer/ISP path)",
             __func__, CameraUtils::pixelCode2String(mbusCode), sensorEntityName.c_str());
        return false;
    }

    // Enumerate every native mode the sensor supports at this mbus code, without
    // switching the currently active one (VIDIOC_SUBDEV_ENUM_FRAME_SIZE is read-only).
    V4L2Subdevice sensorSubdev(hops[0].node.devName);
    if (sensorSubdev.Open(O_RDWR) != 0) {
        LOGW("%s: failed to (re)open sensor %s to enumerate modes", __func__,
             hops[0].node.devName.c_str());
        return false;
    }

    outConfs->clear();
    outStreamMap->clear();
    if (outFpsRange) {
        outFpsRange->clear();
        discoverFpsRange(hops[0].node.devName, hops[0].node.srcPad, mbusCode, hops[0].width,
                         hops[0].height, outFpsRange);
    }
    if (outVcInfo) {
        *outVcInfo = DiscoveredVcInfo();
        discoverVcInfo(hops, outVcInfo);
    }
    int mcId = 0;
    bool sawActiveMode = false;
    for (int index = 0; index < kMaxEnumeratedModes; index++) {
        int width = 0, height = 0;
        int ret = sensorSubdev.EnumFrameSize(hops[0].node.srcPad, mbusCode, index, &width, &height);
        if (ret != 0) break;

        if ((width == hops[0].width) && (height == hops[0].height)) sawActiveMode = true;

        stream_t cfg;
        CLEAR(cfg);
        cfg.format = format;
        cfg.width = width;
        cfg.height = height;
        cfg.field = hops[0].field;

        outConfs->push_back(buildMediaCtlConf(hops, mcId, width, height, format, cfg.field));
        (*outStreamMap)[mcId].push_back(cfg);
        mcId++;
    }
    sensorSubdev.Close();

    if (outConfs->empty()) {
        // Sensor doesn't support frame size enumeration (older/simpler subdev driver);
        // fall back to a single mode built from the currently active format only.
        stream_t cfg;
        CLEAR(cfg);
        cfg.format = format;
        cfg.width = hops[0].width;
        cfg.height = hops[0].height;
        cfg.field = hops[0].field;

        outConfs->push_back(
            buildMediaCtlConf(hops, 0, hops[0].width, hops[0].height, format, cfg.field));
        (*outStreamMap)[0].push_back(cfg);
        sawActiveMode = true;
    }

    if (!sawActiveMode) {
        LOGW("%s: enumerated modes for %s didn't include the currently active %dx%d; "
             "discovery may be incomplete",
             __func__, sensorEntityName.c_str(), hops[0].width, hops[0].height);
    }

    LOGI("%s: self discovered %zu mode(s) for %s (mbus %s -> %s)", __func__, outConfs->size(),
        sensorEntityName.c_str(), CameraUtils::pixelCode2String(mbusCode),
        CameraUtils::pixelCode2String(format));
    return true;
}

}  // namespace icamera
