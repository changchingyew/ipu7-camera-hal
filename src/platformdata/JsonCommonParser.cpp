/*
 * Copyright (C) 2022-2024 Intel Corporation.
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

#define LOG_TAG JsonCommonParser

#include "JsonCommonParser.h"

#include <string>
#include <vector>

namespace icamera {

CameraCommonParser::CameraCommonParser(PlatformData::StaticCfg* cfg) : mStaticCfg(cfg) {}

CameraCommonParser::~CameraCommonParser() {}

bool CameraCommonParser::run(const std::string& filename) {
    auto root = openJsonFile(filename);
    if (root.empty()) {
        return true;
    }

    if (!root.isMember("Common")) {
        return false;
    }

    const Json::Value& node = root["Common"];

    if (node.isMember("version")) {
        mStaticCfg->mCommonConfig.xmlVersion = node["version"].asFloat();
    }
    if (node.isMember("platform")) {
        mStaticCfg->mCommonConfig.ipuName = node["platform"].asString();
    }
    if (node.isMember("availableSensors")) {
        auto ele = node["availableSensors"];
        for (Json::Value::ArrayIndex i = 0; i < ele.size(); ++i)
            mStaticCfg->mCommonConfig.availableSensors.push_back(ele[i].asString());
    }

    if (node.isMember("cameraNumber")) {
        mStaticCfg->mCommonConfig.cameraNumber = node["cameraNumber"].asInt();
    }
    if (node.isMember("videoStreamNum")) {
        mStaticCfg->mCommonConfig.videoStreamNum = node["videoStreamNum"].asInt();
    }
    if (node.isMember("useGpuProcessor")) {
        mStaticCfg->mCommonConfig.useGpuProcessor = node["useGpuProcessor"].asBool();
    }
    if (node.isMember("autoDiscoverSensors")) {
        auto ele = node["autoDiscoverSensors"];
        for (Json::Value::ArrayIndex i = 0; i < ele.size(); ++i)
            mStaticCfg->mCommonConfig.autoDiscoverSensors.push_back(ele[i].asString());
    }
    return true;
}

}  // namespace icamera
