/****************************************************************************
 *
 *   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#pragma once


#include <uORB/uorbNew/uorb.h>
#include <inttypes.h>
#include <string>
#include <Eigen/Core>


#ifndef __cplusplus

#endif
#define LIDAR_MSG_MAX_LEN 1000000

#ifdef __cplusplus
struct __EXPORT lidar_msg_s {
#else
    struct lidar_msg_s {
#endif
    uint64_t timestamp;
    uint64_t device_id;

    struct EIGEN_ALIGN16 PointXYZIRT ///< user defined point type
    {
        float x;
        float y;
        float z;
        uint8_t intensity;
        double timestamp;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct tmp {
        char frame_id[64] = "";      ///< Point cloud frame id
        uint32_t seq = 0;               ///< Sequence number of message
        uint32_t height = 0;            ///< Height of point cloud
        uint32_t width = 0;             ///< Width of point cloud
        bool is_dense = false;          ///< If is_dense=true, the point cloud does not contain NAN points
        PointXYZIRT buffer[LIDAR_MSG_MAX_LEN];
        int points_num;
    } points;

    lidar_msg_s() {};

#ifdef __cplusplus

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(lidar_msg, lidar_msg_s);

