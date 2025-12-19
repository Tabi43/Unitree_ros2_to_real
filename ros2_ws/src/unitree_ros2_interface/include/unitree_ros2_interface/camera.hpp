#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>

#ifndef UNITREE_ROS2_INTERFACE_CAMERA_HPP_
#define UNITREE_ROS2_INTERFACE_CAMERA_HPP_

/**
 * @file camera.hpp
 * @brief Camera information and enumerations for the Unitree ROS 2 interface.
 *
 * This header mirrors the definitions from the original ROS 1
 * ``camera.hpp`` found in the ``unitree_ros_interface`` package.  It
 * provides an enumeration describing the side of a stereo pair and a
 * simple structure describing the IP, port and naming conventions
 * associated with each physical camera on a Unitree robot.  Helper
 * functions allow lookup of a ``CameraInfo`` by name or numeric ID.
 *
 * The definitions here are deliberately minimal: all ROS 1
 * dependencies have been removed so that this header can be used
 * directly from a ROS 2 package.  Paths to calibration files and
 * configuration YAML are resolved at runtime by the camera node via
 * ``ament_index_cpp::get_package_share_directory()`` rather than
 * ``ros::package::getPath()``.  See ``camera_interface_node.cpp`` for
 * details.
 */

#include <string>

namespace unitree_ros2_interface {

/// Identifies which side of a stereo camera pair is being referred to.
enum class CameraSide {
    LEFT = 0,
    RIGHT = 1,
    STEREO = 2,
    DEPTH = 3,
};

/**
 * @brief Simple description of a physical camera on the robot.
 *
 * The ``CameraInfo`` struct records the camera's IP address, UDP
 * communication port, device ID and node ID used by the Unitree
 * camera SDK.  It also records human‑friendly names for the camera
 * itself and for the coordinate frames associated with the left and
 * right lenses of a stereo pair.  A default constructor is provided
 * for cases where fields will be filled programmatically, as well as
 * a convenience constructor for initialising all members in one
 * expression.
 */
struct CameraInfo {
    std::string ip;              ///< IP address of the camera.
    unsigned int port;           ///< UDP port used for image transfer.
    unsigned int devId;          ///< Device ID used by the SDK.
    unsigned int nodeId;         ///< Node ID used by the SDK when
                                 ///< opening the device by node number.
    std::string name;            ///< Canonical name (e.g. "front_camera").
    std::string camera_frame;    ///< Frame id for the stereo pair.
    std::string camera_frame_left;  ///< Frame id for the left lens.
    std::string camera_frame_right; ///< Frame id for the right lens.

    /// Default constructor initialises fields with sensible defaults.
    CameraInfo(): ip(""), port(0), devId(0), nodeId(0), name("") {
        camera_frame = "";
        camera_frame_left = "";
        camera_frame_right = "";
    }

    /**
     * @brief Convenience constructor.
     *
     * Constructs a ``CameraInfo`` specifying all relevant fields.  When
     * specifying a stereo pair, ``frame_name`` should identify the
     * common parent frame of the two lenses; ``camera_frame_left`` and
     * ``camera_frame_right`` will be derived automatically by
     * appending ``"_left"`` and ``"_right"`` respectively.  If a
     * different naming scheme is required, assign to these members
     * directly after construction.
     */
    CameraInfo(const std::string &ip_, unsigned int port_, unsigned int devId_,
               const std::string &name_, const std::string &frame_name_, unsigned int nodeId_ = 0)
        : ip(ip_), port(port_), devId(devId_), nodeId(nodeId_), name(name_),
          camera_frame(frame_name_),
          camera_frame_left(frame_name_ + "_left"),
          camera_frame_right(frame_name_ + "_right") {}
};

/**
 * Predefined camera configurations for the Unitree Go1 robot.  These
 * correspond to the values used by the Camera SDK Configuration.  Each camera
 * has a unique IP address on the robot's internal network and a
 * corresponding UDP port.  The ``devId`` field identifies the camera
 * within the SDK, and the ``nodeId`` field identifies the device
 * number when opening by node (e.g. /dev/video0).
*/
static inline CameraInfo frontCamera  = CameraInfo("192.168.123.13", 9201, 1, "front_camera",  "camera_face", 1);
static inline CameraInfo chinCamera   = CameraInfo("192.168.123.13", 9202, 2, "chin_camera",   "camera_chin");
static inline CameraInfo leftCamera   = CameraInfo("192.168.123.14", 9203, 3, "left_camera",   "camera_left", 1);
static inline CameraInfo rightCamera  = CameraInfo("192.168.123.14", 9204, 4, "right_camera",  "camera_right");
static inline CameraInfo bottomCamera = CameraInfo("192.168.123.15", 9205, 5, "bottom_camera", "camera_rearDown");

/**
 * @brief Retrieve camera information by name.
 *
 * Looks up a camera by its canonical name and returns a copy of the
 * associated ``CameraInfo`` via the ``cameraInfo`` argument.  If the
 * name is not recognised, the function returns ``false`` and leaves
 * ``cameraInfo`` unchanged.
 *
 * @param camera_name The canonical camera name (e.g. "front_camera").
 * @param cameraInfo  Output parameter to receive the corresponding
 *                    ``CameraInfo`` on success.
 * @return ``true`` if the camera name was recognised, ``false`` otherwise.
 */
inline bool getCameraInfo(const std::string &camera_name, CameraInfo &cameraInfo) {
    if (camera_name == "front_camera") {
        cameraInfo = frontCamera;
        return true;
    } else if (camera_name == "chin_camera") {
        cameraInfo = chinCamera;
        return true;
    } else if (camera_name == "left_camera") {
        cameraInfo = leftCamera;
        return true;
    } else if (camera_name == "right_camera") {
        cameraInfo = rightCamera;
        return true;
    } else if (camera_name == "bottom_camera") {
        cameraInfo = bottomCamera;
        return true;
    } else {
        return false;
    }
    return false;
}

/**
 * @brief Retrieve camera information by numerical ID.
 *
 * Looks up a camera by its ID (as used in the original ROS 1
 * implementation) and returns a copy of the associated ``CameraInfo``
 * via the ``cameraInfo`` argument.  If the ID is not recognised,
 * the function returns ``false`` and leaves ``cameraInfo`` unchanged.
 *
 * @param id         Numeric identifier of the camera.
 * @param cameraInfo Output parameter to receive the corresponding
 *                   ``CameraInfo`` on success.
 * @return ``true`` if the camera ID was recognised, ``false`` otherwise.
 */
inline bool getCameraInfo(const unsigned int id, CameraInfo &cameraInfo) {
    switch (id) {
    case 1:
        cameraInfo = frontCamera;
        return true;
    case 2:
        cameraInfo = chinCamera;
        return true;
    case 3:
        cameraInfo = leftCamera;
        return true;
    case 4:
        cameraInfo = rightCamera;
        return true;
    case 5:
        cameraInfo = bottomCamera;
        return true;
    default:
        return false;
    }
    return false;
}

} // namespace unitree_ros2_interface

#endif  // UNITREE_ROS2_INTERFACE_CAMERA_HPP_