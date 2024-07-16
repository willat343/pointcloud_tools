#ifndef POINTCLOUD_TOOLS_PCLPOINTCLOUD2_UTILITIES_HPP
#define POINTCLOUD_TOOLS_PCLPOINTCLOUD2_UTILITIES_HPP

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_traits.h>
#include <statistics_msgs/SummaryStatistics.h>
#include <statistics_msgs/SummaryStatisticsArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace pct {

/**
 * @brief Create a new pointcloud with an additional field appended.
 *
 * This function copies all the src data and modifies the pointcloud and field metadata to account for the new field.
 * The bytes corresponding to the new data fields are uninitialised.
 *
 * @param src
 * @param name
 * @param field_type
 * @param count
 * @return pcl::PCLPointCloud2
 */
pcl::PCLPointCloud2 add_field(const pcl::PCLPointCloud2& src, const std::string& name,
        const pcl::PCLPointField::PointFieldTypes datatype, const std::uint32_t count = 1);

/**
 * @brief Create a new pointcloud with additional fields appended.
 *
 * @param src
 * @param names
 * @param datatype
 * @param count
 * @return pcl::PCLPointCloud2
 */
pcl::PCLPointCloud2 add_fields(const pcl::PCLPointCloud2& src, const std::vector<std::string>& names,
        const pcl::PCLPointField::PointFieldTypes datatype, const std::uint32_t count = 1);

pcl::PCLPointCloud2 add_unit_vectors(const pcl::PCLPointCloud2& src);

/**
 * @brief Cast field from one type to another. Currently the types must be of the same bit depth.
 *
 * See also cast_field_with_scale().
 *
 * @tparam type
 * @param pointcloud
 * @param name
 */
template<std::uint8_t type>
void cast_field(pcl::PCLPointCloud2& pointcloud, const std::string& name);

/**
 * @brief Cast a field from one type to another with a scale. The scaling is done as a double multiplication.
 *
 * @tparam type
 * @param pointcloud
 * @param name
 * @param scale
 */
template<std::uint8_t type>
void cast_field_with_scale(pcl::PCLPointCloud2& pointcloud, const std::string& name, const double scale);

/**
 * @brief Change a point cloud field name.
 *
 * @param pointcloud
 * @param from
 * @param to
 */
void change_field_name(pcl::PCLPointCloud2& pointcloud, const std::string& from, const std::string& to);

/**
 * @brief Return a count of the number of normals which have a norm different from 1 by a threshold. Therefore it is
 * desirable for this number to be 0. It will always be less than or equal to the number of points in the cloud.
 *
 * @param pointcloud
 * @param threshold
 * @return int
 */
int check_normals(const pcl::PCLPointCloud2& pointcloud, const float threshold = 0.000001f);

/**
 * @brief Get a function that will get the data of type T in a pointcloud as type OutT.
 *
 * The function returns a lambda with signature (but with by-value capturing):
 *  `OutT (*)(pcl::PCLPointCloud2& pointcloud, const std::size_t i)`
 * where:
 *  - pointcloud is the pointcloud to get the data from
 *  - i is the index of the point to get
 *
 * Example Usage:
 *      auto get_field_data = pct::create_get_field_data_function<float, float>(field);
 *      for (std::size_t i = 0; i < pct::size_points(pointcloud); ++i) {
 *          float f = get_field_data(pointcloud, i);
 *      }
 *
 * @tparam OutT
 * @tparam T
 * @tparam int
 * @param field
 * @return auto
 */
template<typename OutT, typename T, int>
auto create_get_field_data_function(const pcl::PCLPointField& field);

/**
 * @brief Get a function that will get the data of runtime type in a pointcloud as type OutT.
 *
 * The function returns a lambda with signature (but with by-value capturing):
 *  `OutT (*)(pcl::PCLPointCloud2& pointcloud, const std::size_t i)`
 * where:
 *  - pointcloud is the pointcloud to get the data from
 *  - i is the index of the point to get
 *
 * Example Usage:
 *      auto get_field_data = pct::create_get_field_data_function<float>(field);
 *      for (std::size_t i = 0; i < pct::size_points(pointcloud); ++i) {
 *          float f = get_field_data(pointcloud, i);
 *      }
 *
 * @tparam OutT
 * @param field
 * @return auto
 */
template<typename OutT>
auto create_get_field_data_function(const pcl::PCLPointField& field);

/**
 * @brief Get a function that will set the data of type T in a pointcloud with input data of type InT.
 *
 * The function returns a lambda with signature (but with by-value capturing):
 *  `void (*)(pcl::PCLPointCloud2& pointcloud, const std::size_t i, const InT value)`
 * where:
 *  - pointcloud is the pointcloud to be modified
 *  - i is the index of the point to be modified
 *  - value is the desired value for the point's field
 *
 * Example Usage:
 *      auto set_field_data = pct::create_set_field_data_function<float, float>(field);
 *      for (std::size_t i = 0; i < pct::size_points(pointcloud); ++i) {
 *          set_field_data(pointcloud, i, 5.f);
 *      }
 *
 * @tparam InT
 * @tparam T
 * @tparam int
 * @param field
 * @return auto
 */
template<typename InT, typename T, int>
auto create_set_field_data_function(const pcl::PCLPointField& field);

/**
 * @brief Get a function that will set the data of runtime type in a pointcloud with input data of type InT.
 *
 * The function returns a lambda with signature (but with by-value capturing):
 *  `void (*)(pcl::PCLPointCloud2& pointcloud, const std::size_t i, const InT value)`
 * where:
 *  - pointcloud is the pointcloud to be modified
 *  - i is the index of the point to be modified
 *  - value is the desired value for the point's field
 *
 * Example Usage:
 *      auto set_field_data = pct::create_set_field_data_function<float>(field);
 *      for (std::size_t i = 0; i < pct::size_points(pointcloud); ++i) {
 *          set_field_data(pointcloud, i, 5.f);
 *      }
 *
 * @tparam InT
 * @param field
 * @return auto
 */
template<typename InT>
auto create_set_field_data_function(const pcl::PCLPointField& field);

/**
 * @brief Deskew a pointcloud to `new_time` under the assumption that there has been a constant twist applied over a
 * time `dt` resulting in a transform `skew`. The `src` and `dest` point clouds must have fields [x, y, z]. The
 * [x, y, z] fields of the `dest` point cloud are modified, but no data is copied from `src`. The header time of `dest`
 * is set to new_time.
 *
 * See also `deskew_offset_constant_twist`, `deskew_spin_constant_twist`.
 *
 * @tparam InterpCoeffFunction
 * @param skew skew transformation over `dt` assuming constant twist
 * @param new_time skew start time in microseconds
 * @param new_time new time in microseconds
 * @param dt duration of skew in seconds
 * @param interp_coeff_function function to compute interpolation coefficient for the i-th point in form `(*)(const
 * pcl::PointCloud2& src, const std::size_t i) -> double`
 * @param src source point cloud
 * @param dest destination point cloud
 */
template<typename InterpCoeffFunction>
void deskew_constant_twist(const Eigen::Isometry3d& skew, const std::uint64_t skew_start_time,
        const std::uint64_t new_time, const double dt, const InterpCoeffFunction& interp_coeff_function,
        const pcl::PCLPointCloud2& src, pcl::PCLPointCloud2& dest);

/**
 * @brief Deskew a pointcloud under constant twist assumption when the `src` point cloud has a time field containing
 * absolute times (correct with respect to the header time). Calls `deskew_constant_twist`.
 *
 * @param skew
 * @param new_time
 * @param dt
 * @param time_field
 * @param time_ratio_to_seconds
 * @param src
 * @return pcl::PCLPointCloud2
 */
pcl::PCLPointCloud2 deskew_absolute_constant_twist(const Eigen::Isometry3d& skew, const std::uint64_t skew_start_time,
        const std::uint64_t new_time, const double dt, const std::string& time_field,
        const double time_ratio_to_seconds, const pcl::PCLPointCloud2& src);

/**
 * @brief Deskew a pointcloud under constant twist assumption when the `src` point cloud has a time field containing
 * offset times since the header time. Calls `deskew_constant_twist`.
 *
 * @param skew skew transform
 * @param dt time of point cloud sweep
 * @param new_time target time to deskew to
 * @param time_field name of the time field
 * @param time_ratio_to_seconds scale time field by this ratio to get seconds
 * @param src skewed point cloud
 */
pcl::PCLPointCloud2 deskew_offset_constant_twist(const Eigen::Isometry3d& skew, const std::uint64_t skew_start_time,
        const std::uint64_t new_time, const double dt, const std::string& time_field,
        const double time_ratio_to_seconds, const pcl::PCLPointCloud2& src);

/**
 * @brief Deskew a pointcloud under constant twist assumption when the `src` point cloud has come from one complete
 * rotation of a spinning LiDAR. Calls `deskew_constant_twist`.
 *
 * @param skew
 * @param new_time
 * @param dt
 * @param spin_cw_from_top
 * @param src
 * @return pcl::PCLPointCloud2
 */
pcl::PCLPointCloud2 deskew_spin_constant_twist(const Eigen::Isometry3d& skew, const std::uint64_t skew_start_time,
        const std::uint64_t new_time, const double dt, const bool spin_cw_from_top, const pcl::PCLPointCloud2& src);

/**
 * @brief Check if a point cloud has no points.
 *
 * @param pointcloud
 * @return true
 * @return false
 */
bool empty(const pcl::PCLPointCloud2& pointcloud);

/**
 * @brief Warning: This function should be used for debugging or singular data acquisition. Prefer to access data
 * through the functions generated by create_get_field_data_function(field), especially if accessing the data of many
 * points.
 *
 * @tparam OutT
 * @param pointcloud point cloud
 * @param field point field
 * @param i point index
 * @return OutT
 */
template<typename OutT>
OutT field_data(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field, const std::size_t i);

std::string field_type_to_string(const std::uint8_t field_type);

template<typename T>
void filter_max(const pcl::PCLPointCloud2& src, pcl::PCLPointCloud2& dest, const pcl::PCLPointField& field,
        const T max);

template<typename T>
void filter_max(const pcl::PCLPointCloud2& src, pcl::PCLPointCloud2& dest, const std::string& field_name, const T max);

const pcl::PCLPointField& get_field(const pcl::PCLPointCloud2& pointcloud, const std::string& name);

pcl::PCLPointField& get_field(pcl::PCLPointCloud2& pointcloud, const std::string& name);

bool has_field(const pcl::PCLPointCloud2& pointcloud, const std::string& name);

template<int>
struct is_8bit_type {};
template<int>
struct is_16bit_type {};
template<int>
struct is_32bit_type {};
template<int>
struct is_64bit_type {};

template<std::uint8_t type>
bool is_same_bit_depth(const std::uint8_t runtime_type);

bool is_8bit(const pcl::PCLPointField::PointFieldTypes type);

bool is_8bit(const std::uint8_t type);

bool is_16bit(const pcl::PCLPointField::PointFieldTypes type);

bool is_16bit(const std::uint8_t type);

bool is_32bit(const pcl::PCLPointField::PointFieldTypes type);

bool is_32bit(const std::uint8_t type);

bool is_64bit(const pcl::PCLPointField::PointFieldTypes type);

bool is_64bit(const std::uint8_t type);

template<typename T>
T max(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

template<typename T>
T max(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name);

std::string max_str(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

template<typename T = double>
T mean(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

template<typename T = double>
T mean(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name);

template<typename T>
T min(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

template<typename T>
T min(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name);

std::string min_str(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

std::uint32_t point_step(const pcl::PCLPointField& last_field);

pcl::PCLPointCloud2 remove_field(const pcl::PCLPointCloud2& src, const std::string& name);

pcl::PCLPointCloud2 remove_fields(const pcl::PCLPointCloud2& src, const std::vector<std::string>& names);

void resize(pcl::PCLPointCloud2& pointcloud, const std::uint32_t width, const std::uint32_t height = 1);

std::uint32_t row_step(const pcl::PCLPointCloud2& pointcloud);

void scale_field(pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field, const double scale);

void scale_field(pcl::PCLPointCloud2& pointcloud, const std::string& name, const double scale);

/**
 * @brief Warning: This function should be used for debugging or singular data acquisition. Prefer to access data
 * through the functions generated by create_set_field_data_function(field), especially if accessing the data of many
 * points.
 *
 * @tparam InT
 * @param pointcloud
 * @param field
 * @param i
 * @param value
 */
template<typename InT>
void set_field_data(pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field, const std::size_t i,
        const InT value);

std::uint32_t size_bytes(const pcl::PCLPointCloud2& pointcloud);

std::uint32_t size_points(const pcl::PCLPointCloud2& pointcloud);

template<typename T = float>
std::vector<Eigen::Matrix<T, 3, 1>> spherical_coordinates(const pcl::PCLPointCloud2& pointcloud);

template<typename T = double>
T standard_deviation(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field, const double mean);

template<typename T = double>
T standard_deviation(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

template<typename T = double>
T standard_deviation(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name, const double mean);

template<typename T = double>
T standard_deviation(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name);

statistics_msgs::SummaryStatistics statistics(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

statistics_msgs::SummaryStatisticsArray statistics(const pcl::PCLPointCloud2& pointcloud);

template<typename T>
T sum(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

template<typename T>
T sum(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name);

std::string summary(const pcl::PCLPointCloud2& pointcloud);

std::string summary(const pcl::PCLPointCloud2& pointcloud,
        const std::vector<statistics_msgs::SummaryStatistics>& statistics);

std::string to_string(const pcl::PCLPointField& field);

std::string to_string(const pcl::PCLPointField::PointFieldTypes field_type);

template<typename T = float>
Eigen::Matrix<T, 3, Eigen::Dynamic> unit_vectors(const pcl::PCLPointCloud2& pointcloud);

template<typename T = double>
T variance(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field, const double mean);

template<typename T = double>
T variance(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

template<typename T = double>
T variance(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name, const double mean);

template<typename T = double>
T variance(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name);

}

namespace pcl {

bool operator==(const pcl::PCLPointField& f1, const pcl::PCLPointField& f2);

}

#include "pointcloud_tools/impl/pclpointcloud2_utilities.hpp"

#endif
