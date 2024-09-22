#include "pointcloud_tools/pclpointcloud2_utilities.hpp"

#include <pcl/PCLPointField.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sstream>

namespace pct {

pcl::PCLPointCloud2 add_field(const pcl::PCLPointCloud2& src, const std::string& name,
        const pcl::PCLPointField::PointFieldTypes datatype, const std::uint32_t count) {
    return add_fields(src, {name}, datatype, count);
}

pcl::PCLPointCloud2 add_fields(const pcl::PCLPointCloud2& src, const std::vector<std::string>& names,
        const pcl::PCLPointField::PointFieldTypes datatype, const std::uint32_t count) {
    pcl::PCLPointCloud2 dest;
    dest.header = src.header;
    dest.height = src.height;
    dest.width = src.width;
    dest.fields = src.fields;
    for (const auto& name : names) {
        dest.fields.emplace_back(pcl::PCLPointField{.name = name,
                .offset = dest.fields.empty() ? 0 : point_step(dest.fields.back()),
                .datatype = datatype,
                .count = count});
    }
    dest.is_bigendian = src.is_bigendian;
    dest.point_step = point_step(dest.fields.back());
    dest.row_step = row_step(dest);
    dest.data.resize(size_bytes(dest));
    dest.is_dense = src.is_dense;
    for (std::size_t i = 0, j = 0; i < dest.data.size(); i += dest.point_step, j += src.point_step) {
        std::memcpy(&dest.data[i], &src.data[j], src.point_step);
    }
    return dest;
}

pcl::PCLPointCloud2 add_unit_vectors(const pcl::PCLPointCloud2& src) {
    pcl::PCLPointCloud2 dest = add_fields(src, {"ux", "uy", "uz"}, pcl::PCLPointField::PointFieldTypes::FLOAT32, 1);
    auto get_x_field_data = create_get_field_data_function<float>(get_field(dest, "x"));
    auto get_y_field_data = create_get_field_data_function<float>(get_field(dest, "y"));
    auto get_z_field_data = create_get_field_data_function<float>(get_field(dest, "z"));
    auto set_ux_field_data = create_set_field_data_function<float, float>(get_field(dest, "ux"));
    auto set_uy_field_data = create_set_field_data_function<float, float>(get_field(dest, "uy"));
    auto set_uz_field_data = create_set_field_data_function<float, float>(get_field(dest, "uz"));
    const std::size_t num_points = size_points(dest);
    for (std::size_t i = 0; i < num_points; ++i) {
        const Eigen::Vector3f unit_vector = Eigen::Matrix<float, 3, 1>{get_x_field_data(dest, i),
                get_y_field_data(dest, i), get_z_field_data(dest, i)}
                                                    .normalized();
        set_ux_field_data(dest, i, unit_vector[0]);
        set_uy_field_data(dest, i, unit_vector[1]);
        set_uz_field_data(dest, i, unit_vector[2]);
    }
    return dest;
}

void change_field_name(pcl::PCLPointCloud2& pointcloud, const std::string& from, const std::string& to) {
    get_field(pointcloud, from).name = to;
}

int check_normals(const pcl::PCLPointCloud2& pointcloud, const float threshold) {
    auto get_normal_x_field_data = create_get_field_data_function<float>(get_field(pointcloud, "normal_x"));
    auto get_normal_y_field_data = create_get_field_data_function<float>(get_field(pointcloud, "normal_y"));
    auto get_normal_z_field_data = create_get_field_data_function<float>(get_field(pointcloud, "normal_z"));
    int unnormalised_count{0};
    const std::size_t num_points = size_points(pointcloud);
    for (std::size_t i = 0; i < num_points; ++i) {
        const Eigen::Vector3f normal{get_normal_x_field_data(pointcloud, i), get_normal_y_field_data(pointcloud, i),
                get_normal_z_field_data(pointcloud, i)};
        if (std::abs(normal.norm() - 1.f) > threshold) {
            ++unnormalised_count;
        }
    }
    return unnormalised_count;
}

/**
 * @brief Template specialisation of create_get_field_data_function<InT, T> for when OutT == T == std::uint8_t.
 *
 * It is more efficient as we can directly access the vector data.
 *
 * @tparam
 * @param field
 * @return auto
 */
template<>
auto create_get_field_data_function<std::uint8_t, std::uint8_t>(const pcl::PCLPointField& field) {
    if (pcl::traits::asEnum<std::uint8_t>::value != field.datatype) {
        throw std::runtime_error("datatype did not match type T");
    }
    const std::size_t offset = field.offset;
    return [offset](const pcl::PCLPointCloud2& pointcloud, const std::size_t i) -> std::uint8_t {
        return pointcloud.data[i * pointcloud.point_step + offset];
    };
}

/**
 * @brief Template specialisation of create_set_field_data_function<InT, T> for when InT == T == std::uint8_t.
 *
 * It is more efficient as we can directly access the vector data.
 *
 * @tparam
 * @param field
 * @return auto
 */
template<>
auto create_set_field_data_function<std::uint8_t, std::uint8_t>(const pcl::PCLPointField& field) {
    if (pcl::traits::asEnum<std::uint8_t>::value != field.datatype) {
        throw std::runtime_error("datatype did not match type std::uint8_t");
    }
    const std::size_t offset = field.offset;
    return [offset](pcl::PCLPointCloud2& pointcloud, const std::size_t i, const std::uint8_t value) -> void {
        pointcloud.data[i * pointcloud.point_step + offset] = value;
    };
}

pcl::PCLPointCloud2 deskew_absolute_constant_twist(const Eigen::Isometry3d& skew, const std::uint64_t skew_start_time,
        const std::uint64_t new_time, const double dt, const std::string& time_field,
        const double time_ratio_to_seconds, const pcl::PCLPointCloud2& src) {
    // Error handling
    if (dt <= 0.0) {
        throw std::runtime_error("dt cannot be <= 0.0 for deskewing");
    }

    // Create interp coefficient function for time field as an offset
    auto get_t_field_data = create_get_field_data_function<double>(get_field(src, time_field));
    const double skew_start_time_s = 1.0e-6 * static_cast<double>(skew_start_time);
    auto interp_coeff_function = [get_t_field_data, dt, time_ratio_to_seconds, skew_start_time_s](
                                         const pcl::PCLPointCloud2& src, const std::size_t i) -> double {
        return (time_ratio_to_seconds * get_t_field_data(src, i) - skew_start_time_s) / dt;
    };

    // Create deskew point cloud without time field
    pcl::PCLPointCloud2 dest = remove_field(src, time_field);
    deskew_constant_twist(skew, skew_start_time, new_time, dt, interp_coeff_function, src, dest);
    return dest;
}

pcl::PCLPointCloud2 deskew_absolute_constant_twist(const std::vector<std::pair<double, Eigen::Isometry3d>>& poses,
        const std::uint64_t new_time, const std::string& time_field, const double time_ratio_to_seconds,
        const pcl::PCLPointCloud2& src) {
    auto get_t_field_data = create_get_field_data_function<double>(get_field(src, time_field));
    auto get_point_time = [get_t_field_data, time_ratio_to_seconds](const pcl::PCLPointCloud2& src,
                                  const std::size_t i) -> double {
        return time_ratio_to_seconds * get_t_field_data(src, i);
    };

    // Create deskew point cloud without time field
    pcl::PCLPointCloud2 dest = remove_field(src, time_field);
    deskew_constant_twist(poses, new_time, get_point_time, src, dest);
    return dest;
}

pcl::PCLPointCloud2 deskew_offset_constant_twist(const Eigen::Isometry3d& skew, const std::uint64_t skew_start_time,
        const std::uint64_t new_time, const double dt, const std::string& time_field,
        const double time_ratio_to_seconds, const pcl::PCLPointCloud2& src) {
    // Error handling
    if (dt <= 0.0) {
        throw std::runtime_error("dt cannot be <= 0.0 for deskewing");
    }

    // Create interp coefficient function for time field as an offset
    auto get_t_field_data = create_get_field_data_function<double>(get_field(src, time_field));
    auto interp_coeff_function = [get_t_field_data, dt, time_ratio_to_seconds](const pcl::PCLPointCloud2& src,
                                         const std::size_t i) -> double {
        return time_ratio_to_seconds * get_t_field_data(src, i) / dt;
    };

    // Create deskew point cloud without time field
    pcl::PCLPointCloud2 dest = remove_field(src, time_field);
    deskew_constant_twist(skew, skew_start_time, new_time, dt, interp_coeff_function, src, dest);
    return dest;
}

pcl::PCLPointCloud2 deskew_offset_constant_twist(const std::vector<std::pair<double, Eigen::Isometry3d>>& poses,
        const std::uint64_t new_time, const std::string& time_field, const double time_ratio_to_seconds,
        const pcl::PCLPointCloud2& src) {
    auto get_t_field_data = create_get_field_data_function<double>(get_field(src, time_field));
    const double src_header_s = timestamp_as_seconds(src);
    auto get_point_time = [get_t_field_data, time_ratio_to_seconds, src_header_s](const pcl::PCLPointCloud2& src,
                                  const std::size_t i) -> double {
        return src_header_s + time_ratio_to_seconds * get_t_field_data(src, i);
    };

    // Create deskew point cloud without time field
    pcl::PCLPointCloud2 dest = remove_field(src, time_field);
    deskew_constant_twist(poses, new_time, get_point_time, src, dest);
    return dest;
}

pcl::PCLPointCloud2 deskew_spin_constant_twist(const Eigen::Isometry3d& skew, const std::uint64_t skew_start_time,
        const std::uint64_t new_time, const double dt, const bool spin_cw_from_top, const pcl::PCLPointCloud2& src) {
    // There may be some small overhead in the lambda calls and double access of x,y fields.
    auto get_x_field_data = create_get_field_data_function<double>(get_field(src, "x"));
    auto get_y_field_data = create_get_field_data_function<double>(get_field(src, "y"));
    const double flip = spin_cw_from_top ? -1.0 : 1.0;
    auto interp_coeff_function = [get_x_field_data, get_y_field_data, flip](const pcl::PCLPointCloud2& src,
                                         const std::size_t i) -> double {
        // atan2 -> [-pi, pi]. Take negative angle for CW spin.
        double angle = flip * std::atan2(get_y_field_data(src, i), get_x_field_data(src, i));
        // wrap to [0, 2pi]
        if (angle < 0.0) {
            angle += 2.0 * M_PI;
        }
        // map to [0, 1]
        return angle / (2.0 * M_PI);
    };

    // Create deskew point cloud with all fields
    pcl::PCLPointCloud2 dest = src;
    deskew_constant_twist(skew, skew_start_time, new_time, dt, interp_coeff_function, src, dest);
    return dest;
}

pcl::PCLPointCloud2 deskew_spin_constant_twist(const std::vector<std::pair<double, Eigen::Isometry3d>>& poses,
        const std::uint64_t new_time, const double spin_period, const bool spin_cw_from_top,
        const bool stamped_at_spin_end, const pcl::PCLPointCloud2& src) {
    // There may be some small overhead in the lambda calls and double access of x,y fields.
    auto get_x_field_data = create_get_field_data_function<double>(get_field(src, "x"));
    auto get_y_field_data = create_get_field_data_function<double>(get_field(src, "y"));
    const double src_start_s = timestamp_as_seconds(src) - (stamped_at_spin_end ? spin_period : 0.0);
    const double flip = spin_cw_from_top ? -1.0 : 1.0;
    auto get_point_time = [get_x_field_data, get_y_field_data, src_start_s, spin_period, flip](
                                  const pcl::PCLPointCloud2& src, const std::size_t i) -> double {
        // atan2 -> [-pi, pi].
        double angle = flip * std::atan2(get_y_field_data(src, i), get_x_field_data(src, i));
        // wrap to [0, 2pi]
        if (angle < 0.0) {
            angle += 2.0 * M_PI;
        }
        // map to [0, spin_period]
        const double t_offset = spin_period * angle / (2.0 * M_PI);
        // add offset to start time
        return src_start_s + t_offset;
    };

    // Create deskew point cloud with all fields
    pcl::PCLPointCloud2 dest = src;
    deskew_constant_twist(poses, new_time, get_point_time, src, dest);
    return dest;
}

bool empty(const pcl::PCLPointCloud2& pointcloud) {
    return size_points(pointcloud) == 0;
}

std::string to_string(const pcl::PCLPointField& field) {
    std::stringstream ss;
    ss << "name: " << field.name << ", offset: " << field.offset
       << ", datatype: " << field_type_to_string(field.datatype) << ", count: " << field.count;
    return ss.str();
}

std::string field_type_to_string(const std::uint8_t field_type) {
    return to_string(static_cast<pcl::PCLPointField::PointFieldTypes>(field_type));
}

const pcl::PCLPointField& get_field(const pcl::PCLPointCloud2& pointcloud, const std::string& name) {
    for (const auto& field : pointcloud.fields) {
        if (field.name == name) {
            return field;
        }
    }
    throw std::runtime_error("Field " + name + " not found in PCLPointCloud2");
}

pcl::PCLPointField& get_field(pcl::PCLPointCloud2& pointcloud, const std::string& name) {
    for (auto& field : pointcloud.fields) {
        if (field.name == name) {
            return field;
        }
    }
    throw std::runtime_error("Field " + name + " not found in PCLPointCloud2");
}

bool has_field(const pcl::PCLPointCloud2& pointcloud, const std::string& name) {
    for (const auto& field : pointcloud.fields) {
        if (field.name == name) {
            return true;
        }
    }
    return false;
}

bool is_8bit(const pcl::PCLPointField::PointFieldTypes type) {
    switch (type) {
        case pcl::PCLPointField::PointFieldTypes::INT8:   // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT8:  // fallthrough
            return true;
        case pcl::PCLPointField::PointFieldTypes::INT16:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT16:   // fallthrough
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:  // fallthrough
        case pcl::PCLPointField::PointFieldTypes::INT32:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT32:   // fallthrough
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return false;
        default:
            throw std::runtime_error("pcl::PCLPointField::PointFieldTypes not recognised.");
    }
}

bool is_8bit(const std::uint8_t type) {
    return is_8bit(static_cast<pcl::PCLPointField::PointFieldTypes>(type));
}

bool is_16bit(const pcl::PCLPointField::PointFieldTypes type) {
    switch (type) {
        case pcl::PCLPointField::PointFieldTypes::INT16:   // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT16:  // fallthrough
            return true;
        case pcl::PCLPointField::PointFieldTypes::INT8:     // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT8:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:  // fallthrough
        case pcl::PCLPointField::PointFieldTypes::INT32:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT32:   // fallthrough
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return false;
        default:
            throw std::runtime_error("pcl::PCLPointField::PointFieldTypes not recognised.");
    }
}

bool is_16bit(const std::uint8_t type) {
    return is_16bit(static_cast<pcl::PCLPointField::PointFieldTypes>(type));
}

bool is_32bit(const pcl::PCLPointField::PointFieldTypes type) {
    switch (type) {
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:  // fallthrough
        case pcl::PCLPointField::PointFieldTypes::INT32:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT32:
            return true;
        case pcl::PCLPointField::PointFieldTypes::INT8:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT8:   // fallthrough
        case pcl::PCLPointField::PointFieldTypes::INT16:   // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT16:  // fallthrough
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return false;
        default:
            throw std::runtime_error("pcl::PCLPointField::PointFieldTypes not recognised.");
    }
}

bool is_32bit(const std::uint8_t type) {
    return is_32bit(static_cast<pcl::PCLPointField::PointFieldTypes>(type));
}

bool is_64bit(const pcl::PCLPointField::PointFieldTypes type) {
    switch (type) {
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return true;
        case pcl::PCLPointField::PointFieldTypes::INT8:     // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT8:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::INT16:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT16:   // fallthrough
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:  // fallthrough
        case pcl::PCLPointField::PointFieldTypes::INT32:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT32:
            return false;
        default:
            throw std::runtime_error("pcl::PCLPointField::PointFieldTypes not recognised.");
    }
}

bool is_64bit(const std::uint8_t type) {
    return is_64bit(static_cast<pcl::PCLPointField::PointFieldTypes>(type));
}

std::string max_str(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    switch (field.datatype) {
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:
            return std::to_string(max<float>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return std::to_string(max<double>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::INT8:
            return std::to_string(max<std::int8_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT8:
            return std::to_string(max<std::uint8_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::INT16:
            return std::to_string(max<std::int16_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT16:
            return std::to_string(max<std::uint16_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::INT32:
            return std::to_string(max<std::int32_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT32:
            return std::to_string(max<std::uint32_t>(pointcloud, field));
        default:
            throw std::runtime_error("Failed to get max value string");
    }
}

std::string min_str(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    switch (field.datatype) {
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:
            return std::to_string(min<float>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return std::to_string(min<double>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::INT8:
            return std::to_string(min<std::int8_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT8:
            return std::to_string(min<std::uint8_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::INT16:
            return std::to_string(min<std::int16_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT16:
            return std::to_string(min<std::uint16_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::INT32:
            return std::to_string(min<std::int32_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT32:
            return std::to_string(min<std::uint32_t>(pointcloud, field));
        default:
            throw std::runtime_error("Failed to get max value string");
    }
}

std::uint32_t point_step(const pcl::PCLPointField& last_field) {
    return last_field.offset + pcl::getFieldSize(last_field.datatype) * last_field.count;
}

pcl::PCLPointCloud2 remove_field(const pcl::PCLPointCloud2& src, const std::string& name) {
    return remove_fields(src, {name});
}

pcl::PCLPointCloud2 remove_fields(const pcl::PCLPointCloud2& src, const std::vector<std::string>& names) {
    pcl::PCLPointCloud2 dest;
    dest.header = src.header;
    dest.height = src.height;
    dest.width = src.width;
    std::vector<pcl::PCLPointField> retained_src_fields;
    for (const auto& field : src.fields) {
        if (std::find(names.cbegin(), names.cend(), field.name) == names.cend()) {
            retained_src_fields.emplace_back(field);
            dest.fields.emplace_back(pcl::PCLPointField{.name = field.name,
                    .offset = dest.fields.empty() ? 0 : point_step(dest.fields.back()),
                    .datatype = field.datatype,
                    .count = field.count});
        }
    }
    dest.is_bigendian = src.is_bigendian;
    dest.point_step = point_step(dest.fields.back());
    dest.row_step = row_step(dest);
    dest.data.resize(size_bytes(dest));
    dest.is_dense = src.is_dense;
    // Copy data field by field
    for (std::size_t f = 0; f < dest.fields.size(); ++f) {
        const pcl::PCLPointField& src_field = retained_src_fields[f];
        const pcl::PCLPointField& dest_field = dest.fields[f];
        const std::size_t field_bytes = pcl::getFieldSize(dest_field.datatype) * dest_field.count;
        for (std::size_t i = 0, j = 0; i < dest.data.size(); i += dest.point_step, j += src.point_step) {
            std::memcpy(&dest.data[i + dest_field.offset], &src.data[j + src_field.offset], field_bytes);
        }
    }
    return dest;
}

void resize(pcl::PCLPointCloud2& pointcloud, const std::uint32_t width, const std::uint32_t height) {
    pointcloud.height = height;
    pointcloud.width = width;
    pointcloud.row_step = row_step(pointcloud);
    pointcloud.data.resize(size_bytes(pointcloud));
}

std::uint32_t row_step(const pcl::PCLPointCloud2& pointcloud) {
    return pointcloud.point_step * pointcloud.width;
}

void scale_field(pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field, const double scale) {
    auto get_field_data = create_get_field_data_function<double>(field);
    auto set_field_data = create_set_field_data_function<double>(field);
    const std::size_t num_points = size_points(pointcloud);
    for (std::size_t i = 0; i < num_points; ++i) {
        set_field_data(pointcloud, i, get_field_data(pointcloud, i) * scale);
    }
}

void scale_field(pcl::PCLPointCloud2& pointcloud, const std::string& name, const double scale) {
    return scale_field(pointcloud, get_field(pointcloud, name), scale);
}

// Could also use pointcloud.data.size(), which should be identical if the metadata is correct
std::uint32_t size_bytes(const pcl::PCLPointCloud2& pointcloud) {
    return pointcloud.height * pointcloud.row_step;
}

std::uint32_t size_points(const pcl::PCLPointCloud2& pointcloud) {
    return pointcloud.height * pointcloud.width;
}

double timestamp_as_seconds(const pcl::PCLPointCloud2& pointcloud) {
    // Convert from (unsigned) integer microseconds
    return 1.0e-6 * static_cast<double>(pointcloud.header.stamp);
}

std::string to_string(const pcl::PCLPointField::PointFieldTypes field_type) {
    switch (field_type) {
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:
            return "FLOAT32";
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return "FLOAT64";
        case pcl::PCLPointField::PointFieldTypes::INT8:
            return "INT8";
        case pcl::PCLPointField::PointFieldTypes::INT16:
            return "INT16";
        case pcl::PCLPointField::PointFieldTypes::INT32:
            return "INT32";
        case pcl::PCLPointField::PointFieldTypes::UINT8:
            return "UINT8";
        case pcl::PCLPointField::PointFieldTypes::UINT16:
            return "UINT16";
        case pcl::PCLPointField::PointFieldTypes::UINT32:
            return "UINT32";
        default:
            throw std::runtime_error("Unknown PointFieldType " + std::to_string(field_type));
    }
}

}

namespace pcl {

bool operator==(const pcl::PCLPointField& f1, const pcl::PCLPointField& f2) {
    return f1.name == f2.name && f1.offset == f2.offset && f1.datatype == f2.datatype && f1.count == f2.count;
}

}
