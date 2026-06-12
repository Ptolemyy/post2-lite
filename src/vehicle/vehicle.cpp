#include "post2/vehicle/vehicle.hpp"

#include <cmath>

namespace post2::vehicle {

Vec3 operator+(const Vec3& lhs, const Vec3& rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 operator-(const Vec3& lhs, const Vec3& rhs)
{
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 operator*(const Vec3& value, double scale)
{
    return {value.x * scale, value.y * scale, value.z * scale};
}

Vec3 operator*(double scale, const Vec3& value)
{
    return value * scale;
}

Vec3 operator/(const Vec3& value, double scale)
{
    return {value.x / scale, value.y / scale, value.z / scale};
}

double dot(const Vec3& lhs, const Vec3& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

double norm(const Vec3& value)
{
    return std::sqrt(dot(value, value));
}

bool operator==(const TankRef& lhs, const TankRef& rhs)
{
    return lhs.stage_name == rhs.stage_name && lhs.tank_name == rhs.tank_name;
}

bool operator!=(const TankRef& lhs, const TankRef& rhs)
{
    return !(lhs == rhs);
}

} // namespace post2::vehicle
