#ifndef LIORF_GPS_UTIL_H
#define LIORF_GPS_UTIL_H

#include <cmath>
#include <algorithm>
#include <iostream>
#include <gdal/ogr_spatialref.h>

constexpr double LIORF_PI = 3.14159265358979323846;
constexpr double LIORF_PIX = LIORF_PI * 3000.0 / 180.0;
constexpr double LIORF_EE = 0.00669342162296594323;
constexpr double LIORF_A = 6378245.0;

namespace GPSUtil {

inline void transformLat(double lng, double lat, double &ret)
{
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * std::sqrt(std::fabs(lng));
    ret += (20.0 * std::sin(6.0 * lng * LIORF_PI) + 20.0 * std::sin(2.0 * lng * LIORF_PI)) * 2.0 / 3.0;
    ret += (20.0 * std::sin(lat * LIORF_PI) + 40.0 * std::sin(lat / 3.0 * LIORF_PI)) * 2.0 / 3.0;
    ret += (160.0 * std::sin(lat / 12.0 * LIORF_PI) + 320.0 * std::sin(lat * LIORF_PI / 30.0)) * 2.0 / 3.0;
}

inline void transformLng(double lng, double lat, double &ret)
{
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * std::sqrt(std::fabs(lng));
    ret += (20.0 * std::sin(6.0 * lng * LIORF_PI) + 20.0 * std::sin(2.0 * lng * LIORF_PI)) * 2.0 / 3.0;
    ret += (20.0 * std::sin(lng * LIORF_PI) + 40.0 * std::sin(lng / 3.0 * LIORF_PI)) * 2.0 / 3.0;
    ret += (150.0 * std::sin(lng / 12.0 * LIORF_PI) + 300.0 * std::sin(lng / 30.0 * LIORF_PI)) * 2.0 / 3.0;
}

inline bool outOfChina(double lat, double lon)
{
    return (lon < 72.004 || lon > 137.8347) || (lat < 0.8293 || lat > 55.8271);
}

inline void wgs84_to_gcj02(double lng, double lat, double &out_lng, double &out_lat)
{
    double dlat, dlng;
    transformLat(lng - 105.0, lat - 35.0, dlat);
    transformLng(lng - 105.0, lat - 35.0, dlng);

    const double radlat = lat / 180.0 * LIORF_PI;
    double magic = std::sin(radlat);
    magic = 1 - LIORF_EE * magic * magic;
    const double sqrtmagic = std::sqrt(magic);

    dlat = (dlat * 180.0) / ((LIORF_A * (1 - LIORF_EE)) / (magic * sqrtmagic) * LIORF_PI);
    dlng = (dlng * 180.0) / (LIORF_A / sqrtmagic * std::cos(radlat) * LIORF_PI);

    out_lng = lng + dlng;
    out_lat = lat + dlat;
}

inline bool wgs84_to_cgcs2000_3deg(double lon, double lat, double &x, double &y, int lon0 = 117)
{
    OGRSpatialReference oSrcSRS;
    oSrcSRS.importFromEPSG(4326);
    oSrcSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    OGRSpatialReference oDstSRS;
    const char *proj4Str = CPLSPrintf(
        "+proj=tmerc +lat_0=0 +lon_0=%d +k=1.0 +x_0=500000 +y_0=0 +ellps=GRS80 +units=m +no_defs", lon0);
    oDstSRS.importFromProj4(proj4Str);
    oDstSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    OGRCoordinateTransformation *poCT = OGRCreateCoordinateTransformation(&oSrcSRS, &oDstSRS);
    if (poCT == nullptr) {
        std::cerr << "Error: Failed to create coordinate transformation." << std::endl;
        return false;
    }

    x = lon;
    y = lat;
    if (!poCT->Transform(1, &x, &y)) {
        std::cerr << "Error: Transformation failed." << std::endl;
        delete poCT;
        return false;
    }

    delete poCT;
    return true;
}

inline bool ConvertLonLatToXY(double lon, double lat, double &x, double &y, int lon0 = 117)
{
    double out_lng, out_lat;
    wgs84_to_gcj02(lon, lat, out_lng, out_lat);
    return wgs84_to_cgcs2000_3deg(out_lng, out_lat, x, y, lon0);
}

}  // namespace GPSUtil

#endif  // LIORF_GPS_UTIL_H
